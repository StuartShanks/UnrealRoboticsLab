// Copyright (c) 2026 Jonathan Embley-Riches. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// --- LEGAL DISCLAIMER ---
// UnrealRoboticsLab is an independent software plugin. It is NOT affiliated with,
// endorsed by, or sponsored by Epic Games, Inc. "Unreal" and "Unreal Engine" are
// trademarks or registered trademarks of Epic Games, Inc. in the US and elsewhere.
//
// This plugin incorporates third-party software: MuJoCo (Apache 2.0),
// CoACD (MIT), and libzmq (MPL 2.0). See ThirdPartyNotices.txt for details.

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "MinkConfiguration.h"
#include "MinkSolveIK.h"
#include "MinkTestUtils.h"
#include "Tasks/MinkFrameTask.h"
#include "Tasks/MinkPostureTask.h"
#include "Tasks/MinkDampingTask.h"
#include "Tasks/MinkComTask.h"
#include "Tasks/MinkDofFreezingTask.h"
#include "Limits/MinkLimit.h"
#include "Limits/MinkConfigurationLimit.h"
#include "Limits/MinkCollisionAvoidanceLimit.h"

#if WITH_DEV_AUTOMATION_TESTS

// NOTE: TOL_IK collides in spirit with TOL_QP/TOL_OBJ/TOL_LIE/TOL_KIN/TOL_OBJ_LIMITS already
// defined as file-statics in sibling test files; with unity builds concatenating all
// Private/*.cpp into one translation unit, a differently-named constant avoids any redefinition.
static const double TOL_IK_SOLVE = 1e-6;

namespace
{
/** gen_solve_ik's task descriptors store cost fields (position_cost/orientation_cost/cost) as a
 * bare JSON number when the python call site passed a python float (mink itself broadcasts a
 * scalar to a 1-vector internally via np.atleast_1d) — but as a JSON array when a full vector
 * was passed. Handle both so the C++ side mirrors mink's implicit broadcast. */
FMinkVec JsonCostVec(const TSharedPtr<FJsonObject>& D, const TCHAR* Field)
{
	if (D->HasTypedField<EJson::Array>(Field))
	{
		return MinkJsonVec(D->GetArrayField(Field));
	}
	FMinkVec V(1);
	V(0) = D->GetNumberField(Field);
	return V;
}

/** Reconstructs one scenario's task list from its JSON descriptor array (mirrors gen_solve_ik's
 * `tasks_desc` dispatch in gen_golden.py). Frame/posture/damping/com are all supported even
 * though the current fixture only exercises frame/posture/damping. */
bool BuildTasksFromJson(const TArray<TSharedPtr<FJsonValue>>& TasksJson, const mjModel* Model,
	TArray<TUniquePtr<FMinkBaseTask>>& OutOwnedTasks, TArray<const FMinkBaseTask*>& OutTaskPtrs)
{
	for (const TSharedPtr<FJsonValue>& TaskVal : TasksJson)
	{
		const TSharedPtr<FJsonObject> D = TaskVal->AsObject();
		const FString Type = D->GetStringField(TEXT("type"));

		if (Type == TEXT("frame"))
		{
			const FString FrameName = D->GetStringField(TEXT("frame"));
			EMinkFrameType FrameType;
			MinkFrameTypeFromString(D->GetStringField(TEXT("frame_type")), FrameType);
			const FMinkVec PositionCost = JsonCostVec(D, TEXT("position_cost"));
			const FMinkVec OrientationCost = JsonCostVec(D, TEXT("orientation_cost"));
			const double Gain = D->HasField(TEXT("gain")) ? D->GetNumberField(TEXT("gain")) : 1.0;
			const double Lm = D->HasField(TEXT("lm")) ? D->GetNumberField(TEXT("lm")) : 0.0;
			const FMinkVec Target = MinkJsonVec(D->GetArrayField(TEXT("target")));

			TUniquePtr<FMinkFrameTask> Task =
				MakeUnique<FMinkFrameTask>(FrameName, FrameType, PositionCost, OrientationCost, Gain, Lm);
			if (!Task->bIsValid)
			{
				return false;
			}
			Task->SetTarget(FMinkSE3::FromWxyzXyz(Target.data()));
			OutTaskPtrs.Add(Task.Get());
			OutOwnedTasks.Add(MoveTemp(Task));
		}
		else if (Type == TEXT("posture"))
		{
			const FMinkVec Cost = JsonCostVec(D, TEXT("cost"));
			const double Gain = D->HasField(TEXT("gain")) ? D->GetNumberField(TEXT("gain")) : 1.0;
			const double Lm = D->HasField(TEXT("lm")) ? D->GetNumberField(TEXT("lm")) : 0.0;
			const FMinkVec TargetQ = MinkJsonVec(D->GetArrayField(TEXT("target_q")));

			TUniquePtr<FMinkPostureTask> Task = MakeUnique<FMinkPostureTask>(Model, Cost, Gain, Lm);
			if (!Task->bIsValid || !Task->SetTarget(TargetQ))
			{
				return false;
			}
			OutTaskPtrs.Add(Task.Get());
			OutOwnedTasks.Add(MoveTemp(Task));
		}
		else if (Type == TEXT("damping"))
		{
			const FMinkVec Cost = JsonCostVec(D, TEXT("cost"));
			TUniquePtr<FMinkDampingTask> Task = MakeUnique<FMinkDampingTask>(Model, Cost);
			if (!Task->bIsValid)
			{
				return false;
			}
			OutTaskPtrs.Add(Task.Get());
			OutOwnedTasks.Add(MoveTemp(Task));
		}
		else if (Type == TEXT("com"))
		{
			const FMinkVec Cost = JsonCostVec(D, TEXT("cost"));
			const double Gain = D->HasField(TEXT("gain")) ? D->GetNumberField(TEXT("gain")) : 1.0;
			const double Lm = D->HasField(TEXT("lm")) ? D->GetNumberField(TEXT("lm")) : 0.0;
			const FMinkVec TargetCom = MinkJsonVec(D->GetArrayField(TEXT("target_com")));

			TUniquePtr<FMinkComTask> Task = MakeUnique<FMinkComTask>(Cost, Gain, Lm);
			if (!Task->bIsValid || !Task->SetTarget(FMinkVec3(TargetCom(0), TargetCom(1), TargetCom(2))))
			{
				return false;
			}
			OutTaskPtrs.Add(Task.Get());
			OutOwnedTasks.Add(MoveTemp(Task));
		}
		else
		{
			return false;
		}
	}
	return true;
}

/** Reconstructs the "constraints" JSON array (null or a list of {"dofs": [...]}) as
 * FMinkDofFreezingTask instances (mirrors gen_solve_ik's DofFreezingTask construction). */
bool BuildConstraintsFromJson(const TSharedPtr<FJsonObject>& Scenario, const mjModel* Model,
	TArray<TUniquePtr<FMinkTask>>& OutOwnedConstraints, TArray<const FMinkTask*>& OutConstraintPtrs)
{
	const TArray<TSharedPtr<FJsonValue>>* ConstraintsJson = nullptr;
	if (!Scenario->TryGetArrayField(TEXT("constraints"), ConstraintsJson) || ConstraintsJson == nullptr)
	{
		return true;
	}
	for (const TSharedPtr<FJsonValue>& ConstraintVal : *ConstraintsJson)
	{
		const TSharedPtr<FJsonObject> D = ConstraintVal->AsObject();
		TArray<int32> Dofs;
		for (const TSharedPtr<FJsonValue>& DofVal : D->GetArrayField(TEXT("dofs")))
		{
			Dofs.Add(static_cast<int32>(DofVal->AsNumber()));
		}
		TUniquePtr<FMinkDofFreezingTask> Task = MakeUnique<FMinkDofFreezingTask>(Model, Dofs);
		if (!Task->bIsValid)
		{
			return false;
		}
		OutConstraintPtrs.Add(Task.Get());
		OutOwnedConstraints.Add(MoveTemp(Task));
	}
	return true;
}

/** Reconstructs the "limits" JSON field, which is either the plain string mode ("default" ->
 * nullptr, meaning MinkSolveIK's own default single ConfigurationLimit; "none" -> an explicit
 * empty array) or — for the collision-avoidance scenario — a rich object descriptor naming an
 * explicit ["configuration"?, "collision"?] limit list (mirrors gen_solve_ik's limits_desc).
 * When explicit, OutLimitPtrs is populated and bOutExplicit is set so the caller passes
 * &OutLimitPtrs rather than nullptr/&EmptyLimits. */
bool BuildLimitsFromJson(const TSharedPtr<FJsonObject>& Scenario, const mjModel* Model,
	TArray<TUniquePtr<FMinkLimit>>& OutOwnedLimits, TArray<const FMinkLimit*>& OutLimitPtrs, bool& bOutExplicit,
	FString& OutLimitsMode)
{
	const TSharedPtr<FJsonObject>* LimitsObj = nullptr;
	if (!Scenario->TryGetObjectField(TEXT("limits"), LimitsObj) || LimitsObj == nullptr)
	{
		bOutExplicit = false;
		OutLimitsMode = Scenario->GetStringField(TEXT("limits"));
		return true;
	}

	bOutExplicit = true;
	const TSharedPtr<FJsonObject>& D = *LimitsObj;

	if (D->HasField(TEXT("configuration")))
	{
		const TSharedPtr<FJsonObject> ConfigDesc = D->GetObjectField(TEXT("configuration"));
		const double Gain = ConfigDesc->GetNumberField(TEXT("gain"));

		TUniquePtr<FMinkConfigurationLimit> Limit = MakeUnique<FMinkConfigurationLimit>(Model, Gain);
		if (!Limit->bIsValid)
		{
			return false;
		}
		OutLimitPtrs.Add(Limit.Get());
		OutOwnedLimits.Add(MoveTemp(Limit));
	}

	if (D->HasField(TEXT("collision")))
	{
		const TSharedPtr<FJsonObject> CollisionDesc = D->GetObjectField(TEXT("collision"));
		const double Gain = CollisionDesc->GetNumberField(TEXT("gain"));
		const double MinDist = CollisionDesc->GetNumberField(TEXT("min_dist"));
		const double DetectionDist = CollisionDesc->GetNumberField(TEXT("detection_dist"));
		const double BoundRelaxation = CollisionDesc->GetNumberField(TEXT("bound_relaxation"));

		TArray<FMinkCollisionPair> GeomPairs;
		for (const TSharedPtr<FJsonValue>& PairVal : CollisionDesc->GetArrayField(TEXT("geom_pairs")))
		{
			const TArray<TSharedPtr<FJsonValue>> PairArr = PairVal->AsArray();
			FMinkGeomGroup GroupA;
			for (const TSharedPtr<FJsonValue>& NameVal : PairArr[0]->AsArray())
			{
				GroupA.Names.Add(NameVal->AsString());
			}
			FMinkGeomGroup GroupB;
			for (const TSharedPtr<FJsonValue>& NameVal : PairArr[1]->AsArray())
			{
				GroupB.Names.Add(NameVal->AsString());
			}
			GeomPairs.Emplace(MoveTemp(GroupA), MoveTemp(GroupB));
		}

		TUniquePtr<FMinkCollisionAvoidanceLimit> Limit =
			MakeUnique<FMinkCollisionAvoidanceLimit>(Model, GeomPairs, Gain, MinDist, DetectionDist, BoundRelaxation);
		if (!Limit->bIsValid)
		{
			return false;
		}
		OutLimitPtrs.Add(Limit.Get());
		OutOwnedLimits.Add(MoveTemp(Limit));
	}

	return true;
}
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMinkSolveIKGoldenTest,
	"URLab.Mink.SolveIK.Golden",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMinkSolveIKGoldenTest::RunTest(const FString& Parameters)
{
	TSharedPtr<FJsonObject> Root;
	if (!MinkLoadFixture(TEXT("solve_ik"), Root))
	{
		AddError(TEXT("solve_ik.json missing — run gen_golden.py"));
		return false;
	}

	const TArray<TSharedPtr<FJsonValue>>& Scenarios = Root->GetArrayField(TEXT("scenarios"));
	TestTrue(TEXT("scenarios non-empty"), Scenarios.Num() > 0);

	for (const TSharedPtr<FJsonValue>& ScenarioVal : Scenarios)
	{
		const TSharedPtr<FJsonObject> Scenario = ScenarioVal->AsObject();

		const FString ModelName = Scenario->GetStringField(TEXT("model"));
		mjModel* Model = MinkLoadModel(ModelName + TEXT(".xml"));
		if (Model == nullptr)
		{
			AddError(FString::Printf(TEXT("failed to load model '%s'"), *ModelName));
			continue;
		}

		const FMinkVec Q0 = MinkJsonVec(Scenario->GetArrayField(TEXT("q0")));
		const double Damping = Scenario->GetNumberField(TEXT("damping"));
		const double Dt = Scenario->GetNumberField(TEXT("dt"));
		const int32 Steps = static_cast<int32>(Scenario->GetNumberField(TEXT("steps")));

		TArray<TUniquePtr<FMinkLimit>> OwnedLimits;
		TArray<const FMinkLimit*> LimitPtrs;
		bool bExplicitLimits = false;
		FString LimitsMode;
		if (!TestTrue(TEXT("BuildLimitsFromJson"),
				BuildLimitsFromJson(Scenario, Model, OwnedLimits, LimitPtrs, bExplicitLimits, LimitsMode)))
		{
			mj_deleteModel(Model);
			continue;
		}

		TArray<TUniquePtr<FMinkBaseTask>> OwnedTasks;
		TArray<const FMinkBaseTask*> TaskPtrs;
		if (!TestTrue(TEXT("BuildTasksFromJson"),
				BuildTasksFromJson(Scenario->GetArrayField(TEXT("tasks")), Model, OwnedTasks, TaskPtrs)))
		{
			mj_deleteModel(Model);
			continue;
		}

		TArray<TUniquePtr<FMinkTask>> OwnedConstraints;
		TArray<const FMinkTask*> ConstraintPtrs;
		if (!TestTrue(TEXT("BuildConstraintsFromJson"),
				BuildConstraintsFromJson(Scenario, Model, OwnedConstraints, ConstraintPtrs)))
		{
			mj_deleteModel(Model);
			continue;
		}
		const TArray<const FMinkTask*>* ConstraintsArg = ConstraintPtrs.Num() > 0 ? &ConstraintPtrs : nullptr;

		// "default" -> nullptr (MinkSolveIK defaults to a ConfigurationLimit); "none" -> an
		// explicit empty array (no limits at all); an object descriptor -> the explicit
		// [configuration?, collision?] list just built above.
		TArray<const FMinkLimit*> EmptyLimits;
		const TArray<const FMinkLimit*>* LimitsArg =
			bExplicitLimits ? &LimitPtrs : (LimitsMode == TEXT("none") ? &EmptyLimits : nullptr);

		{
			FMinkConfiguration Cfg(Model, Q0.data());

			const FMinkIKResult Result = MinkSolveIK(Cfg, TaskPtrs, Dt, Damping,
				/*bSafetyBreak=*/false, LimitsArg, ConstraintsArg);
			if (TestTrue(FString::Printf(TEXT("%s: MinkSolveIK success"), *ModelName), Result.IsSuccess()))
			{
				MinkExpectNear(*this, TEXT("v"), Result.Velocity, MinkJsonVec(Scenario->GetArrayField(TEXT("v"))),
					TOL_IK_SOLVE);
			}
		}

		{
			FMinkConfiguration Cfg2(Model, Q0.data());
			bool bAllOk = true;
			for (int32 Step = 0; Step < Steps; ++Step)
			{
				const FMinkIKResult StepResult = MinkSolveIK(Cfg2, TaskPtrs, Dt, Damping,
					/*bSafetyBreak=*/false, LimitsArg, ConstraintsArg);
				if (!StepResult.IsSuccess())
				{
					bAllOk = false;
					break;
				}
				Cfg2.IntegrateInplace(StepResult.Velocity, Dt);
			}
			if (TestTrue(FString::Printf(TEXT("%s: integrate loop success"), *ModelName), bAllOk))
			{
				MinkExpectNear(*this, TEXT("q_final"), Cfg2.GetQ(),
					MinkJsonVec(Scenario->GetArrayField(TEXT("q_final"))), TOL_IK_SOLVE);
			}
		}

		mj_deleteModel(Model);
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMinkSolveIKSafetyBreakTest,
	"URLab.Mink.SolveIK.SafetyBreak",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMinkSolveIKSafetyBreakTest::RunTest(const FString& Parameters)
{
	mjModel* Model = MinkLoadModel(TEXT("arm3.xml"));
	if (Model == nullptr)
	{
		AddError(TEXT("failed to load model 'arm3'"));
		return false;
	}

	// j2 (qpos/dof index 1) has range [-1.9, 1.9]; 3.0 is well outside it.
	double Q[3] = {0.0, 3.0, 0.0};
	FMinkConfiguration Cfg(Model, Q);

	FMinkVec DampingCost(1);
	DampingCost(0) = 1.0;
	FMinkDampingTask DampingTask(Model, DampingCost);
	TArray<const FMinkBaseTask*> Tasks = {&DampingTask};

	AddExpectedErrorPlain(TEXT("violates configuration limits"));
	const FMinkIKResult Result = MinkSolveIK(Cfg, Tasks, /*Dt=*/0.02, /*Damping=*/1e-12, /*bSafetyBreak=*/true);
	TestEqual(TEXT("Status == NotWithinConfigurationLimits"), Result.Status,
		EMinkIKStatus::NotWithinConfigurationLimits);

	mj_deleteModel(Model);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMinkSolveIKTaskErrorTest,
	"URLab.Mink.SolveIK.TaskError",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMinkSolveIKTaskErrorTest::RunTest(const FString& Parameters)
{
	mjModel* Model = MinkLoadModel(TEXT("arm3.xml"));
	if (Model == nullptr)
	{
		AddError(TEXT("failed to load model 'arm3'"));
		return false;
	}

	FMinkVec PositionCost(1);
	PositionCost(0) = 1.0;
	FMinkVec OrientationCost(1);
	OrientationCost(0) = 1.0;
	FMinkFrameTask UnsetTask(TEXT("ee"), EMinkFrameType::Site, PositionCost, OrientationCost);
	TArray<const FMinkBaseTask*> Tasks = {&UnsetTask};

	FMinkConfiguration Cfg(Model);

	AddExpectedErrorPlain(TEXT("No target set for FMinkFrameTask"));
	const FMinkIKResult Result = MinkSolveIK(Cfg, Tasks, /*Dt=*/0.02);
	TestEqual(TEXT("Status == TaskError"), Result.Status, EMinkIKStatus::TaskError);

	mj_deleteModel(Model);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
