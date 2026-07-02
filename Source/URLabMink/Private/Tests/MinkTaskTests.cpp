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
#include "MinkTestUtils.h"
#include "Tasks/MinkComTask.h"
#include "Tasks/MinkDampingTask.h"
#include "Tasks/MinkDofFreezingTask.h"
#include "Tasks/MinkFrameTask.h"
#include "Tasks/MinkKineticEnergyRegularizationTask.h"
#include "Tasks/MinkPostureTask.h"
#include "Tasks/MinkRelativeFrameTask.h"

#if WITH_DEV_AUTOMATION_TESTS

static const double TOL_OBJ = 1e-8;

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMinkTaskPostureTest,
	"URLab.Mink.Tasks.Posture",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMinkTaskPostureTest::RunTest(const FString& Parameters)
{
	TSharedPtr<FJsonObject> Root;
	if (!MinkLoadFixture(TEXT("task_posture"), Root))
	{
		AddError(TEXT("task_posture.json missing — run gen_golden.py"));
		return false;
	}
	const TSharedPtr<FJsonObject> Models = Root->GetObjectField(TEXT("models"));

	static const TArray<FString> ModelNames = {TEXT("arm3"), TEXT("floating")};
	for (const FString& ModelName : ModelNames)
	{
		const TArray<TSharedPtr<FJsonValue>>& Cases = Models->GetArrayField(ModelName);
		TestTrue(FString::Printf(TEXT("%s cases non-empty"), *ModelName), Cases.Num() > 0);
		if (Cases.Num() == 0)
		{
			continue;
		}

		mjModel* Model = MinkLoadModel(ModelName + TEXT(".xml"));
		if (Model == nullptr)
		{
			AddError(FString::Printf(TEXT("failed to load model '%s'"), *ModelName));
			continue;
		}

		{
			FMinkConfiguration Cfg(Model);

			for (const TSharedPtr<FJsonValue>& CaseVal : Cases)
			{
				const TSharedPtr<FJsonObject> Case = CaseVal->AsObject();

				const FMinkVec CostVec = MinkJsonVec(Case->GetArrayField(TEXT("cost")));
				const double Gain = Case->GetNumberField(TEXT("gain"));
				const double Lm = Case->GetNumberField(TEXT("lm"));
				const FMinkVec TargetQ = MinkJsonVec(Case->GetArrayField(TEXT("target_q")));
				const FMinkVec Q = MinkJsonVec(Case->GetArrayField(TEXT("q")));

				FMinkPostureTask Task(Model, CostVec, Gain, Lm);
				if (!TestTrue(TEXT("task.bIsValid"), Task.bIsValid))
				{
					continue;
				}
				if (!TestTrue(TEXT("SetTarget"), Task.SetTarget(TargetQ)))
				{
					continue;
				}
				Cfg.Update(Q.data());

				FMinkVec Error;
				if (TestTrue(TEXT("ComputeError"), Task.ComputeError(Cfg, Error)))
				{
					MinkExpectNear(*this, TEXT("error"), Error, MinkJsonVec(Case->GetArrayField(TEXT("error"))),
						TOL_OBJ);
				}

				FMinkMat Jacobian;
				if (TestTrue(TEXT("ComputeJacobian"), Task.ComputeJacobian(Cfg, Jacobian)))
				{
					MinkExpectNear(*this, TEXT("jacobian"), Jacobian,
						MinkJsonMat(Case->GetArrayField(TEXT("jacobian"))), TOL_OBJ);
				}

				FMinkObjective Objective;
				if (TestTrue(TEXT("ComputeQpObjective"), Task.ComputeQpObjective(Cfg, Objective)))
				{
					MinkExpectNear(*this, TEXT("H"), Objective.H, MinkJsonMat(Case->GetArrayField(TEXT("H"))),
						TOL_OBJ);
					MinkExpectNear(*this, TEXT("c"), Objective.C, MinkJsonVec(Case->GetArrayField(TEXT("c"))),
						TOL_OBJ);
				}

				FMinkResidual Residual;
				if (TestTrue(TEXT("ComputeQpResidual == Ok"),
						Task.ComputeQpResidual(Cfg, Residual) == EMinkTaskStatus::Ok))
				{
					const TSharedPtr<FJsonObject> ResidualJson = Case->GetObjectField(TEXT("residual"));
					MinkExpectNear(*this, TEXT("residual.wjac"), Residual.WeightedJacobian,
						MinkJsonMat(ResidualJson->GetArrayField(TEXT("wjac"))), TOL_OBJ);
					MinkExpectNear(*this, TEXT("residual.werr"), Residual.WeightedError,
						MinkJsonVec(ResidualJson->GetArrayField(TEXT("werr"))), TOL_OBJ);

					const double ExpectedMu = ResidualJson->GetNumberField(TEXT("mu"));
					FMinkMat ActualMuMat(1, 1);
					ActualMuMat(0, 0) = Residual.Mu;
					FMinkMat ExpectedMuMat(1, 1);
					ExpectedMuMat(0, 0) = ExpectedMu;
					MinkExpectNear(*this, TEXT("residual.mu"), ActualMuMat, ExpectedMuMat, TOL_OBJ);
				}
			}
		}

		mj_deleteModel(Model);
	}

	// Validation: gain outside [0, 1] => bIsValid == false; negative cost => SetCost returns false.
	{
		mjModel* Model = MinkLoadModel(TEXT("arm3.xml"));
		if (Model == nullptr)
		{
			AddError(TEXT("failed to load model 'arm3'"));
			return false;
		}

		FMinkVec Cost(1);
		Cost(0) = 1.0;

		AddExpectedErrorPlain(TEXT("`gain` must be in the range [0, 1]"));
		FMinkPostureTask InvalidGainTask(Model, Cost, /*Gain=*/1.5, /*LmDamping=*/0.0);
		TestFalse(TEXT("gain 1.5 => bIsValid false"), InvalidGainTask.bIsValid);

		FMinkPostureTask ValidTask(Model, Cost, /*Gain=*/1.0, /*LmDamping=*/0.0);
		TestTrue(TEXT("ValidTask.bIsValid"), ValidTask.bIsValid);
		const FMinkVec NegativeCost = FMinkVec::Constant(Model->nv, -1.0);
		AddExpectedErrorPlain(TEXT("FMinkPostureTask cost should be >= 0"));
		TestFalse(TEXT("negative cost => SetCost false"), ValidTask.SetCost(NegativeCost));

		mj_deleteModel(Model);
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMinkTaskFrameTest,
	"URLab.Mink.Tasks.Frame",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMinkTaskFrameTest::RunTest(const FString& Parameters)
{
	TSharedPtr<FJsonObject> Root;
	if (!MinkLoadFixture(TEXT("task_frame"), Root))
	{
		AddError(TEXT("task_frame.json missing — run gen_golden.py"));
		return false;
	}
	const TSharedPtr<FJsonObject> Models = Root->GetObjectField(TEXT("models"));

	static const TArray<FString> ModelNames = {TEXT("arm3"), TEXT("floating")};
	for (const FString& ModelName : ModelNames)
	{
		const TArray<TSharedPtr<FJsonValue>>& Cases = Models->GetArrayField(ModelName);
		TestTrue(FString::Printf(TEXT("%s cases non-empty"), *ModelName), Cases.Num() > 0);
		if (Cases.Num() == 0)
		{
			continue;
		}

		mjModel* Model = MinkLoadModel(ModelName + TEXT(".xml"));
		if (Model == nullptr)
		{
			AddError(FString::Printf(TEXT("failed to load model '%s'"), *ModelName));
			continue;
		}

		{
			FMinkConfiguration Cfg(Model);

			for (const TSharedPtr<FJsonValue>& CaseVal : Cases)
			{
				const TSharedPtr<FJsonObject> Case = CaseVal->AsObject();

				const FString FrameName = Case->GetStringField(TEXT("frame"));
				EMinkFrameType FrameType;
				MinkFrameTypeFromString(Case->GetStringField(TEXT("frame_type")), FrameType);
				const FMinkVec PositionCost = MinkJsonVec(Case->GetArrayField(TEXT("position_cost")));
				const FMinkVec OrientationCost = MinkJsonVec(Case->GetArrayField(TEXT("orientation_cost")));
				const double Gain = Case->GetNumberField(TEXT("gain"));
				const double Lm = Case->GetNumberField(TEXT("lm"));
				const FMinkVec Target = MinkJsonVec(Case->GetArrayField(TEXT("target")));
				const FMinkVec Q = MinkJsonVec(Case->GetArrayField(TEXT("q")));

				FMinkFrameTask Task(FrameName, FrameType, PositionCost, OrientationCost, Gain, Lm);
				if (!TestTrue(TEXT("task.bIsValid"), Task.bIsValid))
				{
					continue;
				}
				Task.SetTarget(FMinkSE3::FromWxyzXyz(Target.data()));
				Cfg.Update(Q.data());

				FMinkVec Error;
				if (TestTrue(TEXT("ComputeError"), Task.ComputeError(Cfg, Error)))
				{
					MinkExpectNear(*this, TEXT("error"), Error, MinkJsonVec(Case->GetArrayField(TEXT("error"))),
						TOL_OBJ);
				}

				FMinkMat Jacobian;
				if (TestTrue(TEXT("ComputeJacobian"), Task.ComputeJacobian(Cfg, Jacobian)))
				{
					MinkExpectNear(*this, TEXT("jacobian"), Jacobian,
						MinkJsonMat(Case->GetArrayField(TEXT("jacobian"))), TOL_OBJ);
				}

				FMinkObjective Objective;
				if (TestTrue(TEXT("ComputeQpObjective"), Task.ComputeQpObjective(Cfg, Objective)))
				{
					MinkExpectNear(*this, TEXT("H"), Objective.H, MinkJsonMat(Case->GetArrayField(TEXT("H"))),
						TOL_OBJ);
					MinkExpectNear(*this, TEXT("c"), Objective.C, MinkJsonVec(Case->GetArrayField(TEXT("c"))),
						TOL_OBJ);
				}

				FMinkResidual Residual;
				if (TestTrue(TEXT("ComputeQpResidual == Ok"),
						Task.ComputeQpResidual(Cfg, Residual) == EMinkTaskStatus::Ok))
				{
					const TSharedPtr<FJsonObject> ResidualJson = Case->GetObjectField(TEXT("residual"));
					MinkExpectNear(*this, TEXT("residual.wjac"), Residual.WeightedJacobian,
						MinkJsonMat(ResidualJson->GetArrayField(TEXT("wjac"))), TOL_OBJ);
					MinkExpectNear(*this, TEXT("residual.werr"), Residual.WeightedError,
						MinkJsonVec(ResidualJson->GetArrayField(TEXT("werr"))), TOL_OBJ);

					const double ExpectedMu = ResidualJson->GetNumberField(TEXT("mu"));
					FMinkMat ActualMuMat(1, 1);
					ActualMuMat(0, 0) = Residual.Mu;
					FMinkMat ExpectedMuMat(1, 1);
					ExpectedMuMat(0, 0) = ExpectedMu;
					MinkExpectNear(*this, TEXT("residual.mu"), ActualMuMat, ExpectedMuMat, TOL_OBJ);
				}
			}
		}

		mj_deleteModel(Model);
	}

	// Validation: unset target => ComputeError false; negative position cost => SetPositionCost false.
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

		FMinkFrameTask Task(TEXT("ee"), EMinkFrameType::Site, PositionCost, OrientationCost);
		TestTrue(TEXT("Task.bIsValid"), Task.bIsValid);

		FMinkConfiguration Cfg(Model);
		FMinkVec Error;
		AddExpectedErrorPlain(TEXT("No target set for FMinkFrameTask"));
		TestFalse(TEXT("unset target => ComputeError false"), Task.ComputeError(Cfg, Error));

		const FMinkVec NegativePositionCost = FMinkVec::Constant(3, -1.0);
		AddExpectedErrorPlain(TEXT("FMinkFrameTask position cost should be >= 0"));
		TestFalse(TEXT("negative position cost => SetPositionCost false"), Task.SetPositionCost(NegativePositionCost));

		const FMinkVec NegativeOrientationCost = FMinkVec::Constant(3, -1.0);
		AddExpectedErrorPlain(TEXT("FMinkFrameTask position cost should be >= 0")); // parity: mink v1.2.0's orientation setter really says "position cost" (upstream copy-paste quirk)
		TestFalse(TEXT("negative orientation cost => SetOrientationCost false"), Task.SetOrientationCost(NegativeOrientationCost));

		mj_deleteModel(Model);
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMinkTaskRelativeFrameTest,
	"URLab.Mink.Tasks.RelativeFrame",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMinkTaskRelativeFrameTest::RunTest(const FString& Parameters)
{
	TSharedPtr<FJsonObject> Root;
	if (!MinkLoadFixture(TEXT("task_relative_frame"), Root))
	{
		AddError(TEXT("task_relative_frame.json missing — run gen_golden.py"));
		return false;
	}
	const TSharedPtr<FJsonObject> Models = Root->GetObjectField(TEXT("models"));

	static const TArray<FString> ModelNames = {TEXT("arm3"), TEXT("floating")};
	for (const FString& ModelName : ModelNames)
	{
		const TArray<TSharedPtr<FJsonValue>>& Cases = Models->GetArrayField(ModelName);
		TestTrue(FString::Printf(TEXT("%s cases non-empty"), *ModelName), Cases.Num() > 0);
		if (Cases.Num() == 0)
		{
			continue;
		}

		mjModel* Model = MinkLoadModel(ModelName + TEXT(".xml"));
		if (Model == nullptr)
		{
			AddError(FString::Printf(TEXT("failed to load model '%s'"), *ModelName));
			continue;
		}

		{
			FMinkConfiguration Cfg(Model);

			for (const TSharedPtr<FJsonValue>& CaseVal : Cases)
			{
				const TSharedPtr<FJsonObject> Case = CaseVal->AsObject();

				const FString FrameName = Case->GetStringField(TEXT("frame"));
				EMinkFrameType FrameType;
				MinkFrameTypeFromString(Case->GetStringField(TEXT("frame_type")), FrameType);
				const FString RootName = Case->GetStringField(TEXT("root"));
				EMinkFrameType RootType;
				MinkFrameTypeFromString(Case->GetStringField(TEXT("root_type")), RootType);
				const FMinkVec PositionCost = MinkJsonVec(Case->GetArrayField(TEXT("position_cost")));
				const FMinkVec OrientationCost = MinkJsonVec(Case->GetArrayField(TEXT("orientation_cost")));
				const double Gain = Case->GetNumberField(TEXT("gain"));
				const double Lm = Case->GetNumberField(TEXT("lm"));
				const FMinkVec Target = MinkJsonVec(Case->GetArrayField(TEXT("target")));
				const FMinkVec Q = MinkJsonVec(Case->GetArrayField(TEXT("q")));

				FMinkRelativeFrameTask Task(
					FrameName, FrameType, RootName, RootType, PositionCost, OrientationCost, Gain, Lm);
				if (!TestTrue(TEXT("task.bIsValid"), Task.bIsValid))
				{
					continue;
				}
				Task.SetTarget(FMinkSE3::FromWxyzXyz(Target.data()));
				Cfg.Update(Q.data());

				FMinkVec Error;
				if (TestTrue(TEXT("ComputeError"), Task.ComputeError(Cfg, Error)))
				{
					MinkExpectNear(*this, TEXT("error"), Error, MinkJsonVec(Case->GetArrayField(TEXT("error"))),
						TOL_OBJ);
				}

				FMinkMat Jacobian;
				if (TestTrue(TEXT("ComputeJacobian"), Task.ComputeJacobian(Cfg, Jacobian)))
				{
					MinkExpectNear(*this, TEXT("jacobian"), Jacobian,
						MinkJsonMat(Case->GetArrayField(TEXT("jacobian"))), TOL_OBJ);
				}

				FMinkObjective Objective;
				if (TestTrue(TEXT("ComputeQpObjective"), Task.ComputeQpObjective(Cfg, Objective)))
				{
					MinkExpectNear(*this, TEXT("H"), Objective.H, MinkJsonMat(Case->GetArrayField(TEXT("H"))),
						TOL_OBJ);
					MinkExpectNear(*this, TEXT("c"), Objective.C, MinkJsonVec(Case->GetArrayField(TEXT("c"))),
						TOL_OBJ);
				}

				FMinkResidual Residual;
				if (TestTrue(TEXT("ComputeQpResidual == Ok"),
						Task.ComputeQpResidual(Cfg, Residual) == EMinkTaskStatus::Ok))
				{
					const TSharedPtr<FJsonObject> ResidualJson = Case->GetObjectField(TEXT("residual"));
					MinkExpectNear(*this, TEXT("residual.wjac"), Residual.WeightedJacobian,
						MinkJsonMat(ResidualJson->GetArrayField(TEXT("wjac"))), TOL_OBJ);
					MinkExpectNear(*this, TEXT("residual.werr"), Residual.WeightedError,
						MinkJsonVec(ResidualJson->GetArrayField(TEXT("werr"))), TOL_OBJ);

					const double ExpectedMu = ResidualJson->GetNumberField(TEXT("mu"));
					FMinkMat ActualMuMat(1, 1);
					ActualMuMat(0, 0) = Residual.Mu;
					FMinkMat ExpectedMuMat(1, 1);
					ExpectedMuMat(0, 0) = ExpectedMu;
					MinkExpectNear(*this, TEXT("residual.mu"), ActualMuMat, ExpectedMuMat, TOL_OBJ);
				}
			}
		}

		mj_deleteModel(Model);
	}

	// Validation: unset target => ComputeError false; negative position/orientation cost => setters false.
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

		FMinkRelativeFrameTask Task(
			TEXT("ee"), EMinkFrameType::Site, TEXT("link1"), EMinkFrameType::Body, PositionCost, OrientationCost);
		TestTrue(TEXT("Task.bIsValid"), Task.bIsValid);

		FMinkConfiguration Cfg(Model);
		FMinkVec Error;
		AddExpectedErrorPlain(TEXT("No target set for FMinkRelativeFrameTask"));
		TestFalse(TEXT("unset target => ComputeError false"), Task.ComputeError(Cfg, Error));

		const FMinkVec NegativePositionCost = FMinkVec::Constant(3, -1.0);
		AddExpectedErrorPlain(TEXT("FMinkRelativeFrameTask position cost should be >= 0"));
		TestFalse(TEXT("negative position cost => SetPositionCost false"), Task.SetPositionCost(NegativePositionCost));

		const FMinkVec NegativeOrientationCost = FMinkVec::Constant(3, -1.0);
		AddExpectedErrorPlain(TEXT("FMinkRelativeFrameTask position cost should be >= 0")); // parity: mink v1.2.0's orientation setter really says "position cost" (upstream copy-paste quirk)
		TestFalse(
			TEXT("negative orientation cost => SetOrientationCost false"), Task.SetOrientationCost(NegativeOrientationCost));

		mj_deleteModel(Model);
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMinkTaskComTest,
	"URLab.Mink.Tasks.Com",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMinkTaskComTest::RunTest(const FString& Parameters)
{
	TSharedPtr<FJsonObject> Root;
	if (!MinkLoadFixture(TEXT("task_com"), Root))
	{
		AddError(TEXT("task_com.json missing — run gen_golden.py"));
		return false;
	}
	const TSharedPtr<FJsonObject> Models = Root->GetObjectField(TEXT("models"));

	static const TArray<FString> ModelNames = {TEXT("arm3"), TEXT("floating")};
	for (const FString& ModelName : ModelNames)
	{
		const TArray<TSharedPtr<FJsonValue>>& Cases = Models->GetArrayField(ModelName);
		TestTrue(FString::Printf(TEXT("%s cases non-empty"), *ModelName), Cases.Num() > 0);
		if (Cases.Num() == 0)
		{
			continue;
		}

		mjModel* Model = MinkLoadModel(ModelName + TEXT(".xml"));
		if (Model == nullptr)
		{
			AddError(FString::Printf(TEXT("failed to load model '%s'"), *ModelName));
			continue;
		}

		{
			FMinkConfiguration Cfg(Model);

			for (const TSharedPtr<FJsonValue>& CaseVal : Cases)
			{
				const TSharedPtr<FJsonObject> Case = CaseVal->AsObject();

				const FMinkVec CostVec = MinkJsonVec(Case->GetArrayField(TEXT("cost")));
				const double Gain = Case->GetNumberField(TEXT("gain"));
				const double Lm = Case->GetNumberField(TEXT("lm"));
				const FMinkVec Target = MinkJsonVec(Case->GetArrayField(TEXT("target")));
				const FMinkVec Q = MinkJsonVec(Case->GetArrayField(TEXT("q")));

				FMinkComTask Task(CostVec, Gain, Lm);
				if (!TestTrue(TEXT("task.bIsValid"), Task.bIsValid))
				{
					continue;
				}
				TestTrue(TEXT("SetTarget"), Task.SetTarget(FMinkVec3(Target(0), Target(1), Target(2))));
				Cfg.Update(Q.data());

				FMinkVec Error;
				if (TestTrue(TEXT("ComputeError"), Task.ComputeError(Cfg, Error)))
				{
					MinkExpectNear(*this, TEXT("error"), Error, MinkJsonVec(Case->GetArrayField(TEXT("error"))),
						TOL_OBJ);
				}

				FMinkMat Jacobian;
				if (TestTrue(TEXT("ComputeJacobian"), Task.ComputeJacobian(Cfg, Jacobian)))
				{
					MinkExpectNear(*this, TEXT("jacobian"), Jacobian,
						MinkJsonMat(Case->GetArrayField(TEXT("jacobian"))), TOL_OBJ);
				}

				FMinkObjective Objective;
				if (TestTrue(TEXT("ComputeQpObjective"), Task.ComputeQpObjective(Cfg, Objective)))
				{
					MinkExpectNear(*this, TEXT("H"), Objective.H, MinkJsonMat(Case->GetArrayField(TEXT("H"))),
						TOL_OBJ);
					MinkExpectNear(*this, TEXT("c"), Objective.C, MinkJsonVec(Case->GetArrayField(TEXT("c"))),
						TOL_OBJ);
				}

				FMinkResidual Residual;
				if (TestTrue(TEXT("ComputeQpResidual == Ok"),
						Task.ComputeQpResidual(Cfg, Residual) == EMinkTaskStatus::Ok))
				{
					const TSharedPtr<FJsonObject> ResidualJson = Case->GetObjectField(TEXT("residual"));
					MinkExpectNear(*this, TEXT("residual.wjac"), Residual.WeightedJacobian,
						MinkJsonMat(ResidualJson->GetArrayField(TEXT("wjac"))), TOL_OBJ);
					MinkExpectNear(*this, TEXT("residual.werr"), Residual.WeightedError,
						MinkJsonVec(ResidualJson->GetArrayField(TEXT("werr"))), TOL_OBJ);

					const double ExpectedMu = ResidualJson->GetNumberField(TEXT("mu"));
					FMinkMat ActualMuMat(1, 1);
					ActualMuMat(0, 0) = Residual.Mu;
					FMinkMat ExpectedMuMat(1, 1);
					ExpectedMuMat(0, 0) = ExpectedMu;
					MinkExpectNear(*this, TEXT("residual.mu"), ActualMuMat, ExpectedMuMat, TOL_OBJ);
				}
			}
		}

		mj_deleteModel(Model);
	}

	// Validation: unset target => ComputeError false; negative cost => SetCost false.
	{
		mjModel* Model = MinkLoadModel(TEXT("arm3.xml"));
		if (Model == nullptr)
		{
			AddError(TEXT("failed to load model 'arm3'"));
			return false;
		}

		FMinkVec Cost(1);
		Cost(0) = 1.0;

		FMinkComTask Task(Cost);
		TestTrue(TEXT("Task.bIsValid"), Task.bIsValid);

		FMinkConfiguration Cfg(Model);
		FMinkVec Error;
		AddExpectedErrorPlain(TEXT("No target set for FMinkComTask"));
		TestFalse(TEXT("unset target => ComputeError false"), Task.ComputeError(Cfg, Error));

		const FMinkVec NegativeCost = FMinkVec::Constant(3, -1.0);
		AddExpectedErrorPlain(TEXT("FMinkComTask cost must be >= 0"));
		TestFalse(TEXT("negative cost => SetCost false"), Task.SetCost(NegativeCost));

		mj_deleteModel(Model);
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMinkTaskDampingTest,
	"URLab.Mink.Tasks.Damping",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMinkTaskDampingTest::RunTest(const FString& Parameters)
{
	TSharedPtr<FJsonObject> Root;
	if (!MinkLoadFixture(TEXT("task_damping"), Root))
	{
		AddError(TEXT("task_damping.json missing — run gen_golden.py"));
		return false;
	}
	const TSharedPtr<FJsonObject> Models = Root->GetObjectField(TEXT("models"));

	static const TArray<FString> ModelNames = {TEXT("arm3"), TEXT("floating")};
	for (const FString& ModelName : ModelNames)
	{
		const TArray<TSharedPtr<FJsonValue>>& Cases = Models->GetArrayField(ModelName);
		TestTrue(FString::Printf(TEXT("%s cases non-empty"), *ModelName), Cases.Num() > 0);
		if (Cases.Num() == 0)
		{
			continue;
		}

		mjModel* Model = MinkLoadModel(ModelName + TEXT(".xml"));
		if (Model == nullptr)
		{
			AddError(FString::Printf(TEXT("failed to load model '%s'"), *ModelName));
			continue;
		}

		{
			FMinkConfiguration Cfg(Model);

			for (const TSharedPtr<FJsonValue>& CaseVal : Cases)
			{
				const TSharedPtr<FJsonObject> Case = CaseVal->AsObject();

				const FMinkVec CostVec = MinkJsonVec(Case->GetArrayField(TEXT("cost")));
				const FMinkVec Q = MinkJsonVec(Case->GetArrayField(TEXT("q")));

				FMinkDampingTask Task(Model, CostVec);
				if (!TestTrue(TEXT("task.bIsValid"), Task.bIsValid))
				{
					continue;
				}
				Cfg.Update(Q.data());

				FMinkVec Error;
				if (TestTrue(TEXT("ComputeError"), Task.ComputeError(Cfg, Error)))
				{
					MinkExpectNear(*this, TEXT("error"), Error, MinkJsonVec(Case->GetArrayField(TEXT("error"))),
						TOL_OBJ);
				}

				FMinkMat Jacobian;
				if (TestTrue(TEXT("ComputeJacobian"), Task.ComputeJacobian(Cfg, Jacobian)))
				{
					MinkExpectNear(*this, TEXT("jacobian"), Jacobian,
						MinkJsonMat(Case->GetArrayField(TEXT("jacobian"))), TOL_OBJ);
				}

				FMinkObjective Objective;
				if (TestTrue(TEXT("ComputeQpObjective"), Task.ComputeQpObjective(Cfg, Objective)))
				{
					MinkExpectNear(*this, TEXT("H"), Objective.H, MinkJsonMat(Case->GetArrayField(TEXT("H"))),
						TOL_OBJ);
					MinkExpectNear(*this, TEXT("c"), Objective.C, MinkJsonVec(Case->GetArrayField(TEXT("c"))),
						TOL_OBJ);
				}

				FMinkResidual Residual;
				if (TestTrue(TEXT("ComputeQpResidual == Ok"),
						Task.ComputeQpResidual(Cfg, Residual) == EMinkTaskStatus::Ok))
				{
					const TSharedPtr<FJsonObject> ResidualJson = Case->GetObjectField(TEXT("residual"));
					MinkExpectNear(*this, TEXT("residual.wjac"), Residual.WeightedJacobian,
						MinkJsonMat(ResidualJson->GetArrayField(TEXT("wjac"))), TOL_OBJ);
					MinkExpectNear(*this, TEXT("residual.werr"), Residual.WeightedError,
						MinkJsonVec(ResidualJson->GetArrayField(TEXT("werr"))), TOL_OBJ);

					const double ExpectedMu = ResidualJson->GetNumberField(TEXT("mu"));
					FMinkMat ActualMuMat(1, 1);
					ActualMuMat(0, 0) = Residual.Mu;
					FMinkMat ExpectedMuMat(1, 1);
					ExpectedMuMat(0, 0) = ExpectedMu;
					MinkExpectNear(*this, TEXT("residual.mu"), ActualMuMat, ExpectedMuMat, TOL_OBJ);
				}
			}
		}

		mj_deleteModel(Model);
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMinkTaskDofFreezingTest,
	"URLab.Mink.Tasks.DofFreezing",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMinkTaskDofFreezingTest::RunTest(const FString& Parameters)
{
	TSharedPtr<FJsonObject> Root;
	if (!MinkLoadFixture(TEXT("task_dof_freezing"), Root))
	{
		AddError(TEXT("task_dof_freezing.json missing — run gen_golden.py"));
		return false;
	}
	const TSharedPtr<FJsonObject> Models = Root->GetObjectField(TEXT("models"));

	static const TArray<FString> ModelNames = {TEXT("arm3"), TEXT("floating")};
	for (const FString& ModelName : ModelNames)
	{
		const TArray<TSharedPtr<FJsonValue>>& Cases = Models->GetArrayField(ModelName);
		TestTrue(FString::Printf(TEXT("%s cases non-empty"), *ModelName), Cases.Num() > 0);
		if (Cases.Num() == 0)
		{
			continue;
		}

		mjModel* Model = MinkLoadModel(ModelName + TEXT(".xml"));
		if (Model == nullptr)
		{
			AddError(FString::Printf(TEXT("failed to load model '%s'"), *ModelName));
			continue;
		}

		{
			FMinkConfiguration Cfg(Model);

			for (const TSharedPtr<FJsonValue>& CaseVal : Cases)
			{
				const TSharedPtr<FJsonObject> Case = CaseVal->AsObject();

				TArray<int32> DofIndices;
				for (const TSharedPtr<FJsonValue>& V : Case->GetArrayField(TEXT("dof_indices")))
				{
					DofIndices.Add((int32)V->AsNumber());
				}
				const double Gain = Case->GetNumberField(TEXT("gain"));
				const FMinkVec Q = MinkJsonVec(Case->GetArrayField(TEXT("q")));

				FMinkDofFreezingTask Task(Model, DofIndices, Gain);
				if (!TestTrue(TEXT("task.bIsValid"), Task.bIsValid))
				{
					continue;
				}
				Cfg.Update(Q.data());

				FMinkVec Error;
				if (TestTrue(TEXT("ComputeError"), Task.ComputeError(Cfg, Error)))
				{
					MinkExpectNear(*this, TEXT("error"), Error, MinkJsonVec(Case->GetArrayField(TEXT("error"))),
						TOL_OBJ);
				}

				FMinkMat Jacobian;
				if (TestTrue(TEXT("ComputeJacobian"), Task.ComputeJacobian(Cfg, Jacobian)))
				{
					MinkExpectNear(*this, TEXT("jacobian"), Jacobian,
						MinkJsonMat(Case->GetArrayField(TEXT("jacobian"))), TOL_OBJ);
				}

				FMinkObjective Objective;
				if (TestTrue(TEXT("ComputeQpObjective"), Task.ComputeQpObjective(Cfg, Objective)))
				{
					MinkExpectNear(*this, TEXT("H"), Objective.H, MinkJsonMat(Case->GetArrayField(TEXT("H"))),
						TOL_OBJ);
					MinkExpectNear(*this, TEXT("c"), Objective.C, MinkJsonVec(Case->GetArrayField(TEXT("c"))),
						TOL_OBJ);
				}

				FMinkResidual Residual;
				if (TestTrue(TEXT("ComputeQpResidual == Ok"),
						Task.ComputeQpResidual(Cfg, Residual) == EMinkTaskStatus::Ok))
				{
					const TSharedPtr<FJsonObject> ResidualJson = Case->GetObjectField(TEXT("residual"));
					MinkExpectNear(*this, TEXT("residual.wjac"), Residual.WeightedJacobian,
						MinkJsonMat(ResidualJson->GetArrayField(TEXT("wjac"))), TOL_OBJ);
					MinkExpectNear(*this, TEXT("residual.werr"), Residual.WeightedError,
						MinkJsonVec(ResidualJson->GetArrayField(TEXT("werr"))), TOL_OBJ);

					const double ExpectedMu = ResidualJson->GetNumberField(TEXT("mu"));
					FMinkMat ActualMuMat(1, 1);
					ActualMuMat(0, 0) = Residual.Mu;
					FMinkMat ExpectedMuMat(1, 1);
					ExpectedMuMat(0, 0) = ExpectedMu;
					MinkExpectNear(*this, TEXT("residual.mu"), ActualMuMat, ExpectedMuMat, TOL_OBJ);
				}
			}
		}

		mj_deleteModel(Model);
	}

	// Validation: duplicate indices and out-of-range indices both => bIsValid false.
	{
		mjModel* Model = MinkLoadModel(TEXT("arm3.xml"));
		if (Model == nullptr)
		{
			AddError(TEXT("failed to load model 'arm3'"));
			return false;
		}

		TArray<int32> EmptyIndices;
		AddExpectedErrorPlain(TEXT("FMinkDofFreezingTask requires at least one DOF index."));
		FMinkDofFreezingTask EmptyTask(Model, EmptyIndices);
		TestFalse(TEXT("empty indices => bIsValid false"), EmptyTask.bIsValid);

		TArray<int32> OutOfRangeIndices = {0, Model->nv};
		AddExpectedErrorPlain(TEXT("is out of range"));
		FMinkDofFreezingTask OutOfRangeTask(Model, OutOfRangeIndices);
		TestFalse(TEXT("out-of-range index => bIsValid false"), OutOfRangeTask.bIsValid);

		TArray<int32> DuplicateIndices = {0, 1, 0};
		AddExpectedErrorPlain(TEXT("Duplicate DOF indices found"));
		FMinkDofFreezingTask DuplicateTask(Model, DuplicateIndices);
		TestFalse(TEXT("duplicate indices => bIsValid false"), DuplicateTask.bIsValid);

		mj_deleteModel(Model);
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMinkTaskKineticEnergyTest,
	"URLab.Mink.Tasks.KineticEnergy",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMinkTaskKineticEnergyTest::RunTest(const FString& Parameters)
{
	TSharedPtr<FJsonObject> Root;
	if (!MinkLoadFixture(TEXT("task_kinetic_energy"), Root))
	{
		AddError(TEXT("task_kinetic_energy.json missing — run gen_golden.py"));
		return false;
	}
	const TSharedPtr<FJsonObject> Models = Root->GetObjectField(TEXT("models"));

	static const TArray<FString> ModelNames = {TEXT("arm3"), TEXT("floating")};
	for (const FString& ModelName : ModelNames)
	{
		const TArray<TSharedPtr<FJsonValue>>& Cases = Models->GetArrayField(ModelName);
		TestTrue(FString::Printf(TEXT("%s cases non-empty"), *ModelName), Cases.Num() > 0);
		if (Cases.Num() == 0)
		{
			continue;
		}

		mjModel* Model = MinkLoadModel(ModelName + TEXT(".xml"));
		if (Model == nullptr)
		{
			AddError(FString::Printf(TEXT("failed to load model '%s'"), *ModelName));
			continue;
		}

		{
			FMinkConfiguration Cfg(Model);

			for (const TSharedPtr<FJsonValue>& CaseVal : Cases)
			{
				const TSharedPtr<FJsonObject> Case = CaseVal->AsObject();

				const double Cost = Case->GetNumberField(TEXT("cost"));
				const double Dt = Case->GetNumberField(TEXT("dt"));
				const FMinkVec Q = MinkJsonVec(Case->GetArrayField(TEXT("q")));

				FMinkKineticEnergyRegularizationTask Task(Cost);
				if (!TestTrue(TEXT("task.bIsValid"), Task.bIsValid))
				{
					continue;
				}
				Task.SetDt(Dt);
				Cfg.Update(Q.data());

				FMinkObjective Objective;
				if (TestTrue(TEXT("ComputeQpObjective"), Task.ComputeQpObjective(Cfg, Objective)))
				{
					MinkExpectNear(*this, TEXT("H"), Objective.H, MinkJsonMat(Case->GetArrayField(TEXT("H"))),
						TOL_OBJ);
					MinkExpectNear(*this, TEXT("c"), Objective.C, MinkJsonVec(Case->GetArrayField(TEXT("c"))),
						TOL_OBJ);
				}

				FMinkResidual Residual;
				TestTrue(TEXT("KE task inherits dense fallback (NoResidualForm)"),
					Task.ComputeQpResidual(Cfg, Residual) == EMinkTaskStatus::NoResidualForm);
			}
		}

		mj_deleteModel(Model);
	}

	// Validation: negative cost => bIsValid false; unset dt => ComputeQpObjective false.
	{
		mjModel* Model = MinkLoadModel(TEXT("arm3.xml"));
		if (Model == nullptr)
		{
			AddError(TEXT("failed to load model 'arm3'"));
			return false;
		}

		AddExpectedErrorPlain(TEXT("FMinkKineticEnergyRegularizationTask cost should be >= 0"));
		FMinkKineticEnergyRegularizationTask InvalidTask(-1.0);
		TestFalse(TEXT("negative cost => bIsValid false"), InvalidTask.bIsValid);

		FMinkKineticEnergyRegularizationTask Task(1e-4);
		TestTrue(TEXT("Task.bIsValid"), Task.bIsValid);

		FMinkConfiguration Cfg(Model);
		FMinkObjective Objective;
		AddExpectedErrorPlain(TEXT("No integration timestep set for FMinkKineticEnergyRegularizationTask"));
		TestFalse(TEXT("unset dt => ComputeQpObjective false"), Task.ComputeQpObjective(Cfg, Objective));

		mj_deleteModel(Model);
	}

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
