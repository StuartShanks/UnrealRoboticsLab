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
#include "Tasks/MinkPostureTask.h"

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

#endif // WITH_DEV_AUTOMATION_TESTS
