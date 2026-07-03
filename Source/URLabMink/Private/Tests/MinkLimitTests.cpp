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
#include "Limits/MinkConfigurationLimit.h"
#include "Limits/MinkVelocityLimit.h"
#include "Limits/MinkCollisionAvoidanceLimit.h"

#if WITH_DEV_AUTOMATION_TESTS

// NOTE: TOL_OBJ (task/limit matrices, 1e-8 per the global constraints) is already defined as a
// file-static in MinkTaskTests.cpp; with unity builds concatenating all Private/*.cpp into one
// translation unit, redefining it here would collide. Match its value locally instead.
static const double TOL_OBJ_LIMITS = 1e-8;

// Unity-unique name for the CollisionAvoidance test's own tolerance constant (see the note
// above — every sibling test file picks its own name to avoid unity-build redefinition).
static const double TOL_OBJ_CA = 1e-8;

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMinkLimitConfigurationTest,
	"URLab.Mink.Limits.Configuration",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMinkLimitConfigurationTest::RunTest(const FString& Parameters)
{
	TSharedPtr<FJsonObject> Root;
	if (!MinkLoadFixture(TEXT("limit_configuration"), Root))
	{
		AddError(TEXT("limit_configuration.json missing — run gen_golden.py"));
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

				const double Gain = Case->GetNumberField(TEXT("gain"));
				const double MinDistanceFromLimits = Case->GetNumberField(TEXT("min_distance_from_limits"));
				const FMinkVec Q = MinkJsonVec(Case->GetArrayField(TEXT("q")));

				FMinkConfigurationLimit Limit(Model, Gain, MinDistanceFromLimits);
				if (!TestTrue(TEXT("Limit.bIsValid"), Limit.bIsValid))
				{
					continue;
				}
				Cfg.Update(Q.data());

				FMinkInequality Inequality;
				if (TestTrue(TEXT("ComputeQpInequalities"), Limit.ComputeQpInequalities(Cfg, /*Dt=*/0.0, Inequality)))
				{
					if (TestTrue(TEXT("Inequality active"), !Inequality.IsInactive()))
					{
						TestTrue(TEXT("G non-empty"), Inequality.G->rows() > 0 && Inequality.G->cols() > 0);
						TestTrue(TEXT("h non-empty"), Inequality.H->size() > 0);
						MinkExpectNear(*this, TEXT("G"), *Inequality.G, MinkJsonMat(Case->GetArrayField(TEXT("G"))),
							TOL_OBJ_LIMITS);
						MinkExpectNear(*this, TEXT("h"), *Inequality.H, MinkJsonVec(Case->GetArrayField(TEXT("h"))),
							TOL_OBJ_LIMITS);
					}
				}
			}
		}

		mj_deleteModel(Model);
	}

	// Validation: gain outside (0, 1] => bIsValid false.
	{
		mjModel* Model = MinkLoadModel(TEXT("arm3.xml"));
		if (Model == nullptr)
		{
			AddError(TEXT("failed to load model 'arm3'"));
			return false;
		}

		AddExpectedErrorPlain(TEXT("FMinkConfigurationLimit gain must be in the range (0, 1]"));
		FMinkConfigurationLimit InvalidGainLimit(Model, /*Gain=*/1.5);
		TestFalse(TEXT("gain 1.5 => bIsValid false"), InvalidGainLimit.bIsValid);

		mj_deleteModel(Model);
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMinkLimitVelocityTest,
	"URLab.Mink.Limits.Velocity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMinkLimitVelocityTest::RunTest(const FString& Parameters)
{
	TSharedPtr<FJsonObject> Root;
	if (!MinkLoadFixture(TEXT("limit_velocity"), Root))
	{
		AddError(TEXT("limit_velocity.json missing — run gen_golden.py"));
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

				// Ordered pairs, in the exact order the fixture recorded (mirrors the Python
				// dict insertion order and therefore the expected G/h row order).
				TArray<TPair<FString, FMinkVec>> Velocities;
				for (const TSharedPtr<FJsonValue>& PairVal : Case->GetArrayField(TEXT("joint_order")))
				{
					const TArray<TSharedPtr<FJsonValue>> PairArr = PairVal->AsArray();
					const FString JointName = PairArr[0]->AsString();
					const FMinkVec Values = MinkJsonVec(PairArr[1]->AsArray());
					Velocities.Emplace(JointName, Values);
				}
				const double Dt = Case->GetNumberField(TEXT("dt"));
				const FMinkVec Q = MinkJsonVec(Case->GetArrayField(TEXT("q")));

				FMinkVelocityLimit Limit(Model, Velocities);
				if (!TestTrue(TEXT("Limit.bIsValid"), Limit.bIsValid))
				{
					continue;
				}
				Cfg.Update(Q.data());

				FMinkInequality Inequality;
				if (TestTrue(TEXT("ComputeQpInequalities"), Limit.ComputeQpInequalities(Cfg, Dt, Inequality)))
				{
					if (TestTrue(TEXT("Inequality active"), !Inequality.IsInactive()))
					{
						TestTrue(TEXT("G non-empty"), Inequality.G->rows() > 0 && Inequality.G->cols() > 0);
						TestTrue(TEXT("h non-empty"), Inequality.H->size() > 0);
						MinkExpectNear(*this, TEXT("G"), *Inequality.G, MinkJsonMat(Case->GetArrayField(TEXT("G"))),
							TOL_OBJ_LIMITS);
						MinkExpectNear(*this, TEXT("h"), *Inequality.H, MinkJsonVec(Case->GetArrayField(TEXT("h"))),
							TOL_OBJ_LIMITS);
					}
				}
			}
		}

		mj_deleteModel(Model);
	}

	// Validation: a free joint name => bIsValid false.
	{
		mjModel* Model = MinkLoadModel(TEXT("floating.xml"));
		if (Model == nullptr)
		{
			AddError(TEXT("failed to load model 'floating'"));
			return false;
		}

		FMinkVec DummyLimit(1);
		DummyLimit(0) = 1.0;
		TArray<TPair<FString, FMinkVec>> FreeJointVelocities;
		FreeJointVelocities.Emplace(TEXT("root"), DummyLimit);

		AddExpectedErrorPlain(TEXT("Free joint root is not supported"));
		FMinkVelocityLimit InvalidLimit(Model, FreeJointVelocities);
		TestFalse(TEXT("free joint name => bIsValid false"), InvalidLimit.bIsValid);

		mj_deleteModel(Model);
	}

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMinkLimitCollisionAvoidanceTest,
	"URLab.Mink.Limits.CollisionAvoidance",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMinkLimitCollisionAvoidanceTest::RunTest(const FString& Parameters)
{
	TSharedPtr<FJsonObject> Root;
	if (!MinkLoadFixture(TEXT("limit_collision"), Root))
	{
		AddError(TEXT("limit_collision.json missing — run gen_golden.py"));
		return false;
	}

	const TArray<TSharedPtr<FJsonValue>>& Cases =
		Root->GetObjectField(TEXT("models"))->GetArrayField(TEXT("scene_collision"));
	TestTrue(TEXT("scene_collision cases non-empty"), Cases.Num() > 0);
	if (Cases.Num() == 0)
	{
		return false;
	}

	mjModel* Model = MinkLoadModel(TEXT("scene_collision.xml"));
	if (Model == nullptr)
	{
		AddError(TEXT("failed to load model 'scene_collision'"));
		return false;
	}

	// Mirrors gen_limit_collision's single geom_pairs spec, exercised across every case.
	FMinkGeomGroup ArmGroup;
	ArmGroup.Names = {TEXT("g1"), TEXT("g2"), TEXT("g3")};
	FMinkGeomGroup ObstacleGroup;
	ObstacleGroup.Names = {TEXT("obstacle1"), TEXT("obstacle2"), TEXT("floor")};
	FMinkGeomGroup SelfA;
	SelfA.Names = {TEXT("g1")};
	FMinkGeomGroup SelfB;
	SelfB.Names = {TEXT("g3")};
	TArray<FMinkCollisionPair> GeomPairs;
	GeomPairs.Emplace(ArmGroup, ObstacleGroup);
	GeomPairs.Emplace(SelfA, SelfB);

	{
		FMinkConfiguration Cfg(Model);

		for (const TSharedPtr<FJsonValue>& CaseVal : Cases)
		{
			const TSharedPtr<FJsonObject> Case = CaseVal->AsObject();

			const FString CaseLabel = FString::Printf(
				TEXT("%s/%s"), *Case->GetStringField(TEXT("q_label")), *Case->GetStringField(TEXT("variant")));

			const double Gain = Case->GetNumberField(TEXT("gain"));
			const double MinDist = Case->GetNumberField(TEXT("min_dist"));
			const double DetectionDist = Case->GetNumberField(TEXT("detection_dist"));
			const double BoundRelaxation = Case->GetNumberField(TEXT("bound_relaxation"));
			const double Dt = Case->GetNumberField(TEXT("dt"));
			const FMinkVec Q = MinkJsonVec(Case->GetArrayField(TEXT("q")));

			TArray<TPair<int32, int32>> ExpectedPairs;
			for (const TSharedPtr<FJsonValue>& PairVal : Case->GetArrayField(TEXT("geom_id_pairs")))
			{
				const TArray<TSharedPtr<FJsonValue>> P = PairVal->AsArray();
				ExpectedPairs.Emplace(static_cast<int32>(P[0]->AsNumber()), static_cast<int32>(P[1]->AsNumber()));
			}
			const FMinkMat ExpectedG = MinkJsonMat(Case->GetArrayField(TEXT("G")));
			const FMinkVec ExpectedH = MinkJsonVec(Case->GetArrayField(TEXT("h")));

			Cfg.Update(Q.data());

			// Pass 1: linear scan (bBroadphase = false).
			{
				FMinkCollisionAvoidanceLimit Limit(
					Model, GeomPairs, Gain, MinDist, DetectionDist, BoundRelaxation, /*bBroadphase=*/false);
				if (!TestTrue(FString::Printf(TEXT("%s: Limit.bIsValid (linear)"), *CaseLabel), Limit.bIsValid))
				{
					continue;
				}

				const TArray<TPair<int32, int32>>& ActualPairs = Limit.GetGeomIdPairs();
				bool bPairsEqual = ActualPairs.Num() == ExpectedPairs.Num();
				for (int32 Index = 0; bPairsEqual && Index < ActualPairs.Num(); ++Index)
				{
					bPairsEqual = ActualPairs[Index].Key == ExpectedPairs[Index].Key
							   && ActualPairs[Index].Value == ExpectedPairs[Index].Value;
				}
				TestTrue(FString::Printf(TEXT("%s: geom_id_pairs exact match"), *CaseLabel), bPairsEqual);

				FMinkInequality Inequality;
				if (TestTrue(FString::Printf(TEXT("%s: ComputeQpInequalities (linear)"), *CaseLabel),
						Limit.ComputeQpInequalities(Cfg, Dt, Inequality)))
				{
					TestTrue(TEXT("G non-empty"), Inequality.G.IsSet() && Inequality.G->rows() > 0
													  && Inequality.G->cols() > 0);
					TestTrue(TEXT("h non-empty"), Inequality.H.IsSet() && Inequality.H->size() > 0);
					MinkExpectNear(*this, TEXT("G (linear)"), *Inequality.G, ExpectedG, TOL_OBJ_CA);
					MinkExpectNear(*this, TEXT("h (linear)"), *Inequality.H, ExpectedH, TOL_OBJ_CA);
				}
			}

			// Pass 2: forced broadphase (BroadphaseMinPairs = 0) — must match the exact same
			// fixture, proving the broadphase is a pure pre-filter.
			{
				FMinkCollisionAvoidanceLimit Limit(
					Model, GeomPairs, Gain, MinDist, DetectionDist, BoundRelaxation, /*bBroadphase=*/true);
				Limit.BroadphaseMinPairs = 0;
				if (!TestTrue(FString::Printf(TEXT("%s: Limit.bIsValid (broadphase)"), *CaseLabel), Limit.bIsValid))
				{
					continue;
				}

				FMinkInequality Inequality;
				if (TestTrue(FString::Printf(TEXT("%s: ComputeQpInequalities (broadphase)"), *CaseLabel),
						Limit.ComputeQpInequalities(Cfg, Dt, Inequality)))
				{
					MinkExpectNear(*this, TEXT("G (broadphase)"), *Inequality.G, ExpectedG, TOL_OBJ_CA);
					MinkExpectNear(*this, TEXT("h (broadphase)"), *Inequality.H, ExpectedH, TOL_OBJ_CA);
				}
			}
		}
	}

	mj_deleteModel(Model);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
