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

#include "Misc/AutomationTest.h"

#include "MinkConfiguration.h"
#include "MinkSolveIK.h"
#include "MinkTestUtils.h"
#include "Tasks/MinkFrameTask.h"
#include "Tasks/MinkPostureTask.h"
#include "Tasks/MinkDampingTask.h"
#include "Lie/MinkSE3.h"

#include <mujoco/mujoco.h>

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
/** Solve/integrate NIters steps of the given stack; return final EE translation. */
FMinkVec3 RunStack(FMinkConfiguration& Cfg, const TArray<const FMinkBaseTask*>& Tasks,
	int32 NIters, double Dt)
{
	for (int32 i = 0; i < NIters; ++i)
	{
		const FMinkIKResult R = MinkSolveIK(Cfg, Tasks, Dt, /*Damping*/ 1e-3);
		if (!R.IsSuccess())
		{
			break;
		}
		Cfg.IntegrateInplace(R.Velocity, Dt);
	}
	FMinkSE3 Out;
	Cfg.GetTransformFrameToWorld(TEXT("ee"), EMinkFrameType::Site, Out);
	return Out.Translation();
}
} // namespace

// Composition test backing the data-driven IK controller: a Frame+Posture stack
// converges to a reachable target, and adding a heavy Damping task on one joint
// freezes that joint (the mobile-base "fix_base" pattern) while others still move.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMinkTaskCompositionTest,
	"URLab.Mink.Tasks.Composition",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMinkTaskCompositionTest::RunTest(const FString& Parameters)
{
	mjModel* M = MinkLoadModel(TEXT("arm3.xml"));
	if (!TestNotNull(TEXT("arm3.xml model loaded"), M))
	{
		return false;
	}

	// Reachable target = FK of a pose that needs a j1 yaw of 0.5.
	const double QTarget[3] = {0.5, 0.6, -0.5};
	FMinkVec3 TargetPos;
	{
		FMinkConfiguration Probe(M, QTarget);
		FMinkSE3 T;
		Probe.GetTransformFrameToWorld(TEXT("ee"), EMinkFrameType::Site, T);
		TargetPos = T.Translation();
	}

	FMinkFrameTask Frame(TEXT("ee"), EMinkFrameType::Site,
		FMinkVec::Constant(1, 1.0), FMinkVec::Constant(1, 0.0), /*Gain*/ 1.0, /*Lm*/ 1e-3);
	Frame.SetTarget(FMinkSE3::FromTranslation(TargetPos));

	const double Dt = 0.01;
	const int32 NIters = 300;

	// --- Phase A: Frame + Posture converges, and j1 does the yaw work ---------
	{
		FMinkConfiguration Cfg(M);
		Cfg.UpdateFromKeyframe(TEXT("home"));
		const double Q1Start = Cfg.GetQ()[0];

		FMinkPostureTask Posture(M, FMinkVec::Constant(M->nv, 1e-3));
		Posture.SetTargetFromConfiguration(Cfg);

		const FMinkVec3 Final = RunStack(Cfg, {&Frame, &Posture}, NIters, Dt);
		const double Err = (Final - TargetPos).norm();
		TestTrue(FString::Printf(TEXT("A: frame+posture converged (err %.5f m)"), Err), Err < 1e-2);
		const double Q1Moved = FMath::Abs(Cfg.GetQ()[0] - Q1Start);
		TestTrue(FString::Printf(TEXT("A: j1 moved to yaw the arm (|dq1| %.3f)"), Q1Moved), Q1Moved > 0.3);
	}

	// --- Phase B: heavy Damping on j1 freezes it (fix-base pattern) ------------
	{
		FMinkConfiguration Cfg(M);
		Cfg.UpdateFromKeyframe(TEXT("home"));
		const double Q1Start = Cfg.GetQ()[0];

		FMinkPostureTask Posture(M, FMinkVec::Constant(M->nv, 1e-3));
		Posture.SetTargetFromConfiguration(Cfg);

		FMinkVec DampCost = FMinkVec::Zero(M->nv);
		DampCost[M->jnt_dofadr[mj_name2id(M, mjOBJ_JOINT, "j1")]] = 1e4;
		FMinkDampingTask Damping(M, DampCost);

		RunStack(Cfg, {&Frame, &Posture, &Damping}, NIters, Dt);
		const double Q1Moved = FMath::Abs(Cfg.GetQ()[0] - Q1Start);
		TestTrue(FString::Printf(TEXT("B: damped j1 stayed frozen (|dq1| %.4f)"), Q1Moved), Q1Moved < 0.05);
		// The rest of the arm must still have chased the target (pitch joints move).
		const double Q2Moved = FMath::Abs(Cfg.GetQ()[1] - 0.4);
		TestTrue(FString::Printf(TEXT("B: undamped j2 still moved (|dq2| %.4f)"), Q2Moved), Q2Moved > 0.05);
	}

	mj_deleteModel(M);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
