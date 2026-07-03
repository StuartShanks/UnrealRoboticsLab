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

#include "MinkEndEffectorIK.h"
#include "MinkSolveIK.h"
#include "MinkTestUtils.h"
#include "Lie/MinkSE3.h"

#include <mujoco/mujoco.h>

#if WITH_DEV_AUTOMATION_TESTS

namespace
{
/** Read a site's world position out of live data as an Eigen 3-vector. */
FMinkVec3 SitePos(const mjData* D, int32 SiteId)
{
	return FMinkVec3(D->site_xpos[3 * SiteId + 0], D->site_xpos[3 * SiteId + 1],
		D->site_xpos[3 * SiteId + 2]);
}
} // namespace

// Drives the arm3 end-effector site to a reachable Cartesian target from a different start
// configuration and asserts the differential-IK loop converges the site to that target.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMinkLiveIKConvergesTest,
	"URLab.Mink.LiveIK.Converges",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMinkLiveIKConvergesTest::RunTest(const FString& Parameters)
{
	mjModel* M = MinkLoadModel(TEXT("arm3.xml"));
	if (!TestNotNull(TEXT("arm3.xml model loaded"), M))
	{
		return false;
	}
	mjData* D = mj_makeData(M);

	const int32 SiteId = mj_name2id(M, mjOBJ_SITE, "ee");
	TestTrue(TEXT("ee site exists"), SiteId >= 0);

	// Reachable target = forward kinematics of a known joint configuration.
	const double QTarget[3] = {0.5, 0.6, -0.5};
	mju_copy(D->qpos, QTarget, 3);
	mj_forward(M, D);
	const FMinkVec3 TargetPos = SitePos(D, SiteId);
	const FMinkSE3 Target = FMinkSE3::FromTranslation(TargetPos);

	// Start from the "home" keyframe (a different pose) and solve toward the target.
	const int32 KeyId = mj_name2id(M, mjOBJ_KEY, "home");
	if (KeyId >= 0)
	{
		mju_copy(D->qpos, M->key_qpos + (int64)KeyId * M->nq, M->nq);
	}
	else
	{
		mju_zero(D->qpos, M->nq);
	}
	mju_zero(D->qvel, M->nv);
	mj_forward(M, D);
	const double StartErr = (SitePos(D, SiteId) - TargetPos).norm();

	FMinkEndEffectorIKConfig Cfg;
	Cfg.FrameName = TEXT("ee");
	Cfg.FrameType = EMinkFrameType::Site;
	Cfg.DriveJointNames = {TEXT("j1"), TEXT("j2"), TEXT("j3")};
	Cfg.PositionCost = 1.0;
	Cfg.OrientationCost = 0.0; // 3-DOF arm: position only
	Cfg.PostureCost = 1e-3;
	Cfg.Damping = 1e-3;

	FMinkEndEffectorIK Ik(M, Cfg);
	if (!TestTrue(TEXT("IK driver constructed valid"), Ik.IsValid()))
	{
		mj_deleteData(D);
		mj_deleteModel(M);
		return false;
	}

	EMinkIKStatus Status = EMinkIKStatus::Success;
	const double Dt = 0.01;
	for (int32 i = 0; i < 500; ++i)
	{
		Status = Ik.SolveStep(D, Target, Dt);
		if (Status != EMinkIKStatus::Success)
		{
			break;
		}
		mj_forward(M, D); // refresh site_xpos for the next read/step
	}
	TestEqual(TEXT("solver status stayed Success"), static_cast<int32>(Status),
		static_cast<int32>(EMinkIKStatus::Success));

	const double FinalErr = (SitePos(D, SiteId) - TargetPos).norm();
	TestTrue(FString::Printf(TEXT("EE converged: start err %.4f m -> final err %.5f m"), StartErr, FinalErr),
		FinalErr < 1e-2);
	// Sanity: the target really was non-trivially far at the start.
	TestTrue(TEXT("start error was non-trivial"), StartErr > 0.05);

	mj_deleteData(D);
	mj_deleteModel(M);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
