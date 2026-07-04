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
#include "Tests/MjTestHelpers.h"
#include "MuJoCo/Components/Joints/MjJoint.h"
#include "MuJoCo/Components/Actuators/MjPositionActuator.h"
#include "MuJoCo/Components/Actuators/MjMotorActuator.h"
#include "MuJoCo/Components/Controllers/MjBaseDriveController.h"
#include "MuJoCo/Input/MjTwistController.h"
#include "mujoco/mujoco.h"

namespace
{
struct FBaseDriveRig
{
	UMjBaseDriveController* Ctrl = nullptr;
	UMjTwistController* Twist = nullptr;
	UMjMotorActuator* ArmAct = nullptr; // pass-through probe

	/** Configure FMjUESession's default hierarchy into a planar base:
	 *  TestJoint becomes the X slide; add Y slide + yaw hinge + a spare
	 *  hinge ("arm_j") with a motor actuator to prove pass-through. */
	void Configure(FMjUESession& Sess)
	{
		Sess.Robot->ControlSource = 1; // UI source → SetControl feeds pass-through

		Sess.Joint->Type = EMjJointType::Slide;
		Sess.Joint->bOverride_Type = true;
		Sess.Joint->Axis = FVector(1, 0, 0);
		Sess.Joint->bOverride_Axis = true;

		auto MakeJoint = [&](const TCHAR* Name, EMjJointType Type, FVector Axis) {
			UMjJoint* J = NewObject<UMjJoint>(Sess.Robot, Name);
			J->Type = Type;
			J->bOverride_Type = true;
			J->Axis = Axis;
			J->bOverride_Axis = true;
			J->RegisterComponent();
			J->AttachToComponent(Sess.Body, FAttachmentTransformRules::KeepRelativeTransform);
			return J;
		};
		// NOTE on the Y axis: UE→MuJoCo flips Y. If the DriveWorldY test
		// below fails with inverted sign, flip this to FVector(0, 1, 0) —
		// the assertion on d->xpos is the source of truth.
		MakeJoint(TEXT("joint_y"), EMjJointType::Slide, FVector(0, -1, 0));
		MakeJoint(TEXT("joint_th"), EMjJointType::Hinge, FVector(0, 0, 1));
		MakeJoint(TEXT("arm_j"), EMjJointType::Hinge, FVector(1, 0, 0));

		auto MakePosAct = [&](const TCHAR* Name, const TCHAR* TargetJoint) {
			UMjPositionActuator* A = NewObject<UMjPositionActuator>(Sess.Robot, Name);
			A->TargetName = TargetJoint;
			A->kp = 500.f;
			A->bOverride_kp = true;
			A->RegisterComponent();
			return A;
		};
		MakePosAct(TEXT("act_x"), TEXT("TestJoint"));
		MakePosAct(TEXT("act_y"), TEXT("joint_y"));
		MakePosAct(TEXT("act_th"), TEXT("joint_th"));

		ArmAct = NewObject<UMjMotorActuator>(Sess.Robot, TEXT("arm_act"));
		ArmAct->TargetName = TEXT("arm_j");
		ArmAct->RegisterComponent();

		Twist = NewObject<UMjTwistController>(Sess.Robot, TEXT("TwistCtrl"));
		Twist->RegisterComponent();

		Ctrl = NewObject<UMjBaseDriveController>(Sess.Robot, TEXT("BaseDrive"));
		Ctrl->BaseJointNames = {TEXT("TestJoint"), TEXT("joint_y"), TEXT("joint_th")};
		Ctrl->RegisterComponent();
	}

	/** Drive N physics steps with the controller in the loop. */
	static void Drive(FMjUESession& Sess, int32 N)
	{
		for (int32 i = 0; i < N; ++i)
		{
			Sess.Robot->ApplyControls();
			Sess.Step(1);
		}
	}
};

double BaseDriveBodyX(FMjUESession& S)
{ /* base body world x (MuJoCo m) */
	mjModel* m = S.Manager->PhysicsEngine->m_model;
	mjData* d = S.Manager->PhysicsEngine->m_data;
	// RootBody is the only non-world body carrying our joints:
	int Bid = -1;
	for (int i = 1; i < m->nbody; ++i)
		if (m->body_jntnum[i] > 0)
		{
			Bid = i;
			break;
		}
	return (Bid >= 0) ? d->xpos[Bid * 3 + 0] : 0.0;
}
double BaseDriveBodyY(FMjUESession& S)
{
	mjModel* m = S.Manager->PhysicsEngine->m_model;
	mjData* d = S.Manager->PhysicsEngine->m_data;
	int Bid = -1;
	for (int i = 1; i < m->nbody; ++i)
		if (m->body_jntnum[i] > 0)
		{
			Bid = i;
			break;
		}
	return (Bid >= 0) ? d->xpos[Bid * 3 + 1] : 0.0;
}
} // namespace

// URLab.Nav.BaseDrive.Binds — controller resolves 3 base bindings + arm
// binding survives; controller is the articulation's cached controller.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjBaseDriveBinds,
	"URLab.Nav.BaseDrive.Binds",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjBaseDriveBinds::RunTest(const FString&)
{
	FBaseDriveRig Rig;
	FMjUESession S;
	if (!S.Init([&Rig](FMjUESession& Sess) { Rig.Configure(Sess); }))
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}
	TestTrue(TEXT("controller bound"), Rig.Ctrl->IsBound());
	TestEqual(TEXT("nu == 4"), (int)S.Manager->PhysicsEngine->m_model->nu, 4);
	S.Cleanup();
	return true;
}

// URLab.Nav.BaseDrive.DriveForwardX — twist vx=0.5 for 1 s → base at ~+0.5 m
// MuJoCo X, ~0 Y.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjBaseDriveForwardX,
	"URLab.Nav.BaseDrive.DriveForwardX",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjBaseDriveForwardX::RunTest(const FString&)
{
	FBaseDriveRig Rig;
	FMjUESession S;
	if (!S.Init([&Rig](FMjUESession& Sess) { Rig.Configure(Sess); }))
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}
	const int StepsPerSec = FMath::RoundToInt(
		1.0 / S.Manager->PhysicsEngine->m_model->opt.timestep);
	Rig.Twist->SetTwist(0.5f, 0.f, 0.f);
	FBaseDriveRig::Drive(S, StepsPerSec);
	TestTrue(TEXT("moved ~0.5 m in +X"), BaseDriveBodyX(S) > 0.30 && BaseDriveBodyX(S) < 0.70);
	TestTrue(TEXT("negligible Y drift"), FMath::Abs(BaseDriveBodyY(S)) < 0.05);
	S.Cleanup();
	return true;
}

// URLab.Nav.BaseDrive.DriveWorldY — vy=+0.5 (bus LEFT = MuJoCo +Y) for 1 s
// → base displaces +Y MuJoCo. Catches axis-convention mistakes.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjBaseDriveWorldY,
	"URLab.Nav.BaseDrive.DriveWorldY",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjBaseDriveWorldY::RunTest(const FString&)
{
	FBaseDriveRig Rig;
	FMjUESession S;
	if (!S.Init([&Rig](FMjUESession& Sess) { Rig.Configure(Sess); }))
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}
	const int StepsPerSec = FMath::RoundToInt(
		1.0 / S.Manager->PhysicsEngine->m_model->opt.timestep);
	Rig.Twist->SetTwist(0.f, 0.5f, 0.f);
	FBaseDriveRig::Drive(S, StepsPerSec);
	TestTrue(TEXT("moved ~+0.5 m in MuJoCo +Y"), BaseDriveBodyY(S) > 0.30 && BaseDriveBodyY(S) < 0.70);
	S.Cleanup();
	return true;
}

// URLab.Nav.BaseDrive.YawThenForward — rotate ~90° CCW, then drive vx →
// world displacement is +Y MuJoCo (robot frame rotated). Proves the
// world-frame rotation in the servo.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjBaseDriveYawThenForward,
	"URLab.Nav.BaseDrive.YawThenForward",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjBaseDriveYawThenForward::RunTest(const FString&)
{
	FBaseDriveRig Rig;
	FMjUESession S;
	if (!S.Init([&Rig](FMjUESession& Sess) { Rig.Configure(Sess); }))
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}
	const int StepsPerSec = FMath::RoundToInt(
		1.0 / S.Manager->PhysicsEngine->m_model->opt.timestep);
	Rig.Twist->SetTwist(0.f, 0.f, HALF_PI); // 90°/s CCW for 1 s
	FBaseDriveRig::Drive(S, StepsPerSec);
	Rig.Twist->SetTwist(0.5f, 0.f, 0.f); // forward in ROBOT frame
	FBaseDriveRig::Drive(S, StepsPerSec);
	TestTrue(TEXT("second leg went +Y (world)"), BaseDriveBodyY(S) > 0.25);
	TestTrue(TEXT("little +X in second leg"), FMath::Abs(BaseDriveBodyX(S)) < 0.25);
	S.Cleanup();
	return true;
}

// URLab.Nav.BaseDrive.LeashBoundsBlocked — X joint limited to [0, 5 cm];
// command vx=1.0 for 1 s: without the leash ctrl would integrate to ~1.0;
// with leash + range clamp it stays ≤ range max + leash.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjBaseDriveLeash,
	"URLab.Nav.BaseDrive.LeashBoundsBlocked",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjBaseDriveLeash::RunTest(const FString&)
{
	FBaseDriveRig Rig;
	FMjUESession S;
	if (!S.Init([&Rig](FMjUESession& Sess) {
			Rig.Configure(Sess);
			Sess.Joint->limited = true;
			Sess.Joint->bOverride_limited = true;
			Sess.Joint->range = {0.f, 5.f}; // slide range in cm per tooltip
			Sess.Joint->bOverride_range = true;
		}))
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}
	mjModel* m = S.Manager->PhysicsEngine->m_model;
	mjData* d = S.Manager->PhysicsEngine->m_data;
	const int StepsPerSec = FMath::RoundToInt(1.0 / m->opt.timestep);
	Rig.Twist->SetTwist(1.0f, 0.f, 0.f);
	FBaseDriveRig::Drive(S, StepsPerSec);
	const int ActX = mj_name2id(m, mjOBJ_ACTUATOR,
		TCHAR_TO_UTF8(*FString(S.Robot->GetName() + TEXT("_act_x"))));
	// Fallback: scan for the actuator whose name ends with act_x.
	int Id = ActX;
	if (Id < 0)
		for (int i = 0; i < m->nu; ++i)
			if (FString(mj_id2name(m, mjOBJ_ACTUATOR, i)).EndsWith(TEXT("act_x")))
			{
				Id = i;
				break;
			}
	TestTrue(TEXT("found act_x"), Id >= 0);
	if (Id >= 0)
		TestTrue(TEXT("ctrl leashed (< 0.12, not ~1.0)"), d->ctrl[Id] < 0.12);
	S.Cleanup();
	return true;
}

// URLab.Nav.BaseDrive.PassThrough — non-base actuator still receives its
// UI value while the controller is bound (constraint: bound controller
// replaces the default ctrl path).
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMjBaseDrivePassThrough,
	"URLab.Nav.BaseDrive.PassThrough",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)
bool FMjBaseDrivePassThrough::RunTest(const FString&)
{
	FBaseDriveRig Rig;
	FMjUESession S;
	if (!S.Init([&Rig](FMjUESession& Sess) { Rig.Configure(Sess); }))
	{
		AddError(S.LastError);
		S.Cleanup();
		return false;
	}
	mjModel* m = S.Manager->PhysicsEngine->m_model;
	mjData* d = S.Manager->PhysicsEngine->m_data;
	Rig.ArmAct->SetControl(3.3f);
	S.Robot->ApplyControls();
	int Id = -1;
	for (int i = 0; i < m->nu; ++i)
		if (FString(mj_id2name(m, mjOBJ_ACTUATOR, i)).EndsWith(TEXT("arm_act")))
		{
			Id = i;
			break;
		}
	TestTrue(TEXT("found arm_act"), Id >= 0);
	if (Id >= 0)
		TestEqual(TEXT("arm ctrl passed through"), (float)d->ctrl[Id], 3.3f, 1e-3f);
	S.Cleanup();
	return true;
}
