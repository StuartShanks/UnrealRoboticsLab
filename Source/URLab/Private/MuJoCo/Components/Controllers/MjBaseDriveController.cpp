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

#include "MuJoCo/Components/Controllers/MjBaseDriveController.h"
#include "Utils/URLabLogging.h" // LogURLab — Bind diagnostics must be visible

#include "MuJoCo/Components/Actuators/MjActuator.h"
#include "MuJoCo/Input/MjTwistController.h"

#include <mujoco/mujoco.h>

namespace
{
/** Suffix-tolerant joint resolution (same pattern as the mink controller):
 *  exact, else compiled name ends with "_Name" or "/Name". */
int32 ResolveJointByName(const mjModel* m, const FString& Name)
{
	int32 Id = mj_name2id(m, mjOBJ_JOINT, TCHAR_TO_UTF8(*Name));
	if (Id >= 0)
		return Id;
	const FString US = TEXT("_") + Name, SL = TEXT("/") + Name;
	for (int32 i = 0; i < m->njnt; ++i)
	{
		const char* Nm = mj_id2name(m, mjOBJ_JOINT, i);
		if (!Nm)
			continue;
		const FString S = UTF8_TO_TCHAR(Nm);
		if (S == Name || S.EndsWith(US) || S.EndsWith(SL))
			return i;
	}
	return -1;
}
} // namespace

void UMjBaseDriveController::Bind(mjModel* m, mjData* d, const TMap<int32, UMjActuator*>& Map)
{
	Super::Bind(m, d, Map); // fills Bindings

	// Sibling twist source — Bind runs on the game thread (PostSetup), so
	// component discovery is safe here; GetTwist() is thread-safe later.
	TwistSource = GetOwner() ? GetOwner()->FindComponentByClass<UMjTwistController>() : nullptr;
	if (!TwistSource)
		UE_LOG(LogURLab, Warning,
			TEXT("UMjBaseDriveController '%s': no UMjTwistController sibling — base will hold position."),
			*GetName());

	// Resolve the 3 base joints to Bindings entries via their qpos address.
	bBaseResolved = false;
	BaseBindingIdx[0] = BaseBindingIdx[1] = BaseBindingIdx[2] = -1;
	if (BaseJointNames.Num() != 3)
	{
		UE_LOG(LogURLab, Error,
			TEXT("UMjBaseDriveController '%s': BaseJointNames must have exactly 3 entries (X, Y, TH)."),
			*GetName());
		return;
	}
	for (int32 k = 0; k < 3; ++k)
	{
		const int32 Jid = ResolveJointByName(m, BaseJointNames[k]);
		if (Jid < 0)
		{
			UE_LOG(LogURLab, Error,
				TEXT("UMjBaseDriveController '%s': base joint '%s' not found in compiled model."),
				*GetName(), *BaseJointNames[k]);
			return;
		}
		const int32 QposAddr = m->jnt_qposadr[Jid];
		for (int32 b = 0; b < Bindings.Num(); ++b)
		{
			if (Bindings[b].QposAddr == QposAddr)
			{
				BaseBindingIdx[k] = b;
				break;
			}
		}
		if (BaseBindingIdx[k] < 0)
		{
			UE_LOG(LogURLab, Error,
				TEXT("UMjBaseDriveController '%s': no actuator bound to base joint '%s'."),
				*GetName(), *BaseJointNames[k]);
			return;
		}
		Target[k] = d->qpos[QposAddr]; // seed
	}
	bBaseResolved = true;
}

void UMjBaseDriveController::ComputeAndApply(mjModel* m, mjData* d, uint8 Source)
{
	// Pass-through FIRST for every non-base binding: a bound controller
	// replaces the articulation's default ctrl path entirely, so the arm
	// actuators must keep receiving their UI/network values through us.
	for (int32 b = 0; b < Bindings.Num(); ++b)
	{
		if (bBaseResolved
			&& (b == BaseBindingIdx[0] || b == BaseBindingIdx[1] || b == BaseBindingIdx[2]))
			continue;
		const FActuatorBinding& B = Bindings[b];
		if (B.ActuatorMjID >= 0 && B.Component)
			d->ctrl[B.ActuatorMjID] = B.Component->ResolveDesiredControl(Source);
	}

	if (!bBaseResolved)
		return;

	// Twist (robot frame, bus convention) → world-frame velocity.
	double Vx = 0, Vy = 0, W = 0;
	if (TwistSource)
	{
		const FVector T = TwistSource->GetTwist(); // (Vx, Vy, YawRate)
		Vx = T.X;
		Vy = T.Y;
		W = T.Z;
	}
	const double Theta = d->qpos[Bindings[BaseBindingIdx[2]].QposAddr];
	const double C = FMath::Cos(Theta), S = FMath::Sin(Theta);
	const double VWorld[3] = {C * Vx - S * Vy, S * Vx + C * Vy, W};

	const double Dt = m->opt.timestep;
	for (int32 k = 0; k < 3; ++k)
	{
		const FActuatorBinding& B = Bindings[BaseBindingIdx[k]];
		const double Qpos = d->qpos[B.QposAddr];

		if (ActuatorMode == EMjBaseDriveActuatorMode::VelocityDirect)
		{
			d->ctrl[B.ActuatorMjID] = VWorld[k];
			continue;
		}

		// PositionIntegrate: integrate, re-seed on discontinuity, leash,
		// clamp to joint range.
		Target[k] += VWorld[k] * Dt;
		const double Leash = (k == 2) ? MaxLeashAngular : MaxLeashLinear;
		if (FMath::Abs(Target[k] - Qpos) > ReseedThreshold)
			Target[k] = Qpos;
		Target[k] = FMath::Clamp(Target[k], Qpos - Leash, Qpos + Leash);

		// Respect joint limits if present.
		const int32 Jid = m->actuator_trnid[B.ActuatorMjID * 2];
		if (Jid >= 0 && m->jnt_limited[Jid])
			Target[k] = FMath::Clamp(Target[k],
				(double)m->jnt_range[Jid * 2], (double)m->jnt_range[Jid * 2 + 1]);

		d->ctrl[B.ActuatorMjID] = Target[k];
	}
}
