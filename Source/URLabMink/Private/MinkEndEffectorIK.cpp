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

#include "MinkEndEffectorIK.h"

#include "Tasks/MinkFrameTask.h"
#include "Tasks/MinkPostureTask.h"

#include <mujoco/mujoco.h>

namespace
{
/** mjtObj for a mink frame type. */
int MinkFrameObjType(EMinkFrameType Type)
{
	switch (Type)
	{
		case EMinkFrameType::Site:
			return mjOBJ_SITE;
		case EMinkFrameType::Geom:
			return mjOBJ_GEOM;
		case EMinkFrameType::Body:
		default:
			return mjOBJ_BODY;
	}
}
} // namespace

FMinkEndEffectorIK::FMinkEndEffectorIK(const mjModel* InModel, const FMinkEndEffectorIKConfig& InConfig)
	: Model(InModel)
	, Config(InConfig)
{
	check(Model);

	Configuration = MakeUnique<FMinkConfiguration>(Model);

	const FMinkVec PosCost = FMinkVec::Constant(1, Config.PositionCost);
	const FMinkVec OriCost = FMinkVec::Constant(1, Config.OrientationCost);
	FrameTask = MakeUnique<FMinkFrameTask>(
		Config.FrameName, Config.FrameType, PosCost, OriCost, Config.Gain, Config.LmDamping);

	const FMinkVec PostCost = FMinkVec::Constant(1, Config.PostureCost);
	PostureTask = MakeUnique<FMinkPostureTask>(Model, PostCost, Config.Gain, Config.LmDamping);
	// Default posture target = the model's initial configuration.
	PostureTask->SetTargetFromConfiguration(*Configuration);

	// Resolve the target frame.
	const bool bFrameOk =
		mj_name2id(Model, MinkFrameObjType(Config.FrameType), TCHAR_TO_UTF8(*Config.FrameName)) >= 0;
	if (!bFrameOk)
	{
		UE_LOG(LogURLabMink, Error, TEXT("FMinkEndEffectorIK: frame '%s' not found in model."),
			*Config.FrameName);
	}

	// Resolve the drive joints (their qpos / qvel addresses).
	bool bJointsOk = true;
	if (Config.DriveJointNames.Num() == 0)
	{
		for (int32 j = 0; j < Model->njnt; ++j)
		{
			DriveQposAdr.Add(Model->jnt_qposadr[j]);
			DriveDofAdr.Add(Model->jnt_dofadr[j]);
		}
	}
	else
	{
		for (const FString& Name : Config.DriveJointNames)
		{
			const int32 Id = mj_name2id(Model, mjOBJ_JOINT, TCHAR_TO_UTF8(*Name));
			if (Id < 0)
			{
				UE_LOG(LogURLabMink, Error, TEXT("FMinkEndEffectorIK: joint '%s' not found in model."),
					*Name);
				bJointsOk = false;
				continue;
			}
			DriveQposAdr.Add(Model->jnt_qposadr[Id]);
			DriveDofAdr.Add(Model->jnt_dofadr[Id]);
		}
	}

	bValid = bFrameOk && bJointsOk && DriveQposAdr.Num() > 0;
}

FMinkEndEffectorIK::~FMinkEndEffectorIK() = default;

void FMinkEndEffectorIK::SetPostureTarget(const FMinkVec& TargetQ)
{
	if (PostureTask.IsValid())
	{
		PostureTask->SetTarget(TargetQ);
	}
}

EMinkIKStatus FMinkEndEffectorIK::SolveStep(mjData* LiveData, const FMinkSE3& TargetInMjWorld, double Dt)
{
	if (!bValid || LiveData == nullptr)
	{
		return EMinkIKStatus::TaskError;
	}

	// Load the live configuration into the solver's scratch data, aim the task, solve.
	Configuration->Update(LiveData->qpos);
	FrameTask->SetTarget(TargetInMjWorld);

	TArray<const FMinkBaseTask*> Tasks;
	Tasks.Add(FrameTask.Get());
	Tasks.Add(PostureTask.Get());

	const FMinkIKResult Result =
		MinkSolveIK(*Configuration, Tasks, Dt, Config.Damping, Config.bSafetyBreak);
	if (!Result.IsSuccess())
	{
		return Result.Status;
	}

	// Integrate the tangent velocity and write only the drive joints back (kinematic apply).
	const FMinkVec Q = Configuration->Integrate(Result.Velocity, Dt);
	for (int32 i = 0; i < DriveQposAdr.Num(); ++i)
	{
		LiveData->qpos[DriveQposAdr[i]] = Q[DriveQposAdr[i]];
		LiveData->qvel[DriveDofAdr[i]] = 0.0;
	}

	return EMinkIKStatus::Success;
}

bool FMinkEndEffectorIK::GetFrameTransform(const mjData* LiveData, FMinkSE3& Out) const
{
	if (!bValid || LiveData == nullptr)
	{
		return false;
	}
	Configuration->Update(LiveData->qpos);
	return Configuration->GetTransformFrameToWorld(Config.FrameName, Config.FrameType, Out);
}
