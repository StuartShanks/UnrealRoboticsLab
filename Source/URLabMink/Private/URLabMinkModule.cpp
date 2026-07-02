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

#include "MinkTypes.h"
#include "Modules/ModuleManager.h"

DEFINE_LOG_CATEGORY(LogURLabMink);

mjtObj MinkFrameTypeToObj(EMinkFrameType Type)
{
	switch (Type)
	{
		case EMinkFrameType::Body:
			return mjOBJ_BODY;
		case EMinkFrameType::Geom:
			return mjOBJ_GEOM;
		default:
			return mjOBJ_SITE;
	}
}

bool MinkFrameTypeFromString(const FString& S, EMinkFrameType& Out)
{
	if (S == TEXT("body"))
	{
		Out = EMinkFrameType::Body;
		return true;
	}
	if (S == TEXT("geom"))
	{
		Out = EMinkFrameType::Geom;
		return true;
	}
	if (S == TEXT("site"))
	{
		Out = EMinkFrameType::Site;
		return true;
	}
	return false;
}

int32 MinkDofWidth(int32 JointType)
{
	switch (JointType)
	{
		case mjJNT_FREE:
			return 6;
		case mjJNT_BALL:
			return 3;
		default:
			return 1; // slide, hinge
	}
}

int32 MinkQposWidth(int32 JointType)
{
	switch (JointType)
	{
		case mjJNT_FREE:
			return 7;
		case mjJNT_BALL:
			return 4;
		default:
			return 1; // slide, hinge
	}
}

int32 MinkConstraintWidth(int32 EqType)
{
	switch (EqType)
	{
		case mjEQ_CONNECT:
			return 3;
		case mjEQ_WELD:
			return 6;
		default:
			return 1; // joint, tendon
	}
}

FMinkMat3 MinkSkew(const FMinkVec3& X)
{
	FMinkMat3 M;
	// clang-format off
	M <<   0.0, -X(2),  X(1),
	      X(2),   0.0, -X(0),
	     -X(1),  X(0),   0.0;
	// clang-format on
	return M;
}

IMPLEMENT_MODULE(FDefaultModuleImpl, URLabMink)
