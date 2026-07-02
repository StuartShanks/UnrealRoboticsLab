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

#pragma once

#include "CoreMinimal.h"
#include <mujoco/mujoco.h>

THIRD_PARTY_INCLUDES_START
#include "Eigen/Dense"
THIRD_PARTY_INCLUDES_END

URLABMINK_API DECLARE_LOG_CATEGORY_EXTERN(LogURLabMink, Log, All);

using FMinkVec = Eigen::VectorXd;
using FMinkMat = Eigen::MatrixXd;
using FMinkVec3 = Eigen::Vector3d;
using FMinkVec6 = Eigen::Matrix<double, 6, 1>;
using FMinkMat3 = Eigen::Matrix3d;
using FMinkMat6 = Eigen::Matrix<double, 6, 6>;
using FMinkRowMat = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

/** mink lie/utils.py get_epsilon for float64. */
constexpr double MinkEpsilon = 1e-10;

/** mink constants.py SUPPORTED_FRAMES. */
enum class EMinkFrameType : uint8
{
	Body,
	Geom,
	Site
};

URLABMINK_API mjtObj MinkFrameTypeToObj(EMinkFrameType Type);
URLABMINK_API bool MinkFrameTypeFromString(const FString& S, EMinkFrameType& Out);

/** mink constants.py dof_width / qpos_width / constraint_width. */
URLABMINK_API int32 MinkDofWidth(int32 JointType);
URLABMINK_API int32 MinkQposWidth(int32 JointType);
URLABMINK_API int32 MinkConstraintWidth(int32 EqType);

/** mink lie/utils.py skew. */
URLABMINK_API FMinkMat3 MinkSkew(const FMinkVec3& X);
