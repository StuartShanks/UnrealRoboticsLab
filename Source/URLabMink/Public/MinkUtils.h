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
#include "MinkTypes.h"

/**
 * Port of mink utils.py — free functions operating directly on an mjModel/mjData, independent
 * of FMinkConfiguration.
 */

/** mink utils.py move_mocap_to_frame. False + error log if MocapName is not a mocap body or FrameName is unknown. */
URLABMINK_API bool MinkMoveMocapToFrame(
	const mjModel* M, mjData* D, const FString& MocapName, const FString& FrameName, EMinkFrameType FrameType);

/** mink utils.py get_freejoint_dims. */
URLABMINK_API void MinkGetFreejointDims(const mjModel* M, TArray<int32>& OutQIds, TArray<int32>& OutVIds);

/**
 * mink utils.py custom_configuration_vector. KeyName empty means "use qpos0" (Python key_name=None).
 * JointValues is an explicit (name, value) list — preserves call-site order (mink's kwargs are Python
 * dicts, insertion-ordered, but that order does not affect the result here since writes are disjoint
 * per joint; the array form is used for the deliberate FString-key-order convention used elsewhere in
 * this port). Returns false + error log if a joint name is unknown or its value has the wrong width.
 */
URLABMINK_API bool MinkCustomConfigurationVector(const mjModel* M, const FString& KeyName,
	const TArray<TPair<FString, FMinkVec>>& JointValues, FMinkVec& OutQ);

/** mink utils.py get_body_body_ids — immediate children bodies of BodyId. */
URLABMINK_API TArray<int32> MinkGetBodyBodyIds(const mjModel* M, int32 BodyId);

/** mink utils.py get_subtree_body_ids — all bodies in the subtree rooted at BodyId. */
URLABMINK_API TArray<int32> MinkGetSubtreeBodyIds(const mjModel* M, int32 BodyId);

/** mink utils.py get_body_geom_ids — geoms directly attached to BodyId. */
URLABMINK_API TArray<int32> MinkGetBodyGeomIds(const mjModel* M, int32 BodyId);

/** mink utils.py get_body_joint_ids — joints directly attached to BodyId. */
URLABMINK_API TArray<int32> MinkGetBodyJointIds(const mjModel* M, int32 BodyId);

/** mink utils.py get_subtree_geom_ids — all geoms in the subtree rooted at BodyId. */
URLABMINK_API TArray<int32> MinkGetSubtreeGeomIds(const mjModel* M, int32 BodyId);

/** mink utils.py get_subtree_joint_ids — all joints in the subtree rooted at BodyId. */
URLABMINK_API TArray<int32> MinkGetSubtreeJointIds(const mjModel* M, int32 BodyId);
