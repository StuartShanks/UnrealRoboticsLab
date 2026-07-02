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
#include "Lie/MinkSE3.h"

/**
 * Port of mink configuration.py Configuration — encapsulates a MuJoCo model and its own
 * mjData for convenient access to kinematic quantities (frame transforms, Jacobians, the
 * joint-space inertia matrix). Runs forward kinematics on construction and on every Update().
 *
 * Owns its mjData (mj_makeData in the ctor, mj_deleteData in the dtor), exactly like the
 * Python Configuration.__init__ creating its own MjData. The mjModel is NOT owned; the
 * caller is responsible for its lifetime (and for mj_deleteModel-ing it).
 */
class URLABMINK_API FMinkConfiguration
{
public:
	explicit FMinkConfiguration(const mjModel* InModel, const double* Q = nullptr);
	~FMinkConfiguration();

	FMinkConfiguration(const FMinkConfiguration&) = delete;
	FMinkConfiguration& operator=(const FMinkConfiguration&) = delete;

	/** Run forward kinematics. If Q is non-null, overwrites Data->qpos first (mirrors Python update(q=...)). */
	void Update(const double* Q = nullptr);

	/** Update the configuration from a keyframe. False + error log if KeyName is unknown. */
	bool UpdateFromKeyframe(const FString& KeyName);

	/**
	 * Check that the current configuration is within bounds.
	 * Returns false iff the configuration violates limits AND bSafetyBreak is true (mirrors
	 * Python's check_limits raising NotWithinConfigurationLimits only when safety_break=True).
	 */
	bool CheckLimits(double Tol = 1e-6, bool bSafetyBreak = true) const;

	/** Frame Jacobian expressed in the local (body) frame; OutJac is resized to 6 x nv. */
	bool GetFrameJacobian(const FString& FrameName, EMinkFrameType FrameType, FMinkMat& OutJac) const;

	/** Pose of a frame in the world frame at the current configuration. */
	bool GetTransformFrameToWorld(const FString& FrameName, EMinkFrameType FrameType, FMinkSE3& Out) const;

	/** Pose of SourceName with respect to DestName at the current configuration. */
	bool GetTransform(const FString& SourceName, EMinkFrameType SourceType, const FString& DestName,
		EMinkFrameType DestType, FMinkSE3& Out) const;

	/** Integrate a velocity from the current configuration, returning the new configuration. */
	FMinkVec Integrate(const FMinkVec& Velocity, double Dt) const;

	/** Integrate a velocity and update the current configuration in place (re-runs Update()). */
	void IntegrateInplace(const FMinkVec& Velocity, double Dt);

	/** Joint-space inertia matrix M(q) at the current configuration. */
	FMinkMat GetInertiaMatrix() const;

	/** The current configuration vector (copy of Data->qpos). */
	FMinkVec GetQ() const;

	int32 Nv() const { return Model->nv; }
	int32 Nq() const { return Model->nq; }

	const mjModel* Model;
	mjData* Data;
	FMinkMat EyeNv; // cached identity, mirrors configuration._eye_nv

private:
	int32 ResolveFrameId(const FString& FrameName, EMinkFrameType FrameType) const; // -1 invalid; cached
	mutable TMap<TPair<FString, uint8>, int32> FrameIdCache;

	// Precomputed limited-joint arrays (jnt ids, qpos adr, ranges), mirrors __init__.
	TArray<int32> LimitedJntIds;
	TArray<int32> LimitedQposAdr;
	TArray<double> LimitedLower, LimitedUpper;
};
