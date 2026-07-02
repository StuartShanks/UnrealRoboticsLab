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
#include "Lie/MinkSO3.h"

/**
 * Port of mink lie/se3.py SE3 — special Euclidean group for proper rigid transforms in 3D.
 * Internal parameterization is (qw, qx, qy, qz, x, y, z). Tangent parameterization is
 * (vx, vy, vz, wx, wy, wz) — translation FIRST.
 */
struct URLABMINK_API FMinkSE3
{
	double WxyzXyz[7] = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	static FMinkSE3 Identity();
	static FMinkSE3 FromWxyzXyz(const double* In);
	static FMinkSE3 FromRotationAndTranslation(const FMinkSO3& Rotation, const FMinkVec3& Translation);
	static FMinkSE3 FromRotation(const FMinkSO3& Rotation);
	static FMinkSE3 FromTranslation(const FMinkVec3& Translation);
	static FMinkSE3 FromMatrix(const Eigen::Matrix4d& M);
	static FMinkSE3 FromMocapId(const mjData* Data, int32 MocapId);
	static FMinkSE3 Exp(const FMinkVec6& Tangent);
	static FMinkSE3 SampleUniform(FRandomStream& Rng);

	FMinkSO3 Rotation() const;
	FMinkVec3 Translation() const;
	Eigen::Matrix4d AsMatrix() const;
	FMinkVec6 Log() const;
	FMinkMat6 Adjoint() const;
	FMinkSE3 Inverse() const;
	FMinkSE3 Normalize() const;
	FMinkSE3 Multiply(const FMinkSE3& Other) const;
	FMinkVec3 Apply(const FMinkVec3& Target) const;

	static FMinkMat6 Ljac(const FMinkVec6& Other);
	static FMinkMat6 Ljacinv(const FMinkVec6& Other);
	static FMinkMat6 Rjac(const FMinkVec6& Other);    // Ljac(-Other)
	static FMinkMat6 Rjacinv(const FMinkVec6& Other); // Ljacinv(-Other)
	FMinkMat6 Jlog() const;                           // Rjacinv(Log())

	FMinkSE3 RPlus(const FMinkVec6& Other) const;  // this * Exp(Other)
	FMinkVec6 RMinus(const FMinkSE3& Other) const; // (Other^-1 * this).Log()
	FMinkSE3 Interpolate(const FMinkSE3& Other, double Alpha) const;
};
