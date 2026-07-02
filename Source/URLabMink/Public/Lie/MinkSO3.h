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
 * Port of mink lie/so3.py SO3 — special orthogonal group for 3D rotations.
 * Internal parameterization is (qw, qx, qy, qz). Tangent parameterization is (wx, wy, wz).
 */
struct URLABMINK_API FMinkSO3
{
	double Wxyz[4] = {1.0, 0.0, 0.0, 0.0};

	static FMinkSO3 Identity();
	static FMinkSO3 FromWxyz(const double* InWxyz);
	static FMinkSO3 FromMatrix(const FMinkMat3& M);
	static FMinkSO3 FromXRadians(double Theta);
	static FMinkSO3 FromYRadians(double Theta);
	static FMinkSO3 FromZRadians(double Theta);
	static FMinkSO3 FromRpyRadians(double Roll, double Pitch, double Yaw);
	static FMinkSO3 Exp(const FMinkVec3& Tangent);
	static FMinkSO3 SampleUniform(FRandomStream& Rng);

	FMinkMat3 AsMatrix() const;
	FMinkVec3 Log() const;
	FMinkMat3 Adjoint() const; // == AsMatrix()
	FMinkSO3 Inverse() const;
	FMinkSO3 Normalize() const;
	FMinkSO3 Multiply(const FMinkSO3& Other) const;
	FMinkVec3 Apply(const FMinkVec3& Target) const;
	double ComputeRollRadians() const;
	double ComputePitchRadians() const;
	double ComputeYawRadians() const;
	FMinkVec3 AsRpyRadians() const; // (roll, pitch, yaw)

	static FMinkMat3 Ljac(const FMinkVec3& Other);
	static FMinkMat3 Ljacinv(const FMinkVec3& Other);
	static FMinkMat3 Rjac(const FMinkVec3& Other);    // Ljac(-Other)
	static FMinkMat3 Rjacinv(const FMinkVec3& Other); // Ljacinv(-Other)

	FMinkSO3 RPlus(const FMinkVec3& Other) const;  // this * Exp(Other)
	FMinkVec3 RMinus(const FMinkSO3& Other) const; // (Other^-1 * this).Log()
	FMinkSO3 Interpolate(const FMinkSO3& Other, double Alpha) const;
	FMinkSO3 Clamp(const FMinkVec3& RpyLower, const FMinkVec3& RpyUpper) const;
};
