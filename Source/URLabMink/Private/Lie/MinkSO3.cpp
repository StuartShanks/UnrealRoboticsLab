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

#include "Lie/MinkSO3.h"

FMinkSO3 FMinkSO3::Identity()
{
	return FMinkSO3();
}

FMinkSO3 FMinkSO3::FromWxyz(const double* InWxyz)
{
	FMinkSO3 Result;
	for (int32 i = 0; i < 4; ++i)
	{
		Result.Wxyz[i] = InWxyz[i];
	}
	return Result;
}

FMinkSO3 FMinkSO3::FromMatrix(const FMinkMat3& M)
{
	// mju_mat2Quat expects a row-major 3x3 buffer; Eigen's default storage is col-major.
	double RowMajor[9];
	for (int32 r = 0; r < 3; ++r)
	{
		for (int32 c = 0; c < 3; ++c)
		{
			RowMajor[r * 3 + c] = M(r, c);
		}
	}
	FMinkSO3 Result;
	mju_mat2Quat(Result.Wxyz, RowMajor);
	return Result;
}

FMinkSO3 FMinkSO3::FromXRadians(double Theta)
{
	return FMinkSO3::Exp(FMinkVec3(Theta, 0.0, 0.0));
}

FMinkSO3 FMinkSO3::FromYRadians(double Theta)
{
	return FMinkSO3::Exp(FMinkVec3(0.0, Theta, 0.0));
}

FMinkSO3 FMinkSO3::FromZRadians(double Theta)
{
	return FMinkSO3::Exp(FMinkVec3(0.0, 0.0, Theta));
}

FMinkSO3 FMinkSO3::FromRpyRadians(double Roll, double Pitch, double Yaw)
{
	return FromZRadians(Yaw).Multiply(FromYRadians(Pitch)).Multiply(FromXRadians(Roll));
}

FMinkSO3 FMinkSO3::Exp(const FMinkVec3& Tangent)
{
	double Axis[3] = {Tangent(0), Tangent(1), Tangent(2)};
	const double Theta = mju_normalize3(Axis);
	FMinkSO3 Result;
	// NOTE mju_axisAngle2Quat does not normalize the quaternion but is guaranteed to
	// return a unit quaternion when axis is a unit vector, which mju_normalize3 ensures.
	mju_axisAngle2Quat(Result.Wxyz, Axis, Theta);
	return Result;
}

FMinkSO3 FMinkSO3::SampleUniform(FRandomStream& Rng)
{
	// Ref: https://lavalle.pl/planning/node198.html
	const double U1 = Rng.FRandRange(0.0, 1.0);
	const double U2 = Rng.FRandRange(0.0, 2.0 * PI);
	const double U3 = Rng.FRandRange(0.0, 2.0 * PI);
	const double A = FMath::Sqrt(1.0 - U1);
	const double B = FMath::Sqrt(U1);
	FMinkSO3 Result;
	Result.Wxyz[0] = A * FMath::Sin(U2);
	Result.Wxyz[1] = A * FMath::Cos(U2);
	Result.Wxyz[2] = B * FMath::Sin(U3);
	Result.Wxyz[3] = B * FMath::Cos(U3);
	return Result;
}

FMinkMat3 FMinkSO3::AsMatrix() const
{
	double RowMajor[9];
	mju_quat2Mat(RowMajor, Wxyz);
	FMinkMat3 M;
	for (int32 r = 0; r < 3; ++r)
	{
		for (int32 c = 0; c < 3; ++c)
		{
			M(r, c) = RowMajor[r * 3 + c];
		}
	}
	return M;
}

FMinkVec3 FMinkSO3::Log() const
{
	// NOTE: np.sign(0.0) == 0.0 in Python, so when Wxyz[0] == 0.0 the whole
	// quaternion is zeroed and the subsequent norm branch returns zeros.
	const double Sign = static_cast<double>((Wxyz[0] > 0.0) - (Wxyz[0] < 0.0));
	double Q[4];
	for (int32 i = 0; i < 4; ++i)
	{
		Q[i] = Wxyz[i] * Sign;
	}
	const double W = Q[0];
	double V[3] = {Q[1], Q[2], Q[3]};
	const double Norm = mju_normalize3(V);
	if (Norm < MinkEpsilon)
	{
		return FMinkVec3::Zero();
	}
	const double Scale = 2.0 * FMath::Atan2(Norm, W);
	return FMinkVec3(Scale * V[0], Scale * V[1], Scale * V[2]);
}

FMinkMat3 FMinkSO3::Adjoint() const
{
	return AsMatrix();
}

FMinkSO3 FMinkSO3::Inverse() const
{
	FMinkSO3 Result;
	mju_negQuat(Result.Wxyz, Wxyz);
	return Result;
}

FMinkSO3 FMinkSO3::Normalize() const
{
	FMinkSO3 Result = *this;
	mju_normalize4(Result.Wxyz);
	return Result;
}

FMinkSO3 FMinkSO3::Multiply(const FMinkSO3& Other) const
{
	FMinkSO3 Result;
	mju_mulQuat(Result.Wxyz, Wxyz, Other.Wxyz);
	return Result;
}

FMinkVec3 FMinkSO3::Apply(const FMinkVec3& Target) const
{
	const double V[3] = {Target(0), Target(1), Target(2)};
	double Out[3];
	mju_rotVecQuat(Out, V, Wxyz);
	return FMinkVec3(Out[0], Out[1], Out[2]);
}

double FMinkSO3::ComputeRollRadians() const
{
	const double Q0 = Wxyz[0], Q1 = Wxyz[1], Q2 = Wxyz[2], Q3 = Wxyz[3];
	return FMath::Atan2(2.0 * (Q0 * Q1 + Q2 * Q3), 1.0 - 2.0 * (Q1 * Q1 + Q2 * Q2));
}

double FMinkSO3::ComputePitchRadians() const
{
	const double Q0 = Wxyz[0], Q1 = Wxyz[1], Q2 = Wxyz[2], Q3 = Wxyz[3];
	return FMath::Asin(2.0 * (Q0 * Q2 - Q3 * Q1));
}

double FMinkSO3::ComputeYawRadians() const
{
	const double Q0 = Wxyz[0], Q1 = Wxyz[1], Q2 = Wxyz[2], Q3 = Wxyz[3];
	return FMath::Atan2(2.0 * (Q0 * Q3 + Q1 * Q2), 1.0 - 2.0 * (Q2 * Q2 + Q3 * Q3));
}

FMinkVec3 FMinkSO3::AsRpyRadians() const
{
	return FMinkVec3(ComputeRollRadians(), ComputePitchRadians(), ComputeYawRadians());
}

FMinkMat3 FMinkSO3::Ljac(const FMinkVec3& Other)
{
	const double Theta = Other.norm();
	const double T2 = Theta * Theta;
	double Alpha, Beta;
	if (Theta < MinkEpsilon)
	{
		Alpha = (1.0 / 2.0) * (1.0 - T2 / 12.0 * (1.0 - T2 / 30.0 * (1.0 - T2 / 56.0)));
		Beta = (1.0 / 6.0) * (1.0 - T2 / 20.0 * (1.0 - T2 / 42.0 * (1.0 - T2 / 72.0)));
	}
	else
	{
		const double T3 = T2 * Theta;
		Alpha = (1.0 - FMath::Cos(Theta)) / T2;
		Beta = (Theta - FMath::Sin(Theta)) / T3;
	}
	// skew(other) @ skew(other) == outer(other, other) - dot(other, other) * I
	const double InnerProduct = Other.dot(Other);
	FMinkMat3 Result = Beta * (Other * Other.transpose() - InnerProduct * FMinkMat3::Identity())
					 + Alpha * MinkSkew(Other) + FMinkMat3::Identity();
	return Result;
}

FMinkMat3 FMinkSO3::Ljacinv(const FMinkVec3& Other)
{
	const double Theta = Other.norm();
	const double T2 = Theta * Theta;
	double Beta;
	if (Theta < MinkEpsilon)
	{
		Beta = (1.0 / 12.0) * (1.0 + T2 / 60.0 * (1.0 + T2 / 42.0 * (1.0 + T2 / 40.0)));
	}
	else
	{
		Beta = (1.0 / T2) * (1.0 - (Theta * FMath::Sin(Theta) / (2.0 * (1.0 - FMath::Cos(Theta)))));
	}
	// skew(other) @ skew(other) == outer(other, other) - dot(other, other) * I
	const double InnerProduct = Other.dot(Other);
	FMinkMat3 Result = Beta * (Other * Other.transpose() - InnerProduct * FMinkMat3::Identity())
					 - 0.5 * MinkSkew(Other) + FMinkMat3::Identity();
	return Result;
}

FMinkMat3 FMinkSO3::Rjac(const FMinkVec3& Other)
{
	return Ljac(-Other);
}

FMinkMat3 FMinkSO3::Rjacinv(const FMinkVec3& Other)
{
	return Ljacinv(-Other);
}

FMinkSO3 FMinkSO3::RPlus(const FMinkVec3& Other) const
{
	return Multiply(FMinkSO3::Exp(Other));
}

FMinkVec3 FMinkSO3::RMinus(const FMinkSO3& Other) const
{
	return Other.Inverse().Multiply(*this).Log();
}

FMinkSO3 FMinkSO3::Interpolate(const FMinkSO3& Other, double Alpha) const
{
	if (Alpha < 0.0 || Alpha > 1.0)
	{
		UE_LOG(LogURLabMink, Error, TEXT("Expected alpha within [0.0, 1.0] but received %f"), Alpha);
		return *this;
	}
	const FMinkVec3 Delta = Inverse().Multiply(Other).Log();
	return Multiply(FMinkSO3::Exp(Alpha * Delta));
}

FMinkSO3 FMinkSO3::Clamp(const FMinkVec3& RpyLower, const FMinkVec3& RpyUpper) const
{
	const double Roll = FMath::Clamp(ComputeRollRadians(), RpyLower(0), RpyUpper(0));
	const double Pitch = FMath::Clamp(ComputePitchRadians(), RpyLower(1), RpyUpper(1));
	const double Yaw = FMath::Clamp(ComputeYawRadians(), RpyLower(2), RpyUpper(2));
	return FromRpyRadians(Roll, Pitch, Yaw);
}
