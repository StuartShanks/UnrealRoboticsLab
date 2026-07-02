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

#include "Lie/MinkSE3.h"

namespace
{
// Eqn 180.
FMinkMat3 GetQ(const FMinkVec6& Other)
{
	const double OmegaArr[3] = {Other(3), Other(4), Other(5)};
	const double Theta = mju_norm3(OmegaArr);
	const double T2 = Theta * Theta;
	const double A = 0.5;
	double B, C, D;
	if (T2 < MinkEpsilon)
	{
		B = (1.0 / 6.0) + (1.0 / 120.0) * T2;
		C = -(1.0 / 24.0) + (1.0 / 720.0) * T2;
		D = -(1.0 / 60.0);
	}
	else
	{
		const double T4 = T2 * T2;
		const double SinTheta = FMath::Sin(Theta);
		const double CosTheta = FMath::Cos(Theta);
		B = (Theta - SinTheta) / (T2 * Theta);
		C = (1.0 - 0.5 * T2 - CosTheta) / T4;
		D = (2.0 * Theta - 3.0 * SinTheta + Theta * CosTheta) / (2.0 * T4 * Theta);
	}
	const FMinkMat3 V = MinkSkew(Other.head<3>());
	const FMinkMat3 W = MinkSkew(Other.tail<3>());
	const FMinkMat3 VW = V * W;
	const FMinkMat3 WV = VW.transpose();
	const FMinkMat3 WVW = WV * W;
	const FMinkMat3 VWW = VW * W;
	return A * V + B * (WV + VW + WVW) - C * (VWW - VWW.transpose() - 3.0 * WVW) + D * (WVW * W + W * WVW);
}
} // namespace

FMinkSE3 FMinkSE3::Identity()
{
	return FMinkSE3();
}

FMinkSE3 FMinkSE3::FromWxyzXyz(const double* In)
{
	FMinkSE3 Result;
	for (int32 i = 0; i < 7; ++i)
	{
		Result.WxyzXyz[i] = In[i];
	}
	return Result;
}

FMinkSE3 FMinkSE3::FromRotationAndTranslation(const FMinkSO3& Rotation, const FMinkVec3& Translation)
{
	FMinkSE3 Result;
	for (int32 i = 0; i < 4; ++i)
	{
		Result.WxyzXyz[i] = Rotation.Wxyz[i];
	}
	Result.WxyzXyz[4] = Translation(0);
	Result.WxyzXyz[5] = Translation(1);
	Result.WxyzXyz[6] = Translation(2);
	return Result;
}

FMinkSE3 FMinkSE3::FromRotation(const FMinkSO3& Rotation)
{
	return FromRotationAndTranslation(Rotation, FMinkVec3::Zero());
}

FMinkSE3 FMinkSE3::FromTranslation(const FMinkVec3& Translation)
{
	return FromRotationAndTranslation(FMinkSO3::Identity(), Translation);
}

FMinkSE3 FMinkSE3::FromMatrix(const Eigen::Matrix4d& M)
{
	const FMinkMat3 R = M.block<3, 3>(0, 0);
	const FMinkVec3 T = M.block<3, 1>(0, 3);
	return FromRotationAndTranslation(FMinkSO3::FromMatrix(R), T);
}

FMinkSE3 FMinkSE3::FromMocapId(const mjData* Data, int32 MocapId)
{
	const FMinkSO3 Rot = FMinkSO3::FromWxyz(Data->mocap_quat + 4 * MocapId);
	const FMinkVec3 Trans(
		Data->mocap_pos[3 * MocapId], Data->mocap_pos[3 * MocapId + 1], Data->mocap_pos[3 * MocapId + 2]);
	return FromRotationAndTranslation(Rot, Trans);
}

FMinkSE3 FMinkSE3::Exp(const FMinkVec6& Tangent)
{
	const FMinkSO3 Rotation = FMinkSO3::Exp(Tangent.tail<3>());
	const double OmegaArr[3] = {Tangent(3), Tangent(4), Tangent(5)};
	const double Theta = mju_norm3(OmegaArr);
	const double T2 = Theta * Theta;
	FMinkMat3 VMat;
	if (T2 < MinkEpsilon)
	{
		VMat = Rotation.AsMatrix();
	}
	else
	{
		const FMinkMat3 SkewOmega = MinkSkew(Tangent.tail<3>());
		VMat = FMinkMat3::Identity() + (1.0 - FMath::Cos(Theta)) / T2 * SkewOmega
			 + (Theta - FMath::Sin(Theta)) / (T2 * Theta) * (SkewOmega * SkewOmega);
	}
	return FromRotationAndTranslation(Rotation, VMat * Tangent.head<3>());
}

FMinkSE3 FMinkSE3::SampleUniform(FRandomStream& Rng)
{
	const FMinkSO3 Rotation = FMinkSO3::SampleUniform(Rng);
	const FMinkVec3 Translation(
		Rng.FRandRange(-1.0, 1.0), Rng.FRandRange(-1.0, 1.0), Rng.FRandRange(-1.0, 1.0));
	return FromRotationAndTranslation(Rotation, Translation);
}

FMinkSO3 FMinkSE3::Rotation() const
{
	return FMinkSO3::FromWxyz(WxyzXyz);
}

FMinkVec3 FMinkSE3::Translation() const
{
	return FMinkVec3(WxyzXyz[4], WxyzXyz[5], WxyzXyz[6]);
}

Eigen::Matrix4d FMinkSE3::AsMatrix() const
{
	Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
	M.block<3, 3>(0, 0) = Rotation().AsMatrix();
	M.block<3, 1>(0, 3) = Translation();
	return M;
}

FMinkVec6 FMinkSE3::Log() const
{
	const FMinkVec3 Omega = Rotation().Log();
	const double OmegaArr[3] = {Omega(0), Omega(1), Omega(2)};
	const double Theta = mju_norm3(OmegaArr);
	const double T2 = Theta * Theta;
	const FMinkMat3 SkewOmega = MinkSkew(Omega);
	const FMinkMat3 SkewOmega2 = SkewOmega * SkewOmega;
	FMinkMat3 VInvMat;
	if (T2 < MinkEpsilon)
	{
		VInvMat = FMinkMat3::Identity() - 0.5 * SkewOmega + SkewOmega2 / 12.0;
	}
	else
	{
		const double HalfTheta = 0.5 * Theta;
		VInvMat = FMinkMat3::Identity() - 0.5 * SkewOmega
				+ (1.0 - 0.5 * Theta * FMath::Cos(HalfTheta) / FMath::Sin(HalfTheta)) / T2 * SkewOmega2;
	}
	FMinkVec6 Tangent;
	Tangent.head<3>() = VInvMat * Translation();
	Tangent.tail<3>() = Omega;
	return Tangent;
}

FMinkMat6 FMinkSE3::Adjoint() const
{
	const FMinkMat3 R = Rotation().AsMatrix();
	const FMinkMat3 TangentMat = MinkSkew(Translation()) * R;
	FMinkMat6 Result = FMinkMat6::Zero();
	Result.block<3, 3>(0, 0) = R;
	Result.block<3, 3>(0, 3) = TangentMat;
	Result.block<3, 3>(3, 3) = R;
	return Result;
}

FMinkSE3 FMinkSE3::Inverse() const
{
	FMinkSE3 Result;
	mju_negQuat(Result.WxyzXyz, WxyzXyz);
	const double NegXyz[3] = {-WxyzXyz[4], -WxyzXyz[5], -WxyzXyz[6]};
	mju_rotVecQuat(Result.WxyzXyz + 4, NegXyz, Result.WxyzXyz);
	return Result;
}

FMinkSE3 FMinkSE3::Normalize() const
{
	FMinkSE3 Result = *this;
	mju_normalize4(Result.WxyzXyz);
	return Result;
}

FMinkSE3 FMinkSE3::Multiply(const FMinkSE3& Other) const
{
	FMinkSE3 Result;
	mju_mulQuat(Result.WxyzXyz, WxyzXyz, Other.WxyzXyz);
	double T[3];
	mju_rotVecQuat(T, Other.WxyzXyz + 4, WxyzXyz);
	Result.WxyzXyz[4] = T[0] + WxyzXyz[4];
	Result.WxyzXyz[5] = T[1] + WxyzXyz[5];
	Result.WxyzXyz[6] = T[2] + WxyzXyz[6];
	return Result;
}

FMinkVec3 FMinkSE3::Apply(const FMinkVec3& Target) const
{
	const double V[3] = {Target(0), Target(1), Target(2)};
	double Out[3];
	mju_rotVecQuat(Out, V, WxyzXyz);
	return FMinkVec3(Out[0] + WxyzXyz[4], Out[1] + WxyzXyz[5], Out[2] + WxyzXyz[6]);
}

FMinkMat6 FMinkSE3::Ljac(const FMinkVec6& Other)
{
	const FMinkVec3 W = Other.tail<3>();
	const double WArr[3] = {Other(3), Other(4), Other(5)};
	const double ThetaSquared = mju_dot3(WArr, WArr);
	if (ThetaSquared < MinkEpsilon)
	{
		return FMinkMat6::Identity();
	}
	FMinkMat6 Result = FMinkMat6::Zero();
	const FMinkMat3 LjacTranslation = GetQ(Other);
	const FMinkMat3 LjacSo3 = FMinkSO3::Ljac(W);
	Result.block<3, 3>(0, 0) = LjacSo3;
	Result.block<3, 3>(0, 3) = LjacTranslation;
	Result.block<3, 3>(3, 3) = LjacSo3;
	return Result;
}

FMinkMat6 FMinkSE3::Ljacinv(const FMinkVec6& Other)
{
	const FMinkVec3 W = Other.tail<3>();
	const double WArr[3] = {Other(3), Other(4), Other(5)};
	const double ThetaSquared = mju_dot3(WArr, WArr);
	if (ThetaSquared < MinkEpsilon)
	{
		return FMinkMat6::Identity();
	}
	FMinkMat6 Result = FMinkMat6::Zero();
	const FMinkMat3 LjacTranslation = GetQ(Other);
	const FMinkMat3 LjacinvSo3 = FMinkSO3::Ljacinv(W);
	Result.block<3, 3>(0, 0) = LjacinvSo3;
	Result.block<3, 3>(0, 3) = -LjacinvSo3 * LjacTranslation * LjacinvSo3;
	Result.block<3, 3>(3, 3) = LjacinvSo3;
	return Result;
}

FMinkMat6 FMinkSE3::Rjac(const FMinkVec6& Other)
{
	return Ljac(-Other);
}

FMinkMat6 FMinkSE3::Rjacinv(const FMinkVec6& Other)
{
	return Ljacinv(-Other);
}

FMinkMat6 FMinkSE3::Jlog() const
{
	return Rjacinv(Log());
}

FMinkSE3 FMinkSE3::RPlus(const FMinkVec6& Other) const
{
	return Multiply(FMinkSE3::Exp(Other));
}

FMinkVec6 FMinkSE3::RMinus(const FMinkSE3& Other) const
{
	return Other.Inverse().Multiply(*this).Log();
}

FMinkSE3 FMinkSE3::Interpolate(const FMinkSE3& Other, double Alpha) const
{
	if (Alpha < 0.0 || Alpha > 1.0)
	{
		UE_LOG(LogURLabMink, Error, TEXT("Expected alpha within [0.0, 1.0] but received %f"), Alpha);
		return *this;
	}
	const FMinkVec6 Delta = Inverse().Multiply(Other).Log();
	return Multiply(FMinkSE3::Exp(Alpha * Delta));
}
