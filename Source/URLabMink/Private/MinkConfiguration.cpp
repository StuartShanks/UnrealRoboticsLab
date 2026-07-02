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

#include "MinkConfiguration.h"

#include "Lie/MinkSO3.h"

namespace
{
const double* FramePos(const mjData* Data, EMinkFrameType Type, int32 Id)
{
	switch (Type)
	{
		case EMinkFrameType::Body:
			return Data->xpos + 3 * Id;
		case EMinkFrameType::Geom:
			return Data->geom_xpos + 3 * Id;
		default:
			return Data->site_xpos + 3 * Id;
	}
}

const double* FrameXmat(const mjData* Data, EMinkFrameType Type, int32 Id)
{
	switch (Type)
	{
		case EMinkFrameType::Body:
			return Data->xmat + 9 * Id;
		case EMinkFrameType::Geom:
			return Data->geom_xmat + 9 * Id;
		default:
			return Data->site_xmat + 9 * Id;
	}
}

const TCHAR* FrameTypeName(EMinkFrameType Type)
{
	switch (Type)
	{
		case EMinkFrameType::Body:
			return TEXT("body");
		case EMinkFrameType::Geom:
			return TEXT("geom");
		default:
			return TEXT("site");
	}
}
} // namespace

FMinkConfiguration::FMinkConfiguration(const mjModel* InModel, const double* Q)
	: Model(InModel)
	, Data(mj_makeData(InModel))
	, EyeNv(FMinkMat::Identity(InModel->nv, InModel->nv))
{
	// Precompute limited joint indices for vectorized CheckLimits, mirrors Python __init__.
	for (int32 Jnt = 0; Jnt < Model->njnt; ++Jnt)
	{
		if (Model->jnt_limited[Jnt] && Model->jnt_type[Jnt] != mjJNT_FREE)
		{
			LimitedJntIds.Add(Jnt);
			LimitedQposAdr.Add(Model->jnt_qposadr[Jnt]);
			LimitedLower.Add(Model->jnt_range[2 * Jnt + 0]);
			LimitedUpper.Add(Model->jnt_range[2 * Jnt + 1]);
		}
	}

	Update(Q);
}

FMinkConfiguration::~FMinkConfiguration()
{
	if (Data != nullptr)
	{
		mj_deleteData(Data);
	}
}

void FMinkConfiguration::Update(const double* Q)
{
	if (Q != nullptr)
	{
		FMemory::Memcpy(Data->qpos, Q, sizeof(double) * Model->nq);
	}
	// The minimal function call required to get updated frame transforms is mj_kinematics.
	// An extra call to mj_comPos is required for updated Jacobians.
	mj_kinematics(Model, Data);
	mj_comPos(Model, Data);
	if (Model->neq > 0)
	{
		mj_makeConstraint(Model, Data);
	}
}

bool FMinkConfiguration::UpdateFromKeyframe(const FString& KeyName)
{
	const int32 KeyId = mj_name2id(Model, mjOBJ_KEY, TCHAR_TO_UTF8(*KeyName));
	if (KeyId == -1)
	{
		UE_LOG(LogURLabMink, Error,
			TEXT("Keyframe %s does not exist in the model."), *KeyName);
		return false;
	}
	Update(Model->key_qpos + (int64)KeyId * Model->nq);
	return true;
}

bool FMinkConfiguration::CheckLimits(double Tol, bool bSafetyBreak) const
{
	if (LimitedJntIds.Num() == 0)
	{
		return true;
	}

	TArray<int32> ViolatedIndices;
	for (int32 Idx = 0; Idx < LimitedJntIds.Num(); ++Idx)
	{
		const double QVal = Data->qpos[LimitedQposAdr[Idx]];
		if (QVal < LimitedLower[Idx] - Tol || QVal > LimitedUpper[Idx] + Tol)
		{
			ViolatedIndices.Add(Idx);
		}
	}
	if (ViolatedIndices.Num() == 0)
	{
		return true;
	}

	if (bSafetyBreak)
	{
		// Mirrors np.argmax(violations): the first True index.
		const int32 Idx = ViolatedIndices[0];
		const int32 Jnt = LimitedJntIds[Idx];
		const double QVal = Data->qpos[LimitedQposAdr[Idx]];
		const char* JointNameUtf8 = mj_id2name(Model, mjOBJ_JOINT, Jnt);
		const FString JointName = JointNameUtf8 != nullptr ? FString(UTF8_TO_TCHAR(JointNameUtf8)) : FString();
		UE_LOG(LogURLabMink, Error, TEXT("Joint %d (%s) violates configuration limits %g <= %g <= %g"), Jnt,
			*JointName, LimitedLower[Idx], QVal, LimitedUpper[Idx]);
		return false;
	}

	for (const int32 Idx : ViolatedIndices)
	{
		const int32 Jnt = LimitedJntIds[Idx];
		const double QVal = Data->qpos[LimitedQposAdr[Idx]];
		UE_LOG(LogURLabMink, Verbose, TEXT("Value %.2f at joint %d is outside of its limits: [%.2f, %.2f]"), QVal,
			Jnt, LimitedLower[Idx], LimitedUpper[Idx]);
	}
	return true;
}

int32 FMinkConfiguration::ResolveFrameId(const FString& FrameName, EMinkFrameType FrameType) const
{
	const TPair<FString, uint8> Key(FrameName, static_cast<uint8>(FrameType));
	if (const int32* Found = FrameIdCache.Find(Key))
	{
		return *Found;
	}

	const int32 FrameId = mj_name2id(Model, MinkFrameTypeToObj(FrameType), TCHAR_TO_UTF8(*FrameName));
	if (FrameId == -1)
	{
		UE_LOG(LogURLabMink, Error, TEXT("%s '%s' does not exist in the model."), FrameTypeName(FrameType),
			*FrameName);
		return -1;
	}

	FrameIdCache.Add(Key, FrameId);
	return FrameId;
}

bool FMinkConfiguration::GetFrameJacobian(const FString& FrameName, EMinkFrameType FrameType, FMinkMat& OutJac) const
{
	const int32 Id = ResolveFrameId(FrameName, FrameType);
	if (Id < 0)
	{
		return false;
	}

	FMinkRowMat Jac(6, Model->nv);
	switch (FrameType)
	{
		case EMinkFrameType::Body:
			mj_jacBody(Model, Data, Jac.data(), Jac.data() + 3 * Model->nv, Id);
			break;
		case EMinkFrameType::Geom:
			mj_jacGeom(Model, Data, Jac.data(), Jac.data() + 3 * Model->nv, Id);
			break;
		case EMinkFrameType::Site:
			mj_jacSite(Model, Data, Jac.data(), Jac.data() + 3 * Model->nv, Id);
			break;
	}

	// MuJoCo jacobians have a frame of reference centered at the local frame but aligned with
	// the world frame. To obtain a jacobian expressed in the local frame we left-multiply by
	// A[T_fw] = blockdiag(R_fw, R_fw) where R_fw = R_wf^T.
	const double* Xmat = FrameXmat(Data, FrameType, Id);
	Eigen::Map<const FMinkRowMat> RWf(Xmat, 3, 3);
	OutJac.resize(6, Model->nv);
	OutJac.topRows(3) = RWf.transpose() * Jac.topRows(3);
	OutJac.bottomRows(3) = RWf.transpose() * Jac.bottomRows(3);
	return true;
}

bool FMinkConfiguration::GetTransformFrameToWorld(
	const FString& FrameName, EMinkFrameType FrameType, FMinkSE3& Out) const
{
	const int32 Id = ResolveFrameId(FrameName, FrameType);
	if (Id < 0)
	{
		return false;
	}

	const double* Pos = FramePos(Data, FrameType, Id);
	const double* Xmat = FrameXmat(Data, FrameType, Id);
	const FMinkMat3 R = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(Xmat);
	Out = FMinkSE3::FromRotationAndTranslation(FMinkSO3::FromMatrix(R), Eigen::Map<const FMinkVec3>(Pos));
	return true;
}

bool FMinkConfiguration::GetTransform(const FString& SourceName, EMinkFrameType SourceType,
	const FString& DestName, EMinkFrameType DestType, FMinkSE3& Out) const
{
	FMinkSE3 Source;
	FMinkSE3 Dest;
	if (!GetTransformFrameToWorld(SourceName, SourceType, Source))
	{
		return false;
	}
	if (!GetTransformFrameToWorld(DestName, DestType, Dest))
	{
		return false;
	}
	Out = Dest.Inverse().Multiply(Source);
	return true;
}

FMinkVec FMinkConfiguration::Integrate(const FMinkVec& Velocity, double Dt) const
{
	FMinkVec Q = GetQ();
	mj_integratePos(Model, Q.data(), Velocity.data(), Dt);
	return Q;
}

void FMinkConfiguration::IntegrateInplace(const FMinkVec& Velocity, double Dt)
{
	mj_integratePos(Model, Data->qpos, Velocity.data(), Dt);
	Update();
}

FMinkMat FMinkConfiguration::GetInertiaMatrix() const
{
	// Run the composite rigid body inertia (CRB) algorithm to populate Data->M.
	mj_makeM(Model, Data);
	// Data->M is stored in a lower-triangular implicitly-symmetric CSR format and is converted
	// to a dense symmetric matrix via mju_sym2dense.
	FMinkMat M(Model->nv, Model->nv);
	mju_sym2dense(M.data(), Data->M, Model->nv, Model->M_rownnz, Model->M_rowadr, Model->M_colind);
	return M;
}

FMinkVec FMinkConfiguration::GetQ() const
{
	return Eigen::Map<const FMinkVec>(Data->qpos, Model->nq);
}
