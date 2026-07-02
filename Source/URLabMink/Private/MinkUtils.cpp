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

#include "MinkUtils.h"

namespace
{
const double* UtilsFramePos(const mjData* D, EMinkFrameType Type, int32 Id)
{
	switch (Type)
	{
		case EMinkFrameType::Body:
			return D->xpos + 3 * Id;
		case EMinkFrameType::Geom:
			return D->geom_xpos + 3 * Id;
		default:
			return D->site_xpos + 3 * Id;
	}
}

const double* UtilsFrameXmat(const mjData* D, EMinkFrameType Type, int32 Id)
{
	switch (Type)
	{
		case EMinkFrameType::Body:
			return D->xmat + 9 * Id;
		case EMinkFrameType::Geom:
			return D->geom_xmat + 9 * Id;
		default:
			return D->site_xmat + 9 * Id;
	}
}
} // namespace

bool MinkMoveMocapToFrame(
	const mjModel* M, mjData* D, const FString& MocapName, const FString& FrameName, EMinkFrameType FrameType)
{
	const int32 BodyId = mj_name2id(M, mjOBJ_BODY, TCHAR_TO_UTF8(*MocapName));
	const int32 MocapId = (BodyId >= 0) ? M->body_mocapid[BodyId] : -1;
	if (MocapId < 0)
	{
		UE_LOG(LogURLabMink, Error, TEXT("Body '%s' is not a mocap body."), *MocapName);
		return false;
	}

	const int32 ObjId = mj_name2id(M, MinkFrameTypeToObj(FrameType), TCHAR_TO_UTF8(*FrameName));
	if (ObjId == -1)
	{
		UE_LOG(LogURLabMink, Error, TEXT("Frame '%s' does not exist in the model."), *FrameName);
		return false;
	}

	const double* Xpos = UtilsFramePos(D, FrameType, ObjId);
	const double* Xmat = UtilsFrameXmat(D, FrameType, ObjId);
	FMemory::Memcpy(D->mocap_pos + 3 * MocapId, Xpos, sizeof(double) * 3);
	mju_mat2Quat(D->mocap_quat + 4 * MocapId, Xmat);
	return true;
}

void MinkGetFreejointDims(const mjModel* M, TArray<int32>& OutQIds, TArray<int32>& OutVIds)
{
	OutQIds.Reset();
	OutVIds.Reset();
	for (int32 Jnt = 0; Jnt < M->njnt; ++Jnt)
	{
		if (M->jnt_type[Jnt] == mjJNT_FREE)
		{
			const int32 QAdr = M->jnt_qposadr[Jnt];
			const int32 VAdr = M->jnt_dofadr[Jnt];
			for (int32 K = 0; K < 7; ++K)
			{
				OutQIds.Add(QAdr + K);
			}
			for (int32 K = 0; K < 6; ++K)
			{
				OutVIds.Add(VAdr + K);
			}
		}
	}
}

bool MinkCustomConfigurationVector(const mjModel* M, const FString& KeyName,
	const TArray<TPair<FString, FMinkVec>>& JointValues, FMinkVec& OutQ)
{
	mjData* D = mj_makeData(M);
	if (!KeyName.IsEmpty())
	{
		const int32 KeyId = mj_name2id(M, mjOBJ_KEY, TCHAR_TO_UTF8(*KeyName));
		if (KeyId == -1)
		{
			UE_LOG(LogURLabMink, Error, TEXT("Keyframe %s does not exist in the model."), *KeyName);
			mj_deleteData(D);
			return false;
		}
		mj_resetDataKeyframe(M, D, KeyId);
	}
	else
	{
		mj_resetData(M, D);
	}

	FMinkVec Q = Eigen::Map<const FMinkVec>(D->qpos, M->nq);
	mj_deleteData(D);

	for (const TPair<FString, FMinkVec>& Entry : JointValues)
	{
		const int32 Jid = mj_name2id(M, mjOBJ_JOINT, TCHAR_TO_UTF8(*Entry.Key));
		if (Jid == -1)
		{
			UE_LOG(LogURLabMink, Error, TEXT("Joint '%s' does not exist in the model."), *Entry.Key);
			return false;
		}
		const int32 JntDim = MinkQposWidth(M->jnt_type[Jid]);
		const int32 Qid = M->jnt_qposadr[Jid];
		if (Entry.Value.size() != JntDim)
		{
			UE_LOG(LogURLabMink, Error,
				TEXT("Joint %s should have a qpos value of (%d,) but got (%lld,)"), *Entry.Key, JntDim,
				(long long)Entry.Value.size());
			return false;
		}
		Q.segment(Qid, JntDim) = Entry.Value;
	}

	OutQ = Q;
	return true;
}

TArray<int32> MinkGetBodyBodyIds(const mjModel* M, int32 BodyId)
{
	TArray<int32> Out;
	for (int32 i = 0; i < M->nbody; ++i)
	{
		if (M->body_parentid[i] == BodyId && BodyId != i)
		{
			Out.Add(i);
		}
	}
	return Out;
}

TArray<int32> MinkGetSubtreeBodyIds(const mjModel* M, int32 BodyId)
{
	TArray<int32> BodyIds;
	TArray<int32> Stack;
	Stack.Add(BodyId);
	while (Stack.Num() > 0)
	{
		// Python: body_id = stack.pop() pops the LAST element; TArray::Pop() does the same.
		const int32 Cur = Stack.Pop();
		BodyIds.Add(Cur);
		Stack.Append(MinkGetBodyBodyIds(M, Cur));
	}
	return BodyIds;
}

TArray<int32> MinkGetBodyGeomIds(const mjModel* M, int32 BodyId)
{
	TArray<int32> Out;
	const int32 GeomStart = M->body_geomadr[BodyId];
	const int32 GeomNum = M->body_geomnum[BodyId];
	for (int32 i = 0; i < GeomNum; ++i)
	{
		Out.Add(GeomStart + i);
	}
	return Out;
}

TArray<int32> MinkGetBodyJointIds(const mjModel* M, int32 BodyId)
{
	TArray<int32> Out;
	const int32 JntStart = M->body_jntadr[BodyId];
	if (JntStart < 0)
	{
		return Out;
	}
	const int32 JntNum = M->body_jntnum[BodyId];
	for (int32 i = 0; i < JntNum; ++i)
	{
		Out.Add(JntStart + i);
	}
	return Out;
}

TArray<int32> MinkGetSubtreeGeomIds(const mjModel* M, int32 BodyId)
{
	TArray<int32> Out;
	for (const int32 Bid : MinkGetSubtreeBodyIds(M, BodyId))
	{
		Out.Append(MinkGetBodyGeomIds(M, Bid));
	}
	return Out;
}

TArray<int32> MinkGetSubtreeJointIds(const mjModel* M, int32 BodyId)
{
	TArray<int32> Out;
	for (const int32 Bid : MinkGetSubtreeBodyIds(M, BodyId))
	{
		Out.Append(MinkGetBodyJointIds(M, Bid));
	}
	return Out;
}
