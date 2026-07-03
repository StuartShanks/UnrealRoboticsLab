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

#include "Limits/MinkCollisionAvoidanceLimit.h"

#include <limits>

#include "MinkConfiguration.h"

namespace
{
/** Port of the Python homogenize_geom_id_list helper: resolves Names via mj_name2id and passes
 * raw Ids through unvalidated (mirrors isinstance(g, int) passthrough). */
bool HomogenizeGeomGroup(const mjModel* Model, const FMinkGeomGroup& Group, TArray<int32>& OutIds)
{
	for (const FString& Name : Group.Names)
	{
		const int32 Id = mj_name2id(Model, mjOBJ_GEOM, TCHAR_TO_UTF8(*Name));
		if (Id == -1)
		{
			UE_LOG(LogURLabMink, Error, TEXT("Invalid geom name '%s'."), *Name);
			return false;
		}
		OutIds.Add(Id);
	}
	OutIds.Append(Group.Ids);
	return true;
}

/** Port of _is_welded_together. */
bool IsWeldedTogether(const mjModel* Model, int32 GeomId1, int32 GeomId2)
{
	const int32 Body1 = Model->geom_bodyid[GeomId1];
	const int32 Body2 = Model->geom_bodyid[GeomId2];
	return Model->body_weldid[Body1] == Model->body_weldid[Body2];
}

/** Port of _are_geom_bodies_parent_child (weld-parent-weld logic verbatim). */
bool AreGeomBodiesParentChild(const mjModel* Model, int32 GeomId1, int32 GeomId2)
{
	const int32 BodyId1 = Model->geom_bodyid[GeomId1];
	const int32 BodyId2 = Model->geom_bodyid[GeomId2];

	// body_weldid is the id of the body's weld.
	const int32 BodyWeldId1 = Model->body_weldid[BodyId1];
	const int32 BodyWeldId2 = Model->body_weldid[BodyId2];

	// weld_parent_id is the id of the parent of the body's weld.
	const int32 WeldParentId1 = Model->body_parentid[BodyWeldId1];
	const int32 WeldParentId2 = Model->body_parentid[BodyWeldId2];

	// weld_parent_weldid is the weld id of the parent of the body's weld.
	const int32 WeldParentWeldId1 = Model->body_weldid[WeldParentId1];
	const int32 WeldParentWeldId2 = Model->body_weldid[WeldParentId2];

	const bool Cond1 = BodyWeldId1 == WeldParentWeldId2;
	const bool Cond2 = BodyWeldId2 == WeldParentWeldId1;
	return Cond1 || Cond2;
}

/** Port of _is_pass_contype_conaffinity_check. */
bool PassesContypeConaffinityCheck(const mjModel* Model, int32 GeomId1, int32 GeomId2)
{
	const bool Cond1 = (Model->geom_contype[GeomId1] & Model->geom_conaffinity[GeomId2]) != 0;
	const bool Cond2 = (Model->geom_contype[GeomId2] & Model->geom_conaffinity[GeomId1]) != 0;
	return Cond1 || Cond2;
}

/** Packs a (min,max) geom id pair into a single key for TSet dedup (both ids are non-negative
 * mjModel indices, well within 32 bits). */
uint64 PackPair(int32 A, int32 B)
{
	return (static_cast<uint64>(static_cast<uint32>(A)) << 32) | static_cast<uint32>(B);
}
} // namespace

FMinkCollisionAvoidanceLimit::FMinkCollisionAvoidanceLimit(const mjModel* Model,
	const TArray<FMinkCollisionPair>& GeomPairs, double InGain, double InMinimumDistanceFromCollisions,
	double InCollisionDetectionDistance, double InBoundRelaxation, bool InBroadphase)
	: bBroadphase(InBroadphase)
	, ModelRef(Model)
	, Gain(InGain)
	, MinDist(InMinimumDistanceFromCollisions)
	, DetectionDist(InCollisionDetectionDistance)
	, BoundRelaxation(InBoundRelaxation)
{
	TSet<uint64> Seen;
	TArray<TPair<int32, int32>> Collected;

	for (const FMinkCollisionPair& Pair : GeomPairs)
	{
		TArray<int32> IdsA, IdsB;
		if (!HomogenizeGeomGroup(Model, Pair.Key, IdsA) || !HomogenizeGeomGroup(Model, Pair.Value, IdsB))
		{
			bIsValid = false;
			return;
		}

		// Dedup each side (order is irrelevant: the cartesian product below is itself
		// deduped+sorted, mirroring Python's list(set(id_pair_A))).
		TSet<int32> DedupA(IdsA);
		TSet<int32> DedupB(IdsB);

		for (const int32 GeomA : DedupA)
		{
			for (const int32 GeomB : DedupB)
			{
				const bool bWeldOk = !IsWeldedTogether(Model, GeomA, GeomB);
				const bool bParentChildOk = !AreGeomBodiesParentChild(Model, GeomA, GeomB);
				const bool bContypeOk = PassesContypeConaffinityCheck(Model, GeomA, GeomB);
				if (bWeldOk && bParentChildOk && bContypeOk)
				{
					const int32 Lo = FMath::Min(GeomA, GeomB);
					const int32 Hi = FMath::Max(GeomA, GeomB);
					const uint64 Key = PackPair(Lo, Hi);
					bool bAlreadyInSet = false;
					Seen.Add(Key, &bAlreadyInSet);
					if (!bAlreadyInSet)
					{
						Collected.Emplace(Lo, Hi);
					}
				}
			}
		}
	}

	// Sort ascending (deviation #5): Python's list(set(...)) order is not reproducible, so both
	// this ctor and the fixture generator sort geom_id_pairs after construction.
	Collected.Sort(
		[](const TPair<int32, int32>& A, const TPair<int32, int32>& B) {
			if (A.Key != B.Key)
			{
				return A.Key < B.Key;
			}
			return A.Value < B.Value;
		});
	GeomIdPairs = MoveTemp(Collected);

	// Precompute broadphase partitions (mirrors _init_broadphase).
	for (int32 Index = 0; Index < GeomIdPairs.Num(); ++Index)
	{
		const int32 G1 = GeomIdPairs[Index].Key;
		const int32 G2 = GeomIdPairs[Index].Value;
		const double R1 = Model->geom_rbound[G1];
		const double R2 = Model->geom_rbound[G2];
		const bool bIsPlane1 = Model->geom_type[G1] == mjGEOM_PLANE;
		const bool bIsPlane2 = Model->geom_type[G2] == mjGEOM_PLANE;

		const bool bBothBounded = (R1 > 0.0) && (R2 > 0.0);
		const bool bPlane1 = bIsPlane1 && (R2 > 0.0); // G1 is the plane.
		const bool bPlane2 = bIsPlane2 && (R1 > 0.0); // G2 is the plane.
		const bool bPlanePair = (bPlane1 || bPlane2) && !bBothBounded;

		if (bBothBounded)
		{
			SsIdx.Add(Index);
			SsG1.Add(G1);
			SsG2.Add(G2);
			SsRsum.Add(R1 + R2);
		}
		else if (bPlanePair)
		{
			PgIdx.Add(Index);
			if (bPlane1)
			{
				PgPlane.Add(G1);
				PgOther.Add(G2);
				PgROther.Add(R2);
			}
			else
			{
				PgPlane.Add(G2);
				PgOther.Add(G1);
				PgROther.Add(R1);
			}
		}
		else
		{
			KeepIdx.Add(Index);
		}
	}
}

TArray<int32> FMinkCollisionAvoidanceLimit::BroadphaseSurvivors(const mjData* Data) const
{
	TArray<int32> Survivors = KeepIdx;
	const double Margin = DetectionDist;

	for (int32 K = 0; K < SsIdx.Num(); ++K)
	{
		const double* P1 = &Data->geom_xpos[3 * SsG1[K]];
		const double* P2 = &Data->geom_xpos[3 * SsG2[K]];
		const double Dx = P1[0] - P2[0];
		const double Dy = P1[1] - P2[1];
		const double Dz = P1[2] - P2[2];
		const double DistSq = Dx * Dx + Dy * Dy + Dz * Dz;
		const double Bound = SsRsum[K] + Margin;
		if (DistSq <= Bound * Bound)
		{
			Survivors.Add(SsIdx[K]);
		}
	}

	for (int32 K = 0; K < PgIdx.Num(); ++K)
	{
		// Plane normal is the z-axis of the plane frame: columns 2, 5, 8 of xmat.
		const double* PlaneXmat = &Data->geom_xmat[9 * PgPlane[K]];
		const double NormalX = PlaneXmat[2];
		const double NormalY = PlaneXmat[5];
		const double NormalZ = PlaneXmat[8];

		const double* PlanePos = &Data->geom_xpos[3 * PgPlane[K]];
		const double* OtherPos = &Data->geom_xpos[3 * PgOther[K]];
		const double Dx = OtherPos[0] - PlanePos[0];
		const double Dy = OtherPos[1] - PlanePos[1];
		const double Dz = OtherPos[2] - PlanePos[2];
		const double SignedDist = NormalX * Dx + NormalY * Dy + NormalZ * Dz;

		if (SignedDist <= Margin + PgROther[K])
		{
			Survivors.Add(PgIdx[K]);
		}
	}

	// Row order is irrelevant (each survivor writes an independent row), so no sort is needed.
	return Survivors;
}

bool FMinkCollisionAvoidanceLimit::ComputeQpInequalities(
	const FMinkConfiguration& Configuration, double Dt, FMinkInequality& Out) const
{
	Out = FMinkInequality();

	const int32 MaxContacts = GeomIdPairs.Num();
	const int32 Nv = ModelRef->nv;

	FMinkMat G = FMinkMat::Zero(MaxContacts, Nv);
	FMinkVec H = FMinkVec::Constant(MaxContacts, std::numeric_limits<double>::infinity());

	mjData* Data = Configuration.Data;

	TArray<int32> Indices;
	if (bBroadphase && MaxContacts >= BroadphaseMinPairs)
	{
		Indices = BroadphaseSurvivors(Data);
	}
	else
	{
		Indices.Reserve(MaxContacts);
		for (int32 Index = 0; Index < MaxContacts; ++Index)
		{
			Indices.Add(Index);
		}
	}

	for (const int32 Idx : Indices)
	{
		const int32 Geom1Id = GeomIdPairs[Idx].Key;
		const int32 Geom2Id = GeomIdPairs[Idx].Value;

		double Fromto[6];
		const double Dist = mj_geomDistance(ModelRef, Data, Geom1Id, Geom2Id, DetectionDist, Fromto);
		if (FMath::Abs(Dist - DetectionDist) < 1e-12)
		{
			continue;
		}

		FMinkVec3 Normal(Fromto[3] - Fromto[0], Fromto[4] - Fromto[1], Fromto[5] - Fromto[2]);
		Normal.normalize();

		FMinkRowMat Jac1 = FMinkRowMat::Zero(3, Nv);
		FMinkRowMat Jac2 = FMinkRowMat::Zero(3, Nv);
		const double Point2[3] = {Fromto[3], Fromto[4], Fromto[5]};
		const double Point1[3] = {Fromto[0], Fromto[1], Fromto[2]};
		mj_jac(ModelRef, Data, Jac2.data(), nullptr, Point2, ModelRef->geom_bodyid[Geom2Id]);
		mj_jac(ModelRef, Data, Jac1.data(), nullptr, Point1, ModelRef->geom_bodyid[Geom1Id]);

		const FMinkMat RowJac = Jac2 - Jac1;
		const FMinkMat Row = Normal.transpose() * RowJac; // 1 x Nv row.

		double Bound;
		if (Dist > MinDist)
		{
			Bound = (Gain * (Dist - MinDist) / Dt) + BoundRelaxation;
		}
		else
		{
			Bound = BoundRelaxation;
		}
		const double Sign = (Dist >= 0.0) ? -1.0 : 1.0;

		H(Idx) = Bound;
		G.row(Idx) = Sign * Row;
	}

	Out.G = MoveTemp(G);
	Out.H = MoveTemp(H);
	return true;
}
