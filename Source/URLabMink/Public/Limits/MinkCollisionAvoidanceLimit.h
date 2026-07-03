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
#include "Limits/MinkLimit.h"

class FMinkConfiguration;

/**
 * Port of mink limits/collision_avoidance_limit.py's GeomSequence — a heterogeneous list of
 * geoms specified either by name (resolved via mj_name2id) or by raw geom id (used directly,
 * unvalidated — mirrors the Python homogenize helper's isinstance(g, int) passthrough).
 */
struct URLABMINK_API FMinkGeomGroup
{
	TArray<FString> Names; // resolved via mj_name2id
	TArray<int32> Ids;     // used directly
};

/** Port of mink's CollisionPair — a pair of geom groups between which collision avoidance is
 * performed (every geom in A vs every geom in B, after dedup/filtering). */
using FMinkCollisionPair = TPair<FMinkGeomGroup, FMinkGeomGroup>;

/**
 * Port of mink limits/collision_avoidance_limit.py CollisionAvoidanceLimit — an inequality
 * constraint on the normal velocity between geom pairs, limiting how fast geoms are allowed to
 * approach each other.
 *
 * Pair construction (ctor): each collision pair's two geom groups are homogenized to ids,
 * deduped per side, cartesian-producted, filtered (not welded together, bodies not
 * parent/child, contype/conaffinity check passes), the surviving (min,max) id pairs deduped
 * again across all collision pairs, and finally SORTED ascending (see global constraints
 * deviation #5 — Python's list(set(...)) order is not reproducible, so both the C++ ctor and
 * the fixture generator sort after construction).
 *
 * Broadphase (deviation-free — an additive perf optimization mink itself documents as a strict
 * pre-filter): pairs are partitioned once in the ctor into sphere-sphere (both geoms have a
 * finite bounding sphere), plane-geom (one geom is a plane, the other bounded), and always-keep
 * groups, mirroring MuJoCo's own mj_filterSphere. BroadphaseSurvivors() cheaply culls pairs that
 * are provably farther apart than CollisionDetectionDistance before the expensive narrow-phase
 * mj_geomDistance query; the assembled constraint is identical to the unfiltered computation
 * (rows for culled pairs are left at their zero-initialized / +inf defaults, exactly like rows
 * skipped by the narrow phase's own `dist == distmax` early-out).
 */
class URLABMINK_API FMinkCollisionAvoidanceLimit : public FMinkLimit
{
public:
	FMinkCollisionAvoidanceLimit(const mjModel* Model, const TArray<FMinkCollisionPair>& GeomPairs,
		double Gain = 0.85, double MinimumDistanceFromCollisions = 0.005,
		double CollisionDetectionDistance = 0.01, double BoundRelaxation = 0.0, bool bBroadphase = true);

	bool bIsValid = true;

	/** Port of CollisionAvoidanceLimit.compute_qp_inequalities. Always returns a full
	 * (MaxNumContacts x nv) G / (MaxNumContacts) h pair (never "inactive"), mirroring Python's
	 * Constraint always being fully shaped even when every row is skipped. */
	bool ComputeQpInequalities(const FMinkConfiguration&, double Dt, FMinkInequality& Out) const override;

	int32 MaxNumContacts() const { return GeomIdPairs.Num(); }
	int32 BroadphaseMinPairs = 16; // mirrors _BROADPHASE_MIN_PAIRS; 0 forces broadphase
	bool bBroadphase = true;

	/** Deduped, filtered, sorted-ascending (min,max) geom id pairs (see deviation #5). Exposed
	 * (read-only) so golden-fixture tests can assert exact pair-set equality. */
	const TArray<TPair<int32, int32>>& GetGeomIdPairs() const { return GeomIdPairs; }

private:
	/** Port of _broadphase_survivors. Returns the indices into GeomIdPairs that survive the
	 * cheap pre-filter (order is irrelevant — each survivor writes an independent row). */
	TArray<int32> BroadphaseSurvivors(const mjData* Data) const;

	const mjModel* ModelRef = nullptr;
	double Gain = 0.85;
	double MinDist = 0.005;
	double DetectionDist = 0.01;
	double BoundRelaxation = 0.0;
	TArray<TPair<int32, int32>> GeomIdPairs; // deduped, filtered, SORTED ascending (see deviations)

	// Precomputed broadphase partitions (sphere-sphere / plane-geom / always-keep), indices into
	// GeomIdPairs. Mirrors _init_broadphase.
	TArray<int32> SsIdx, PgIdx, KeepIdx;
	TArray<int32> SsG1, SsG2, PgPlane, PgOther;
	TArray<double> SsRsum, PgROther;
};
