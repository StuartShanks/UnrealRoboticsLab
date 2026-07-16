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

#include "MuJoCo/Navigation/MjNavPursuit.h"

namespace MjNavPursuit
{
float ShortestAngleRad(float FromRad, float ToRad)
{
	float D = FMath::Fmod(ToRad - FromRad, 2.f * PI);
	if (D > PI)
		D -= 2.f * PI;
	else if (D <= -PI)
		D += 2.f * PI;
	return D;
}

namespace
{
/** Closest point on the polyline to P (XY only). Outputs segment index and
 *  parametric t within that segment. */
FVector ClosestOnPath(const TArray<FVector>& Pts, const FVector& P,
	int32& OutSeg, float& OutT)
{
	OutSeg = 0;
	OutT = 0.f;
	if (Pts.Num() == 1)
		return Pts[0];
	float BestD2 = TNumericLimits<float>::Max();
	FVector Best = Pts[0];
	for (int32 i = 0; i + 1 < Pts.Num(); ++i)
	{
		FVector A = Pts[i], B = Pts[i + 1];
		A.Z = B.Z = 0.f;
		FVector Q(P.X, P.Y, 0.f);
		const FVector AB = B - A;
		const float L2 = AB.SizeSquared();
		const float T = (L2 > SMALL_NUMBER)
						  ? FMath::Clamp(FVector::DotProduct(Q - A, AB) / L2, 0.f, 1.f)
						  : 0.f;
		const FVector C = A + T * AB;
		const float D2 = FVector::DistSquared(Q, C);
		if (D2 < BestD2)
		{
			BestD2 = D2;
			Best = C;
			OutSeg = i;
			OutT = T;
		}
	}
	return Best;
}

/** Walk Distance cm forward along the polyline from (Seg, T). Clamps to the
 *  final point. */
FVector WalkAhead(const TArray<FVector>& Pts, int32 Seg, float T, float Distance)
{
	if (Pts.Num() == 1)
		return Pts[0];
	FVector Cur = FMath::Lerp(Pts[Seg], Pts[Seg + 1], T);
	Cur.Z = 0.f;
	float Remaining = Distance;
	int32 i = Seg;
	FVector SegEnd = FVector(Pts[i + 1].X, Pts[i + 1].Y, 0.f);
	while (true)
	{
		const float ToEnd = FVector::Dist(Cur, SegEnd);
		if (Remaining <= ToEnd || i + 2 >= Pts.Num())
		{
			const FVector Dir = (SegEnd - Cur).GetSafeNormal();
			return Cur + Dir * FMath::Min(Remaining, ToEnd);
		}
		Remaining -= ToEnd;
		++i;
		Cur = SegEnd;
		SegEnd = FVector(Pts[i + 1].X, Pts[i + 1].Y, 0.f);
	}
}
} // namespace

FPursuitResult ComputeTwist(const TArray<FVector>& PathPoints,
	const FPursuitState& State, const FPursuitParams& P)
{
	FPursuitResult R;
	R.DesiredYawRad = State.YawRad; // hold current heading unless steering below
	if (PathPoints.Num() == 0)
	{
		R.bArrived = true;
		return R;
	}
	const FVector Goal(PathPoints.Last().X, PathPoints.Last().Y, 0.f);
	const FVector Pos(State.Position.X, State.Position.Y, 0.f);
	const float DistToGoal = FVector::Dist(Pos, Goal);

	// 1. Arrival check.
	if (DistToGoal <= P.AcceptanceRadius)
	{
		R.bArrived = true;
		R.LookaheadPoint = Goal;
		return R;
	}

	// 2. Lookahead point.
	int32 Seg;
	float T;
	ClosestOnPath(PathPoints, Pos, Seg, T);
	R.LookaheadPoint = WalkAhead(PathPoints, Seg, T, P.LookaheadDist);

	// 3. Desired world velocity (UE cm-space direction, m/s magnitude).
	FVector Dir = R.LookaheadPoint - Pos;
	Dir.Z = 0.f;
	Dir = Dir.GetSafeNormal();
	if (Dir.IsNearlyZero())
		Dir = (Goal - Pos).GetSafeNormal();
	const float Speed = P.MaxSpeed * FMath::Min(1.f, DistToGoal / P.DecelRadius);

	// 4. Yaw command toward direction of travel (UE yaw, +CW from above). The
	//    desired heading is always reported (carrot consumers need it); the
	//    yaw-RATE command stays gated on MinSpeedForHeading so a crawling base
	//    doesn't spin in place chasing heading noise.
	R.DesiredYawRad = FMath::Atan2(Dir.Y, Dir.X);
	float UEYawRate = 0.f;
	if (Speed >= P.MinSpeedForHeading)
	{
		const float Err = ShortestAngleRad(State.YawRad, R.DesiredYawRad);
		UEYawRate = FMath::Clamp(P.YawGain * Err, -P.MaxYawRate, P.MaxYawRate);
	}

	// 5. World→robot frame, then UE→bus convention. THE conversion:
	//    UE robot axes at yaw ψ: fwd=(cosψ, sinψ), right=(−sinψ, cosψ).
	//    bus vx = fwd (m/s); bus vy = −right (UE right = bus −left);
	//    bus yaw_rate = −UE yaw rate (UE +CW vs bus +CCW).
	const float C = FMath::Cos(State.YawRad), S = FMath::Sin(State.YawRad);
	const float FwdCmps = (Dir.X * C + Dir.Y * S) * Speed; // unitless dir × m/s
	const float RightCmps = (-Dir.X * S + Dir.Y * C) * Speed;
	R.Vx = FwdCmps;
	R.Vy = -RightCmps;
	R.YawRate = -UEYawRate;
	return R;
}
} // namespace MjNavPursuit
