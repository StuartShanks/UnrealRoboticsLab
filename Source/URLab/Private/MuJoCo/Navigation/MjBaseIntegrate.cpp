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

#include "MuJoCo/Navigation/MjBaseIntegrate.h"

namespace MjBaseIntegrate
{
void TwistToWorld(double Vx, double Vy, double YawRate, double Theta, double OutVWorld[3])
{
	const double C = FMath::Cos(Theta), S = FMath::Sin(Theta);
	OutVWorld[0] = C * Vx - S * Vy;
	OutVWorld[1] = S * Vx + C * Vy;
	OutVWorld[2] = YawRate;
}

double IntegrateLeashed(double Target, double V, double Dt, double Measured,
	bool bAngular, const FLeashParams& P, bool bRangeLimited, double RangeLo, double RangeHi)
{
	Target += V * Dt;
	const double Leash = bAngular ? (double)P.MaxLeashAngular : (double)P.MaxLeashLinear;
	if (FMath::Abs(Target - Measured) > (double)P.ReseedThreshold)
	{
		Target = Measured;
	}
	Target = FMath::Clamp(Target, Measured - Leash, Measured + Leash);
	if (bRangeLimited)
	{
		Target = FMath::Clamp(Target, RangeLo, RangeHi);
	}
	return Target;
}
} // namespace MjBaseIntegrate
