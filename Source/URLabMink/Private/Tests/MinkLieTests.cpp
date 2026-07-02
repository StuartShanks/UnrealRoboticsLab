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

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Lie/MinkSO3.h"
#include "MinkTestUtils.h"

#if WITH_DEV_AUTOMATION_TESTS

static const double TOL_LIE = 1e-10;

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMinkLieSO3Test,
	"URLab.Mink.Lie.SO3",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMinkLieSO3Test::RunTest(const FString& Parameters)
{
	TSharedPtr<FJsonObject> Root;
	if (!MinkLoadFixture(TEXT("lie"), Root))
	{
		AddError(TEXT("lie.json missing — run gen_golden.py"));
		return false;
	}
	const TSharedPtr<FJsonObject> So3 = Root->GetObjectField(TEXT("so3"));

	for (const auto& V : So3->GetArrayField(TEXT("exp")))
	{
		const auto C = V->AsObject();
		const FMinkVec T = MinkJsonVec(C->GetArrayField(TEXT("tangent")));
		const FMinkSO3 G = FMinkSO3::Exp(T.head<3>());
		MinkExpectNear(*this, TEXT("so3.exp"),
			Eigen::Map<const Eigen::Vector4d>(G.Wxyz), MinkJsonVec(C->GetArrayField(TEXT("wxyz"))), TOL_LIE);
	}
	for (const auto& V : So3->GetArrayField(TEXT("log")))
	{
		const auto C = V->AsObject();
		const FMinkVec Q = MinkJsonVec(C->GetArrayField(TEXT("wxyz")));
		MinkExpectNear(*this, TEXT("so3.log"),
			FMinkSO3::FromWxyz(Q.data()).Log(), MinkJsonVec(C->GetArrayField(TEXT("tangent"))), TOL_LIE);
	}
	for (const auto& V : So3->GetArrayField(TEXT("matrix")))
	{
		const auto C = V->AsObject();
		const FMinkVec Q = MinkJsonVec(C->GetArrayField(TEXT("wxyz")));
		const FMinkSO3 G = FMinkSO3::FromWxyz(Q.data());
		MinkExpectNear(*this, TEXT("so3.matrix.as_matrix"),
			G.AsMatrix(), MinkJsonMat(C->GetArrayField(TEXT("matrix"))), TOL_LIE);
		const FMinkSO3 Back = FMinkSO3::FromMatrix(MinkJsonMat(C->GetArrayField(TEXT("matrix"))));
		MinkExpectNear(*this, TEXT("so3.matrix.from_matrix"),
			Eigen::Map<const Eigen::Vector4d>(Back.Wxyz), MinkJsonVec(C->GetArrayField(TEXT("wxyz_back"))), TOL_LIE);
	}
	for (const auto& V : So3->GetArrayField(TEXT("multiply")))
	{
		const auto C = V->AsObject();
		const FMinkVec A = MinkJsonVec(C->GetArrayField(TEXT("a")));
		const FMinkVec B = MinkJsonVec(C->GetArrayField(TEXT("b")));
		const FMinkSO3 Result = FMinkSO3::FromWxyz(A.data()).Multiply(FMinkSO3::FromWxyz(B.data()));
		MinkExpectNear(*this, TEXT("so3.multiply"),
			Eigen::Map<const Eigen::Vector4d>(Result.Wxyz), MinkJsonVec(C->GetArrayField(TEXT("out"))), TOL_LIE);
	}
	for (const auto& V : So3->GetArrayField(TEXT("apply")))
	{
		const auto C = V->AsObject();
		const FMinkVec Q = MinkJsonVec(C->GetArrayField(TEXT("wxyz")));
		const FMinkVec Vec = MinkJsonVec(C->GetArrayField(TEXT("v")));
		MinkExpectNear(*this, TEXT("so3.apply"),
			FMinkSO3::FromWxyz(Q.data()).Apply(Vec.head<3>()), MinkJsonVec(C->GetArrayField(TEXT("out"))), TOL_LIE);
	}
	for (const auto& V : So3->GetArrayField(TEXT("inverse")))
	{
		const auto C = V->AsObject();
		const FMinkVec Q = MinkJsonVec(C->GetArrayField(TEXT("wxyz")));
		const FMinkSO3 Result = FMinkSO3::FromWxyz(Q.data()).Inverse();
		MinkExpectNear(*this, TEXT("so3.inverse"),
			Eigen::Map<const Eigen::Vector4d>(Result.Wxyz), MinkJsonVec(C->GetArrayField(TEXT("out"))), TOL_LIE);
	}
	for (const auto& V : So3->GetArrayField(TEXT("ljac")))
	{
		const auto C = V->AsObject();
		const FMinkVec T = MinkJsonVec(C->GetArrayField(TEXT("tangent")));
		MinkExpectNear(*this, TEXT("so3.ljac"),
			FMinkSO3::Ljac(T.head<3>()), MinkJsonMat(C->GetArrayField(TEXT("m"))), TOL_LIE);
	}
	for (const auto& V : So3->GetArrayField(TEXT("ljacinv")))
	{
		const auto C = V->AsObject();
		const FMinkVec T = MinkJsonVec(C->GetArrayField(TEXT("tangent")));
		MinkExpectNear(*this, TEXT("so3.ljacinv"),
			FMinkSO3::Ljacinv(T.head<3>()), MinkJsonMat(C->GetArrayField(TEXT("m"))), TOL_LIE);
	}
	for (const auto& V : So3->GetArrayField(TEXT("rpy")))
	{
		const auto C = V->AsObject();
		const FMinkVec Q = MinkJsonVec(C->GetArrayField(TEXT("wxyz")));
		MinkExpectNear(*this, TEXT("so3.rpy"),
			FMinkSO3::FromWxyz(Q.data()).AsRpyRadians(), MinkJsonVec(C->GetArrayField(TEXT("rpy"))), TOL_LIE);
	}
	for (const auto& V : So3->GetArrayField(TEXT("rminus")))
	{
		const auto C = V->AsObject();
		const FMinkVec A = MinkJsonVec(C->GetArrayField(TEXT("a")));
		const FMinkVec B = MinkJsonVec(C->GetArrayField(TEXT("b")));
		const FMinkVec3 Result = FMinkSO3::FromWxyz(A.data()).RMinus(FMinkSO3::FromWxyz(B.data()));
		MinkExpectNear(*this, TEXT("so3.rminus"),
			Result, MinkJsonVec(C->GetArrayField(TEXT("out"))), TOL_LIE);
	}
	for (const auto& V : So3->GetArrayField(TEXT("interpolate")))
	{
		const auto C = V->AsObject();
		const FMinkVec A = MinkJsonVec(C->GetArrayField(TEXT("a")));
		const FMinkVec B = MinkJsonVec(C->GetArrayField(TEXT("b")));
		const double Alpha = C->GetNumberField(TEXT("alpha"));
		const FMinkSO3 Result = FMinkSO3::FromWxyz(A.data()).Interpolate(FMinkSO3::FromWxyz(B.data()), Alpha);
		MinkExpectNear(*this, TEXT("so3.interpolate"),
			Eigen::Map<const Eigen::Vector4d>(Result.Wxyz), MinkJsonVec(C->GetArrayField(TEXT("out"))), TOL_LIE);
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
