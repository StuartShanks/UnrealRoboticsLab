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
#include "MinkQp.h"
#include "MinkTestUtils.h"

#if WITH_DEV_AUTOMATION_TESTS

static const double TOL_QP = 1e-7;

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMinkQpGoldenTest,
	"URLab.Mink.Qp.Golden",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMinkQpGoldenTest::RunTest(const FString& Parameters)
{
	TSharedPtr<FJsonObject> Root;
	if (!MinkLoadFixture(TEXT("qp"), Root))
	{
		AddError(TEXT("qp.json missing — run gen_golden.py"));
		return false;
	}

	const TArray<TSharedPtr<FJsonValue>>& Cases = Root->GetArrayField(TEXT("cases"));
	TestTrue(TEXT("cases non-empty"), Cases.Num() > 0);

	for (const TSharedPtr<FJsonValue>& CaseVal : Cases)
	{
		const TSharedPtr<FJsonObject> Case = CaseVal->AsObject();

		FMinkQpProblem Problem;
		Problem.H = MinkJsonMat(Case->GetArrayField(TEXT("H")));
		Problem.C = MinkJsonVec(Case->GetArrayField(TEXT("c")));
		if (Case->HasTypedField<EJson::Array>(TEXT("G")))
		{
			Problem.G = MinkJsonMat(Case->GetArrayField(TEXT("G")));
			// Fixture stores the inequality bound as "h_ineq", not "h" — a sibling "h" field
			// would collide with "H" (the Hessian) since FJsonObject keys on case-insensitive
			// FString equality (see gen_qp() in gen_golden.py for the same note).
			Problem.HIneq = MinkJsonVec(Case->GetArrayField(TEXT("h_ineq")));
		}
		if (Case->HasTypedField<EJson::Array>(TEXT("A")))
		{
			Problem.A = MinkJsonMat(Case->GetArrayField(TEXT("A")));
			Problem.B = MinkJsonVec(Case->GetArrayField(TEXT("b")));
		}

		FMinkVec X;
		if (TestTrue(TEXT("MinkSolveQp"), MinkSolveQp(Problem, X)))
		{
			MinkExpectNear(*this, TEXT("qp.x"), X, MinkJsonVec(Case->GetArrayField(TEXT("x"))), TOL_QP);
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMinkQpInfeasibleTest,
	"URLab.Mink.Qp.Infeasible",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMinkQpInfeasibleTest::RunTest(const FString& Parameters)
{
	// H = I2, c = 0, G = [[1,0],[-1,0]], h = [-1,-1]  =>  x0 <= -1 AND -x0 <= -1 (x0 >= 1): infeasible.
	FMinkQpProblem Problem;
	Problem.H = FMinkMat::Identity(2, 2);
	Problem.C = FMinkVec::Zero(2);
	FMinkMat G(2, 2);
	G << 1, 0, -1, 0;
	Problem.G = G;
	FMinkVec H(2);
	H << -1, -1;
	Problem.HIneq = H;

	FMinkVec X;
	TestFalse(TEXT("MinkSolveQp on infeasible problem returns false"), MinkSolveQp(Problem, X));
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
