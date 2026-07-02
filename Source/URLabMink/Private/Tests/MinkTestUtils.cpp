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

#include "MinkTestUtils.h"

#include "HAL/PlatformFileManager.h"
#include "Interfaces/IPluginManager.h"
#include "Misc/FileHelper.h"
#include "Serialization/JsonSerializer.h"

FString MinkGoldenDir()
{
	TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("UnrealRoboticsLab"));
	if (!Plugin.IsValid())
	{
		UE_LOG(LogURLabMink, Error, TEXT("MinkGoldenDir: could not find plugin 'UnrealRoboticsLab'"));
		return FString();
	}
	return Plugin->GetBaseDir() / TEXT("Scripts/mink_golden");
}

bool MinkLoadFixture(const FString& LayerName, TSharedPtr<FJsonObject>& Out)
{
	const FString Path = MinkGoldenDir() / TEXT("fixtures") / (LayerName + TEXT(".json"));
	FString Contents;
	if (!FFileHelper::LoadFileToString(Contents, *Path))
	{
		UE_LOG(LogURLabMink, Error, TEXT("MinkLoadFixture: failed to read '%s'"), *Path);
		return false;
	}
	const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Contents);
	if (!FJsonSerializer::Deserialize(Reader, Out) || !Out.IsValid())
	{
		UE_LOG(LogURLabMink, Error, TEXT("MinkLoadFixture: failed to parse '%s'"), *Path);
		return false;
	}
	return true;
}

mjModel* MinkLoadModel(const FString& ModelName)
{
	const FString Path = MinkGoldenDir() / TEXT("models") / ModelName;
	char Err[1024] = {};
	mjModel* M = mj_loadXML(TCHAR_TO_UTF8(*Path), nullptr, Err, sizeof(Err));
	if (M == nullptr)
	{
		UE_LOG(LogURLabMink, Error, TEXT("MinkLoadModel: mj_loadXML('%s') failed: %s"), *Path, UTF8_TO_TCHAR(Err));
	}
	return M;
}

FMinkVec MinkJsonVec(const TArray<TSharedPtr<FJsonValue>>& A)
{
	FMinkVec V(A.Num());
	for (int32 i = 0; i < A.Num(); ++i)
	{
		V(i) = A[i]->AsNumber();
	}
	return V;
}

FMinkMat MinkJsonMat(const TArray<TSharedPtr<FJsonValue>>& A)
{
	const int32 Rows = A.Num();
	const int32 Cols = Rows > 0 ? A[0]->AsArray().Num() : 0;
	FMinkMat M(Rows, Cols);
	for (int32 r = 0; r < Rows; ++r)
	{
		const TArray<TSharedPtr<FJsonValue>> Row = A[r]->AsArray();
		for (int32 c = 0; c < Cols; ++c)
		{
			M(r, c) = Row[c]->AsNumber();
		}
	}
	return M;
}

bool MinkExpectNear(
	FAutomationTestBase& Test, const TCHAR* What, const FMinkMat& Actual, const FMinkMat& Expected, double Tol)
{
	if (Actual.rows() != Expected.rows() || Actual.cols() != Expected.cols())
	{
		Test.AddError(FString::Printf(TEXT("%s: shape mismatch actual(%lld,%lld) vs expected(%lld,%lld)"), What,
			(long long)Actual.rows(), (long long)Actual.cols(), (long long)Expected.rows(),
			(long long)Expected.cols()));
		return false;
	}

	double MaxAbsExpected = 1.0;
	for (int32 r = 0; r < Expected.rows(); ++r)
	{
		for (int32 c = 0; c < Expected.cols(); ++c)
		{
			MaxAbsExpected = FMath::Max(MaxAbsExpected, FMath::Abs(Expected(r, c)));
		}
	}
	const double Threshold = Tol * MaxAbsExpected;

	double MaxDiff = 0.0;
	int32 FirstBadIndex = -1;
	int32 FlatIndex = 0;
	for (int32 r = 0; r < Actual.rows(); ++r)
	{
		for (int32 c = 0; c < Actual.cols(); ++c, ++FlatIndex)
		{
			const double Diff = FMath::Abs(Actual(r, c) - Expected(r, c));
			if (Diff > MaxDiff)
			{
				MaxDiff = Diff;
			}
			if (Diff > Threshold && FirstBadIndex < 0)
			{
				FirstBadIndex = FlatIndex;
			}
		}
	}

	if (FirstBadIndex >= 0)
	{
		Test.AddError(FString::Printf(TEXT("%s: max diff %.17g exceeds tol %.17g (first offending flat index %d)"),
			What, MaxDiff, Threshold, FirstBadIndex));
		return false;
	}
	return true;
}
