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
#include "MinkConfiguration.h"
#include "MinkUtils.h"
#include "MinkTestUtils.h"

#if WITH_DEV_AUTOMATION_TESTS

static const double TOL_KIN = 1e-9;

namespace
{
TArray<int32> JsonIntArray(const TArray<TSharedPtr<FJsonValue>>& A)
{
	TArray<int32> Out;
	Out.Reserve(A.Num());
	for (const TSharedPtr<FJsonValue>& V : A)
	{
		Out.Add(static_cast<int32>(V->AsNumber()));
	}
	return Out;
}

bool ExpectIntArrayEqual(
	FAutomationTestBase& Test, const TCHAR* What, const TArray<int32>& Actual, const TArray<int32>& Expected)
{
	if (Actual.Num() != Expected.Num())
	{
		Test.AddError(FString::Printf(
			TEXT("%s: length mismatch actual=%d expected=%d"), What, Actual.Num(), Expected.Num()));
		return false;
	}
	for (int32 i = 0; i < Actual.Num(); ++i)
	{
		if (Actual[i] != Expected[i])
		{
			Test.AddError(FString::Printf(
				TEXT("%s: mismatch at index %d actual=%d expected=%d"), What, i, Actual[i], Expected[i]));
			return false;
		}
	}
	return true;
}
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMinkConfigurationKinematicsTest,
	"URLab.Mink.Configuration.Kinematics",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMinkConfigurationKinematicsTest::RunTest(const FString& Parameters)
{
	TSharedPtr<FJsonObject> Root;
	if (!MinkLoadFixture(TEXT("configuration"), Root))
	{
		AddError(TEXT("configuration.json missing — run gen_golden.py"));
		return false;
	}
	const TSharedPtr<FJsonObject> Models = Root->GetObjectField(TEXT("models"));

	static const TArray<FString> ModelNames = {TEXT("arm3"), TEXT("floating")};
	for (const FString& ModelName : ModelNames)
	{
		const TSharedPtr<FJsonObject> Entry = Models->GetObjectField(ModelName);
		const TArray<TSharedPtr<FJsonValue>>& Cases = Entry->GetArrayField(TEXT("cases"));
		TestTrue(FString::Printf(TEXT("%s cases non-empty"), *ModelName), Cases.Num() > 0);
		if (Cases.Num() == 0)
		{
			continue;
		}

		mjModel* Model = MinkLoadModel(ModelName + TEXT(".xml"));
		if (Model == nullptr)
		{
			AddError(FString::Printf(TEXT("failed to load model '%s'"), *ModelName));
			continue;
		}

		{
			FMinkConfiguration Cfg(Model);

			for (const TSharedPtr<FJsonValue>& CaseVal : Cases)
			{
				const TSharedPtr<FJsonObject> Case = CaseVal->AsObject();
				const FMinkVec Q = MinkJsonVec(Case->GetArrayField(TEXT("q")));
				Cfg.Update(Q.data());

				const TArray<TSharedPtr<FJsonValue>>& Frames = Case->GetArrayField(TEXT("frames"));
				TestTrue(TEXT("frames non-empty"), Frames.Num() > 0);
				for (const TSharedPtr<FJsonValue>& FrameVal : Frames)
				{
					const TSharedPtr<FJsonObject> Frame = FrameVal->AsObject();
					const FString Name = Frame->GetStringField(TEXT("name"));
					EMinkFrameType Type;
					MinkFrameTypeFromString(Frame->GetStringField(TEXT("type")), Type);

					FMinkMat Jac;
					if (TestTrue(FString::Printf(TEXT("GetFrameJacobian(%s)"), *Name),
							Cfg.GetFrameJacobian(Name, Type, Jac)))
					{
						MinkExpectNear(*this, TEXT("frame.jac"), Jac, MinkJsonMat(Frame->GetArrayField(TEXT("jac"))),
							TOL_KIN);
					}

					FMinkSE3 Pose;
					if (TestTrue(FString::Printf(TEXT("GetTransformFrameToWorld(%s)"), *Name),
							Cfg.GetTransformFrameToWorld(Name, Type, Pose)))
					{
						MinkExpectNear(*this, TEXT("frame.pose"),
							Eigen::Map<const Eigen::Matrix<double, 7, 1>>(Pose.WxyzXyz),
							MinkJsonVec(Frame->GetArrayField(TEXT("pose"))), TOL_KIN);
					}
				}

				const TSharedPtr<FJsonObject> Transform = Case->GetObjectField(TEXT("transform"));
				const FString Src = Transform->GetStringField(TEXT("src"));
				EMinkFrameType SrcType;
				MinkFrameTypeFromString(Transform->GetStringField(TEXT("src_type")), SrcType);
				const FString Dst = Transform->GetStringField(TEXT("dst"));
				EMinkFrameType DstType;
				MinkFrameTypeFromString(Transform->GetStringField(TEXT("dst_type")), DstType);

				FMinkSE3 TransformPose;
				if (TestTrue(TEXT("GetTransform"), Cfg.GetTransform(Src, SrcType, Dst, DstType, TransformPose)))
				{
					MinkExpectNear(*this, TEXT("transform.pose"),
						Eigen::Map<const Eigen::Matrix<double, 7, 1>>(TransformPose.WxyzXyz),
						MinkJsonVec(Transform->GetArrayField(TEXT("pose"))), TOL_KIN);
				}

				const TSharedPtr<FJsonObject> Integrate = Case->GetObjectField(TEXT("integrate"));
				const FMinkVec V = MinkJsonVec(Integrate->GetArrayField(TEXT("v")));
				const double Dt = Integrate->GetNumberField(TEXT("dt"));
				const FMinkVec QOut = Cfg.Integrate(V, Dt);
				MinkExpectNear(
					*this, TEXT("integrate.q_out"), QOut, MinkJsonVec(Integrate->GetArrayField(TEXT("q_out"))),
					TOL_KIN);

				const FMinkMat Inertia = Cfg.GetInertiaMatrix();
				MinkExpectNear(
					*this, TEXT("inertia"), Inertia, MinkJsonMat(Case->GetArrayField(TEXT("inertia"))), TOL_KIN);
			}

			if (Entry->HasField(TEXT("keyframe")))
			{
				const TSharedPtr<FJsonObject> Keyframe = Entry->GetObjectField(TEXT("keyframe"));
				const FString KeyName = Keyframe->GetStringField(TEXT("name"));
				if (TestTrue(TEXT("UpdateFromKeyframe"), Cfg.UpdateFromKeyframe(KeyName)))
				{
					MinkExpectNear(
						*this, TEXT("keyframe.q"), Cfg.GetQ(), MinkJsonVec(Keyframe->GetArrayField(TEXT("q"))),
						TOL_KIN);
				}
			}
		}

		mj_deleteModel(Model);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMinkUtilsHelpersTest,
	"URLab.Mink.Utils.Helpers",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMinkUtilsHelpersTest::RunTest(const FString& Parameters)
{
	TSharedPtr<FJsonObject> Root;
	if (!MinkLoadFixture(TEXT("utils"), Root))
	{
		AddError(TEXT("utils.json missing — run gen_golden.py"));
		return false;
	}
	const TSharedPtr<FJsonObject> Models = Root->GetObjectField(TEXT("models"));

	static const TArray<FString> ModelNames = {TEXT("arm3"), TEXT("floating")};
	for (const FString& ModelName : ModelNames)
	{
		const TSharedPtr<FJsonObject> Entry = Models->GetObjectField(ModelName);

		mjModel* Model = MinkLoadModel(ModelName + TEXT(".xml"));
		if (Model == nullptr)
		{
			AddError(FString::Printf(TEXT("failed to load model '%s'"), *ModelName));
			continue;
		}

		TArray<int32> QIds;
		TArray<int32> VIds;
		MinkGetFreejointDims(Model, QIds, VIds);
		ExpectIntArrayEqual(*this, TEXT("freejoint_q_ids"), QIds,
			JsonIntArray(Entry->GetArrayField(TEXT("freejoint_q_ids"))));
		ExpectIntArrayEqual(*this, TEXT("freejoint_v_ids"), VIds,
			JsonIntArray(Entry->GetArrayField(TEXT("freejoint_v_ids"))));

		const TArray<TSharedPtr<FJsonValue>>& Subtree = Entry->GetArrayField(TEXT("subtree"));
		TestTrue(FString::Printf(TEXT("%s subtree non-empty"), *ModelName), Subtree.Num() > 0);
		for (const TSharedPtr<FJsonValue>& V : Subtree)
		{
			const TSharedPtr<FJsonObject> C = V->AsObject();
			const FString BodyName = C->GetStringField(TEXT("body"));
			const int32 Bid = mj_name2id(Model, mjOBJ_BODY, TCHAR_TO_UTF8(*BodyName));
			if (!TestTrue(FString::Printf(TEXT("body '%s' resolves"), *BodyName), Bid >= 0))
			{
				continue;
			}

			ExpectIntArrayEqual(*this, TEXT("subtree.body_ids"), MinkGetSubtreeBodyIds(Model, Bid),
				JsonIntArray(C->GetArrayField(TEXT("body_ids"))));
			ExpectIntArrayEqual(*this, TEXT("subtree.geom_ids"), MinkGetSubtreeGeomIds(Model, Bid),
				JsonIntArray(C->GetArrayField(TEXT("geom_ids"))));
			ExpectIntArrayEqual(*this, TEXT("subtree.joint_ids"), MinkGetSubtreeJointIds(Model, Bid),
				JsonIntArray(C->GetArrayField(TEXT("joint_ids"))));
		}

		if (Entry->HasField(TEXT("move_mocap")))
		{
			const TSharedPtr<FJsonObject> MoveMocap = Entry->GetObjectField(TEXT("move_mocap"));
			const FString MocapName = MoveMocap->GetStringField(TEXT("mocap"));
			const FString FrameName = MoveMocap->GetStringField(TEXT("frame"));
			EMinkFrameType FrameType;
			MinkFrameTypeFromString(MoveMocap->GetStringField(TEXT("type")), FrameType);

			mjData* Data = mj_makeData(Model);
			mj_kinematics(Model, Data);
			if (TestTrue(TEXT("MinkMoveMocapToFrame"),
					MinkMoveMocapToFrame(Model, Data, MocapName, FrameName, FrameType)))
			{
				const int32 MocapId = Model->body_mocapid[mj_name2id(Model, mjOBJ_BODY, TCHAR_TO_UTF8(*MocapName))];
				MinkExpectNear(*this, TEXT("move_mocap.pos"),
					Eigen::Map<const FMinkVec3>(Data->mocap_pos + 3 * MocapId),
					MinkJsonVec(MoveMocap->GetArrayField(TEXT("pos"))), TOL_KIN);
				MinkExpectNear(*this, TEXT("move_mocap.quat"),
					Eigen::Map<const Eigen::Vector4d>(Data->mocap_quat + 4 * MocapId),
					MinkJsonVec(MoveMocap->GetArrayField(TEXT("quat"))), TOL_KIN);
			}
			mj_deleteData(Data);
		}

		mj_deleteModel(Model);
	}
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
