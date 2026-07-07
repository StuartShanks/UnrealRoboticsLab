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

#include "MuJoCo/Components/Actuators/MjPositionActuator.h"

#include "XmlNode.h"
#include "MuJoCo/Utils/MjXmlUtils.h"
#include "Utils/URLabLogging.h"
#include "MuJoCo/Utils/MjOrientationUtils.h"

UMjPositionActuator::UMjPositionActuator()
{
	Type = EMjActuatorType::Position;
}

void UMjPositionActuator::ExportTo(mjsActuator* Element, mjsDefault* def)
{
	if (!Element)
		return;

	Super::ExportTo(Element, def);

	// --- CODEGEN_EXPORT_START ---
	{
		// Feed mjs_setToPosition ONLY the parameters this component explicitly
		// authored. mjs_addActuator already copied the resolved default class's
		// gains onto Element (gainprm[0]=kp, biasprm[1]=-kp, biasprm[2]=-kv, ...),
		// so any parameter we do not override must be preserved, NOT clobbered:
		//   * Unauthored pointer params (kv/dampratio/timeconst) are passed as
		//     nullptr so mjs_setToPosition leaves biasprm[2]/dynprm untouched.
		//   * Unauthored kp is re-asserted from Element->gainprm[0] (the inherited
		//     value) rather than the historic -1.0 sentinel, which used to write
		//     gainprm[0]=-1 / biasprm[1]=+1 (positive feedback -> divergence).
		// The old code also passed kv AND dampratio as always-non-null sentinels,
		// which made mjs_setToPosition return "kv and dampratio cannot both be
		// defined" and skip biasprm[2] entirely (silently dropping kv). Passing
		// only the authored pointer avoids that. We check the returned error.
		double kvBuf[1] = {(double)kv};
		double dampratioBuf[1] = {(double)dampratio};
		double timeconstBuf[1] = {(timeconst.Num() > 0) ? (double)timeconst[0] : 0.0};
		const char* Err = mjs_setToPosition(Element,
			bOverride_kp ? (double)kp : Element->gainprm[0],
			bOverride_kv ? kvBuf : nullptr,
			bOverride_dampratio ? dampratioBuf : nullptr,
			(bOverride_timeconst && timeconst.Num() > 0) ? timeconstBuf : nullptr,
			bOverride_inheritrange ? (double)inheritrange : 0.0);
		if (Err && *Err)
		{
			UE_LOG(LogURLabExport, Warning,
				TEXT("[UMjPositionActuator::ExportTo] mjs_setToPosition error: %s"), UTF8_TO_TCHAR(Err));
		}
	}
	if (bOverride_inheritrange)
		Element->inheritrange = inheritrange;
	// --- CODEGEN_EXPORT_END ---
}

void UMjPositionActuator::ImportFromXml(const FXmlNode* Node, const FMjCompilerSettings& CompilerSettings)
{
	Super::ImportFromXml(Node, CompilerSettings);
	if (!Node)
		return;

	// --- CODEGEN_IMPORT_START ---
	MjXmlUtils::ReadAttrFloat(Node, TEXT("inheritrange"), inheritrange, bOverride_inheritrange);
	MjXmlUtils::ReadAttrFloat(Node, TEXT("kp"), kp, bOverride_kp);
	MjXmlUtils::ReadAttrFloat(Node, TEXT("kv"), kv, bOverride_kv);
	MjXmlUtils::ReadAttrFloat(Node, TEXT("dampratio"), dampratio, bOverride_dampratio);
	MjXmlUtils::ReadAttrFloatArray(Node, TEXT("timeconst"), timeconst, bOverride_timeconst);
	// --- CODEGEN_IMPORT_END ---
}
