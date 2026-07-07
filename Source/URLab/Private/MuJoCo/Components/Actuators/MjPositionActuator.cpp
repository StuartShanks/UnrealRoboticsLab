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
		// Preserve class-default gains: mjs_addActuator already copied the resolved
		// <default> class's gain/bias params onto Element, and mjs_setToPosition
		// writes kp unconditionally (gainprm[0]=kp, biasprm[1]=-kp) — so unauthored
		// params must never reach the call. Pointer params pass nullptr when not
		// overridden, and an unauthored kp re-asserts Element->gainprm[0]. (A -1
		// sentinel here compiled kp=-1 => positive feedback: the TidyBot NaN.)
		// mjs_setToPosition also rejects kv+dampratio both non-null BEFORE writing
		// biasprm[2], so sentinel buffers silently dropped kv; SetToErr surfaces
		// any such rejection instead of discarding it.
		double kvBuf[1] = {(double)kv};
		double dampratioBuf[1] = {(double)dampratio};
		double timeconstBuf[1] = {(bOverride_timeconst && timeconst.Num() > 0) ? (double)timeconst[0] : -1.0};
		const char* SetToErr = mjs_setToPosition(Element, bOverride_kp ? (double)kp : Element->gainprm[0], bOverride_kv ? kvBuf : nullptr, bOverride_dampratio ? dampratioBuf : nullptr, (bOverride_timeconst && timeconst.Num() > 0) ? timeconstBuf : nullptr, bOverride_inheritrange ? (double)inheritrange : 0.0);
		if (SetToErr && *SetToErr)
		{
			UE_LOG(LogURLabExport, Warning, TEXT("[%s] mjs_setToPosition error: %s"), *GetName(), UTF8_TO_TCHAR(SetToErr));
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
