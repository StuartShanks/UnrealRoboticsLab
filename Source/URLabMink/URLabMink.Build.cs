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

using UnrealBuildTool;
using System.IO;

public class URLabMink : ModuleRules
{
	public URLabMink(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

		// qpmad (vendored Goldfarb-Idnani QP solver) throws on ill-formed
		// problems; the FMinkQp wrapper catches at the boundary.
		bEnableExceptions = true;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"Eigen"
		});

		PrivateDependencyModuleNames.AddRange(new string[]
		{
			"CoreUObject",
			"Projects",
			"Json"
		});

		// MuJoCo headers + import lib from the shared third_party install.
		// URLab.Build.cs owns the drift checks and runtime staging of the
		// shared libraries; this module only consumes the installed artifacts.
		string MujocoPath = Path.Combine(PluginDirectory, "third_party", "install", "MuJoCo");
		PublicIncludePaths.Add(Path.Combine(MujocoPath, "include"));
		string LibPath = Path.Combine(MujocoPath, "lib");
		if (Directory.Exists(LibPath))
		{
			if (Target.Platform == UnrealTargetPlatform.Win64)
			{
				foreach (string LibFile in Directory.GetFiles(LibPath, "*.lib", SearchOption.AllDirectories))
				{
					PublicAdditionalLibraries.Add(LibFile);
				}
			}
			else if (Target.Platform == UnrealTargetPlatform.Linux)
			{
				foreach (string LibFile in Directory.GetFiles(LibPath, "*.so", SearchOption.AllDirectories))
				{
					PublicAdditionalLibraries.Add(LibFile);
				}
			}
		}
	}
}
