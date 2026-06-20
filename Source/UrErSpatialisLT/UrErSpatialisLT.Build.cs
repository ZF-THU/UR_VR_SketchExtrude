// Fill out your copyright notice in the Description page of Project Settings.

using UnrealBuildTool;
using System.IO;

public class UrErSpatialisLT : ModuleRules {
	public UrErSpatialisLT(ReadOnlyTargetRules Target) : base(Target) {
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        PublicDefinitions.Add("MANIFOLD_PAR=-1");

        PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "Water", "Json", "JsonUtilities", "ImageWrapper", "ProceduralMeshComponent", "GeometryCore" });

		PrivateDependencyModuleNames.AddRange(new string[] { "Slate", "SlateCore", "UMG", "RenderCore", "RHI" });

        PublicIncludePaths.AddRange(new string[] {
            Path.Combine(ModuleDirectory, "../../ThirdParty/Manifold/manifold/include"),
            Path.Combine(ModuleDirectory, "../../ThirdParty/Manifold/manifold/src")
        });
        // Uncomment if you are using Slate UI
        // PrivateDependencyModuleNames.AddRange(new string[] { "Slate", "SlateCore" });

        // Uncomment if you are using online features
        // PrivateDependencyModuleNames.Add("OnlineSubsystem");

        // To include OnlineSubsystemSteam, add it to the plugins section in your uproject file with the Enabled attribute set to true
    }
}