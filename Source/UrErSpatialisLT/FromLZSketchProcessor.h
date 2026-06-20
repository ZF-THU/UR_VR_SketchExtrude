#pragma once

#include "CoreMinimal.h"

class UWorld;

class FFromLZSketchProcessor
{
public:
	static void ProcessLatestSketch(UWorld* World);
	static void ProcessSketch(UWorld* World, const FString& SketchPng, const FString& CapturePng);

private:
	static FString FindLatestPng(const FString& Directory, bool bRequireCaptureMainPng = false);
	static bool DecodePngToRGBA(const FString& Path, TArray<uint8>& OutPixels, int32& OutWidth, int32& OutHeight);
	static bool SaveRGBAToPng(const TArray<uint8>& RGBAPixels, int32 Width, int32 Height, const FString& OutputPath);
	static bool ExtractAndSaveChannel(const TArray<uint8>& RGBAPixels, int32 Width, int32 Height, uint8 TargetR, uint8 TargetG, uint8 TargetB, const FString& OutputPath);
};
