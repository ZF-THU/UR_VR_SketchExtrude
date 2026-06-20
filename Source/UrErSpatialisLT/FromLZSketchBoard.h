#pragma once

#include "CoreMinimal.h"

class UGameViewportClient;
class UWorld;

class FFromLZSketchBoard
{
public:
	static void ShowForCapture(UWorld* World, UGameViewportClient* ViewportClient, const FString& CapturePngPath);
	static bool SaveCurrentSketchAndProceed(UWorld* World);
	static bool RestoreIfMinimized();
	static void Minimize();
	static void Close();
	static bool IsOpenOrMinimized();
};
