#pragma once

#include "CoreMinimal.h"

class FViewport;
class UGameViewportClient;
class UWorld;

class FFromLZCameraPreview
{
public:
	static void Tick(UWorld* World, UGameViewportClient* ViewportClient, FViewport* Viewport);
	static void Shutdown();
};
