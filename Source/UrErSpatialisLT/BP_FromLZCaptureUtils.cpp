#include "BP_FromLZCaptureUtils.h"

#include "FromLZCaptureUtils.h"

#include "Camera/CameraComponent.h"
#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "Engine/World.h"
#include "UnrealClient.h"

UBP_FromLZCaptureUtils* UBP_FromLZCaptureUtils::CaptureFromCamera(UObject* WorldContextObject, UCameraComponent* CameraComponent)
{
	UBP_FromLZCaptureUtils* Action = NewObject<UBP_FromLZCaptureUtils>();
	Action->WorldContextObject = WorldContextObject;
	Action->CameraComponent = CameraComponent;
	if (WorldContextObject)
	{
		Action->RegisterWithGameInstance(WorldContextObject);
	}
	return Action;
}

void UBP_FromLZCaptureUtils::Activate()
{
	UWorld* World = GEngine
		? GEngine->GetWorldFromContextObject(WorldContextObject.Get(), EGetWorldErrorMode::ReturnNull)
		: nullptr;
	FViewport* Viewport = (GEngine && GEngine->GameViewport)
		? GEngine->GameViewport->Viewport
		: nullptr;

	TWeakObjectPtr<UBP_FromLZCaptureUtils> WeakThis(this);
	FFromLZCaptureUtils::BeginCaptureFromCameraComponent(
		World,
		Viewport,
		CameraComponent.Get(),
		[WeakThis](const FFromLZCaptureResult& Result)
		{
			if (!WeakThis.IsValid())
			{
				return;
			}

			if (Result.bSuccess)
			{
				WeakThis->OnCompleted.Broadcast(Result.OutputFiles, Result.CaptureDirectory, Result.MainPngPath);
			}
			else
			{
				WeakThis->OnFailed.Broadcast(Result.Message, Result.OutputFiles);
			}
			WeakThis->SetReadyToDestroy();
		});
}
