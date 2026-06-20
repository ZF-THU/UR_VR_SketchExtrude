#include "FromLZGameViewportClient.h"

#include "FromLZCameraPreview.h"
#include "FromLZCaptureUtils.h"
#include "FromLZFaceReconstructor.h"
#include "FromLZSketchBoard.h"
#include "FromLZSketchProcessor.h"
#include "FromLZSessionReset.h"
#include "Engine/GameInstance.h"
#include "InputCoreTypes.h"

namespace
{
	constexpr bool bFromLZLegacyViewportInputEnabled = false;
}

void UFromLZGameViewportClient::Init(FWorldContext& WorldContext, UGameInstance* OwningGameInstance, bool bCreateNewAudioDevice)
{
	Super::Init(WorldContext, OwningGameInstance, bCreateNewAudioDevice);

	const FString WorldName = GetWorld() ? GetWorld()->GetName() : TEXT("<null>");
	UE_LOG(LogTemp, Log, TEXT("FromLZGameViewportClient initialized. World=%s GameInstance=%s"), *WorldName, *GetNameSafe(OwningGameInstance));
}

void UFromLZGameViewportClient::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	FFromLZSessionReset::Tick(GetWorld());
	if (FFromLZSessionReset::IsResetPending())
	{
		FFromLZCameraPreview::Shutdown();
		return;
	}
	FFromLZCameraPreview::Tick(GetWorld(), this, Viewport);
	FFromLZCaptureUtils::CompletePendingCapture(GetWorld(), Viewport);
}

void UFromLZGameViewportClient::Draw(FViewport* InViewport, FCanvas* SceneCanvas)
{
	Super::Draw(InViewport, SceneCanvas);
	FFromLZCaptureUtils::NotifyViewportDrawn(GetWorld(), InViewport);
}

bool UFromLZGameViewportClient::InputKey(const FInputKeyEventArgs& EventArgs)
{
	if (FFromLZSessionReset::IsResetPending())
	{
		return true;
	}

	if (!bFromLZLegacyViewportInputEnabled)
	{
		return Super::InputKey(EventArgs);
	}

	if (EventArgs.Event == IE_Pressed)
	{
		if (EventArgs.Key == EKeys::B)
		{
			if (FFromLZSketchBoard::RestoreIfMinimized())
			{
				return true;
			}
		}
		else if (EventArgs.Key == EKeys::Enter)
		{
			UE_LOG(LogTemp, Log, TEXT("CaptureFromLZ invoked from viewport input. Key=%s"), *EventArgs.Key.ToString());
			FFromLZCaptureUtils::BeginCaptureFromWorld(GetWorld(), Viewport);
		}
		else if (EventArgs.Key == EKeys::One)
		{
			UE_LOG(LogTemp, Log, TEXT("CaptureFromLZ tagged-camera capture invoked. Key=%s Tag=FromLZCaptureCamera1"), *EventArgs.Key.ToString());
			FFromLZCaptureUtils::BeginCaptureFromTaggedCamera(GetWorld(), Viewport, TEXT("FromLZCaptureCamera1"));
			return true;
		}
		else if (EventArgs.Key == EKeys::Two)
		{
			UE_LOG(LogTemp, Log, TEXT("CaptureFromLZ tagged-camera capture invoked. Key=%s Tag=FromLZCaptureCamera2"), *EventArgs.Key.ToString());
			FFromLZCaptureUtils::BeginCaptureFromTaggedCamera(GetWorld(), Viewport, TEXT("FromLZCaptureCamera2"));
			return true;
		}
		else if (EventArgs.Key == EKeys::LeftShift || EventArgs.Key == EKeys::RightShift)
		{
			UE_LOG(LogTemp, Log, TEXT("Step11 restore invoked from viewport input. Key=%s"), *EventArgs.Key.ToString());
			FFromLZFaceReconstructor::RestoreStep11RuntimeBooleans(GetWorld());
		}
		else if (EventArgs.Key == EKeys::SpaceBar)
		{
			if (FFromLZSketchBoard::SaveCurrentSketchAndProceed(GetWorld()))
			{
				return true;
			}
			else
			{
				UE_LOG(LogTemp, Log, TEXT("ProcessSketch invoked from viewport input. Key=%s"), *EventArgs.Key.ToString());
				FFromLZSketchProcessor::ProcessLatestSketch(GetWorld());
			}
		}
	}

	return Super::InputKey(EventArgs);
}

void UFromLZGameViewportClient::BeginDestroy()
{
	FFromLZCameraPreview::Shutdown();
	Super::BeginDestroy();
}
