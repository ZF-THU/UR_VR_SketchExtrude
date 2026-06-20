#include "BP_FromLZShiftUndoUtils.h"

#include "FromLZFaceReconstructor.h"

#include "Engine/Engine.h"
#include "Engine/World.h"

UBP_FromLZShiftUndoUtils* UBP_FromLZShiftUndoUtils::UndoLatestStep11Press(UObject* WorldContextObject)
{
	UBP_FromLZShiftUndoUtils* Action = NewObject<UBP_FromLZShiftUndoUtils>();
	Action->WorldContextObject = WorldContextObject;
	return Action;
}

void UBP_FromLZShiftUndoUtils::Activate()
{
	UWorld* World = GEngine
		? GEngine->GetWorldFromContextObject(WorldContextObject.Get(), EGetWorldErrorMode::ReturnNull)
		: nullptr;
	if (!World)
	{
		OnFailed.Broadcast(TEXT("World context is invalid."), FString(), TArray<FString>());
		SetReadyToDestroy();
		return;
	}

	TWeakObjectPtr<UBP_FromLZShiftUndoUtils> WeakThis(this);
	FFromLZFaceReconstructor::RestoreStep11RuntimeBooleans(
		World,
		[WeakThis](const FFromLZStep11UndoResult& Result)
		{
			if (!WeakThis.IsValid())
			{
				return;
			}

			if (Result.bSuccess)
			{
				WeakThis->OnCompleted.Broadcast(
					Result.PressId,
					Result.UndoDiagnosticPath,
					Result.OutputFiles,
					Result.DestroyedActorCount,
					Result.DestroyedBooleanComponentCount,
					Result.RestoredHiddenSourceCount);
			}
			else
			{
				WeakThis->OnFailed.Broadcast(Result.Message, Result.PressId, Result.OutputFiles);
			}
			WeakThis->SetReadyToDestroy();
		});
}
