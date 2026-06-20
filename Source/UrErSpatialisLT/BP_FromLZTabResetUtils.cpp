#include "BP_FromLZTabResetUtils.h"

#include "FromLZSessionReset.h"

#include "Engine/Engine.h"
#include "Engine/World.h"

UBP_FromLZTabResetUtils* UBP_FromLZTabResetUtils::FullSessionReset(UObject* WorldContextObject)
{
	UBP_FromLZTabResetUtils* Action = NewObject<UBP_FromLZTabResetUtils>();
	Action->WorldContextObject = WorldContextObject;
	return Action;
}

void UBP_FromLZTabResetUtils::Activate()
{
	UWorld* World = GEngine
		? GEngine->GetWorldFromContextObject(WorldContextObject.Get(), EGetWorldErrorMode::ReturnNull)
		: nullptr;
	if (!World)
	{
		OnFailed.Broadcast(TEXT("World context is invalid."), FString(), TArray<FString>(), false, FFromLZSessionReset::GetSessionGeneration());
		SetReadyToDestroy();
		return;
	}

	TWeakObjectPtr<UBP_FromLZTabResetUtils> WeakThis(this);
	FFromLZSessionReset::HandleGlobalTab(
		World,
		[WeakThis](const FFromLZSessionResetResult& Result)
		{
			if (!WeakThis.IsValid())
			{
				return;
			}

			if (Result.bSuccess)
			{
				WeakThis->OnCompleted.Broadcast(
					Result.ArchiveDir,
					Result.LevelName,
					Result.RecreatedFolders,
					Result.bArchiveOk,
					Result.bReloadIssued,
					Result.SessionGeneration);
			}
			else
			{
				WeakThis->OnFailed.Broadcast(
					Result.Message,
					Result.ArchiveDir,
					Result.RecreatedFolders,
					Result.bArchiveOk,
					Result.SessionGeneration);
			}
			WeakThis->SetReadyToDestroy();
		});
}
