#pragma once

#include "CoreMinimal.h"

class UWorld;

struct FFromLZSessionResetResult
{
	bool bSuccess = false;
	FString Message;
	FString ArchiveDir;
	FString LevelName;
	TArray<FString> RecreatedFolders;
	bool bArchiveOk = false;
	bool bReloadIssued = false;
	int32 SessionGeneration = INDEX_NONE;
};

using FFromLZSessionResetCompletionCallback = TFunction<void(const FFromLZSessionResetResult& Result)>;

class FFromLZSessionReset
{
public:
	static void Initialize();
	static void Shutdown();
	static void Tick(UWorld* World);
	static bool HandleGlobalTab(
		UWorld* World,
		FFromLZSessionResetCompletionCallback CompletionCallback = nullptr);
	static bool IsResetPending();

	static int32 GetSessionGeneration();
	static bool IsSessionGenerationCurrent(int32 SessionGeneration);
	static void NotifyCompositeTaskStarted();
	static void NotifyCompositeTaskFinished();

	class FScopedCompositeTaskCounter
	{
	public:
		FScopedCompositeTaskCounter();
		~FScopedCompositeTaskCounter();

		FScopedCompositeTaskCounter(const FScopedCompositeTaskCounter&) = delete;
		FScopedCompositeTaskCounter& operator=(const FScopedCompositeTaskCounter&) = delete;

	private:
		bool bActive = true;
	};

private:
	static void FinalizePendingReset(UWorld* World);
};
