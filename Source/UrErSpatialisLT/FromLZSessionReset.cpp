#include "FromLZSessionReset.h"

#include "FromLZCameraPreview.h"
#include "FromLZCaptureUtils.h"
#include "FromLZFaceReconstructor.h"
#include "FromLZSketchBoard.h"

#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "Framework/Application/IInputProcessor.h"
#include "Framework/Application/SlateApplication.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformFileManager.h"
#include "Input/Events.h"
#include "InputCoreTypes.h"
#include "Kismet/GameplayStatics.h"
#include "UObject/UObjectGlobals.h"
#include "Delegates/Delegate.h"
#include "Misc/DateTime.h"
#include "Misc/Paths.h"

namespace
{
	TAtomic<int32> GFromLZSessionGeneration(0);
	TAtomic<int32> GFromLZActiveCompositeTaskCount(0);
	bool bFromLZResetPending = false;
	bool bFromLZReloadIssued = false;
	constexpr bool bFromLZLegacyGlobalTabInputEnabled = false;
	FFromLZSessionResetCompletionCallback GFromLZPendingResetCompletionCallback;
	FDelegateHandle GFromLZPostLoadMapHandle;

	class FFromLZGlobalTabInputProcessor : public IInputProcessor
	{
	public:
		virtual ~FFromLZGlobalTabInputProcessor() override = default;

		virtual void Tick(const float DeltaTime, FSlateApplication& SlateApp, TSharedRef<ICursor> Cursor) override
		{
		}

		virtual bool HandleKeyDownEvent(FSlateApplication& SlateApp, const FKeyEvent& InKeyEvent) override
		{
			if (InKeyEvent.GetKey() != EKeys::Tab)
			{
				return false;
			}

			UWorld* World = nullptr;
			if (GEngine && GEngine->GameViewport)
			{
				World = GEngine->GameViewport->GetWorld();
			}
			return FFromLZSessionReset::HandleGlobalTab(World);
		}

		virtual const TCHAR* GetDebugName() const override
		{
			return TEXT("FromLZGlobalTabInputProcessor");
		}
	};

	TSharedPtr<FFromLZGlobalTabInputProcessor> GFromLZGlobalTabInputProcessor;

	FString BuildArchiveDirName()
	{
		return FString::Printf(TEXT("log_%s"), *FDateTime::Now().ToString(TEXT("%Y%m%d_%H%M%S")));
	}

	FString BuildUniqueArchiveDirPath()
	{
		const FString SavedDir = FPaths::ProjectSavedDir();
		const FString BaseDir = SavedDir / BuildArchiveDirName();
		IFileManager& FileManager = IFileManager::Get();
		if (!FileManager.DirectoryExists(*BaseDir))
		{
			return BaseDir;
		}

		for (int32 SuffixIndex = 1; SuffixIndex < 1000; ++SuffixIndex)
		{
			const FString Candidate = FString::Printf(TEXT("%s_%02d"), *BaseDir, SuffixIndex);
			if (!FileManager.DirectoryExists(*Candidate))
			{
				return Candidate;
			}
		}

		return BaseDir;
	}

	FString ResolveCurrentLevelName(const UWorld* World)
	{
		return World ? UGameplayStatics::GetCurrentLevelName(World, true) : FString();
	}

	bool CopyDirectoryTreeIfExists(const FString& SourceDir, const FString& DestDir)
	{
		IFileManager& FileManager = IFileManager::Get();
		if (!FileManager.DirectoryExists(*SourceDir))
		{
			return true;
		}

		IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
		PlatformFile.CreateDirectoryTree(*DestDir);
		return PlatformFile.CopyDirectoryTree(*DestDir, *SourceDir, true);
	}

	void AppendResetWorkingFolderPaths(TArray<FString>& OutFolders)
	{
		const FString SavedDir = FPaths::ProjectSavedDir();
		const TCHAR* RecreatedDirs[] =
		{
			TEXT("2DDebug"),
			TEXT("FromAction"),
			TEXT("FromLZCaptures"),
			TEXT("FromProcess"),
			TEXT("FromSketch"),
			TEXT("Logs"),
		};

		for (const TCHAR* SubDir : RecreatedDirs)
		{
			OutFolders.Add(SavedDir / SubDir);
		}
	}

	void ResetSavedWorkingFolders()
	{
		const FString SavedDir = FPaths::ProjectSavedDir();
		const TCHAR* ResetDirs[] =
		{
			TEXT("2DDebug"),
			TEXT("FromAction"),
			TEXT("FromLZCaptures"),
			TEXT("FromProcess"),
			TEXT("FromSketch"),
		};

		IFileManager& FileManager = IFileManager::Get();
		for (const TCHAR* SubDir : ResetDirs)
		{
			const FString Dir = SavedDir / SubDir;
			if (FileManager.DirectoryExists(*Dir))
			{
				FileManager.DeleteDirectory(*Dir, false, true);
			}
			FileManager.MakeDirectory(*Dir, true);
		}

		FileManager.MakeDirectory(*(SavedDir / TEXT("Logs")), true);
	}

	void DispatchPendingResetCompletion(const FFromLZSessionResetResult& Result)
	{
		if (!GFromLZPendingResetCompletionCallback)
		{
			return;
		}

		FFromLZSessionResetCompletionCallback CallbackToRun = MoveTemp(GFromLZPendingResetCompletionCallback);
		if (CallbackToRun)
		{
			CallbackToRun(Result);
		}
	}

	void DispatchResetCompletionNow(
		FFromLZSessionResetCompletionCallback& CompletionCallback,
		const FFromLZSessionResetResult& Result)
	{
		if (CompletionCallback)
		{
			CompletionCallback(Result);
		}
	}

	bool ArchiveSavedFolders(FString& OutArchiveDir)
	{
		const FString SavedDir = FPaths::ProjectSavedDir();
		OutArchiveDir = BuildUniqueArchiveDirPath();
		IFileManager& FileManager = IFileManager::Get();
		FileManager.MakeDirectory(*OutArchiveDir, true);

		const TCHAR* ArchiveDirs[] =
		{
			TEXT("2DDebug"),
			TEXT("FromAction"),
			TEXT("FromLZCaptures"),
			TEXT("FromProcess"),
			TEXT("FromSketch"),
			TEXT("Logs"),
		};

		bool bAllCopied = true;
		for (const TCHAR* SubDir : ArchiveDirs)
		{
			const FString SourceDir = SavedDir / SubDir;
			const FString DestDir = OutArchiveDir / SubDir;
			if (!CopyDirectoryTreeIfExists(SourceDir, DestDir))
			{
				bAllCopied = false;
				UE_LOG(LogTemp, Warning, TEXT("FromLZ reset: failed to archive Saved/%s into %s"), SubDir, *DestDir);
			}
		}

		return bAllCopied;
	}

	void OnPostLoadMap(UWorld* LoadedWorld)
	{
		bFromLZResetPending = false;
		bFromLZReloadIssued = false;
		UE_LOG(LogTemp, Log, TEXT("FromLZ reset: map load completed. World=%s"), *GetNameSafe(LoadedWorld));
	}
}

void FFromLZSessionReset::Initialize()
{
	if (bFromLZLegacyGlobalTabInputEnabled &&
		!GFromLZGlobalTabInputProcessor.IsValid() &&
		FSlateApplication::IsInitialized())
	{
		GFromLZGlobalTabInputProcessor = MakeShared<FFromLZGlobalTabInputProcessor>();
		FSlateApplication::Get().RegisterInputPreProcessor(GFromLZGlobalTabInputProcessor);
	}

	if (!GFromLZPostLoadMapHandle.IsValid())
	{
		GFromLZPostLoadMapHandle = FCoreUObjectDelegates::PostLoadMapWithWorld.AddStatic(&OnPostLoadMap);
	}
}

void FFromLZSessionReset::Shutdown()
{
	if (GFromLZPostLoadMapHandle.IsValid())
	{
		FCoreUObjectDelegates::PostLoadMapWithWorld.Remove(GFromLZPostLoadMapHandle);
		GFromLZPostLoadMapHandle.Reset();
	}

	if (GFromLZGlobalTabInputProcessor.IsValid() && FSlateApplication::IsInitialized())
	{
		FSlateApplication::Get().UnregisterInputPreProcessor(GFromLZGlobalTabInputProcessor);
	}
	GFromLZGlobalTabInputProcessor.Reset();
	GFromLZPendingResetCompletionCallback = nullptr;
}

void FFromLZSessionReset::Tick(UWorld* World)
{
	if (!bFromLZResetPending || bFromLZReloadIssued)
	{
		return;
	}

	if (GFromLZActiveCompositeTaskCount.Load() > 0)
	{
		return;
	}

	FinalizePendingReset(World);
}

bool FFromLZSessionReset::HandleGlobalTab(
	UWorld* World,
	FFromLZSessionResetCompletionCallback CompletionCallback)
{
	if (!World)
	{
		UE_LOG(LogTemp, Warning, TEXT("FromLZ reset: request ignored because no game world is available."));
		FFromLZSessionResetResult Result;
		Result.Message = TEXT("No game world is available.");
		Result.SessionGeneration = GFromLZSessionGeneration.Load();
		DispatchResetCompletionNow(CompletionCallback, Result);
		return false;
	}

	if (bFromLZResetPending)
	{
		UE_LOG(LogTemp, Log, TEXT("FromLZ reset: request ignored because a reset is already pending."));
		FFromLZSessionResetResult Result;
		Result.Message = TEXT("Reset already pending.");
		Result.SessionGeneration = GFromLZSessionGeneration.Load();
		DispatchResetCompletionNow(CompletionCallback, Result);
		return true;
	}

	bFromLZResetPending = true;
	bFromLZReloadIssued = false;
	++GFromLZSessionGeneration;
	GFromLZPendingResetCompletionCallback = MoveTemp(CompletionCallback);

	FFromLZCaptureUtils::CancelPendingCapture();
	FFromLZCameraPreview::Shutdown();
	FFromLZSketchBoard::Close();

	if (GFromLZActiveCompositeTaskCount.Load() > 0)
	{
		UE_LOG(LogTemp, Log, TEXT("FromLZ reset: request received; waiting for %d active composite task(s) to finish before resetting."), GFromLZActiveCompositeTaskCount.Load());
	}
	else
	{
		UE_LOG(LogTemp, Log, TEXT("FromLZ reset: request received; reset will finalize on the next tick."));
	}

	return true;
}

bool FFromLZSessionReset::IsResetPending()
{
	return bFromLZResetPending;
}

int32 FFromLZSessionReset::GetSessionGeneration()
{
	return GFromLZSessionGeneration.Load();
}

bool FFromLZSessionReset::IsSessionGenerationCurrent(int32 SessionGeneration)
{
	return SessionGeneration == GFromLZSessionGeneration.Load();
}

void FFromLZSessionReset::NotifyCompositeTaskStarted()
{
	++GFromLZActiveCompositeTaskCount;
}

void FFromLZSessionReset::NotifyCompositeTaskFinished()
{
	--GFromLZActiveCompositeTaskCount;
}

FFromLZSessionReset::FScopedCompositeTaskCounter::FScopedCompositeTaskCounter()
{
	FFromLZSessionReset::NotifyCompositeTaskStarted();
}

FFromLZSessionReset::FScopedCompositeTaskCounter::~FScopedCompositeTaskCounter()
{
	if (bActive)
	{
		FFromLZSessionReset::NotifyCompositeTaskFinished();
		bActive = false;
	}
}

void FFromLZSessionReset::FinalizePendingReset(UWorld* World)
{
	if (!bFromLZResetPending || bFromLZReloadIssued)
	{
		return;
	}

	FFromLZCaptureUtils::CancelPendingCapture();
	FFromLZCameraPreview::Shutdown();
	FFromLZSketchBoard::Close();
	FFromLZFaceReconstructor::ResetAllRuntimeState(World);

	FString ArchiveDir;
	const bool bArchiveOk = ArchiveSavedFolders(ArchiveDir);
	ResetSavedWorkingFolders();

	const FString LevelName = ResolveCurrentLevelName(World);
	if (!World || LevelName.IsEmpty())
	{
		UE_LOG(LogTemp, Warning, TEXT("FromLZ reset: archive=%d, but level reload failed because world or level name is unavailable."), bArchiveOk ? 1 : 0);
		FFromLZSessionResetResult Result;
		Result.bSuccess = false;
		Result.Message = TEXT("Level reload failed because world or level name is unavailable.");
		Result.ArchiveDir = ArchiveDir;
		Result.LevelName = LevelName;
		AppendResetWorkingFolderPaths(Result.RecreatedFolders);
		Result.bArchiveOk = bArchiveOk;
		Result.bReloadIssued = false;
		Result.SessionGeneration = GFromLZSessionGeneration.Load();
		DispatchPendingResetCompletion(Result);
		bFromLZResetPending = false;
		return;
	}

	bFromLZReloadIssued = true;
	FFromLZSessionResetResult Result;
	Result.bSuccess = true;
	Result.Message = bArchiveOk
		? TEXT("Full session reset completed; level reload issued.")
		: TEXT("Full session reset completed with archive copy warnings; level reload issued.");
	Result.ArchiveDir = ArchiveDir;
	Result.LevelName = LevelName;
	AppendResetWorkingFolderPaths(Result.RecreatedFolders);
	Result.bArchiveOk = bArchiveOk;
	Result.bReloadIssued = true;
	Result.SessionGeneration = GFromLZSessionGeneration.Load();
	DispatchPendingResetCompletion(Result);
	UE_LOG(LogTemp, Log, TEXT("FromLZ reset: archived Saved state to %s, cleared working folders, and is reloading level %s."), *ArchiveDir, *LevelName);
	UGameplayStatics::OpenLevel(World, FName(*LevelName), true);
}
