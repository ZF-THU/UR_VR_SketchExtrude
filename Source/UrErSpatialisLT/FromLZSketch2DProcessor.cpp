#include "FromLZSketch2DProcessor.h"

#include "Async/Async.h"
#include "FromLZFaceReconstructor.h"
#include "FromLZImageOps.h"
#include "FromLZPressNaming.h"
#include "FromLZProcessingLimits.h"
#include "FromLZSessionReset.h"
#include "HAL/FileManager.h"
#include "HAL/CriticalSection.h"
#include "HAL/PlatformTime.h"
#include "Misc/DateTime.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Misc/FileHelper.h"
#include "Misc/ScopeExit.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"

namespace
{
	struct FFromLZCompositeWorkItem
	{
		TArray<uint8> Pixels;
		int32 Width = 0;
		int32 Height = 0;
		FString DebugDir;
		FSketchSourceInfo Source;
		TWeakObjectPtr<UWorld> World;
		FFromLZProcessParams Params;
		FFromLZCompositeStartedCallback StartedCallback;
		FFromLZPressCompletionCallback CompletionCallback;
		int32 SessionGeneration = 0;
		uint64 RequestId = 0;
	};

	FCriticalSection GFromLZCompositeSchedulerLock;
	int32 GFromLZActiveCompositeWorkers = 0;
	bool bGFromLZHasPendingCompositeWork = false;
	FFromLZCompositeWorkItem GFromLZPendingCompositeWork;
	uint64 GFromLZNextCompositeRequestId = 0;

	void StartCompositeWorker(FFromLZCompositeWorkItem&& Work);

	void DispatchPressCallback(FFromLZPressCompletionCallback& Callback, const FFromLZPressProcessResult& Result)
	{
		if (!Callback)
		{
			return;
		}

		FFromLZPressCompletionCallback CallbackToRun = MoveTemp(Callback);
		AsyncTask(ENamedThreads::GameThread, [CallbackToRun = MoveTemp(CallbackToRun), Result]() mutable
		{
			CallbackToRun(Result);
		});
	}

	void DispatchStartedCallback(FFromLZCompositeStartedCallback& Callback, const FFromLZPressProcessResult& Result)
	{
		if (!Callback)
		{
			return;
		}

		FFromLZCompositeStartedCallback CallbackToRun = MoveTemp(Callback);
		AsyncTask(ENamedThreads::GameThread, [CallbackToRun = MoveTemp(CallbackToRun), Result]() mutable
		{
			CallbackToRun(Result);
		});
	}

	void FailCompositeWork(FFromLZCompositeWorkItem& Work, const FString& Message)
	{
		FFromLZPressProcessResult Result;
		Result.bSuccess = false;
		Result.Message = Message;
		DispatchPressCallback(Work.CompletionCallback, Result);
	}

	bool ShouldDropPendingCompositeWork(const FFromLZCompositeWorkItem& Work)
	{
		return FFromLZSessionReset::IsResetPending()
			|| !FFromLZSessionReset::IsSessionGenerationCurrent(Work.SessionGeneration);
	}

	void FinishCompositeWorker()
	{
		FFromLZCompositeWorkItem WorkToStart;
		bool bShouldStartPending = false;
		uint64 DroppedRequestId = 0;
		int32 ActiveAfterFinish = 0;
		int32 MaxWorkers = 1;

		{
			FScopeLock Lock(&GFromLZCompositeSchedulerLock);
			GFromLZActiveCompositeWorkers = FMath::Max(0, GFromLZActiveCompositeWorkers - 1);
			MaxWorkers = FromLZProcessing::GetCompositeMaxWorkers();

			if (bGFromLZHasPendingCompositeWork && ShouldDropPendingCompositeWork(GFromLZPendingCompositeWork))
			{
				DroppedRequestId = GFromLZPendingCompositeWork.RequestId;
				FailCompositeWork(GFromLZPendingCompositeWork, TEXT("Composite request was dropped because the session reset or generation changed."));
				bGFromLZHasPendingCompositeWork = false;
				GFromLZPendingCompositeWork = FFromLZCompositeWorkItem();
			}

			if (bGFromLZHasPendingCompositeWork && GFromLZActiveCompositeWorkers < MaxWorkers)
			{
				WorkToStart = MoveTemp(GFromLZPendingCompositeWork);
				bGFromLZHasPendingCompositeWork = false;
				GFromLZPendingCompositeWork = FFromLZCompositeWorkItem();
				++GFromLZActiveCompositeWorkers;
				bShouldStartPending = true;
			}

			ActiveAfterFinish = GFromLZActiveCompositeWorkers;
		}

		if (DroppedRequestId != 0)
		{
			UE_LOG(LogTemp, Log, TEXT("Sketch2D: dropped stale pending composite request %llu."), static_cast<unsigned long long>(DroppedRequestId));
		}

		if (bShouldStartPending)
		{
			UE_LOG(LogTemp, Log, TEXT("Sketch2D: starting pending composite request %llu (active=%d/max=%d)."), static_cast<unsigned long long>(WorkToStart.RequestId), ActiveAfterFinish, MaxWorkers);
			StartCompositeWorker(MoveTemp(WorkToStart));
		}
		else
		{
			UE_LOG(LogTemp, Verbose, TEXT("Sketch2D: composite worker finished (active=%d/max=%d)."), ActiveAfterFinish, MaxWorkers);
		}
	}

	void StartCompositeWorker(FFromLZCompositeWorkItem&& Work)
	{
		TSharedRef<FFromLZSessionReset::FScopedCompositeTaskCounter, ESPMode::ThreadSafe> TaskCounter =
			MakeShared<FFromLZSessionReset::FScopedCompositeTaskCounter, ESPMode::ThreadSafe>();

		Async(EAsyncExecution::ThreadPool, [Work = MoveTemp(Work), TaskCounter]() mutable
		{
			ON_SCOPE_EXIT
			{
				FinishCompositeWorker();
			};

			const bool bDispatched = FFromLZSketch2DProcessor::ProcessCompositeWithGeneration(
				Work.Pixels,
				Work.Width,
				Work.Height,
				Work.DebugDir,
				Work.Source,
				Work.SessionGeneration,
				Work.World,
				Work.Params,
				MoveTemp(Work.StartedCallback),
				MoveTemp(Work.CompletionCallback));
			if (!bDispatched && Work.CompletionCallback)
			{
				FailCompositeWork(Work, TEXT("Composite processing failed before Step 10/11 dispatch."));
			}
		});
	}

	void ScheduleCompositeWork(FFromLZCompositeWorkItem&& Work)
	{
		FFromLZCompositeWorkItem WorkToStart;
		bool bShouldStart = false;
		bool bReplacedPending = false;
		bool bDiscardedPendingForImmediateStart = false;
		uint64 PendingRequestId = 0;
		int32 ActiveWorkers = 0;
		int32 MaxWorkers = 1;

		{
			FScopeLock Lock(&GFromLZCompositeSchedulerLock);
			MaxWorkers = FromLZProcessing::GetCompositeMaxWorkers();
			Work.RequestId = ++GFromLZNextCompositeRequestId;

			if (GFromLZActiveCompositeWorkers < MaxWorkers)
			{
				bDiscardedPendingForImmediateStart = bGFromLZHasPendingCompositeWork;
				if (bGFromLZHasPendingCompositeWork)
				{
					FailCompositeWork(GFromLZPendingCompositeWork, TEXT("Composite request was discarded because a newer request started immediately."));
				}
				bGFromLZHasPendingCompositeWork = false;
				GFromLZPendingCompositeWork = FFromLZCompositeWorkItem();
				++GFromLZActiveCompositeWorkers;
				WorkToStart = MoveTemp(Work);
				bShouldStart = true;
			}
			else
			{
				bReplacedPending = bGFromLZHasPendingCompositeWork;
				if (bGFromLZHasPendingCompositeWork)
				{
					FailCompositeWork(GFromLZPendingCompositeWork, TEXT("Composite request was replaced by a newer pending request."));
				}
				GFromLZPendingCompositeWork = MoveTemp(Work);
				bGFromLZHasPendingCompositeWork = true;
				PendingRequestId = GFromLZPendingCompositeWork.RequestId;
			}

			ActiveWorkers = GFromLZActiveCompositeWorkers;
		}

		if (bShouldStart)
		{
			if (bDiscardedPendingForImmediateStart)
			{
				UE_LOG(LogTemp, Log, TEXT("Sketch2D: discarded older pending composite because a newer request can start immediately."));
			}
			UE_LOG(LogTemp, Log, TEXT("Sketch2D: starting composite request %llu (active=%d/max=%d)."), static_cast<unsigned long long>(WorkToStart.RequestId), ActiveWorkers, MaxWorkers);
			StartCompositeWorker(MoveTemp(WorkToStart));
		}
		else
		{
			UE_LOG(
				LogTemp,
				Log,
				TEXT("Sketch2D: %s pending composite request %llu because active workers are full (active=%d/max=%d)."),
				bReplacedPending ? TEXT("replaced") : TEXT("queued"),
				static_cast<unsigned long long>(PendingRequestId),
				ActiveWorkers,
				MaxWorkers);
		}
	}
}

void FFromLZSketch2DProcessor::ProcessCompositeAsync(TArray<uint8> RGBA, int32 Width, int32 Height, const FString& DebugDir, const FSketchSourceInfo& Source, UWorld* World)
{
	ProcessCompositeAsync(MoveTemp(RGBA), Width, Height, DebugDir, Source, World, FFromLZProcessParams(), nullptr, nullptr);
}

void FFromLZSketch2DProcessor::ProcessCompositeAsync(
	TArray<uint8> RGBA,
	int32 Width,
	int32 Height,
	const FString& DebugDir,
	const FSketchSourceInfo& Source,
	UWorld* World,
	const FFromLZProcessParams& Params,
	FFromLZCompositeStartedCallback StartedCallback,
	FFromLZPressCompletionCallback CompletionCallback)
{
	// RGBA is taken by value; move it into the async task so the heavy work runs off the game thread.
	FFromLZCompositeWorkItem Work;
	Work.Pixels = MoveTemp(RGBA);
	Work.Width = Width;
	Work.Height = Height;
	Work.DebugDir = DebugDir;
	Work.Source = Source;
	Work.World = TWeakObjectPtr<UWorld>(World);
	Work.Params = Params;
	Work.StartedCallback = MoveTemp(StartedCallback);
	Work.CompletionCallback = MoveTemp(CompletionCallback);
	Work.SessionGeneration = FFromLZSessionReset::GetSessionGeneration();

	ScheduleCompositeWork(MoveTemp(Work));
}

bool FFromLZSketch2DProcessor::ProcessComposite(const TArray<uint8>& RGBA, int32 Width, int32 Height, const FString& DebugDir, const FSketchSourceInfo& Source, TWeakObjectPtr<UWorld> World)
{
	return ProcessComposite(RGBA, Width, Height, DebugDir, Source, World, FFromLZProcessParams());
}

bool FFromLZSketch2DProcessor::ProcessComposite(
	const TArray<uint8>& RGBA,
	int32 Width,
	int32 Height,
	const FString& DebugDir,
	const FSketchSourceInfo& Source,
	TWeakObjectPtr<UWorld> World,
	const FFromLZProcessParams& Params)
{
	return ProcessCompositeWithGeneration(RGBA, Width, Height, DebugDir, Source, FFromLZSessionReset::GetSessionGeneration(), World, Params);
}

bool FFromLZSketch2DProcessor::ProcessCompositeWithGeneration(const TArray<uint8>& RGBA, int32 Width, int32 Height, const FString& DebugDir, const FSketchSourceInfo& Source, int32 SessionGeneration, TWeakObjectPtr<UWorld> World)
{
	return ProcessCompositeWithGeneration(RGBA, Width, Height, DebugDir, Source, SessionGeneration, World, FFromLZProcessParams());
}

bool FFromLZSketch2DProcessor::ProcessCompositeWithGeneration(
	const TArray<uint8>& RGBA,
	int32 Width,
	int32 Height,
	const FString& DebugDir,
	const FSketchSourceInfo& Source,
	int32 SessionGeneration,
	TWeakObjectPtr<UWorld> World,
	const FFromLZProcessParams& Params,
	FFromLZCompositeStartedCallback StartedCallback,
	FFromLZPressCompletionCallback CompletionCallback)
{
	if (Width <= 0 || Height <= 0 || RGBA.Num() < Width * Height * 4)
	{
		UE_LOG(LogTemp, Warning, TEXT("Sketch2D: invalid input (%dx%d, %d bytes)"), Width, Height, RGBA.Num());
		FFromLZPressProcessResult Result;
		Result.bSuccess = false;
		Result.Message = TEXT("Invalid composite input.");
		DispatchPressCallback(CompletionCallback, Result);
		return false;
	}
	if (!FFromLZSessionReset::IsSessionGenerationCurrent(SessionGeneration))
	{
		UE_LOG(LogTemp, Log, TEXT("Sketch2D: skipped stale composite because the session generation changed before processing started."));
		FFromLZPressProcessResult Result;
		Result.bSuccess = false;
		Result.Message = TEXT("Skipped stale composite because the session generation changed before processing started.");
		DispatchPressCallback(CompletionCallback, Result);
		return false;
	}

	IFileManager::Get().MakeDirectory(*DebugDir, true);

	// Each pipeline run (one Space press) gets its own Press_## folder. Number continues
	// from the highest existing Press_* so runs accumulate across restarts.
	const FString PressName = FFromLZPressNaming::MakePressName(FFromLZPressNaming::GetNextPressIndex(DebugDir));
	const FString PressDir = DebugDir / PressName;
	IFileManager::Get().MakeDirectory(*PressDir, true);

	// Action.json outputs live under Saved/FromAction/Press_##/ (sibling of 2DDebug).
	const FString ActionPressDir = FPaths::GetPath(DebugDir) / TEXT("FromAction") / PressName;
	IFileManager::Get().MakeDirectory(*ActionPressDir, true);

	{
		FFromLZPressProcessResult StartedResult;
		StartedResult.bSuccess = true;
		StartedResult.Message = TEXT("Composite processing started.");
		StartedResult.PressDir = PressDir;
		StartedResult.ActionPressDir = ActionPressDir;
		DispatchStartedCallback(StartedCallback, StartedResult);
	}

	// Record which FromLZCaptures / FromSketch source files this press consumed
	// (paths relative to Saved/). Written into the Press_## folder as capture_ref.json.
	{
		const FString ProcessedAt = FDateTime::Now().ToString(TEXT("%Y%m%d_%H%M%S"));
		FString RefJson;
		RefJson += TEXT("{\n");
		RefJson += FString::Printf(TEXT("  \"press\": \"%s\",\n"), *PressName);
		RefJson += FString::Printf(TEXT("  \"processed_at\": \"%s\",\n"), *ProcessedAt);
		RefJson += FString::Printf(TEXT("  \"has_capture\": %s,\n"), Source.bHasCapture ? TEXT("true") : TEXT("false"));
		RefJson += FString::Printf(TEXT("  \"capture_stem\": \"%s\",\n"), *Source.CaptureStem);
		RefJson += FString::Printf(TEXT("  \"capture_png\": \"%s\",\n"), *Source.CapturePngRel);
		RefJson += FString::Printf(TEXT("  \"capture_json\": \"%s\",\n"), *Source.CaptureJsonRel);
		RefJson += FString::Printf(TEXT("  \"faces_png\": \"%s\",\n"), *Source.FacesPngRel);
		RefJson += FString::Printf(TEXT("  \"faces_json\": \"%s\",\n"), *Source.FacesJsonRel);
		RefJson += FString::Printf(TEXT("  \"actor_material_png\": \"%s\",\n"), *Source.ActorMaterialPngRel);
		RefJson += FString::Printf(TEXT("  \"actor_material_json\": \"%s\",\n"), *Source.ActorMaterialJsonRel);
		RefJson += FString::Printf(TEXT("  \"sketch_png\": \"%s\"\n"), *Source.SketchPngRel);
		RefJson += TEXT("}\n");
		FFileHelper::SaveStringToFile(RefJson, *(PressDir / TEXT("capture_ref.json")));
	}

	const double StartTime = FPlatformTime::Seconds();

	// ---- Step 1: preprocess -------------------------------------------------
	// Non-white -> foreground (replaces grayscale+Gaussian+Otsu for the colored composite),
	// then remove small components without changing the source-stroke topology.
	TArray<uint8> Bin;
	FromLZImageOps::BinarizeNonWhite(RGBA, Width, Height, Params.Step1WhiteThreshold, Bin);
	FromLZImageOps::RemoveSmallComponents(Bin, Width, Height, Params.Step1MinArea);

	// 00_input is the raw composite (saved for reference), 01_binary is the cleaned mask.
	{
		TArray<uint8> InputRGBA = RGBA;
		IImageWrapperModule& IWM = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
		TSharedPtr<IImageWrapper> IW = IWM.CreateImageWrapper(EImageFormat::PNG);
		if (IW.IsValid())
		{
			IW->SetRaw(InputRGBA.GetData(), InputRGBA.Num(), Width, Height, ERGBFormat::RGBA, 8);
			const TArray64<uint8>& C = IW->GetCompressed();
			FFileHelper::SaveArrayToFile(TArrayView<const uint8>(C.GetData(), static_cast<int32>(C.Num())), *(PressDir / TEXT("00_input.png")));
		}
	}
	FromLZImageOps::SaveMaskPng(Bin, Width, Height, PressDir / TEXT("01_binary.png"), /*bInvertForDisplay*/ true);

	// ---- Step 2: skeletonize ------------------------------------------------
	TArray<uint8> Skel;
	FromLZImageOps::ZhangSuenThinning(Bin, Width, Height, Skel, Params.Step2ThinningMaxIter);
	FromLZImageOps::RemoveSmallComponents(Skel, Width, Height, Params.Step2SkeletonMinArea);
	FromLZImageOps::SaveMaskPng(Skel, Width, Height, PressDir / TEXT("02_skeleton.png"), /*bInvertForDisplay*/ true);

	// Per-pixel color-class map of the composite (red/green/blue/black/none).
	// Step 3 uses it to keep short dangling branches that still correspond to
	// source red/black marks; later steps reuse it for stroke color assignment.
	TArray<uint8> ColorMap;
	FromLZImageOps::BuildColorClassMap(
		RGBA,
		Width,
		Height,
		static_cast<uint8>(FMath::Clamp(Params.ColorWhiteCutoff, 0, 255)),
		Params.ColorDominanceMargin,
		ColorMap);
	const int32 ColorSampleRadius = Params.ColorSampleRadius;

	// ---- Step 3: skeleton gap repair ---------------------------------------
	// Run full-skeleton connection, red/black graph reconnect, full-graph backfill,
	// connector-aware small-loop pruning, and final short-branch cleanup.
	TArray<uint8> SkelConnected, SkelReconnected, SkelSmallLoopPruned, SkelClean;
	TArray<uint8> EffectiveColorMap;
	FromLZImageOps::CleanupSkeletonEndpoints(
		Skel, Width, Height,
		Params.Step3GapTol,
		Params.Step3ConnectThickness,
		Params.Step3SmallLoopBboxAreaThresh,
		Params.Step3BranchPruneMaxPixels,
		ColorMap,
		ColorSampleRadius,
		PressDir / TEXT("03a_red_black_connectors.png"),
		PressDir / TEXT("03a_red_black_reconnected.png"),
		PressDir / TEXT("03b_connector_prune_debug.json"),
		SkelConnected, SkelReconnected, SkelSmallLoopPruned, SkelClean, EffectiveColorMap);
	FromLZImageOps::SaveMaskPng(SkelConnected, Width, Height, PressDir / TEXT("03a_skeleton_connected.png"), /*bInvertForDisplay*/ true);
	FromLZImageOps::SaveMaskPng(SkelReconnected, Width, Height, PressDir / TEXT("03d_skeleton_reconnected.png"), /*bInvertForDisplay*/ true);
	FromLZImageOps::SaveMaskPng(SkelSmallLoopPruned, Width, Height, PressDir / TEXT("03b_skeleton_small_loop_pruned.png"), /*bInvertForDisplay*/ true);
	FromLZImageOps::SaveMaskPng(SkelClean, Width, Height, PressDir / TEXT("03_skeleton_clean.png"), /*bInvertForDisplay*/ true);

	// The color map also keeps RGB strokes from being merged together.
	const float EndpointTol = Params.Step4EndpointTol;
	const float ColorMinRunArc = Params.Step4ColorMinRunArc;

	// ---- Step 4: stroke tracing + color classification --------------------
	// Crossing-number node classification + 8-connected polyline tracing, then
	// assign red/green/blue/black per stroke and split at color boundaries
	// (gap-repair runs reclassified by neighbor color/direction).
	const int32 TraceMinPixels = Params.Step4TraceMinPixels;
	TArray<FromLZImageOps::FStroke> Strokes;
	FromLZImageOps::TraceStrokes(SkelClean, Width, Height, TraceMinPixels, Strokes);
	FromLZImageOps::SaveStrokesPng(Strokes, Width, Height, PressDir / TEXT("04_strokes.png"));

	TArray<FromLZImageOps::FColoredStroke> ColoredStrokes;
	FromLZImageOps::ColorizeAndSplitStrokes(Strokes, EffectiveColorMap, Width, Height, ColorSampleRadius, ColorMinRunArc, ColoredStrokes);
	FromLZImageOps::SaveColoredStrokesPng(ColoredStrokes, Width, Height, PressDir / TEXT("04_strokes_colored.png"));
	FromLZImageOps::SaveColoredStrokesJson(ColoredStrokes, Width, Height, PressDir / TEXT("04_strokes.json"), EndpointTol);

	// ---- Step 5: corner splitting (color preserved, RGB never merged) -----
	TArray<FromLZImageOps::FColoredStroke> SplitColored;
	FromLZImageOps::SplitColoredStrokesAtCorners(
		ColoredStrokes,
		Params.Step5AngleThresh,
		/*MinPixels*/ TraceMinPixels,
		Params.Step5SegmentArc,
		Params.Step5SplitPeakMinDistance,
		Params.Step5MaxIters,
		SplitColored);

	// Raw per-stroke palette render (each piece a distinct color), same style as 04_strokes.png.
	{
		TArray<FromLZImageOps::FStroke> SplitGeom;
		SplitGeom.Reserve(SplitColored.Num());
		for (const FromLZImageOps::FColoredStroke& CS : SplitColored)
		{
			SplitGeom.Add(CS.Points);
		}
		FromLZImageOps::SaveStrokesPng(SplitGeom, Width, Height, PressDir / TEXT("05_strokes_split.png"));
	}
	FromLZImageOps::SaveColoredStrokesPng(SplitColored, Width, Height, PressDir / TEXT("05_strokes_split_colored.png"));
	FromLZImageOps::SaveColoredStrokesJson(SplitColored, Width, Height, PressDir / TEXT("05_strokes_split.json"), EndpointTol);

	// Helper: emit the three standard outputs for a stroke set
	// (per-stroke palette PNG, class-color PNG, detailed JSON).
	auto SaveStrokeTriplet = [&](const TArray<FromLZImageOps::FColoredStroke>& Set, const FString& Stem)
	{
		TArray<FromLZImageOps::FStroke> Geom;
		Geom.Reserve(Set.Num());
		for (const FromLZImageOps::FColoredStroke& CS : Set)
		{
			Geom.Add(CS.Points);
		}
		FromLZImageOps::SaveStrokesPng(Geom, Width, Height, PressDir / (Stem + TEXT(".png")));
		FromLZImageOps::SaveColoredStrokesPng(Set, Width, Height, PressDir / (Stem + TEXT("_colored.png")));
		FromLZImageOps::SaveColoredStrokesJson(Set, Width, Height, PressDir / (Stem + TEXT(".json")), EndpointTol);
	};

	// ---- Step 6: same-color merge ----------------------------------------
	// Only strokes of the same red/green/blue/black class are merged.
	TArray<FromLZImageOps::FColoredStroke> Merged;
	FromLZImageOps::MergeColoredStrokesSameColor(
		SplitColored,
		Params.Step6MaxGap,
		Params.Step6MaxAngle,
		Params.Step6MaxIters,
		Params.Step6ProtectJunctionRadius,
		Merged);
	SaveStrokeTriplet(Merged, TEXT("06_merged"));

	// ---- Step 7: stroke metrics ------------------------------------------
	// arc / chord / straightness / PCA-line errors / direction; JSON gains kind + metrics.
	FromLZImageOps::ComputeStrokeMetrics(Merged);
	SaveStrokeTriplet(Merged, TEXT("07_stroke_info"));

	// ---- Step 8: enclosed-region mask ------------------------------------
	TArray<uint8> EnclosedMask, EnclosedBarrier;
	FromLZImageOps::ComputeEnclosedRegionMask(Merged, Width, Height, Params.Step8Thickness, EnclosedMask, EnclosedBarrier);
	FromLZImageOps::SaveMaskPng(EnclosedBarrier, Width, Height, PressDir / TEXT("08a_enclosed_barrier.png"), /*bInvertForDisplay*/ true);
	FromLZImageOps::SaveMaskPng(EnclosedMask, Width, Height, PressDir / TEXT("08_enclosed_mask.png"), /*bInvertForDisplay*/ true);

	// ---- Step 9: per-component red cap-loops -> per-component side -> translate-copy --
	// Real red/red and red/black intersections are planarized first. Exact red degree-1 endpoints
	// then run black-contact + global endpoint pairing, followed by a temporary mixed
	// graph and an independent real-red-segment/black-segment fallback pass within 20px.
	TArray<FromLZImageOps::FCapExtrusionResult> Caps;
	const int32 NumCaps = FromLZImageOps::RecoverCapExtrusionsPerComponent(
		Merged, Params.Step9ConnectorTol, Params.Step9BlackSelectTol, Width, Height, PressDir, ActionPressDir, Caps);

	// ---- Step 10/11: face validation -> solid rebuild -> runtime spawn/Boolean --
	if (!FFromLZSessionReset::IsSessionGenerationCurrent(SessionGeneration))
	{
		UE_LOG(LogTemp, Log, TEXT("Sketch2D: skipped stale Step10/11 reconstruction because the session generation changed during processing."));
		FFromLZPressProcessResult Result;
		Result.bSuccess = false;
		Result.Message = TEXT("Skipped stale Step10/11 reconstruction because the session generation changed during processing.");
		Result.PressDir = PressDir;
		Result.ActionPressDir = ActionPressDir;
		DispatchPressCallback(CompletionCallback, Result);
		return false;
	}
	FFromLZFaceReconstructor::ProcessPress(PressDir, ActionPressDir, World, SessionGeneration, MoveTemp(CompletionCallback));

	const double Elapsed = FPlatformTime::Seconds() - StartTime;
	UE_LOG(LogTemp, Log, TEXT("Sketch2D: steps 1-11 dispatched in %.3fs (%dx%d): %d merged strokes; %d cap component(s) -> %s"),
		Elapsed, Width, Height, Merged.Num(), NumCaps, *PressDir);

	return true;
}
