#include "BP_FromLZProcessSketchUtils.h"

#include "Engine/Engine.h"
#include "Engine/World.h"
#include "HAL/FileManager.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"

namespace
{
	struct FFromLZCaptureFileBundle
	{
		FString CaptureStem;
		FString CapturePngPath;
		FString CaptureJsonPath;
		FString FacesPngPath;
		FString FacesJsonPath;
		FString ActorMaterialPngPath;
		FString ActorMaterialJsonPath;
	};

	bool IsCaptureMainPngFilename(const FString& Filename)
	{
		const FString BaseName = FPaths::GetBaseFilename(Filename);
		if (FPaths::GetExtension(Filename, false) != TEXT("png") ||
			BaseName.Len() != 22 ||
			!BaseName.StartsWith(TEXT("FromLZ_"), ESearchCase::CaseSensitive) ||
			BaseName[15] != TCHAR('_'))
		{
			return false;
		}

		for (int32 Index = 7; Index < 15; ++Index)
		{
			if (!FChar::IsDigit(BaseName[Index]))
			{
				return false;
			}
		}
		for (int32 Index = 16; Index < 22; ++Index)
		{
			if (!FChar::IsDigit(BaseName[Index]))
			{
				return false;
			}
		}
		return true;
	}

	FString NormalizeInputPath(const FString& InPath)
	{
		if (InPath.IsEmpty())
		{
			return FString();
		}

		FString Candidate = InPath;
		FPaths::NormalizeFilename(Candidate);
		if (IFileManager::Get().FileExists(*Candidate) || IFileManager::Get().DirectoryExists(*Candidate))
		{
			return FPaths::ConvertRelativePathToFull(Candidate);
		}

		if (FPaths::IsRelative(Candidate))
		{
			const FString SavedCandidate = FPaths::ProjectSavedDir() / Candidate;
			if (IFileManager::Get().FileExists(*SavedCandidate) || IFileManager::Get().DirectoryExists(*SavedCandidate))
			{
				return FPaths::ConvertRelativePathToFull(SavedCandidate);
			}

			const FString ProjectCandidate = FPaths::ProjectDir() / Candidate;
			if (IFileManager::Get().FileExists(*ProjectCandidate) || IFileManager::Get().DirectoryExists(*ProjectCandidate))
			{
				return FPaths::ConvertRelativePathToFull(ProjectCandidate);
			}
		}

		return FPaths::ConvertRelativePathToFull(Candidate);
	}

	FString ResolveSketchFolder(const FString& InPath)
	{
		if (InPath.IsEmpty())
		{
			return FPaths::ProjectSavedDir() / TEXT("FromSketch");
		}

		FString Folder = InPath;
		FPaths::NormalizeFilename(Folder);
		if (!FPaths::IsRelative(Folder))
		{
			return FPaths::ConvertRelativePathToFull(Folder);
		}

		if (Folder.StartsWith(TEXT("Saved/"), ESearchCase::IgnoreCase))
		{
			return FPaths::ConvertRelativePathToFull(FPaths::ProjectDir() / Folder);
		}
		if (Folder.Equals(TEXT("FromSketch"), ESearchCase::IgnoreCase) ||
			Folder.StartsWith(TEXT("FromSketch/"), ESearchCase::IgnoreCase))
		{
			return FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / Folder);
		}
		return FPaths::ConvertRelativePathToFull(FPaths::ProjectDir() / Folder);
	}

	FString FindLatestPng(const FString& Directory)
	{
		TArray<FString> Filenames;
		IFileManager::Get().FindFiles(Filenames, *(Directory / TEXT("*.png")), true, false);
		if (Filenames.IsEmpty())
		{
			return FString();
		}

		FString LatestPath;
		FDateTime LatestTime = FDateTime::MinValue();
		for (const FString& Filename : Filenames)
		{
			const FString FullPath = Directory / Filename;
			const FFileStatData Stat = IFileManager::Get().GetStatData(*FullPath);
			const FString CurrentFilename = FPaths::GetCleanFilename(LatestPath);
			if (Stat.ModificationTime > LatestTime ||
				(Stat.ModificationTime == LatestTime &&
					Filename.Compare(CurrentFilename, ESearchCase::CaseSensitive) > 0))
			{
				LatestTime = Stat.ModificationTime;
				LatestPath = FullPath;
			}
		}
		return LatestPath;
	}

	bool DecodePngToRGBA(const FString& Path, TArray<uint8>& OutPixels, int32& OutWidth, int32& OutHeight)
	{
		OutPixels.Reset();
		OutWidth = 0;
		OutHeight = 0;

		TArray<uint8> RawFileData;
		if (!FFileHelper::LoadFileToArray(RawFileData, *Path))
		{
			return false;
		}

		IImageWrapperModule& ImageWrapperModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
		TSharedPtr<IImageWrapper> ImageWrapper = ImageWrapperModule.CreateImageWrapper(EImageFormat::PNG);
		if (!ImageWrapper.IsValid() || !ImageWrapper->SetCompressed(RawFileData.GetData(), RawFileData.Num()))
		{
			return false;
		}
		if (!ImageWrapper->GetRaw(ERGBFormat::RGBA, 8, OutPixels))
		{
			return false;
		}

		OutWidth = ImageWrapper->GetWidth();
		OutHeight = ImageWrapper->GetHeight();
		return true;
	}

	bool SaveRGBAToPng(const TArray<uint8>& RGBAPixels, int32 Width, int32 Height, const FString& OutputPath)
	{
		if (Width <= 0 || Height <= 0 || RGBAPixels.Num() < Width * Height * 4)
		{
			return false;
		}

		IFileManager::Get().MakeDirectory(*FPaths::GetPath(OutputPath), true);
		IImageWrapperModule& ImageWrapperModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
		TSharedPtr<IImageWrapper> OutputWrapper = ImageWrapperModule.CreateImageWrapper(EImageFormat::PNG);
		if (!OutputWrapper.IsValid())
		{
			return false;
		}

		OutputWrapper->SetRaw(RGBAPixels.GetData(), RGBAPixels.Num(), Width, Height, ERGBFormat::RGBA, 8);
		const TArray64<uint8>& CompressedData = OutputWrapper->GetCompressed();
		return FFileHelper::SaveArrayToFile(
			TArrayView<const uint8>(CompressedData.GetData(), static_cast<int32>(CompressedData.Num())),
			*OutputPath);
	}

	bool ExtractAndSaveChannel(
		const TArray<uint8>& RGBAPixels,
		int32 Width,
		int32 Height,
		uint8 TargetR,
		uint8 TargetG,
		uint8 TargetB,
		const FString& OutputPath)
	{
		TArray<uint8> OutputPixels;
		OutputPixels.SetNumUninitialized(RGBAPixels.Num());

		const int32 PixelCount = Width * Height;
		for (int32 i = 0; i < PixelCount; ++i)
		{
			const int32 Offset = i * 4;
			const uint8 R = RGBAPixels[Offset + 0];
			const uint8 G = RGBAPixels[Offset + 1];
			const uint8 B = RGBAPixels[Offset + 2];

			if (R == TargetR && G == TargetG && B == TargetB)
			{
				OutputPixels[Offset + 0] = R;
				OutputPixels[Offset + 1] = G;
				OutputPixels[Offset + 2] = B;
				OutputPixels[Offset + 3] = 255;
			}
			else
			{
				OutputPixels[Offset + 0] = 0;
				OutputPixels[Offset + 1] = 0;
				OutputPixels[Offset + 2] = 0;
				OutputPixels[Offset + 3] = 0;
			}
		}

		return SaveRGBAToPng(OutputPixels, Width, Height, OutputPath);
	}

	bool ParseCaptureOutputFiles(const TArray<FString>& OutputFiles, FFromLZCaptureFileBundle& OutBundle, FString& OutError)
	{
		OutBundle = FFromLZCaptureFileBundle();
		for (const FString& RawPath : OutputFiles)
		{
			const FString Path = NormalizeInputPath(RawPath);
			if (IsCaptureMainPngFilename(FPaths::GetCleanFilename(Path)) &&
				IFileManager::Get().FileExists(*Path))
			{
				OutBundle.CapturePngPath = Path;
				break;
			}
		}

		if (OutBundle.CapturePngPath.IsEmpty())
		{
			OutError = TEXT("CaptureOutputFiles does not contain a valid FromLZ_YYYYMMDD_HHMMSS.png main capture.");
			return false;
		}

		OutBundle.CaptureStem = FPaths::GetBaseFilename(OutBundle.CapturePngPath);
		const FString CaptureDir = FPaths::GetPath(OutBundle.CapturePngPath);
		const FString BasePath = CaptureDir / OutBundle.CaptureStem;
		OutBundle.CaptureJsonPath = BasePath + TEXT(".json");
		OutBundle.FacesPngPath = BasePath + TEXT("_faces.png");
		OutBundle.FacesJsonPath = BasePath + TEXT("_faces.json");
		OutBundle.ActorMaterialPngPath = BasePath + TEXT("_actor_material_id.png");
		OutBundle.ActorMaterialJsonPath = BasePath + TEXT("_actor_material_id.json");

		if (!IFileManager::Get().FileExists(*OutBundle.CaptureJsonPath) ||
			!IFileManager::Get().FileExists(*OutBundle.FacesPngPath) ||
			!IFileManager::Get().FileExists(*OutBundle.FacesJsonPath))
		{
			OutError = FString::Printf(TEXT("Capture file set is incomplete for stem %s."), *OutBundle.CaptureStem);
			return false;
		}

		return true;
	}

	void AddExistingFile(const FString& Path, TArray<FString>& OutFiles)
	{
		if (!Path.IsEmpty() && IFileManager::Get().FileExists(*Path))
		{
			OutFiles.AddUnique(Path);
		}
	}

	void CollectFilesRecursive(const FString& Directory, TArray<FString>& OutFiles)
	{
		if (Directory.IsEmpty() || !IFileManager::Get().DirectoryExists(*Directory))
		{
			return;
		}

		IFileManager::Get().IterateDirectoryRecursively(*Directory, [&OutFiles](const TCHAR* FilenameOrDirectory, bool bIsDirectory) -> bool
		{
			if (!bIsDirectory)
			{
				OutFiles.AddUnique(FString(FilenameOrDirectory));
			}
			return true;
		});
	}

	bool CopyFileBytes(const FString& SourcePath, const FString& DestPath)
	{
		TArray<uint8> Bytes;
		if (!FFileHelper::LoadFileToArray(Bytes, *SourcePath))
		{
			return false;
		}
		IFileManager::Get().MakeDirectory(*FPaths::GetPath(DestPath), true);
		return FFileHelper::SaveArrayToFile(Bytes, *DestPath);
	}
}

FFromLZProcessParams FBPFromLZProcessParams::ToCoreParams() const
{
	FFromLZProcessParams Params;
	Params.Step1WhiteThreshold = static_cast<uint8>(FMath::Clamp(Step1WhiteThreshold, 0, 255));
	Params.Step1MinArea = Step1MinArea;
	Params.Step2ThinningMaxIter = Step2ThinningMaxIter;
	Params.Step2SkeletonMinArea = Step2SkeletonMinArea;
	Params.ColorSampleRadius = ColorSampleRadius;
	Params.ColorWhiteCutoff = ColorWhiteCutoff;
	Params.ColorDominanceMargin = ColorDominanceMargin;
	Params.Step3GapTol = Step3GapTol;
	Params.Step3ConnectThickness = Step3ConnectThickness;
	Params.Step3SmallLoopBboxAreaThresh = Step3SmallLoopBboxAreaThresh;
	Params.Step3BranchPruneMaxPixels = Step3BranchPruneMaxPixels;
	Params.Step4EndpointTol = Step4EndpointTol;
	Params.Step4ColorMinRunArc = Step4ColorMinRunArc;
	Params.Step4TraceMinPixels = Step4TraceMinPixels;
	Params.Step5AngleThresh = Step5AngleThresh;
	Params.Step5SegmentArc = Step5SegmentArc;
	Params.Step5SplitPeakMinDistance = Step5SplitPeakMinDistance;
	Params.Step5MaxIters = Step5MaxIters;
	Params.Step6MaxGap = Step6MaxGap;
	Params.Step6MaxAngle = Step6MaxAngle;
	Params.Step6MaxIters = Step6MaxIters;
	Params.Step6ProtectJunctionRadius = Step6ProtectJunctionRadius;
	Params.Step8Thickness = Step8Thickness;
	Params.Step9ConnectorTol = Step9ConnectorTol;
	Params.Step9BlackSelectTol = Step9BlackSelectTol;
	Params.CapLoopGraphNodeSnapTol = CapLoopGraphNodeSnapTol;
	Params.CapRedOnlyLoopMinBboxArea = CapRedOnlyLoopMinBboxArea;
	Params.CapBorrowedLoopMinBboxArea = CapBorrowedLoopMinBboxArea;
	Params.CapMixedSelectionTimeBudgetSeconds = CapMixedSelectionTimeBudgetSeconds;
	Params.InteriorGreenMinInsideLengthPx = InteriorGreenMinInsideLengthPx;
	Params.GreenTraceEndpointTolPx = GreenTraceEndpointTolPx;
	Params.GreenTraceMaxChainDeviationDeg = GreenTraceMaxChainDeviationDeg;
	Params.CandidateFaceMinOverlapRatio = CandidateFaceMinOverlapRatio;
	Params.CandidateFaceMaxNormalSideAngleDegrees = CandidateFaceMaxNormalSideAngleDegrees;
	Params.CandidateFacePreferredNormalSideAngleDegrees = CandidateFacePreferredNormalSideAngleDegrees;
	Params.CompositeMaxWorkers = CompositeMaxWorkers;
	Params.ParallelForMaxThreads = ParallelForMaxThreads;
	return Params;
}

UBP_FromLZProcessSketchUtils* UBP_FromLZProcessSketchUtils::ProcessLatestSketchFromCapture(
	UObject* WorldContextObject,
	const TArray<FString>& CaptureOutputFiles,
	const FString& SketchFolderPath,
	FBPFromLZProcessParams ProcessParams)
{
	UBP_FromLZProcessSketchUtils* Action = NewObject<UBP_FromLZProcessSketchUtils>();
	Action->WorldContextObject = WorldContextObject;
	Action->CaptureOutputFiles = CaptureOutputFiles;
	Action->SketchFolderPath = SketchFolderPath;
	Action->ProcessParams = ProcessParams;
	if (WorldContextObject)
	{
		Action->RegisterWithGameInstance(WorldContextObject);
	}
	return Action;
}

void UBP_FromLZProcessSketchUtils::Activate()
{
	UWorld* World = GEngine
		? GEngine->GetWorldFromContextObject(WorldContextObject.Get(), EGetWorldErrorMode::ReturnNull)
		: nullptr;
	if (!World)
	{
		OnFailed.Broadcast(TEXT("World context is invalid."), TArray<FString>());
		SetReadyToDestroy();
		return;
	}

	FFromLZCaptureFileBundle CaptureBundle;
	FString CaptureError;
	if (!ParseCaptureOutputFiles(CaptureOutputFiles, CaptureBundle, CaptureError))
	{
		OnFailed.Broadcast(CaptureError, TArray<FString>());
		SetReadyToDestroy();
		return;
	}

	const FString ResolvedSketchFolder = ResolveSketchFolder(SketchFolderPath);
	const FString SketchPngPath = FindLatestPng(ResolvedSketchFolder);
	if (SketchPngPath.IsEmpty())
	{
		OnFailed.Broadcast(FString::Printf(TEXT("No PNG file found in %s."), *ResolvedSketchFolder), TArray<FString>());
		SetReadyToDestroy();
		return;
	}

	TArray<uint8> CompositeRGBA;
	int32 Width = 0;
	int32 Height = 0;
	if (!DecodePngToRGBA(SketchPngPath, CompositeRGBA, Width, Height) || Width <= 0 || Height <= 0)
	{
		OnFailed.Broadcast(FString::Printf(TEXT("Failed to decode sketch PNG: %s."), *SketchPngPath), TArray<FString>());
		SetReadyToDestroy();
		return;
	}
	for (int32 Index = 0; Index + 3 < CompositeRGBA.Num(); Index += 4)
	{
		CompositeRGBA[Index + 3] = 255;
	}

	ProcessDir = FPaths::ProjectSavedDir() / TEXT("FromProcess");
	IFileManager::Get().MakeDirectory(*ProcessDir, true);
	MainCompositePath = ProcessDir / TEXT("wContextSketch_raw.png");
	CompatibilityRedPath = ProcessDir / TEXT("Sketch_R.png");
	CompatibilityGreenPath = ProcessDir / TEXT("Sketch_G.png");
	CompatibilityBluePath = ProcessDir / TEXT("Sketch_B.png");

	TArray<FString> PartialFiles;
	if (!SaveRGBAToPng(CompositeRGBA, Width, Height, MainCompositePath))
	{
		OnFailed.Broadcast(FString::Printf(TEXT("Failed to save composite PNG: %s."), *MainCompositePath), PartialFiles);
		SetReadyToDestroy();
		return;
	}
	AddExistingFile(MainCompositePath, PartialFiles);

	const bool bSavedR = ExtractAndSaveChannel(CompositeRGBA, Width, Height, 255, 0, 0, CompatibilityRedPath);
	const bool bSavedG = ExtractAndSaveChannel(CompositeRGBA, Width, Height, 0, 255, 0, CompatibilityGreenPath);
	const bool bSavedB = ExtractAndSaveChannel(CompositeRGBA, Width, Height, 0, 0, 255, CompatibilityBluePath);
	AddExistingFile(CompatibilityRedPath, PartialFiles);
	AddExistingFile(CompatibilityGreenPath, PartialFiles);
	AddExistingFile(CompatibilityBluePath, PartialFiles);
	if (!bSavedR || !bSavedG || !bSavedB)
	{
		OnFailed.Broadcast(TEXT("Failed to save one or more RGB channel PNGs."), PartialFiles);
		SetReadyToDestroy();
		return;
	}

	FSketchSourceInfo Source;
	Source.bHasCapture = true;
	Source.CaptureStem = CaptureBundle.CaptureStem;
	Source.CapturePngRel = TEXT("FromLZCaptures/") + CaptureBundle.CaptureStem + TEXT(".png");
	Source.CaptureJsonRel = TEXT("FromLZCaptures/") + CaptureBundle.CaptureStem + TEXT(".json");
	Source.FacesPngRel = TEXT("FromLZCaptures/") + CaptureBundle.CaptureStem + TEXT("_faces.png");
	Source.FacesJsonRel = TEXT("FromLZCaptures/") + CaptureBundle.CaptureStem + TEXT("_faces.json");
	Source.ActorMaterialPngRel = TEXT("FromLZCaptures/") + CaptureBundle.CaptureStem + TEXT("_actor_material_id.png");
	Source.ActorMaterialJsonRel = TEXT("FromLZCaptures/") + CaptureBundle.CaptureStem + TEXT("_actor_material_id.json");
	Source.SketchPngRel = TEXT("FromSketch/") + FPaths::GetCleanFilename(SketchPngPath);

	const FString DebugDir = FPaths::ProjectSavedDir() / TEXT("2DDebug");
	TWeakObjectPtr<UBP_FromLZProcessSketchUtils> WeakThis(this);
	FFromLZSketch2DProcessor::ProcessCompositeAsync(
		MoveTemp(CompositeRGBA),
		Width,
		Height,
		DebugDir,
		Source,
		World,
		ProcessParams.ToCoreParams(),
		[WeakThis](const FFromLZPressProcessResult& Result)
		{
			if (WeakThis.IsValid())
			{
				WeakThis->OnStarted.Broadcast(Result.PressDir, Result.ActionPressDir);
			}
		},
		[WeakThis](const FFromLZPressProcessResult& Result)
		{
			if (!WeakThis.IsValid())
			{
				return;
			}

			TArray<FString> OutputFiles = Result.OutputFiles;
			AddExistingFile(WeakThis->MainCompositePath, OutputFiles);
			AddExistingFile(WeakThis->CompatibilityRedPath, OutputFiles);
			AddExistingFile(WeakThis->CompatibilityGreenPath, OutputFiles);
			AddExistingFile(WeakThis->CompatibilityBluePath, OutputFiles);

			const FString PressName = FPaths::GetCleanFilename(Result.PressDir);
			if (!PressName.IsEmpty())
			{
				const FString PressCompositePath = WeakThis->ProcessDir / (PressName + TEXT("_wContextSketch_raw.png"));
				const FString PressRedPath = WeakThis->ProcessDir / (PressName + TEXT("_Sketch_R.png"));
				const FString PressGreenPath = WeakThis->ProcessDir / (PressName + TEXT("_Sketch_G.png"));
				const FString PressBluePath = WeakThis->ProcessDir / (PressName + TEXT("_Sketch_B.png"));
				CopyFileBytes(WeakThis->MainCompositePath, PressCompositePath);
				CopyFileBytes(WeakThis->CompatibilityRedPath, PressRedPath);
				CopyFileBytes(WeakThis->CompatibilityGreenPath, PressGreenPath);
				CopyFileBytes(WeakThis->CompatibilityBluePath, PressBluePath);
				AddExistingFile(PressCompositePath, OutputFiles);
				AddExistingFile(PressRedPath, OutputFiles);
				AddExistingFile(PressGreenPath, OutputFiles);
				AddExistingFile(PressBluePath, OutputFiles);
			}

			CollectFilesRecursive(Result.PressDir, OutputFiles);
			CollectFilesRecursive(Result.ActionPressDir, OutputFiles);

			if (Result.bSuccess)
			{
				WeakThis->OnCompleted.Broadcast(OutputFiles, Result.PressDir, Result.ActionPressDir, WeakThis->MainCompositePath);
			}
			else
			{
				WeakThis->OnFailed.Broadcast(Result.Message, OutputFiles);
			}
			WeakThis->SetReadyToDestroy();
		});
}
