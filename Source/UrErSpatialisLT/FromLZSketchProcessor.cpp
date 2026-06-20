#include "FromLZSketchProcessor.h"

#include "FromLZSketch2DProcessor.h"
#include "HAL/FileManager.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"

namespace
{
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

	// Center-aligned crop/pad of an RGBA buffer to exactly (DstW x DstH). The image
	// center is preserved: an axis larger than the target is cropped equally on both
	// sides, an axis smaller is padded with white. No scaling is performed.
	void CenterFitRGBA(
		const TArray<uint8>& Src, int32 SrcW, int32 SrcH,
		int32 DstW, int32 DstH, TArray<uint8>& Out)
	{
		const int32 DstN = DstW * DstH;
		Out.SetNumUninitialized(DstN * 4);
		for (int32 i = 0; i < DstN; ++i)
		{
			Out[i * 4 + 0] = 255;
			Out[i * 4 + 1] = 255;
			Out[i * 4 + 2] = 255;
			Out[i * 4 + 3] = 255;
		}

		// Source pixel that maps to output (0,0); centers coincide.
		const int32 OffX = (SrcW - DstW) / 2; // >0 -> crop, <0 -> pad
		const int32 OffY = (SrcH - DstH) / 2;
		for (int32 oy = 0; oy < DstH; ++oy)
		{
			const int32 sy = oy + OffY;
			if (sy < 0 || sy >= SrcH)
			{
				continue;
			}
			for (int32 ox = 0; ox < DstW; ++ox)
			{
				const int32 sx = ox + OffX;
				if (sx < 0 || sx >= SrcW)
				{
					continue;
				}
				const int32 si = (sy * SrcW + sx) * 4;
				const int32 di = (oy * DstW + ox) * 4;
				Out[di + 0] = Src[si + 0];
				Out[di + 1] = Src[si + 1];
				Out[di + 2] = Src[si + 2];
				Out[di + 3] = Src[si + 3];
			}
		}
	}
}

void FFromLZSketchProcessor::ProcessLatestSketch(UWorld* World)
{
	const FString SketchDir = FPaths::ProjectSavedDir() / TEXT("FromSketch");
	const FString CaptureDir = FPaths::ProjectSavedDir() / TEXT("FromLZCaptures");
	const FString SketchPng = FindLatestPng(SketchDir);
	if (SketchPng.IsEmpty())
	{
		UE_LOG(LogTemp, Warning, TEXT("ProcessSketch: No PNG found in %s"), *SketchDir);
		return;
	}

	const FString CapturePng = FindLatestPng(CaptureDir, /*bRequireCaptureMainPng*/ true);
	ProcessSketch(World, SketchPng, CapturePng);
}

void FFromLZSketchProcessor::ProcessSketch(UWorld* World, const FString& SketchPng, const FString& CapturePng)
{
	const FString CaptureDir = FPaths::ProjectSavedDir() / TEXT("FromLZCaptures");
	const FString ProcessDir = FPaths::ProjectSavedDir() / TEXT("FromProcess");

	IFileManager::Get().MakeDirectory(*ProcessDir, true);

	if (SketchPng.IsEmpty())
	{
		UE_LOG(LogTemp, Warning, TEXT("ProcessSketch: sketch path is empty."));
		return;
	}

	// 1) Hand sketch explicitly paired with the board's source capture, or the
	// latest sketch when ProcessLatestSketch is used without an open board.
	TArray<uint8> SketchPixels;
	int32 SW = 0;
	int32 SH = 0;
	if (!DecodePngToRGBA(SketchPng, SketchPixels, SW, SH) || SW <= 0 || SH <= 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("ProcessSketch: Failed to decode sketch %s"), *SketchPng);
		return;
	}
	UE_LOG(LogTemp, Log, TEXT("ProcessSketch: sketch=%s (%dx%d)"), *SketchPng, SW, SH);

	// 2) Captured line-art explicitly paired with this sketch.
	TArray<uint8> CapturePixels;
	int32 CW = 0;
	int32 CH = 0;
	const bool bHasCapture = !CapturePng.IsEmpty() && DecodePngToRGBA(CapturePng, CapturePixels, CW, CH) && CW > 0 && CH > 0;
	if (!CapturePng.IsEmpty() && !bHasCapture)
	{
		UE_LOG(LogTemp, Warning, TEXT("ProcessSketch: explicitly paired capture is unavailable or invalid: %s"), *CapturePng);
		return;
	}
	if (bHasCapture)
	{
		const FString CaptureStem = FPaths::GetBaseFilename(CapturePng);
		const FString CaptureDirectory = FPaths::GetPath(CapturePng);
		const FString CaptureJson = CaptureDirectory / (CaptureStem + TEXT(".json"));
		const FString FacesPng = CaptureDirectory / (CaptureStem + TEXT("_faces.png"));
		const FString FacesJson = CaptureDirectory / (CaptureStem + TEXT("_faces.json"));
		if (!IFileManager::Get().FileExists(*CaptureJson) ||
			!IFileManager::Get().FileExists(*FacesPng) ||
			!IFileManager::Get().FileExists(*FacesJson))
		{
			UE_LOG(
				LogTemp,
				Warning,
				TEXT("ProcessSketch: paired capture set is incomplete and will not be replaced by a newer capture: %s"),
				*CaptureStem);
			return;
		}
		UE_LOG(LogTemp, Log, TEXT("ProcessSketch: capture=%s (%dx%d)"), *CapturePng, CW, CH);
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("ProcessSketch: no usable PNG in %s; composite will contain sketch lines only."), *CaptureDir);
	}

	// 2b) Fit the sketch to the camera render resolution (the capture/faces resolution)
	//     by a center-preserving crop/pad (no scaling), so the composite, cap polygons,
	//     faces image and camera projection all share one pixel coordinate frame.
	if (bHasCapture)
	{
		if (SW != CW || SH != CH)
		{
			TArray<uint8> Fitted;
			CenterFitRGBA(SketchPixels, SW, SH, CW, CH, Fitted);
			SketchPixels = MoveTemp(Fitted);
			UE_LOG(LogTemp, Log, TEXT("ProcessSketch: center-fit sketch %dx%d -> render %dx%d"), SW, SH, CW, CH);
			SW = CW;
			SH = CH;
		}
		SaveRGBAToPng(SketchPixels, SW, SH, ProcessDir / TEXT("Sketch_fitted.png"));
	}

	// 3) Composite onto a clean white canvas in the active processing resolution:
	//      - non-white sketch pixels (the red/green/blue lines) are copied as-is
	//      - elsewhere, black capture pixels are drawn black
	//    Result: white background with black + red + green + blue lines.
	TArray<uint8> Composite;
	Composite.SetNumUninitialized(SW * SH * 4);

	for (int32 y = 0; y < SH; ++y)
	{
		for (int32 x = 0; x < SW; ++x)
		{
			const int32 Idx = (y * SW + x) * 4;
			const uint8 sr = SketchPixels[Idx + 0];
			const uint8 sg = SketchPixels[Idx + 1];
			const uint8 sb = SketchPixels[Idx + 2];

			uint8 r = 255;
			uint8 g = 255;
			uint8 b = 255;

			const bool bSketchMark = !(sr > 240 && sg > 240 && sb > 240);
			if (bSketchMark)
			{
				r = sr;
				g = sg;
				b = sb;
			}
			else if (bHasCapture)
			{
				const int32 cx = FMath::Clamp((x * CW) / SW, 0, CW - 1);
				const int32 cy = FMath::Clamp((y * CH) / SH, 0, CH - 1);
				const int32 CIdx = (cy * CW + cx) * 4;
				const bool bCaptureLine = CapturePixels[CIdx + 0] < 128 && CapturePixels[CIdx + 1] < 128 && CapturePixels[CIdx + 2] < 128;
				if (bCaptureLine)
				{
					r = 0;
					g = 0;
					b = 0;
				}
			}

			Composite[Idx + 0] = r;
			Composite[Idx + 1] = g;
			Composite[Idx + 2] = b;
			Composite[Idx + 3] = 255;
		}
	}

	const FString CompositePath = ProcessDir / TEXT("wContextSketch_raw.png");
	if (SaveRGBAToPng(Composite, SW, SH, CompositePath))
	{
		UE_LOG(LogTemp, Log, TEXT("ProcessSketch: saved composite %s"), *CompositePath);
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("ProcessSketch: failed to save composite %s"), *CompositePath);
	}

	// Run the migrated sketch-analysis/reconstruction dispatch pipeline on the
	// composite off the game thread. Debug artifacts go to <ProjectSaved>/2DDebug/.
	{
		const FString TwoDDebugDir = FPaths::ProjectSavedDir() / TEXT("2DDebug");

		// Record which source files this press consumed (paths relative to Saved/).
		FSketchSourceInfo Source;
		Source.bHasCapture = bHasCapture;
		Source.SketchPngRel = TEXT("FromSketch/") + FPaths::GetCleanFilename(SketchPng);
		if (bHasCapture)
		{
			FString CaptureStem = FPaths::GetBaseFilename(CapturePng); // FromLZ_<timestamp>
			Source.CaptureStem = CaptureStem;
			Source.CapturePngRel = TEXT("FromLZCaptures/") + CaptureStem + TEXT(".png");
			Source.CaptureJsonRel = TEXT("FromLZCaptures/") + CaptureStem + TEXT(".json");
			Source.FacesPngRel = TEXT("FromLZCaptures/") + CaptureStem + TEXT("_faces.png");
			Source.FacesJsonRel = TEXT("FromLZCaptures/") + CaptureStem + TEXT("_faces.json");
			Source.ActorMaterialPngRel = TEXT("FromLZCaptures/") + CaptureStem + TEXT("_actor_material_id.png");
			Source.ActorMaterialJsonRel = TEXT("FromLZCaptures/") + CaptureStem + TEXT("_actor_material_id.json");
		}

		FFromLZSketch2DProcessor::ProcessCompositeAsync(Composite, SW, SH, TwoDDebugDir, Source, World);
	}

	// 4) Continue with red/green/blue line decomposition, now from the composite.
	const bool bR = ExtractAndSaveChannel(Composite, SW, SH, 255, 0, 0, ProcessDir / TEXT("Sketch_R.png"));
	const bool bG = ExtractAndSaveChannel(Composite, SW, SH, 0, 255, 0, ProcessDir / TEXT("Sketch_G.png"));
	const bool bB = ExtractAndSaveChannel(Composite, SW, SH, 0, 0, 255, ProcessDir / TEXT("Sketch_B.png"));

	if (bR && bG && bB)
	{
		UE_LOG(LogTemp, Log, TEXT("ProcessSketch: Saved Sketch_R.png, Sketch_G.png, Sketch_B.png to %s"), *ProcessDir);
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("ProcessSketch: One or more channel files failed to save (R=%d G=%d B=%d)"), bR, bG, bB);
	}
}

bool FFromLZSketchProcessor::DecodePngToRGBA(const FString& Path, TArray<uint8>& OutPixels, int32& OutWidth, int32& OutHeight)
{
	OutWidth = 0;
	OutHeight = 0;
	OutPixels.Reset();

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

bool FFromLZSketchProcessor::SaveRGBAToPng(const TArray<uint8>& RGBAPixels, int32 Width, int32 Height, const FString& OutputPath)
{
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
		*OutputPath
	);
}

FString FFromLZSketchProcessor::FindLatestPng(const FString& Directory, bool bRequireCaptureMainPng)
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
		if (bRequireCaptureMainPng && !IsCaptureMainPngFilename(Filename))
		{
			continue;
		}

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

bool FFromLZSketchProcessor::ExtractAndSaveChannel(const TArray<uint8>& RGBAPixels, int32 Width, int32 Height, uint8 TargetR, uint8 TargetG, uint8 TargetB, const FString& OutputPath)
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

	IImageWrapperModule& ImageWrapperModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
	TSharedPtr<IImageWrapper> OutputWrapper = ImageWrapperModule.CreateImageWrapper(EImageFormat::PNG);

	if (!OutputWrapper.IsValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("ProcessSketch: Failed to create image wrapper for %s"), *OutputPath);
		return false;
	}

	OutputWrapper->SetRaw(OutputPixels.GetData(), OutputPixels.Num(), Width, Height, ERGBFormat::RGBA, 8);

	const TArray64<uint8>& CompressedData = OutputWrapper->GetCompressed();
	return FFileHelper::SaveArrayToFile(
		TArrayView<const uint8>(CompressedData.GetData(), static_cast<int32>(CompressedData.Num())),
		*OutputPath
	);
}
