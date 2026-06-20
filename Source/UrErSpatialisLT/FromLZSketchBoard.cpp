#include "FromLZSketchBoard.h"

#include "FromLZFaceReconstructor.h"
#include "FromLZPressNaming.h"
#include "FromLZSketchProcessor.h"
#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "Engine/Texture2D.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "HAL/FileManager.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Input/Events.h"
#include "InputCoreTypes.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"
#include "Rendering/DrawElements.h"
#include "Styling/CoreStyle.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/SCompoundWidget.h"
#include "Widgets/SLeafWidget.h"
#include "Widgets/SOverlay.h"
#include "Widgets/Images/SImage.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Text/STextBlock.h"

namespace
{
	enum class ESketchBoardTool : uint8
	{
		Red,
		Green,
		Blue,
		Eraser
	};

	struct FSketchBoardFit
	{
		FVector2D TopLeft = FVector2D::ZeroVector;
		FVector2D Size = FVector2D::ZeroVector;
		double Scale = 1.0;
	};

	class FSketchBoardState : public TSharedFromThis<FSketchBoardState>
	{
	public:
		~FSketchBoardState()
		{
			if (CompositeTexture)
			{
				CompositeTexture->RemoveFromRoot();
				CompositeTexture = nullptr;
			}
		}

		bool LoadCapture(const FString& InCapturePngPath)
		{
			CapturePngPath = InCapturePngPath;
			if (!DecodePngToRGBA(CapturePngPath, BackgroundRGBA, Width, Height))
			{
				UE_LOG(LogTemp, Warning, TEXT("SketchBoard: failed to decode capture png %s"), *CapturePngPath);
				return false;
			}
			if (Width <= 0 || Height <= 0 || BackgroundRGBA.Num() != Width * Height * 4)
			{
				UE_LOG(LogTemp, Warning, TEXT("SketchBoard: invalid capture image %s (%dx%d, bytes=%d)"), *CapturePngPath, Width, Height, BackgroundRGBA.Num());
				return false;
			}

			DrawingRGBA.SetNumUninitialized(BackgroundRGBA.Num());
			for (int32 i = 0; i < Width * Height; ++i)
			{
				const int32 Off = i * 4;
				DrawingRGBA[Off + 0] = 255;
				DrawingRGBA[Off + 1] = 255;
				DrawingRGBA[Off + 2] = 255;
				DrawingRGBA[Off + 3] = 255;
			}

			CompositeBGRA.SetNumUninitialized(BackgroundRGBA.Num());
			CompositeTexture = UTexture2D::CreateTransient(Width, Height, PF_B8G8R8A8);
			if (!CompositeTexture)
			{
				UE_LOG(LogTemp, Warning, TEXT("SketchBoard: failed to create transient texture (%dx%d)"), Width, Height);
				return false;
			}
			CompositeTexture->SRGB = true;
			CompositeTexture->Filter = TF_Bilinear;
			CompositeTexture->AddToRoot();
			Brush.SetResourceObject(CompositeTexture);
			Brush.ImageSize = FVector2D(Width, Height);
			UpdateCompositeTexture();
			return true;
		}

		FVector2D GetImageSize() const
		{
			return FVector2D(Width, Height);
		}

		FSketchBoardFit FitTo(const FVector2D& AvailableSize) const
		{
			FSketchBoardFit Fit;
			if (Width <= 0 || Height <= 0 || AvailableSize.X <= 0.0 || AvailableSize.Y <= 0.0)
			{
				return Fit;
			}

			Fit.Scale = FMath::Min(AvailableSize.X / double(Width), AvailableSize.Y / double(Height));
			Fit.Scale = FMath::Max(0.001, Fit.Scale);
			Fit.Size = FVector2D(double(Width) * Fit.Scale, double(Height) * Fit.Scale);
			Fit.TopLeft = (AvailableSize - Fit.Size) * 0.5;
			return Fit;
		}

		bool LocalToImagePixel(const FVector2D& LocalPos, const FVector2D& AvailableSize, FIntPoint& OutPixel) const
		{
			const FSketchBoardFit Fit = FitTo(AvailableSize);
			if (Fit.Size.X <= 0.0 || Fit.Size.Y <= 0.0)
			{
				return false;
			}

			const FVector2D Rel = LocalPos - Fit.TopLeft;
			if (Rel.X < 0.0 || Rel.Y < 0.0 || Rel.X >= Fit.Size.X || Rel.Y >= Fit.Size.Y)
			{
				return false;
			}

			const int32 X = FMath::Clamp(FMath::FloorToInt(Rel.X / Fit.Scale), 0, Width - 1);
			const int32 Y = FMath::Clamp(FMath::FloorToInt(Rel.Y / Fit.Scale), 0, Height - 1);
			OutPixel = FIntPoint(X, Y);
			return true;
		}

		void Paint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect, FSlateWindowElementList& OutDrawElements, int32 LayerId) const
		{
			const FSketchBoardFit Fit = FitTo(AllottedGeometry.GetLocalSize());
			if (Fit.Size.X <= 0.0 || Fit.Size.Y <= 0.0)
			{
				return;
			}

			FSlateDrawElement::MakeBox(
				OutDrawElements,
				LayerId,
				AllottedGeometry.ToPaintGeometry(Fit.Size, FSlateLayoutTransform(Fit.TopLeft)),
				&Brush,
				ESlateDrawEffect::None,
				FLinearColor::White);
		}

		void SetTool(ESketchBoardTool InTool)
		{
			Tool = InTool;
		}

		void ClearDrawing()
		{
			for (int32 i = 0; i < Width * Height; ++i)
			{
				const int32 Off = i * 4;
				DrawingRGBA[Off + 0] = 255;
				DrawingRGBA[Off + 1] = 255;
				DrawingRGBA[Off + 2] = 255;
				DrawingRGBA[Off + 3] = 255;
			}
			UpdateCompositeTexture();
		}

		void DrawLine(const FIntPoint& A, const FIntPoint& B)
		{
			const int32 Dx = FMath::Abs(B.X - A.X);
			const int32 Dy = FMath::Abs(B.Y - A.Y);
			const int32 Steps = FMath::Max(Dx, Dy);
			if (Steps <= 0)
			{
				Stamp(A);
				UpdateCompositeTexture();
				return;
			}

			for (int32 i = 0; i <= Steps; ++i)
			{
				const double T = double(i) / double(Steps);
				const int32 X = FMath::RoundToInt(FMath::Lerp(double(A.X), double(B.X), T));
				const int32 Y = FMath::RoundToInt(FMath::Lerp(double(A.Y), double(B.Y), T));
				Stamp(FIntPoint(X, Y));
			}
			UpdateCompositeTexture();
		}

		bool SaveSketchForNextPress(FString& OutPath)
		{
			TArray<uint8> CompositeRGBA;
			BuildCompositeRGBA(CompositeRGBA);

			const FString TwoDDebugDir = FPaths::ProjectSavedDir() / TEXT("2DDebug");
			const int32 PressIndex = FFromLZPressNaming::GetNextPressIndex(TwoDDebugDir);
			const FString SketchDir = FPaths::ProjectSavedDir() / TEXT("FromSketch");
			IFileManager::Get().MakeDirectory(*SketchDir, true);
			OutPath = SketchDir / FString::Printf(TEXT("Sketch_%02d.png"), PressIndex);
			return SaveRGBAToPng(CompositeRGBA, Width, Height, OutPath);
		}

		const FString& GetCapturePngPath() const
		{
			return CapturePngPath;
		}

	private:
		bool DecodePngToRGBA(const FString& Path, TArray<uint8>& OutPixels, int32& OutWidth, int32& OutHeight)
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

		bool SaveRGBAToPng(const TArray<uint8>& RGBAPixels, int32 InWidth, int32 InHeight, const FString& OutputPath)
		{
			IImageWrapperModule& ImageWrapperModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
			TSharedPtr<IImageWrapper> OutputWrapper = ImageWrapperModule.CreateImageWrapper(EImageFormat::PNG);
			if (!OutputWrapper.IsValid())
			{
				return false;
			}

			OutputWrapper->SetRaw(RGBAPixels.GetData(), RGBAPixels.Num(), InWidth, InHeight, ERGBFormat::RGBA, 8);
			const TArray64<uint8>& CompressedData = OutputWrapper->GetCompressed();
			return FFileHelper::SaveArrayToFile(
				TArrayView<const uint8>(CompressedData.GetData(), static_cast<int32>(CompressedData.Num())),
				*OutputPath);
		}

		void Stamp(const FIntPoint& P)
		{
			const int32 Radius = Tool == ESketchBoardTool::Eraser ? 10 : 1;
			uint8 R = 255;
			uint8 G = 255;
			uint8 B = 255;
			if (Tool == ESketchBoardTool::Red)
			{
				G = 0;
				B = 0;
			}
			else if (Tool == ESketchBoardTool::Green)
			{
				R = 0;
				B = 0;
			}
			else if (Tool == ESketchBoardTool::Blue)
			{
				R = 0;
				G = 0;
			}

			const int32 R2 = Radius * Radius;
			for (int32 Y = P.Y - Radius; Y <= P.Y + Radius; ++Y)
			{
				for (int32 X = P.X - Radius; X <= P.X + Radius; ++X)
				{
					if (X < 0 || X >= Width || Y < 0 || Y >= Height)
					{
						continue;
					}
					const int32 Ox = X - P.X;
					const int32 Oy = Y - P.Y;
					if (Ox * Ox + Oy * Oy > R2)
					{
						continue;
					}
					const int32 Off = (Y * Width + X) * 4;
					DrawingRGBA[Off + 0] = R;
					DrawingRGBA[Off + 1] = G;
					DrawingRGBA[Off + 2] = B;
					DrawingRGBA[Off + 3] = 255;
				}
			}
		}

		void BuildCompositeRGBA(TArray<uint8>& OutComposite) const
		{
			OutComposite.SetNumUninitialized(BackgroundRGBA.Num());
			for (int32 i = 0; i < Width * Height; ++i)
			{
				const int32 Off = i * 4;
				const bool bHasDrawing =
					DrawingRGBA[Off + 0] != 255 ||
					DrawingRGBA[Off + 1] != 255 ||
					DrawingRGBA[Off + 2] != 255;
				const TArray<uint8>& Src = bHasDrawing ? DrawingRGBA : BackgroundRGBA;
				OutComposite[Off + 0] = Src[Off + 0];
				OutComposite[Off + 1] = Src[Off + 1];
				OutComposite[Off + 2] = Src[Off + 2];
				OutComposite[Off + 3] = 255;
			}
		}

		void UpdateCompositeTexture()
		{
			if (!CompositeTexture || BackgroundRGBA.Num() != Width * Height * 4 || DrawingRGBA.Num() != BackgroundRGBA.Num())
			{
				return;
			}

			for (int32 i = 0; i < Width * Height; ++i)
			{
				const int32 Off = i * 4;
				const bool bHasDrawing =
					DrawingRGBA[Off + 0] != 255 ||
					DrawingRGBA[Off + 1] != 255 ||
					DrawingRGBA[Off + 2] != 255;
				const TArray<uint8>& Src = bHasDrawing ? DrawingRGBA : BackgroundRGBA;
				CompositeBGRA[Off + 0] = Src[Off + 2];
				CompositeBGRA[Off + 1] = Src[Off + 1];
				CompositeBGRA[Off + 2] = Src[Off + 0];
				CompositeBGRA[Off + 3] = 255;
			}

			FTexturePlatformData* PlatformData = CompositeTexture->GetPlatformData();
			if (!PlatformData || PlatformData->Mips.Num() == 0)
			{
				return;
			}

			FTexture2DMipMap& Mip = PlatformData->Mips[0];
			void* Data = Mip.BulkData.Lock(LOCK_READ_WRITE);
			if (Data)
			{
				FMemory::Memcpy(Data, CompositeBGRA.GetData(), CompositeBGRA.Num());
			}
			Mip.BulkData.Unlock();
			CompositeTexture->UpdateResource();
		}

		FString CapturePngPath;
		int32 Width = 0;
		int32 Height = 0;
		TArray<uint8> BackgroundRGBA;
		TArray<uint8> DrawingRGBA;
		TArray<uint8> CompositeBGRA;
		UTexture2D* CompositeTexture = nullptr;
		FSlateBrush Brush;
		ESketchBoardTool Tool = ESketchBoardTool::Red;
	};

	class SFromLZSketchCanvas : public SLeafWidget
	{
	public:
		SLATE_BEGIN_ARGS(SFromLZSketchCanvas) {}
			SLATE_ARGUMENT(TSharedPtr<FSketchBoardState>, State)
		SLATE_END_ARGS()

		void Construct(const FArguments& InArgs)
		{
			State = InArgs._State;
		}

		virtual FVector2D ComputeDesiredSize(float LayoutScaleMultiplier) const override
		{
			return State.IsValid() ? State->GetImageSize() : FVector2D(512.0, 512.0);
		}

		virtual int32 OnPaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect, FSlateWindowElementList& OutDrawElements, int32 LayerId, const FWidgetStyle& InWidgetStyle, bool bParentEnabled) const override
		{
			if (State.IsValid())
			{
				State->Paint(Args, AllottedGeometry, MyCullingRect, OutDrawElements, LayerId);
			}
			return LayerId + 1;
		}

		virtual FReply OnMouseButtonDown(const FGeometry& MyGeometry, const FPointerEvent& MouseEvent) override
		{
			if (!State.IsValid() || MouseEvent.GetEffectingButton() != EKeys::LeftMouseButton)
			{
				return FReply::Unhandled();
			}

			FIntPoint Pixel;
			if (!State->LocalToImagePixel(MyGeometry.AbsoluteToLocal(MouseEvent.GetScreenSpacePosition()), MyGeometry.GetLocalSize(), Pixel))
			{
				return FReply::Handled();
			}

			bDrawing = true;
			LastPixel = Pixel;
			State->DrawLine(Pixel, Pixel);
			Invalidate(EInvalidateWidgetReason::Paint);
			return FReply::Handled().CaptureMouse(AsShared());
		}

		virtual FReply OnMouseMove(const FGeometry& MyGeometry, const FPointerEvent& MouseEvent) override
		{
			if (!bDrawing || !State.IsValid())
			{
				return FReply::Unhandled();
			}

			FIntPoint Pixel;
			if (State->LocalToImagePixel(MyGeometry.AbsoluteToLocal(MouseEvent.GetScreenSpacePosition()), MyGeometry.GetLocalSize(), Pixel))
			{
				State->DrawLine(LastPixel, Pixel);
				LastPixel = Pixel;
				Invalidate(EInvalidateWidgetReason::Paint);
			}
			return FReply::Handled();
		}

		virtual FReply OnMouseButtonUp(const FGeometry& MyGeometry, const FPointerEvent& MouseEvent) override
		{
			if (MouseEvent.GetEffectingButton() != EKeys::LeftMouseButton)
			{
				return FReply::Unhandled();
			}

			bDrawing = false;
			return FReply::Handled().ReleaseMouseCapture();
		}

	private:
		TSharedPtr<FSketchBoardState> State;
		bool bDrawing = false;
		FIntPoint LastPixel = FIntPoint::ZeroValue;
	};

	TSharedPtr<FSketchBoardState> GSketchBoardState;
	TSharedPtr<SWidget> GSketchBoardWidget;
	TWeakObjectPtr<UWorld> GSketchBoardWorld;
	TWeakObjectPtr<UGameViewportClient> GSketchBoardViewportClient;
	TWeakObjectPtr<APlayerController> GSketchBoardPlayerController;
	bool bSketchBoardMinimized = false;
	bool bHadPreviousCursor = false;
	bool bPreviousCursorVisible = false;

	class SFromLZSketchBoardWidget : public SCompoundWidget
	{
	public:
		SLATE_BEGIN_ARGS(SFromLZSketchBoardWidget) {}
			SLATE_ARGUMENT(TSharedPtr<FSketchBoardState>, State)
		SLATE_END_ARGS()

		void Construct(const FArguments& InArgs)
		{
			State = InArgs._State;

			ChildSlot
			[
				SNew(SBorder)
				.Padding(FMargin(48.0f))
				.BorderBackgroundColor(FLinearColor(0.0f, 0.0f, 0.0f, 0.35f))
				[
					SNew(SHorizontalBox)
					+ SHorizontalBox::Slot()
					.FillWidth(1.0f)
					[
						SNew(SBorder)
						.Padding(0.0f)
						.BorderBackgroundColor(FLinearColor(0.03f, 0.03f, 0.03f, 1.0f))
						[
							SNew(SFromLZSketchCanvas)
							.State(State)
						]
					]
					+ SHorizontalBox::Slot()
					.AutoWidth()
					.Padding(FMargin(12.0f, 0.0f, 0.0f, 0.0f))
					[
						SNew(SBorder)
						.Padding(8.0f)
						.BorderBackgroundColor(FLinearColor(0.08f, 0.08f, 0.08f, 0.96f))
						[
							SNew(SVerticalBox)
							+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 3.0f) [MakeButton(TEXT("Red"), FLinearColor(1.0f, 0.0f, 0.0f), [this]() { State->SetTool(ESketchBoardTool::Red); })]
							+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 3.0f) [MakeButton(TEXT("Green"), FLinearColor(0.0f, 1.0f, 0.0f), [this]() { State->SetTool(ESketchBoardTool::Green); })]
							+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 3.0f) [MakeButton(TEXT("Blue"), FLinearColor(0.0f, 0.0f, 1.0f), [this]() { State->SetTool(ESketchBoardTool::Blue); })]
							+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 3.0f) [MakeButton(TEXT("Eraser"), FLinearColor::White, [this]() { State->SetTool(ESketchBoardTool::Eraser); })]
							+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 3.0f) [MakeButton(TEXT("Clear"), FLinearColor(0.8f, 0.8f, 0.8f), [this]() { State->ClearDrawing(); })]
							+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 3.0f) [MakeButton(TEXT("Proceed"), FLinearColor(0.3f, 0.8f, 1.0f), []() { FFromLZSketchBoard::SaveCurrentSketchAndProceed(nullptr); })]
							+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 3.0f) [MakeButton(TEXT("Undo"), FLinearColor(1.0f, 0.8f, 0.2f), []() { FFromLZFaceReconstructor::RestoreStep11RuntimeBooleans(GSketchBoardWorld.Get()); })]
							+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 3.0f) [MakeButton(TEXT("Min"), FLinearColor(0.8f, 0.8f, 0.8f), []() { FFromLZSketchBoard::Minimize(); })]
							+ SVerticalBox::Slot().AutoHeight().Padding(0.0f, 3.0f) [MakeButton(TEXT("Close"), FLinearColor(1.0f, 0.3f, 0.3f), []() { FFromLZSketchBoard::Close(); })]
						]
					]
				]
			];
		}

	private:
		static TSharedRef<SWidget> MakeButton(const TCHAR* Label, const FLinearColor& Color, TFunction<void()> OnClicked)
		{
			return SNew(SButton)
				.ContentPadding(FMargin(8.0f, 6.0f))
				.OnClicked_Lambda([OnClicked = MoveTemp(OnClicked)]()
				{
					OnClicked();
					return FReply::Handled();
				})
				[
					SNew(STextBlock)
					.Text(FText::FromString(Label))
					.ColorAndOpacity(Color)
				];
		}

		TSharedPtr<FSketchBoardState> State;
	};

	void SetBoardCursorVisible(UWorld* World, bool bVisible)
	{
		APlayerController* PlayerController = World ? World->GetFirstPlayerController() : nullptr;
		if (!PlayerController)
		{
			return;
		}

		if (bVisible)
		{
			if (!GSketchBoardPlayerController.IsValid())
			{
				GSketchBoardPlayerController = PlayerController;
				bHadPreviousCursor = true;
				bPreviousCursorVisible = PlayerController->bShowMouseCursor;
			}
			PlayerController->bShowMouseCursor = true;
			FInputModeGameAndUI InputMode;
			InputMode.SetHideCursorDuringCapture(false);
			PlayerController->SetInputMode(InputMode);
		}
		else if (GSketchBoardPlayerController.IsValid())
		{
			GSketchBoardPlayerController->bShowMouseCursor = bHadPreviousCursor ? bPreviousCursorVisible : false;
			GSketchBoardPlayerController->SetInputMode(FInputModeGameOnly());
			GSketchBoardPlayerController.Reset();
			bHadPreviousCursor = false;
			bPreviousCursorVisible = false;
		}
	}

	UGameViewportClient* ResolveViewportClient()
	{
		if (GSketchBoardViewportClient.IsValid())
		{
			return GSketchBoardViewportClient.Get();
		}
		return GEngine ? GEngine->GameViewport : nullptr;
	}

	void RemoveBoardWidget()
	{
		if (GSketchBoardWidget.IsValid())
		{
			if (UGameViewportClient* ViewportClient = ResolveViewportClient())
			{
				ViewportClient->RemoveViewportWidgetContent(GSketchBoardWidget.ToSharedRef());
			}
			GSketchBoardWidget.Reset();
		}
	}

	void AddBoardWidget()
	{
		if (!GSketchBoardState.IsValid() || GSketchBoardWidget.IsValid())
		{
			return;
		}
		UGameViewportClient* ViewportClient = ResolveViewportClient();
		if (!ViewportClient)
		{
			UE_LOG(LogTemp, Warning, TEXT("SketchBoard: cannot add widget because viewport client is unavailable."));
			return;
		}

		GSketchBoardWidget =
			SNew(SFromLZSketchBoardWidget)
			.State(GSketchBoardState);
		ViewportClient->AddViewportWidgetContent(GSketchBoardWidget.ToSharedRef(), 200);
	}
}

void FFromLZSketchBoard::ShowForCapture(UWorld* World, UGameViewportClient* ViewportClient, const FString& CapturePngPath)
{
	if (!World || !ViewportClient || CapturePngPath.IsEmpty())
	{
		return;
	}

	if (GSketchBoardState.IsValid() || GSketchBoardWidget.IsValid() || bSketchBoardMinimized)
	{
		Close();
	}

	GSketchBoardWorld = World;
	GSketchBoardViewportClient = ViewportClient;
	bSketchBoardMinimized = false;

	TSharedPtr<FSketchBoardState> NewState = MakeShared<FSketchBoardState>();
	if (!NewState->LoadCapture(CapturePngPath))
	{
		return;
	}

	GSketchBoardState = NewState;
	AddBoardWidget();
	SetBoardCursorVisible(World, true);
	UE_LOG(LogTemp, Log, TEXT("SketchBoard: opened for capture %s"), *CapturePngPath);
}

bool FFromLZSketchBoard::SaveCurrentSketchAndProceed(UWorld* World)
{
	if (!GSketchBoardState.IsValid())
	{
		return false;
	}

	UWorld* WorldToUse = World ? World : GSketchBoardWorld.Get();
	if (!WorldToUse)
	{
		UE_LOG(LogTemp, Warning, TEXT("SketchBoard: cannot proceed because world is unavailable."));
		return true;
	}

	FString SketchPath;
	if (!GSketchBoardState->SaveSketchForNextPress(SketchPath))
	{
		UE_LOG(LogTemp, Warning, TEXT("SketchBoard: failed to save sketch before proceeding."));
		return true;
	}

	UE_LOG(LogTemp, Log, TEXT("SketchBoard: saved sketch %s and proceeding."), *SketchPath);
	const FString CapturePngPath = GSketchBoardState->GetCapturePngPath();
	Close();
	FFromLZSketchProcessor::ProcessSketch(WorldToUse, SketchPath, CapturePngPath);
	return true;
}

bool FFromLZSketchBoard::RestoreIfMinimized()
{
	if (!GSketchBoardState.IsValid() || !bSketchBoardMinimized)
	{
		return false;
	}

	AddBoardWidget();
	bSketchBoardMinimized = false;
	if (UWorld* World = GSketchBoardWorld.Get())
	{
		SetBoardCursorVisible(World, true);
	}
	return true;
}

void FFromLZSketchBoard::Minimize()
{
	if (!GSketchBoardState.IsValid() || bSketchBoardMinimized)
	{
		return;
	}

	RemoveBoardWidget();
	bSketchBoardMinimized = true;
	if (UWorld* World = GSketchBoardWorld.Get())
	{
		SetBoardCursorVisible(World, false);
	}
	UE_LOG(LogTemp, Log, TEXT("SketchBoard: minimized. Press B to restore."));
}

void FFromLZSketchBoard::Close()
{
	RemoveBoardWidget();
	GSketchBoardState.Reset();
	bSketchBoardMinimized = false;
	if (UWorld* World = GSketchBoardWorld.Get())
	{
		SetBoardCursorVisible(World, false);
	}
	GSketchBoardWorld.Reset();
	UE_LOG(LogTemp, Log, TEXT("SketchBoard: closed."));
}

bool FFromLZSketchBoard::IsOpenOrMinimized()
{
	return GSketchBoardState.IsValid();
}
