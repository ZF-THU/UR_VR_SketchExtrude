#include "FromLZCameraPreview.h"

#include "FromLZCaptureUtils.h"
#include "FromLZSketchBoard.h"
#include "Camera/CameraComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "InputCoreTypes.h"
#include "Slate/DeferredCleanupSlateBrush.h"
#include "Widgets/Images/SImage.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/SOverlay.h"

namespace
{
	static constexpr int32 PreviewMaxWidth = 360;
	static constexpr int32 PreviewMaxHeight = 240;
	static constexpr double PreviewAspectTolerance = 0.001;
	static constexpr float PreviewDefaultOrthoWidth = 1536.0f;
	static constexpr float PreviewOrthoNearPlane = 0.0f;
	static constexpr float PreviewOrthoFarPlane = 2097152.0f;

	struct FFromLZPreviewState
	{
		TWeakObjectPtr<UWorld> World;
		TWeakObjectPtr<UGameViewportClient> ViewportClient;
		TObjectPtr<USceneCaptureComponent2D> SceneCapture = nullptr;
		TObjectPtr<UTextureRenderTarget2D> RenderTarget = nullptr;
		TSharedPtr<SWidget> Widget;
		FSlateBrush Brush;
		FIntPoint RenderTargetSize = FIntPoint::ZeroValue;
		bool bWidgetAdded = false;

		~FFromLZPreviewState()
		{
			Shutdown();
		}

		void Shutdown()
		{
			RemoveWidget();
			if (SceneCapture)
			{
				SceneCapture->TextureTarget = nullptr;
				if (SceneCapture->IsRegistered())
				{
					SceneCapture->UnregisterComponent();
				}
				if (SceneCapture->IsRooted())
				{
					SceneCapture->RemoveFromRoot();
				}
				SceneCapture->DestroyComponent();
				SceneCapture = nullptr;
			}
			Brush.SetResourceObject(nullptr);
			if (RenderTarget)
			{
				if (RenderTarget->IsRooted())
				{
					RenderTarget->RemoveFromRoot();
				}
				RenderTarget = nullptr;
			}
			RenderTargetSize = FIntPoint::ZeroValue;
			World.Reset();
			ViewportClient.Reset();
		}

		void RemoveWidget()
		{
			if (Widget.IsValid() && bWidgetAdded)
			{
				if (UGameViewportClient* Client = ViewportClient.Get())
				{
					Client->RemoveViewportWidgetContent(Widget.ToSharedRef());
				}
			}
			bWidgetAdded = false;
			Widget.Reset();
		}
	};

	static TUniquePtr<FFromLZPreviewState> GPreviewState;

	static bool IsUsableOrthographicProjectionMatrix(const FMatrix& Matrix)
	{
		return
			FMath::IsFinite(Matrix.M[0][0]) &&
			FMath::IsFinite(Matrix.M[1][1]) &&
			!FMath::IsNearlyZero(Matrix.M[0][0]) &&
			!FMath::IsNearlyZero(Matrix.M[1][1]) &&
			FMath::IsNearlyZero(Matrix.M[2][3]) &&
			FMath::IsNearlyEqual(Matrix.M[3][3], 1.0f);
	}

	static bool CalculatePreviewSize(double AspectRatio, FIntPoint& OutSize)
	{
		if (!FMath::IsFinite(AspectRatio) || AspectRatio <= 1e-6)
		{
			return false;
		}

		int32 Width = PreviewMaxWidth;
		int32 Height = FMath::RoundToInt(double(Width) / AspectRatio);
		if (Height > PreviewMaxHeight)
		{
			Height = PreviewMaxHeight;
			Width = FMath::RoundToInt(double(Height) * AspectRatio);
		}

		OutSize = FIntPoint(
			FMath::Clamp(Width, 1, PreviewMaxWidth),
			FMath::Clamp(Height, 1, PreviewMaxHeight));
		return OutSize.X > 0 && OutSize.Y > 0;
	}

	static bool BuildPreviewProjection(const UCameraComponent* Camera, float SourceOrthoWidth, FMatrix& OutProjectionMatrix)
	{
		if (!Camera)
		{
			return false;
		}

		const FTransform CameraTransform = Camera->GetComponentTransform();
		const float EffectiveOrthoWidth = FMath::IsFinite(SourceOrthoWidth) && SourceOrthoWidth > 1e-6f
			? SourceOrthoWidth
			: PreviewDefaultOrthoWidth;

		FMinimalViewInfo ViewInfo;
		ViewInfo.Location = CameraTransform.GetLocation();
		ViewInfo.Rotation = CameraTransform.Rotator();
		ViewInfo.FOV = Camera->FieldOfView;
		ViewInfo.DesiredFOV = Camera->FieldOfView;
		ViewInfo.AspectRatio = Camera->AspectRatio;
		ViewInfo.ProjectionMode = ECameraProjectionMode::Orthographic;
		ViewInfo.OrthoWidth = EffectiveOrthoWidth;
		ViewInfo.OrthoNearClipPlane = PreviewOrthoNearPlane;
		ViewInfo.OrthoFarClipPlane = PreviewOrthoFarPlane;
		ViewInfo.bAutoCalculateOrthoPlanes = false;
		ViewInfo.AutoPlaneShift = 0.0f;
		ViewInfo.bUpdateOrthoPlanes = false;
		ViewInfo.bUseCameraHeightAsViewTarget = false;
		ViewInfo.bConstrainAspectRatio = true;

		OutProjectionMatrix = ViewInfo.CalculateProjectionMatrix();
		return IsUsableOrthographicProjectionMatrix(OutProjectionMatrix);
	}

	static APawn* FindPlayerPawn(UWorld* World)
	{
		if (!World)
		{
			return nullptr;
		}

		for (FConstPlayerControllerIterator It = World->GetPlayerControllerIterator(); It; ++It)
		{
			const APlayerController* PlayerController = It->Get();
			if (PlayerController && PlayerController->GetPawn())
			{
				return PlayerController->GetPawn();
			}
		}
		return nullptr;
	}

	static bool EnsureRenderTarget(FFromLZPreviewState& State, const FIntPoint& Size)
	{
		if (State.RenderTarget && State.RenderTargetSize == Size)
		{
			return true;
		}

		if (State.RenderTarget)
		{
			State.RenderTarget->RemoveFromRoot();
			State.RenderTarget = nullptr;
		}

		UTextureRenderTarget2D* RenderTarget = NewObject<UTextureRenderTarget2D>(GetTransientPackage(), NAME_None, RF_Transient);
		if (!RenderTarget)
		{
			return false;
		}

		RenderTarget->RenderTargetFormat = RTF_RGBA8;
		RenderTarget->ClearColor = FLinearColor::Black;
		RenderTarget->bAutoGenerateMips = false;
		RenderTarget->InitAutoFormat(Size.X, Size.Y);
		RenderTarget->UpdateResourceImmediate(true);
		RenderTarget->AddToRoot();

		State.RenderTarget = RenderTarget;
		State.RenderTargetSize = Size;
		State.Brush.SetResourceObject(RenderTarget);
		State.Brush.ImageSize = FVector2D(Size.X, Size.Y);
		return true;
	}

	static bool EnsureSceneCapture(FFromLZPreviewState& State, UWorld* World)
	{
		if (State.SceneCapture && State.SceneCapture->GetWorld() == World)
		{
			return true;
		}

		if (State.SceneCapture)
		{
			State.SceneCapture->TextureTarget = nullptr;
			if (State.SceneCapture->IsRegistered())
			{
				State.SceneCapture->UnregisterComponent();
			}
			if (State.SceneCapture->IsRooted())
			{
				State.SceneCapture->RemoveFromRoot();
			}
			State.SceneCapture->DestroyComponent();
			State.SceneCapture = nullptr;
		}

		USceneCaptureComponent2D* SceneCapture = NewObject<USceneCaptureComponent2D>(GetTransientPackage(), NAME_None, RF_Transient);
		if (!SceneCapture)
		{
			return false;
		}

		SceneCapture->bCaptureEveryFrame = false;
		SceneCapture->bCaptureOnMovement = false;
		SceneCapture->bAlwaysPersistRenderingState = false;
		SceneCapture->ShowFlags.SetMotionBlur(false);
		SceneCapture->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
		SceneCapture->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_UseShowOnlyList;
		SceneCapture->AddToRoot();
		SceneCapture->RegisterComponentWithWorld(World);
		State.SceneCapture = SceneCapture;
		return true;
	}

	static void EnsureWidget(FFromLZPreviewState& State, UGameViewportClient* ViewportClient)
	{
		if (State.Widget.IsValid() && State.bWidgetAdded)
		{
			return;
		}

		State.ViewportClient = ViewportClient;
		State.Widget =
			SNew(SOverlay)
			.Visibility(EVisibility::HitTestInvisible)
			+ SOverlay::Slot()
			.HAlign(HAlign_Right)
			.VAlign(VAlign_Bottom)
			.Padding(FMargin(0.0f, 0.0f, 18.0f, 18.0f))
			[
				SNew(SBorder)
				.Padding(3.0f)
				.BorderBackgroundColor(FLinearColor(0.02f, 0.02f, 0.02f, 0.82f))
				[
					SNew(SBox)
					.WidthOverride_Lambda([&State]() { return State.Brush.ImageSize.X; })
					.HeightOverride_Lambda([&State]() { return State.Brush.ImageSize.Y; })
					[
						SNew(SImage)
						.Image(&State.Brush)
					]
				]
			];

		ViewportClient->AddViewportWidgetContent(State.Widget.ToSharedRef(), 50);
		State.bWidgetAdded = true;
	}

	static void HideWidgetIfNeeded(FFromLZPreviewState& State)
	{
		if (State.Widget.IsValid() && State.bWidgetAdded)
		{
			State.RemoveWidget();
		}
	}
}

void FFromLZCameraPreview::Tick(UWorld* World, UGameViewportClient* ViewportClient, FViewport* Viewport)
{
	if (!World || !ViewportClient || !Viewport)
	{
		Shutdown();
		return;
	}

	if (FFromLZSketchBoard::IsOpenOrMinimized())
	{
		if (GPreviewState)
		{
			HideWidgetIfNeeded(*GPreviewState);
		}
		return;
	}

	if (!GPreviewState)
	{
		GPreviewState = MakeUnique<FFromLZPreviewState>();
	}

	FFromLZPreviewState& State = *GPreviewState;
	if (State.World.IsValid() && State.World.Get() != World)
	{
		State.Shutdown();
	}
	State.World = World;
	State.ViewportClient = ViewportClient;

	APawn* Pawn = FindPlayerPawn(World);
	UCameraComponent* Camera = FFromLZCaptureUtils::FindFromLZCamera(Pawn);
	if (!Pawn || !Camera)
	{
		HideWidgetIfNeeded(State);
		return;
	}

	FIntPoint PreviewSize;
	if (!CalculatePreviewSize(Camera->AspectRatio, PreviewSize) ||
		!EnsureRenderTarget(State, PreviewSize) ||
		!EnsureSceneCapture(State, World))
	{
		HideWidgetIfNeeded(State);
		return;
	}

	FMatrix ProjectionMatrix;
	TArray<AActor*> SubjectActors;
	FString SubjectMode;
	FFromLZCaptureUtils::BuildCaptureSubjectActors(
		World,
		Pawn,
		Pawn,
		Camera->GetComponentLocation(),
		SubjectActors,
		SubjectMode);

	double DerivedOrthoWidth = 0.0;
	FVector SubjectBoundsCenter = FVector::ZeroVector;
	double SubjectFocusDepth = 0.0;
	TArray<AActor*> FramingActors;
	FFromLZCaptureUtils::BuildOrthoFramingActors(SubjectActors, FramingActors);
	float OrthoWidth = FMath::IsFinite(Camera->OrthoWidth) && Camera->OrthoWidth > 1e-6f
		? Camera->OrthoWidth
		: PreviewDefaultOrthoWidth;
	if (FFromLZCaptureUtils::CalculateSubjectBoundsCenterOrthoWidth(
		Camera,
		FramingActors,
		DerivedOrthoWidth,
		SubjectBoundsCenter,
		SubjectFocusDepth))
	{
		OrthoWidth = static_cast<float>(DerivedOrthoWidth);
	}

	if (!BuildPreviewProjection(Camera, OrthoWidth, ProjectionMatrix))
	{
		HideWidgetIfNeeded(State);
		return;
	}

	State.SceneCapture->SetWorldTransform(Camera->GetComponentTransform());
	State.SceneCapture->ProjectionType = ECameraProjectionMode::Orthographic;
	State.SceneCapture->FOVAngle = Camera->FieldOfView;
	State.SceneCapture->OrthoWidth = OrthoWidth;
	State.SceneCapture->bAutoCalculateOrthoPlanes = false;
	State.SceneCapture->AutoPlaneShift = 0.0f;
	State.SceneCapture->bUpdateOrthoPlanes = false;
	State.SceneCapture->bUseCameraHeightAsViewTarget = false;
	State.SceneCapture->bUseCustomProjectionMatrix = true;
	State.SceneCapture->CustomProjectionMatrix = ProjectionMatrix;
	State.SceneCapture->TextureTarget = State.RenderTarget;

	FFromLZCaptureUtils::ApplyCaptureSubjectActors(State.SceneCapture, SubjectActors);

	State.SceneCapture->CaptureScene();
	EnsureWidget(State, ViewportClient);
}

void FFromLZCameraPreview::Shutdown()
{
	if (GPreviewState)
	{
		GPreviewState->Shutdown();
		GPreviewState.Reset();
	}
}
