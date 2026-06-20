#include "FromLZCaptureUtils.h"

#include "FromLZFaceReconstructor.h"
#include "FromLZSketchBoard.h"

#include "Camera/CameraComponent.h"
#include "Camera/PlayerCameraManager.h"
#include "Components/PrimitiveComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "Engine/Engine.h"
#include "Engine/Scene.h"
#include "Engine/StaticMesh.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/SpringArmComponent.h"
#include "HAL/FileManager.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Materials/MaterialInterface.h"
#include "Math/Float16Color.h"
#include "Misc/DateTime.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"
#include "RenderingThread.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"
#include "ProceduralMeshComponent.h"
#include "StaticMeshResources.h"
#include "TextureResource.h"
#include "UnrealClient.h"
#include "UObject/FieldIterator.h"
#include "UObject/UnrealType.h"

static constexpr double FromLZDefaultOrthoWidth = 1536.0;
static constexpr float FromLZCaptureOrthoNearPlane = 0.0f;
static constexpr float FromLZCaptureOrthoFarPlane = 2097152.0f;
static const FName FromLZCaptureSubjectTag(TEXT("FromLZCaptureSubject"));
static const FName FromLZCapturePlaneTag(TEXT("FromLZCapturePlane"));

struct FFromLZCaptureView
{
	FTransform SourceViewTransform = FTransform::Identity;
	FTransform Transform = FTransform::Identity;
	FVector FocusPoint = FVector::ZeroVector;
	FVector ReferencePoint = FVector::ZeroVector;
	FString FocusSource;
	FString ReferenceSource;
	FString SourceViewTransformSource;
	FString TransformSource;
	FString OrthoWidthMode;
	FString FramingMode;
	FIntPoint MaxViewportSize = FIntPoint(1920, 1080);
	FIntPoint ViewportSize = FIntPoint(1920, 1080);
	double ViewportAspectRatio = 16.0 / 9.0;
	double Fov = 90.0;
	double FocusDepth = 0.0;
	double ReferenceDepth = 0.0;
	double OrthoWidth = FromLZDefaultOrthoWidth;
	double OrthoBackoff = FromLZDefaultOrthoWidth;
	FMatrix ProjectionMatrix = FMatrix::Identity;
	bool bHasPlayerCameraManager = false;
	bool bUseCustomProjectionMatrix = false;
	bool bUsedDeprojectWidth = false;
	bool bUsedFormulaFallback = false;
	bool bUsedDepthFallback = false;
	bool bUsedFocusTrace = false;
};

struct FFromLZCameraProjectionSnapshot
{
	ECameraProjectionMode::Type ProjectionMode = ECameraProjectionMode::Perspective;
	float FieldOfView = 90.0f;
	float OrthoWidth = static_cast<float>(FromLZDefaultOrthoWidth);
	float OrthoNearClipPlane = FromLZCaptureOrthoNearPlane;
	float OrthoFarClipPlane = FromLZCaptureOrthoFarPlane;
	float AutoPlaneShift = 0.0f;
	float AspectRatio = 16.0f / 9.0f;
	bool bAutoCalculateOrthoPlanes = false;
	bool bUpdateOrthoPlanes = false;
	bool bUseCameraHeightAsViewTarget = false;
	bool bConstrainAspectRatio = false;
};

static bool FromLZIsUsableOrthographicProjectionMatrix(const FMatrix& Matrix)
{
	return
		FMath::IsFinite(Matrix.M[0][0]) &&
		FMath::IsFinite(Matrix.M[1][1]) &&
		FMath::Abs(Matrix.M[0][0]) > UE_SMALL_NUMBER &&
		FMath::Abs(Matrix.M[1][1]) > UE_SMALL_NUMBER &&
		FMath::IsNearlyZero(Matrix.M[2][3]) &&
		FMath::IsNearlyEqual(Matrix.M[3][3], 1.0);
}

static bool AreProjectionMatricesNearlyEqual(
	const FMatrix& A,
	const FMatrix& B,
	double Tolerance = 1e-9)
{
	for (int32 Row = 0; Row < 4; ++Row)
	{
		for (int32 Column = 0; Column < 4; ++Column)
		{
			if (!FMath::IsNearlyEqual(A.M[Row][Column], B.M[Row][Column], Tolerance))
			{
				return false;
			}
		}
	}
	return true;
}

static bool TryGetOrthographicViewPlanePosition(
	const FMatrix& ProjectionMatrix,
	double NdcX,
	double NdcY,
	double& OutRight,
	double& OutUp)
{
	if (!FromLZIsUsableOrthographicProjectionMatrix(ProjectionMatrix))
	{
		return false;
	}

	OutRight = (NdcX - ProjectionMatrix.M[3][0]) / ProjectionMatrix.M[0][0];
	OutUp = (NdcY - ProjectionMatrix.M[3][1]) / ProjectionMatrix.M[1][1];
	return FMath::IsFinite(OutRight) && FMath::IsFinite(OutUp);
}

static TSharedRef<FJsonObject> SerializeMatrix(const FMatrix& Matrix);

static FFromLZCameraProjectionSnapshot MakeCameraProjectionSnapshot(const UCameraComponent* Camera)
{
	FFromLZCameraProjectionSnapshot Snapshot;
	if (!Camera)
	{
		return Snapshot;
	}

	Snapshot.ProjectionMode = Camera->ProjectionMode;
	Snapshot.FieldOfView = Camera->FieldOfView;
	Snapshot.OrthoWidth = Camera->OrthoWidth;
	Snapshot.OrthoNearClipPlane = Camera->OrthoNearClipPlane;
	Snapshot.OrthoFarClipPlane = Camera->OrthoFarClipPlane;
	Snapshot.AutoPlaneShift = Camera->AutoPlaneShift;
	Snapshot.AspectRatio = Camera->AspectRatio;
	Snapshot.bAutoCalculateOrthoPlanes = Camera->bAutoCalculateOrthoPlanes;
	Snapshot.bUpdateOrthoPlanes = Camera->bUpdateOrthoPlanes;
	Snapshot.bUseCameraHeightAsViewTarget = Camera->bUseCameraHeightAsViewTarget;
	Snapshot.bConstrainAspectRatio = Camera->bConstrainAspectRatio;
	return Snapshot;
}

static FString ProjectionModeToString(ECameraProjectionMode::Type ProjectionMode)
{
	if (const UEnum* Enum = StaticEnum<ECameraProjectionMode::Type>())
	{
		return Enum->GetValueAsString(ProjectionMode);
	}
	return ProjectionMode == ECameraProjectionMode::Orthographic ? TEXT("Orthographic") : TEXT("Perspective");
}

static TSharedRef<FJsonObject> SerializeCameraProjectionSnapshot(const FFromLZCameraProjectionSnapshot& Snapshot)
{
	TSharedRef<FJsonObject> Object = MakeShared<FJsonObject>();
	Object->SetStringField(TEXT("projection_mode"), ProjectionModeToString(Snapshot.ProjectionMode));
	Object->SetNumberField(TEXT("field_of_view"), Snapshot.FieldOfView);
	Object->SetNumberField(TEXT("ortho_width"), Snapshot.OrthoWidth);
	Object->SetNumberField(TEXT("ortho_near_clip_plane"), Snapshot.OrthoNearClipPlane);
	Object->SetNumberField(TEXT("ortho_far_clip_plane"), Snapshot.OrthoFarClipPlane);
	Object->SetNumberField(TEXT("auto_plane_shift"), Snapshot.AutoPlaneShift);
	Object->SetNumberField(TEXT("aspect_ratio"), Snapshot.AspectRatio);
	Object->SetBoolField(TEXT("auto_calculate_ortho_planes"), Snapshot.bAutoCalculateOrthoPlanes);
	Object->SetBoolField(TEXT("update_ortho_planes"), Snapshot.bUpdateOrthoPlanes);
	Object->SetBoolField(TEXT("use_camera_height_as_view_target"), Snapshot.bUseCameraHeightAsViewTarget);
	Object->SetBoolField(TEXT("constrain_aspect_ratio"), Snapshot.bConstrainAspectRatio);
	return Object;
}

struct FPendingFromLZCapture
{
	struct FOutputPaths
	{
		FString Timestamp;
		FString CaptureDirectory;
		FString BaseFilename;
		FString JsonPath;
		FString PngPath;
		FString DebugViewportPerspectivePngPath;
		FString DebugViewportOrthographicPngPath;
		FString DebugFromLZCameraPngPath;
		FString DebugTransformsJsonPath;
		FString ActorMaterialPngPath;
		FString ActorMaterialJsonPath;
	};

	TWeakObjectPtr<APawn> Pawn;
	TWeakObjectPtr<AActor> CameraActor;
	TWeakObjectPtr<UCameraComponent> Camera;
	TWeakObjectPtr<UWorld> World;
	FOutputPaths OutputPaths;
	FFromLZCaptureView CaptureView;
	FFromLZCameraProjectionSnapshot SourceCameraProjection;
	FString CameraSource = TEXT("pawn_fromlz_offscreen");
	FName RequestedCameraTag = NAME_None;
	TArray<TWeakObjectPtr<AActor>> SubjectActors;
	FString SubjectMode;
	FVector SubjectBoundsCenter = FVector::ZeroVector;
	double SubjectFocusDepth = 0.0;
	bool bUsedSubjectBoundsCenterOrthoWidth = false;
	bool bSourceOrthoWidthUsedDefault = false;
	bool bViewportReferenceSaved = false;
	bool bOpenSketchBoardOnSuccess = true;
	FFromLZCaptureCompletionCallback CompletionCallback;
};

static TUniquePtr<FPendingFromLZCapture> GPendingFromLZCapture;

static FPendingFromLZCapture::FOutputPaths MakeCaptureOutputPaths()
{
	FPendingFromLZCapture::FOutputPaths Paths;
	Paths.CaptureDirectory = FPaths::ProjectSavedDir() / TEXT("FromLZCaptures");
	IFileManager::Get().MakeDirectory(*Paths.CaptureDirectory, true);

	Paths.Timestamp = FDateTime::Now().ToString(TEXT("%Y%m%d_%H%M%S"));
	Paths.BaseFilename = FString::Printf(TEXT("FromLZ_%s"), *Paths.Timestamp);
	Paths.JsonPath = Paths.CaptureDirectory / (Paths.BaseFilename + TEXT(".json"));
	Paths.PngPath = Paths.CaptureDirectory / (Paths.BaseFilename + TEXT(".png"));
	Paths.DebugViewportPerspectivePngPath = Paths.CaptureDirectory / (Paths.BaseFilename + TEXT("_debug_viewport_perspective.png"));
	Paths.DebugViewportOrthographicPngPath = Paths.CaptureDirectory / (Paths.BaseFilename + TEXT("_debug_viewport_orthographic.png"));
	Paths.DebugFromLZCameraPngPath = Paths.CaptureDirectory / (Paths.BaseFilename + TEXT("_debug_fromlz_camera.png"));
	Paths.DebugTransformsJsonPath = Paths.CaptureDirectory / (Paths.BaseFilename + TEXT("_debug_transforms.json"));
	Paths.ActorMaterialPngPath = Paths.CaptureDirectory / (Paths.BaseFilename + TEXT("_actor_material_id.png"));
	Paths.ActorMaterialJsonPath = Paths.CaptureDirectory / (Paths.BaseFilename + TEXT("_actor_material_id.json"));
	return Paths;
}

static void AddExistingCaptureFile(const FString& FilePath, TArray<FString>& OutputFiles)
{
	if (!FilePath.IsEmpty() && IFileManager::Get().FileExists(*FilePath))
	{
		OutputFiles.AddUnique(FilePath);
	}
}

static FFromLZCaptureResult MakeCaptureResult(
	const FPendingFromLZCapture::FOutputPaths& Paths,
	bool bSuccess,
	const FString& Message)
{
	FFromLZCaptureResult Result;
	Result.bSuccess = bSuccess;
	Result.Message = Message;
	Result.CaptureDirectory = Paths.CaptureDirectory;
	Result.MainPngPath = Paths.PngPath;
	Result.MainJsonPath = Paths.JsonPath;

	const FString BasePath = Paths.CaptureDirectory / Paths.BaseFilename;
	AddExistingCaptureFile(Paths.PngPath, Result.OutputFiles);
	AddExistingCaptureFile(Paths.JsonPath, Result.OutputFiles);
	AddExistingCaptureFile(BasePath + TEXT("_faces.png"), Result.OutputFiles);
	AddExistingCaptureFile(BasePath + TEXT("_faces.json"), Result.OutputFiles);
	AddExistingCaptureFile(BasePath + TEXT("_faces_debug.png"), Result.OutputFiles);
	AddExistingCaptureFile(Paths.ActorMaterialPngPath, Result.OutputFiles);
	AddExistingCaptureFile(Paths.ActorMaterialJsonPath, Result.OutputFiles);
	AddExistingCaptureFile(Paths.DebugViewportPerspectivePngPath, Result.OutputFiles);
	AddExistingCaptureFile(Paths.DebugViewportOrthographicPngPath, Result.OutputFiles);
	AddExistingCaptureFile(Paths.DebugFromLZCameraPngPath, Result.OutputFiles);
	AddExistingCaptureFile(Paths.DebugTransformsJsonPath, Result.OutputFiles);
	AddExistingCaptureFile(BasePath + TEXT("_debug_depth.png"), Result.OutputFiles);
	AddExistingCaptureFile(BasePath + TEXT("_debug_normal.png"), Result.OutputFiles);
	AddExistingCaptureFile(BasePath + TEXT("_debug_depth_lap.png"), Result.OutputFiles);
	AddExistingCaptureFile(BasePath + TEXT("_debug_normal_grad.png"), Result.OutputFiles);

	return Result;
}

static FFromLZCaptureResult MakeCaptureFailureResult(const FString& Message)
{
	FFromLZCaptureResult Result;
	Result.bSuccess = false;
	Result.Message = Message;
	return Result;
}

static void DispatchCaptureCompletion(
	FFromLZCaptureCompletionCallback& CompletionCallback,
	const FFromLZCaptureResult& Result)
{
	if (CompletionCallback)
	{
		CompletionCallback(Result);
	}
}

// ===================================================================
// Normal-based planar face segmentation for every offscreen capture path.
// Groups foreground pixels whose world normals are continuous and whose depth
// is continuous into planar faces, extracts each face's corner key points, and
// unprojects them to 3D world coordinates using the captured camera parameters.
// ===================================================================
namespace FromLZFaces
{
	static uint32 FaceColorKey(const FColor& Color)
	{
		return (uint32(Color.R) << 16) | (uint32(Color.G) << 8) | uint32(Color.B);
	}

	// 100 visually distinct face colors: 20 hue buckets x 5 saturation/value pairs.
	// These colors are also IDs in the faces PNG, so they must stay unique.
	static const FColor FacePalette100[] = {
		FColor(219, 60, 39), FColor(199, 76, 60), FColor(184, 30, 9), FColor(163, 74, 62), FColor(148, 33, 18),
		FColor(219, 114, 39), FColor(199, 118, 60), FColor(184, 82, 9), FColor(163, 104, 62), FColor(148, 72, 18),
		FColor(219, 168, 39), FColor(199, 159, 60), FColor(184, 134, 9), FColor(163, 135, 62), FColor(148, 111, 18),
		FColor(216, 219, 39), FColor(197, 199, 60), FColor(181, 184, 9), FColor(162, 163, 62), FColor(146, 148, 18),
		FColor(162, 219, 39), FColor(155, 199, 60), FColor(128, 184, 9), FColor(131, 163, 62), FColor(107, 148, 18),
		FColor(108, 219, 39), FColor(113, 199, 60), FColor(76, 184, 9), FColor(101, 163, 62), FColor(68, 148, 18),
		FColor(54, 219, 39), FColor(71, 199, 60), FColor(24, 184, 9), FColor(70, 163, 62), FColor(29, 148, 18),
		FColor(39, 219, 78), FColor(60, 199, 90), FColor(9, 184, 47), FColor(62, 163, 84), FColor(18, 148, 46),
		FColor(39, 219, 132), FColor(60, 199, 132), FColor(9, 184, 99), FColor(62, 163, 114), FColor(18, 148, 85),
		FColor(39, 219, 186), FColor(60, 199, 173), FColor(9, 184, 152), FColor(62, 163, 145), FColor(18, 148, 124),
		FColor(39, 198, 219), FColor(60, 183, 199), FColor(9, 163, 184), FColor(62, 151, 163), FColor(18, 133, 148),
		FColor(39, 144, 219), FColor(60, 141, 199), FColor(9, 111, 184), FColor(62, 121, 163), FColor(18, 94, 148),
		FColor(39, 90, 219), FColor(60, 99, 199), FColor(9, 59, 184), FColor(62, 91, 163), FColor(18, 55, 148),
		FColor(42, 39, 219), FColor(62, 60, 199), FColor(12, 9, 184), FColor(64, 62, 163), FColor(20, 18, 148),
		FColor(96, 39, 219), FColor(104, 60, 199), FColor(64, 9, 184), FColor(94, 62, 163), FColor(59, 18, 148),
		FColor(150, 39, 219), FColor(146, 60, 199), FColor(117, 9, 184), FColor(124, 62, 163), FColor(98, 18, 148),
		FColor(204, 39, 219), FColor(187, 60, 199), FColor(169, 9, 184), FColor(155, 62, 163), FColor(137, 18, 148),
		FColor(219, 39, 180), FColor(199, 60, 169), FColor(184, 9, 146), FColor(163, 62, 141), FColor(148, 18, 120),
		FColor(219, 39, 126), FColor(199, 60, 127), FColor(184, 9, 93), FColor(163, 62, 111), FColor(148, 18, 81),
		FColor(219, 39, 72), FColor(199, 60, 85), FColor(184, 9, 41), FColor(163, 62, 81), FColor(148, 18, 42)
	};
	static_assert(UE_ARRAY_COUNT(FacePalette100) == 100, "FacePalette100 must contain exactly 100 colors.");

	static FColor MakeFallbackFaceColor(int32 FaceId, int32 Attempt)
	{
		uint32 Seed = uint32(FaceId) * 1103515245u + 12345u + uint32(Attempt) * 2654435761u;
		Seed ^= (Seed >> 16);
		const uint8 R = uint8(32u + (Seed & 0x7fu));
		const uint8 G = uint8(32u + ((Seed >> 7) & 0x7fu));
		const uint8 B = uint8(32u + ((Seed >> 14) & 0x7fu));
		return FColor(R, G, B, 255);
	}

	static bool GetUniqueFaceColor(int32 FaceId, TSet<uint32>& UsedColorKeys, FColor& OutColor)
	{
		if (FaceId >= 0 && FaceId < UE_ARRAY_COUNT(FacePalette100))
		{
			const FColor Candidate = FacePalette100[FaceId];
			const uint32 Key = FaceColorKey(Candidate);
			if (!UsedColorKeys.Contains(Key))
			{
				UsedColorKeys.Add(Key);
				OutColor = Candidate;
				return true;
			}
		}

		for (int32 Attempt = 0; Attempt < 4096; ++Attempt)
		{
			const FColor Candidate = MakeFallbackFaceColor(FaceId, Attempt);
			const uint32 Key = FaceColorKey(Candidate);
			if (!UsedColorKeys.Contains(Key) && Key != FaceColorKey(FColor::White) && Key != FaceColorKey(FColor::Black))
			{
				UsedColorKeys.Add(Key);
				OutColor = Candidate;
				return true;
			}
		}
		return false;
	}

	// Perpendicular distance from P to segment-line A-B.
	static float PerpDist(const FVector2D& P, const FVector2D& A, const FVector2D& B)
	{
		const FVector2D AB = B - A;
		const float Len2 = AB.SizeSquared();
		if (Len2 < 1e-6f) { return FVector2D::Distance(P, A); }
		const float T = static_cast<float>(FVector2D::DotProduct(P - A, AB) / Len2);
		return FVector2D::Distance(P, A + AB * T);
	}

	// Ramer-Douglas-Peucker on an OPEN polyline [I0..I1]; appends interior kept indices.
	static void RDP(const TArray<FVector2D>& Pts, int32 I0, int32 I1, float Eps, TArray<int32>& OutKeep)
	{
		float MaxD = -1.f;
		int32 MaxI = -1;
		for (int32 i = I0 + 1; i < I1; ++i)
		{
			const float D = PerpDist(Pts[i], Pts[I0], Pts[I1]);
			if (D > MaxD) { MaxD = D; MaxI = i; }
		}
		if (MaxI >= 0 && MaxD > Eps)
		{
			RDP(Pts, I0, MaxI, Eps, OutKeep);
			OutKeep.Add(MaxI);
			RDP(Pts, MaxI, I1, Eps, OutKeep);
		}
	}

	// Moore-neighbor boundary trace of the region labelled R, starting at StartIdx
	// (the raster-first pixel of the region, so it is on the top boundary).
	static void TraceContour(const TArray<int32>& Label, int32 W, int32 H, int32 R, int32 StartIdx, TArray<FIntPoint>& Out)
	{
		Out.Reset();
		// Clockwise 8-neighborhood: 0=E,1=SE,2=S,3=SW,4=W,5=NW,6=N,7=NE.
		static const int32 DX[8] = { 1, 1, 0, -1, -1, -1, 0, 1 };
		static const int32 DY[8] = { 0, 1, 1, 1, 0, -1, -1, -1 };
		auto At = [&](int32 x, int32 y) -> bool
		{
			return (x >= 0 && x < W && y >= 0 && y < H) && Label[y * W + x] == R;
		};

		const FIntPoint Start(StartIdx % W, StartIdx / W);
		FIntPoint P = Start;
		Out.Add(P);
		int32 Back = 4; // came from the West (region is leftmost on its top row)
		const int32 MaxSteps = W * H * 8;
		for (int32 Step = 0; Step < MaxSteps; ++Step)
		{
			bool bFound = false;
			for (int32 k = 1; k <= 8; ++k)
			{
				const int32 d = (Back + k) & 7;
				const int32 nx = P.X + DX[d];
				const int32 ny = P.Y + DY[d];
				if (At(nx, ny))
				{
					P = FIntPoint(nx, ny);
					Back = (d + 4) & 7; // direction from the new pixel back to the old one
					bFound = true;
					break;
				}
			}
			if (!bFound) { break; } // isolated pixel
			if (P == Start) { break; }
			Out.Add(P);
		}
	}
}

// 5x7 bitmap font for digits 0-9. Each row's bits 4..0 = columns left..right.
static const uint8 DigitGlyph5x7[10][7] = {
	{ 0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E }, // 0
	{ 0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E }, // 1
	{ 0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F }, // 2
	{ 0x0E, 0x11, 0x01, 0x06, 0x01, 0x11, 0x0E }, // 3
	{ 0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02 }, // 4
	{ 0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E }, // 5
	{ 0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E }, // 6
	{ 0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08 }, // 7
	{ 0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E }, // 8
	{ 0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C }, // 9
};

static void DrawDigitGlyph(TArray<FColor>& Buf, int32 W, int32 H, int32 X, int32 Y, int32 Scale, int32 Digit, FColor TextColor)
{
	if (Digit < 0 || Digit > 9 || Scale <= 0) { return; }
	for (int32 row = 0; row < 7; ++row)
	{
		const uint8 RowBits = DigitGlyph5x7[Digit][row];
		for (int32 col = 0; col < 5; ++col)
		{
			if (((RowBits >> (4 - col)) & 1) == 0) { continue; }
			for (int32 dy = 0; dy < Scale; ++dy)
			{
				const int32 Py = Y + row * Scale + dy;
				if (Py < 0 || Py >= H) { continue; }
				for (int32 dx = 0; dx < Scale; ++dx)
				{
					const int32 Px = X + col * Scale + dx;
					if (Px < 0 || Px >= W) { continue; }
					Buf[Py * W + Px] = TextColor;
				}
			}
		}
	}
}

// Draw an integer label centered at (CenterX, CenterY) into Buf with a filled background
// rectangle so the digits stay readable on top of dimmed face colors.
static void DrawIntegerLabel(TArray<FColor>& Buf, int32 W, int32 H, int32 CenterX, int32 CenterY, int32 Scale, int32 Value, FColor TextColor, FColor BgColor)
{
	if (Scale <= 0) { return; }
	int32 DigitsBuf[10];
	int32 NumDigits = 0;
	int32 V = Value < 0 ? 0 : Value;
	if (V == 0)
	{
		DigitsBuf[NumDigits++] = 0;
	}
	else
	{
		int32 Tmp[10];
		int32 N = 0;
		while (V > 0 && N < 10) { Tmp[N++] = V % 10; V /= 10; }
		for (int32 k = N - 1; k >= 0; --k) { DigitsBuf[NumDigits++] = Tmp[k]; }
	}

	const int32 GlyphW = 5;
	const int32 GlyphH = 7;
	const int32 Spacing = 1;
	const int32 TotalW = NumDigits * GlyphW * Scale + (NumDigits > 0 ? (NumDigits - 1) * Spacing * Scale : 0);
	const int32 TotalH = GlyphH * Scale;
	const int32 Pad = Scale;
	int32 X0 = CenterX - TotalW / 2;
	int32 Y0 = CenterY - TotalH / 2;
	X0 = FMath::Clamp(X0, Pad, FMath::Max(Pad, W - TotalW - Pad));
	Y0 = FMath::Clamp(Y0, Pad, FMath::Max(Pad, H - TotalH - Pad));

	for (int32 dy = -Pad; dy < TotalH + Pad; ++dy)
	{
		const int32 Py = Y0 + dy;
		if (Py < 0 || Py >= H) { continue; }
		for (int32 dx = -Pad; dx < TotalW + Pad; ++dx)
		{
			const int32 Px = X0 + dx;
			if (Px < 0 || Px >= W) { continue; }
			Buf[Py * W + Px] = BgColor;
		}
	}

	int32 Cursor = X0;
	for (int32 k = 0; k < NumDigits; ++k)
	{
		DrawDigitGlyph(Buf, W, H, Cursor, Y0, Scale, DigitsBuf[k], TextColor);
		Cursor += (GlyphW + Spacing) * Scale;
	}
}

// Segments the captured depth/normal buffers into planar faces, derives each face's
// corner key points (2D), unprojects them to 3D world coordinates, and writes a
// color-coded faces PNG plus a faces JSON. Reuses the already-read-back buffers.
static bool SaveNormalFaces(
	const TArray<float>& Depth, const TArray<FVector3f>& Normal,
	int32 W, int32 H, const FTransform& CameraTransform, double CameraFov,
	bool bCaptureOrthographic, double CaptureOrthoWidth,
	const FMatrix& CaptureProjectionMatrix,
	const FString& FacesPngPath, const FString& FacesJsonPath)
{
	using namespace FromLZFaces;

	if (bCaptureOrthographic && !FromLZIsUsableOrthographicProjectionMatrix(CaptureProjectionMatrix))
	{
		UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ: faces output requires a valid offscreen orthographic projection matrix."));
		return false;
	}

	// --- tunables ---
	const float NormalAngleTolDeg = 12.0f;
	const float CosTol = FMath::Cos(FMath::DegreesToRadians(NormalAngleTolDeg));
	const float DepthJoinTol = 0.02f;   // relative depth continuity
	const int32 MinFaceArea = 200;      // pixels
	const float RdpEps = 4.0f;          // corner simplification (px)
	const float ForegroundNormalLen = 0.1f;

	const int32 NumPx = W * H;

	// Background detection by depth: UE clears unrendered pixels to the far plane,
	// which appears as a uniform maximum depth (its normal is also a constant fill).
	// Excluding the far depth (and near-zero/invalid depth) drops empty space while
	// KEEPING every real planar surface (slab, ground plane, cube faces) as a face.
	float MaxD = 0.f;
	for (int32 i = 0; i < NumPx; ++i) { MaxD = FMath::Max(MaxD, Depth[i]); }
	const float BgDepthThresh = MaxD * 0.999f;

	// Normalize normals + foreground mask.
	TArray<FVector3f> N;
	N.SetNumUninitialized(NumPx);
	TArray<uint8> Fg;
	Fg.SetNumUninitialized(NumPx);
	for (int32 i = 0; i < NumPx; ++i)
	{
		const float Len = Normal[i].Size();
		const bool bBackground = (Depth[i] >= BgDepthThresh) || (Depth[i] <= 1.f);
		if (Len > ForegroundNormalLen && !bBackground)
		{
			N[i] = Normal[i] / Len;
			Fg[i] = 1;
		}
		else
		{
			N[i] = FVector3f::ZeroVector;
			Fg[i] = 0;
		}
	}

	// Flood-fill into planar regions: grow across neighbors with continuous normal + depth.
	TArray<int32> Label;
	Label.Init(-1, NumPx);
	TArray<int32> RegionArea;
	TArray<int32> Stack;
	const int32 NX[8] = { 1, -1, 0, 0, 1, 1, -1, -1 };
	const int32 NY[8] = { 0, 0, 1, -1, 1, -1, 1, -1 };
	int32 NextLabel = 0;
	for (int32 s = 0; s < NumPx; ++s)
	{
		if (!Fg[s] || Label[s] != -1) { continue; }
		Label[s] = NextLabel;
		Stack.Reset();
		Stack.Add(s);
		int32 Count = 0;
		while (Stack.Num() > 0)
		{
			const int32 c = Stack.Pop(EAllowShrinking::No);
			++Count;
			const int32 cx = c % W;
			const int32 cy = c / W;
			const FVector3f Nc = N[c];
			const float Dc = Depth[c];
			for (int32 k = 0; k < 8; ++k)
			{
				const int32 nx = cx + NX[k];
				const int32 ny = cy + NY[k];
				if (nx < 0 || nx >= W || ny < 0 || ny >= H) { continue; }
				const int32 ni = ny * W + nx;
				if (!Fg[ni] || Label[ni] != -1) { continue; }
				if (FVector3f::DotProduct(Nc, N[ni]) >= CosTol &&
					FMath::Abs(Depth[ni] - Dc) / FMath::Max(Dc, 1.0f) <= DepthJoinTol)
				{
					Label[ni] = NextLabel;
					Stack.Add(ni);
				}
			}
		}
		RegionArea.Add(Count);
		++NextLabel;
	}

	// Keep regions >= MinFaceArea, sorted by area desc -> face ids.
	TArray<int32> KeptLabels;
	for (int32 L = 0; L < NextLabel; ++L)
	{
		if (RegionArea[L] >= MinFaceArea) { KeptLabels.Add(L); }
	}
	KeptLabels.Sort([&](int32 A, int32 B) { return RegionArea[A] > RegionArea[B]; });

	TArray<int32> LabelToFace;
	LabelToFace.Init(-1, NextLabel);
	for (int32 f = 0; f < KeptLabels.Num(); ++f)
	{
		LabelToFace[KeptLabels[f]] = f;
	}
	const int32 NumFaces = KeptLabels.Num();

	// Per-face label mask used for contour tracing + visualization.
	TArray<int32> FaceLabel;
	FaceLabel.Init(-1, NumPx);
	for (int32 i = 0; i < NumPx; ++i)
	{
		if (Label[i] >= 0) { FaceLabel[i] = LabelToFace[Label[i]]; }
	}

	// Camera basis + projection for unprojection.
	const FTransform CamT = CameraTransform;
	const FVector Loc = CamT.GetLocation();
	const FVector Fwd = CamT.GetUnitAxis(EAxis::X);
	const FVector Rgt = CamT.GetUnitAxis(EAxis::Y);
	const FVector Up = CamT.GetUnitAxis(EAxis::Z);
	const float TanX = FMath::Tan(FMath::DegreesToRadians(CameraFov * 0.5));
	const float TanY = TanX * (static_cast<float>(H) / static_cast<float>(W));
	const bool bOrtho = bCaptureOrthographic;

	auto CamRayDir = [&](double px, double py) -> FVector
	{
		const double ndcX = 2.0 * ((px + 0.5) / W) - 1.0;
		const double ndcY = 1.0 - 2.0 * ((py + 0.5) / H);
		return Fwd + Rgt * (ndcX * TanX) + Up * (ndcY * TanY); // forward coeff == 1
	};
	auto OrthoRayOrigin = [&](double px, double py) -> FVector
	{
		const double ndcX = 2.0 * ((px + 0.5) / W) - 1.0;
		const double ndcY = 1.0 - 2.0 * ((py + 0.5) / H);
		double ViewRight = 0.0;
		double ViewUp = 0.0;
		if (!TryGetOrthographicViewPlanePosition(
			CaptureProjectionMatrix,
			ndcX,
			ndcY,
			ViewRight,
			ViewUp))
		{
			return Loc;
		}
		return Loc + Rgt * ViewRight + Up * ViewUp;
	};
	auto Unproject = [&](int32 px, int32 py, float depth) -> FVector
	{
		if (bOrtho)
		{
			return OrthoRayOrigin(px, py) + Fwd * depth;
		}
		// SceneDepth treated as planar (view-space Z); forward coeff of the ray is 1.
		return Loc + CamRayDir(px, py) * depth;
	};

	// Accumulate plane (mean 3D point + mean normal) and 2D centroid per face.
	TArray<FVector> SumPt; SumPt.Init(FVector::ZeroVector, NumFaces);
	TArray<FVector> SumN; SumN.Init(FVector::ZeroVector, NumFaces);
	TArray<int64> Cnt; Cnt.Init(0, NumFaces);
	TArray<FVector2D> SumPx; SumPx.Init(FVector2D::ZeroVector, NumFaces);
	TArray<int32> FirstPixel; FirstPixel.Init(-1, NumFaces);
	for (int32 i = 0; i < NumPx; ++i)
	{
		const int32 f = FaceLabel[i];
		if (f < 0) { continue; }
		const int32 px = i % W;
		const int32 py = i / W;
		SumPt[f] += Unproject(px, py, Depth[i]);
		SumN[f] += FVector(N[i].X, N[i].Y, N[i].Z);
		SumPx[f] += FVector2D(px, py);
		Cnt[f] += 1;
		if (FirstPixel[f] < 0) { FirstPixel[f] = i; }
	}

	// Build JSON + faces visualization.
	TArray<FColor> Out;
	Out.Init(FColor(255, 255, 255, 255), NumPx);
	TSet<uint32> UsedFaceColorKeys;

	FString Json;
	Json += TEXT("{\n");
	Json += FString::Printf(TEXT("  \"image\": { \"w\": %d, \"h\": %d },\n"), W, H);
	Json += FString::Printf(TEXT("  \"num_faces\": %d,\n"), NumFaces);
	Json += TEXT("  \"palette_version\": 2,\n");
	Json += TEXT("  \"unique_face_colors\": true,\n");
	Json += TEXT("  \"projection_source\": \"FMinimalViewInfo::CalculateProjectionMatrix\",\n");
	Json += FString::Printf(TEXT("  \"ortho_width_metadata\": %.17g,\n"), CaptureOrthoWidth);
	Json += FString::Printf(
		TEXT("  \"projection_scale\": { \"m00\": %.17g, \"m11\": %.17g, \"m30\": %.17g, \"m31\": %.17g },\n"),
		CaptureProjectionMatrix.M[0][0],
		CaptureProjectionMatrix.M[1][1],
		CaptureProjectionMatrix.M[3][0],
		CaptureProjectionMatrix.M[3][1]);
	Json += TEXT("  \"faces\": [\n");

	for (int32 f = 0; f < NumFaces; ++f)
	{
		FColor Col;
		if (!GetUniqueFaceColor(f, UsedFaceColorKeys, Col))
		{
			UE_LOG(LogTemp, Error, TEXT("CaptureFromLZ: failed to allocate a unique face color for face_id=%d; faces output skipped."), f);
			return false;
		}
		const uint32 ColKey = FaceColorKey(Col);

		// Paint region pixels.
		for (int32 i = 0; i < NumPx; ++i)
		{
			if (FaceLabel[i] == f) { Out[i] = Col; }
		}

		const double InvCnt = Cnt[f] > 0 ? 1.0 / static_cast<double>(Cnt[f]) : 0.0;
		const FVector PlanePt = SumPt[f] * InvCnt;
		FVector PlaneN = SumN[f];
		PlaneN = PlaneN.GetSafeNormal();
		const FVector2D Centroid2D = SumPx[f] * InvCnt;

		// Contour -> RDP corners (2D). Anchor RDP at the contour point farthest from
		// the centroid so a true corner is an endpoint of the open polyline.
		TArray<FIntPoint> Contour;
		TraceContour(FaceLabel, W, H, f, FirstPixel[f], Contour);

		TArray<FVector2D> Corners2D;
		if (Contour.Num() >= 3)
		{
			TArray<FVector2D> CP;
			CP.Reserve(Contour.Num());
			for (const FIntPoint& P : Contour) { CP.Add(FVector2D(P.X, P.Y)); }

			int32 Anchor = 0;
			double BestD = -1.0;
			for (int32 i = 0; i < CP.Num(); ++i)
			{
				const double D = FVector2D::DistSquared(CP[i], Centroid2D);
				if (D > BestD) { BestD = D; Anchor = i; }
			}
			TArray<FVector2D> Rot;
			Rot.Reserve(CP.Num() + 1);
			for (int32 i = 0; i < CP.Num(); ++i) { Rot.Add(CP[(Anchor + i) % CP.Num()]); }
			const FVector2D ClosePt = Rot[0];
			Rot.Add(ClosePt); // close

			TArray<int32> Keep;
			Keep.Add(0);
			RDP(Rot, 0, Rot.Num() - 1, RdpEps, Keep);
			// drop the duplicated closing point's index if present
			for (int32 idx : Keep)
			{
				if (idx >= 0 && idx < Rot.Num() - 1) { Corners2D.Add(Rot[idx]); }
			}
		}

		// Corner 3D = camera ray through the corner intersected with the fitted plane.
		TArray<FVector> Corners3D;
		Corners3D.Reserve(Corners2D.Num());
		for (const FVector2D& C2 : Corners2D)
		{
			const FVector RayOrigin = bOrtho ? OrthoRayOrigin(C2.X, C2.Y) : Loc;
			const FVector Dir = bOrtho ? Fwd : CamRayDir(C2.X, C2.Y).GetSafeNormal();
			const double Denom = FVector::DotProduct(Dir, PlaneN);
			FVector Hit = PlanePt;
			if (FMath::Abs(Denom) > 1e-5)
			{
				const double T = FVector::DotProduct(PlanePt - RayOrigin, PlaneN) / Denom;
				Hit = RayOrigin + Dir * T;
			}
			Corners3D.Add(Hit);
		}

		// JSON entry.
		Json += TEXT("    {\n");
		Json += FString::Printf(TEXT("      \"id\": %d,\n"), f);
		Json += FString::Printf(TEXT("      \"color_rgb\": [%d, %d, %d],\n"), Col.R, Col.G, Col.B);
		Json += FString::Printf(TEXT("      \"color_key\": %u,\n"), ColKey);
		Json += FString::Printf(TEXT("      \"area_px\": %lld,\n"), static_cast<long long>(Cnt[f]));
		Json += FString::Printf(TEXT("      \"normal_world\": [%.5f, %.5f, %.5f],\n"), PlaneN.X, PlaneN.Y, PlaneN.Z);
		Json += FString::Printf(TEXT("      \"plane_point\": [%.3f, %.3f, %.3f],\n"), PlanePt.X, PlanePt.Y, PlanePt.Z);
		Json += FString::Printf(TEXT("      \"centroid_2d\": [%.2f, %.2f],\n"), Centroid2D.X, Centroid2D.Y);

		Json += TEXT("      \"key_points_2d\": [");
		for (int32 k = 0; k < Corners2D.Num(); ++k)
		{
			Json += FString::Printf(TEXT("%s[%.2f, %.2f]"), (k == 0 ? TEXT("") : TEXT(", ")), Corners2D[k].X, Corners2D[k].Y);
		}
		Json += TEXT("],\n");

		Json += TEXT("      \"key_points_3d\": [");
		for (int32 k = 0; k < Corners3D.Num(); ++k)
		{
			Json += FString::Printf(TEXT("%s[%.3f, %.3f, %.3f]"), (k == 0 ? TEXT("") : TEXT(", ")), Corners3D[k].X, Corners3D[k].Y, Corners3D[k].Z);
		}
		Json += TEXT("]\n");

		Json += FString::Printf(TEXT("    }%s\n"), (f + 1 < NumFaces ? TEXT(",") : TEXT("")));
	}
	Json += TEXT("  ]\n}\n");

	// Save faces PNG (FColor is BGRA in memory).
	IImageWrapperModule& IWM = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
	TSharedPtr<IImageWrapper> IW = IWM.CreateImageWrapper(EImageFormat::PNG);
	bool bSavedPng = false;
	if (IW.IsValid())
	{
		IW->SetRaw(Out.GetData(), Out.Num() * sizeof(FColor), W, H, ERGBFormat::BGRA, 8);
		const TArray64<uint8>& Compressed = IW->GetCompressed();
		bSavedPng = FFileHelper::SaveArrayToFile(TArrayView<const uint8>(Compressed.GetData(), static_cast<int32>(Compressed.Num())), *FacesPngPath);
	}
	const bool bSavedJson = FFileHelper::SaveStringToFile(Json, *FacesJsonPath);

	// === Debug companion: dimmed face colors + decimal face_id text label per region. ===
	// Helps disambiguate palette hue-bucket collisions (faces 0..4 are all reds, 5..9 oranges, etc.)
	// from real flood-fill merges (a single label sprawling across multiple physical surfaces).
	{
		TArray<FColor> OutDebug;
		OutDebug.SetNumUninitialized(NumPx);
		for (int32 i = 0; i < NumPx; ++i)
		{
			const FColor& C = Out[i];
			OutDebug[i] = FColor(
				static_cast<uint8>(static_cast<int32>(C.R) * 6 / 10 + 102),
				static_cast<uint8>(static_cast<int32>(C.G) * 6 / 10 + 102),
				static_cast<uint8>(static_cast<int32>(C.B) * 6 / 10 + 102),
				255);
		}

		// For each face find the in-region pixel closest to its 2D centroid; this is the
		// label anchor (avoids placing digits outside concave regions like L-shapes).
		TArray<FVector2D> Centroids2D;
		Centroids2D.SetNum(NumFaces);
		TArray<int32> LabelAnchorIdx;
		LabelAnchorIdx.SetNum(NumFaces);
		TArray<double> BestAnchorDistSq;
		BestAnchorDistSq.SetNum(NumFaces);
		for (int32 f = 0; f < NumFaces; ++f)
		{
			const double Inv = Cnt[f] > 0 ? 1.0 / static_cast<double>(Cnt[f]) : 0.0;
			Centroids2D[f] = SumPx[f] * Inv;
			LabelAnchorIdx[f] = FirstPixel[f];
			BestAnchorDistSq[f] = TNumericLimits<double>::Max();
		}
		for (int32 i = 0; i < NumPx; ++i)
		{
			const int32 fl = FaceLabel[i];
			if (fl < 0) { continue; }
			const int32 px = i % W;
			const int32 py = i / W;
			const double dx = static_cast<double>(px) - Centroids2D[fl].X;
			const double dy = static_cast<double>(py) - Centroids2D[fl].Y;
			const double D = dx * dx + dy * dy;
			if (D < BestAnchorDistSq[fl]) { BestAnchorDistSq[fl] = D; LabelAnchorIdx[fl] = i; }
		}

		const int32 LabelScale = 3;
		const FColor LabelTextColor(0, 0, 0, 255);
		const FColor LabelBgColor(255, 255, 255, 255);
		for (int32 f = 0; f < NumFaces; ++f)
		{
			const int32 Idx = LabelAnchorIdx[f];
			if (Idx < 0) { continue; }
			const int32 cx = Idx % W;
			const int32 cy = Idx / W;
			DrawIntegerLabel(OutDebug, W, H, cx, cy, LabelScale, f, LabelTextColor, LabelBgColor);
		}

		TSharedPtr<IImageWrapper> IWDebug = IWM.CreateImageWrapper(EImageFormat::PNG);
		if (IWDebug.IsValid())
		{
			IWDebug->SetRaw(OutDebug.GetData(), OutDebug.Num() * sizeof(FColor), W, H, ERGBFormat::BGRA, 8);
			const TArray64<uint8>& DebugCompressed = IWDebug->GetCompressed();
			FString DebugPngPath = FacesPngPath;
			DebugPngPath.RemoveFromEnd(TEXT(".png"));
			DebugPngPath += TEXT("_debug.png");
			const bool bSavedDebug = FFileHelper::SaveArrayToFile(
				TArrayView<const uint8>(DebugCompressed.GetData(), static_cast<int32>(DebugCompressed.Num())),
				*DebugPngPath);
			if (bSavedDebug)
			{
				UE_LOG(LogTemp, Log, TEXT("CaptureFromLZ: saved face-id debug png -> %s"), *DebugPngPath);
			}
			else
			{
				UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ: failed to save face-id debug png to %s"), *DebugPngPath);
			}
		}
	}

	if (bSavedPng && bSavedJson)
	{
		UE_LOG(LogTemp, Log, TEXT("CaptureFromLZ: %d planar face(s) with unique colors -> %s"), NumFaces, *FacesJsonPath);
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ: failed to save faces outputs png=%d json=%d (%s / %s)."), bSavedPng ? 1 : 0, bSavedJson ? 1 : 0, *FacesPngPath, *FacesJsonPath);
	}
	return bSavedPng && bSavedJson;
}

namespace FromLZActorMaterialId
{
	struct FProjectedVertex
	{
		FVector2D Pixel = FVector2D::ZeroVector;
		double Depth = 0.0;
		bool bValid = false;
	};

	static FColor EncodeIdColor(int32 Id)
	{
		return FColor(
			uint8((Id >> 16) & 0xff),
			uint8((Id >> 8) & 0xff),
			uint8(Id & 0xff),
			255);
	}

	static TArray<TSharedPtr<FJsonValue>> JsonColorArray(const FColor& Color)
	{
		TArray<TSharedPtr<FJsonValue>> Values;
		Values.Add(MakeShared<FJsonValueNumber>(Color.R));
		Values.Add(MakeShared<FJsonValueNumber>(Color.G));
		Values.Add(MakeShared<FJsonValueNumber>(Color.B));
		return Values;
	}

	static bool ProjectWorldToCapturePixel(
		const FVector& WorldPosition,
		const FTransform& CameraTransform,
		int32 W,
		int32 H,
		const FMatrix& CaptureProjectionMatrix,
		FProjectedVertex& Out)
	{
		Out = FProjectedVertex();
		if (W <= 0 || H <= 0 || !FromLZIsUsableOrthographicProjectionMatrix(CaptureProjectionMatrix))
		{
			return false;
		}

		const FVector Loc = CameraTransform.GetLocation();
		const FVector Fwd = CameraTransform.GetUnitAxis(EAxis::X);
		const FVector Rgt = CameraTransform.GetUnitAxis(EAxis::Y);
		const FVector Up = CameraTransform.GetUnitAxis(EAxis::Z);
		const FVector Delta = WorldPosition - Loc;
		const double Depth = FVector::DotProduct(Delta, Fwd);
		if (!FMath::IsFinite(Depth) || Depth <= 0.0)
		{
			return false;
		}

		const double ViewRight = FVector::DotProduct(Delta, Rgt);
		const double ViewUp = FVector::DotProduct(Delta, Up);
		const double NdcX =
			ViewRight * CaptureProjectionMatrix.M[0][0] +
			CaptureProjectionMatrix.M[3][0];
		const double NdcY =
			ViewUp * CaptureProjectionMatrix.M[1][1] +
			CaptureProjectionMatrix.M[3][1];
		Out.Pixel.X = ((NdcX + 1.0) * 0.5 * double(W)) - 0.5;
		Out.Pixel.Y = ((1.0 - NdcY) * 0.5 * double(H)) - 0.5;
		Out.Depth = Depth;
		Out.bValid = FMath::IsFinite(Out.Pixel.X) && FMath::IsFinite(Out.Pixel.Y);
		return Out.bValid;
	}

	static bool Barycentric2D(
		const FVector2D& P,
		const FVector2D& A,
		const FVector2D& B,
		const FVector2D& C,
		double& OutA,
		double& OutB,
		double& OutC)
	{
		const double Denom =
			(double(B.Y) - double(C.Y)) * (double(A.X) - double(C.X)) +
			(double(C.X) - double(B.X)) * (double(A.Y) - double(C.Y));
		if (FMath::Abs(Denom) < 1e-8)
		{
			return false;
		}

		OutA = ((double(B.Y) - double(C.Y)) * (double(P.X) - double(C.X)) +
			(double(C.X) - double(B.X)) * (double(P.Y) - double(C.Y))) / Denom;
		OutB = ((double(C.Y) - double(A.Y)) * (double(P.X) - double(C.X)) +
			(double(A.X) - double(C.X)) * (double(P.Y) - double(C.Y))) / Denom;
		OutC = 1.0 - OutA - OutB;
		return FMath::IsFinite(OutA) && FMath::IsFinite(OutB) && FMath::IsFinite(OutC);
	}

	static int32 FindOrAddEntry(
		UPrimitiveComponent* Component,
		int32 MaterialSlot,
		TMap<FString, int32>& EntryIdByKey,
		TArray<TSharedPtr<FJsonValue>>& Entries)
	{
		if (!Component || MaterialSlot < 0)
		{
			return 0;
		}

		const FString Key = FString::Printf(TEXT("%s|%d"), *Component->GetPathName(), MaterialSlot);
		if (const int32* Existing = EntryIdByKey.Find(Key))
		{
			return *Existing;
		}

		const int32 Id = Entries.Num() + 1;
		if (Id > 0x00ffffff)
		{
			return 0;
		}

		AActor* Owner = Component->GetOwner();
		UMaterialInterface* Material = Component->GetMaterial(MaterialSlot);
		const FColor Color = EncodeIdColor(Id);
		TSharedRef<FJsonObject> Entry = MakeShared<FJsonObject>();
		Entry->SetNumberField(TEXT("id"), Id);
		Entry->SetArrayField(TEXT("color_rgb"), JsonColorArray(Color));
		Entry->SetStringField(TEXT("actor_name"), Owner ? Owner->GetName() : FString());
		Entry->SetStringField(TEXT("actor_path"), Owner ? Owner->GetPathName() : FString());
#if WITH_EDITOR
		Entry->SetStringField(TEXT("actor_label"), Owner ? Owner->GetActorLabel() : FString());
#endif
		Entry->SetStringField(TEXT("component_name"), Component->GetName());
		Entry->SetStringField(TEXT("component_path"), Component->GetPathName());
		Entry->SetStringField(TEXT("component_type"), Component->GetClass() ? Component->GetClass()->GetName() : FString());
		Entry->SetNumberField(TEXT("material_slot"), MaterialSlot);
		Entry->SetStringField(TEXT("material_name"), GetNameSafe(Material));
		Entry->SetStringField(TEXT("material_path"), Material ? Material->GetPathName() : FString());
		Entries.Add(MakeShared<FJsonValueObject>(Entry));
		EntryIdByKey.Add(Key, Id);
		return Id;
	}

	static void RasterizeTriangle(
		const FProjectedVertex& A,
		const FProjectedVertex& B,
		const FProjectedVertex& C,
		int32 Id,
		int32 W,
		int32 H,
		TArray<double>& ZBuffer,
		TArray<FColor>& OutPixels)
	{
		if (!A.bValid || !B.bValid || !C.bValid || Id <= 0)
		{
			return;
		}

		const double MinX = FMath::Min3(A.Pixel.X, B.Pixel.X, C.Pixel.X);
		const double MaxX = FMath::Max3(A.Pixel.X, B.Pixel.X, C.Pixel.X);
		const double MinY = FMath::Min3(A.Pixel.Y, B.Pixel.Y, C.Pixel.Y);
		const double MaxY = FMath::Max3(A.Pixel.Y, B.Pixel.Y, C.Pixel.Y);
		int32 X0 = FMath::Clamp(FMath::FloorToInt(MinX), 0, W - 1);
		int32 X1 = FMath::Clamp(FMath::CeilToInt(MaxX), 0, W - 1);
		int32 Y0 = FMath::Clamp(FMath::FloorToInt(MinY), 0, H - 1);
		int32 Y1 = FMath::Clamp(FMath::CeilToInt(MaxY), 0, H - 1);
		if (X1 < X0 || Y1 < Y0)
		{
			return;
		}

		const FColor Color = EncodeIdColor(Id);
		for (int32 Y = Y0; Y <= Y1; ++Y)
		{
			for (int32 X = X0; X <= X1; ++X)
			{
				double Wa = 0.0;
				double Wb = 0.0;
				double Wc = 0.0;
				const FVector2D P(static_cast<double>(X), static_cast<double>(Y));
				if (!Barycentric2D(P, A.Pixel, B.Pixel, C.Pixel, Wa, Wb, Wc))
				{
					continue;
				}

				constexpr double EdgeTolerance = -1e-4;
				if (Wa < EdgeTolerance || Wb < EdgeTolerance || Wc < EdgeTolerance)
				{
					continue;
				}

				const double Depth = Wa * A.Depth + Wb * B.Depth + Wc * C.Depth;
				const int32 PixelIndex = Y * W + X;
				if (Depth > 0.0 && Depth < ZBuffer[PixelIndex])
				{
					ZBuffer[PixelIndex] = Depth;
					OutPixels[PixelIndex] = Color;
				}
			}
		}
	}

	static void RasterizeStaticMeshComponent(
		UStaticMeshComponent* Component,
		const FTransform& CameraTransform,
		const FMatrix& CaptureProjectionMatrix,
		int32 W,
		int32 H,
		TMap<FString, int32>& EntryIdByKey,
		TArray<TSharedPtr<FJsonValue>>& Entries,
		TArray<double>& ZBuffer,
		TArray<FColor>& OutPixels)
	{
		if (!Component || !Component->IsRegistered() || !Component->IsVisible())
		{
			return;
		}

		UStaticMesh* StaticMesh = Component->GetStaticMesh();
		if (!StaticMesh || !StaticMesh->GetRenderData() || StaticMesh->GetRenderData()->LODResources.Num() == 0)
		{
			return;
		}

		const FStaticMeshLODResources& LOD = StaticMesh->GetRenderData()->LODResources[0];
		const FPositionVertexBuffer& PositionBuffer = LOD.VertexBuffers.PositionVertexBuffer;
		const FIndexArrayView Indices = LOD.IndexBuffer.GetArrayView();
		const int32 NumVertices = PositionBuffer.GetNumVertices();
		if (NumVertices <= 0 || Indices.Num() < 3)
		{
			return;
		}

		TArray<FProjectedVertex> Projected;
		Projected.SetNum(NumVertices);
		const FTransform& ComponentTransform = Component->GetComponentTransform();
		for (int32 VertexIndex = 0; VertexIndex < NumVertices; ++VertexIndex)
		{
			const FVector3f Local = PositionBuffer.VertexPosition(VertexIndex);
			const FVector World = ComponentTransform.TransformPosition(FVector(Local.X, Local.Y, Local.Z));
			ProjectWorldToCapturePixel(World, CameraTransform, W, H, CaptureProjectionMatrix, Projected[VertexIndex]);
		}

		for (const FStaticMeshSection& Section : LOD.Sections)
		{
			const int32 MaterialSlot = int32(Section.MaterialIndex);
			const int32 Id = FindOrAddEntry(Component, MaterialSlot, EntryIdByKey, Entries);
			for (uint32 TriIndex = 0; TriIndex < Section.NumTriangles; ++TriIndex)
			{
				const uint32 IndexBase = Section.FirstIndex + TriIndex * 3;
				if (IndexBase + 2 >= uint32(Indices.Num()))
				{
					continue;
				}
				const int32 IA = int32(Indices[IndexBase]);
				const int32 IB = int32(Indices[IndexBase + 1]);
				const int32 IC = int32(Indices[IndexBase + 2]);
				if (!Projected.IsValidIndex(IA) || !Projected.IsValidIndex(IB) || !Projected.IsValidIndex(IC))
				{
					continue;
				}
				RasterizeTriangle(Projected[IA], Projected[IB], Projected[IC], Id, W, H, ZBuffer, OutPixels);
			}
		}
	}

	static void RasterizeProceduralMeshComponent(
		UProceduralMeshComponent* Component,
		const FTransform& CameraTransform,
		const FMatrix& CaptureProjectionMatrix,
		int32 W,
		int32 H,
		TMap<FString, int32>& EntryIdByKey,
		TArray<TSharedPtr<FJsonValue>>& Entries,
		TArray<double>& ZBuffer,
		TArray<FColor>& OutPixels)
	{
		if (!Component || !Component->IsRegistered() || !Component->IsVisible())
		{
			return;
		}

		const FTransform& ComponentTransform = Component->GetComponentTransform();
		for (int32 SectionIndex = 0; SectionIndex < Component->GetNumSections(); ++SectionIndex)
		{
			if (!Component->IsMeshSectionVisible(SectionIndex))
			{
				continue;
			}

			const FProcMeshSection* Section = Component->GetProcMeshSection(SectionIndex);
			if (!Section || Section->ProcVertexBuffer.Num() < 3 || Section->ProcIndexBuffer.Num() < 3)
			{
				continue;
			}

			TArray<FProjectedVertex> Projected;
			Projected.SetNum(Section->ProcVertexBuffer.Num());
			for (int32 VertexIndex = 0; VertexIndex < Section->ProcVertexBuffer.Num(); ++VertexIndex)
			{
				const FVector World = ComponentTransform.TransformPosition(Section->ProcVertexBuffer[VertexIndex].Position);
				ProjectWorldToCapturePixel(World, CameraTransform, W, H, CaptureProjectionMatrix, Projected[VertexIndex]);
			}

			const int32 Id = FindOrAddEntry(Component, SectionIndex, EntryIdByKey, Entries);
			for (int32 Index = 0; Index + 2 < Section->ProcIndexBuffer.Num(); Index += 3)
			{
				const int32 IA = Section->ProcIndexBuffer[Index];
				const int32 IB = Section->ProcIndexBuffer[Index + 1];
				const int32 IC = Section->ProcIndexBuffer[Index + 2];
				if (!Projected.IsValidIndex(IA) || !Projected.IsValidIndex(IB) || !Projected.IsValidIndex(IC))
				{
					continue;
				}
				RasterizeTriangle(Projected[IA], Projected[IB], Projected[IC], Id, W, H, ZBuffer, OutPixels);
			}
		}
	}

	static bool SaveActorMaterialIdBuffer(
		UWorld* World,
		const FTransform& CameraTransform,
		double CaptureOrthoWidth,
		const FMatrix& CaptureProjectionMatrix,
		const TArray<AActor*>& SubjectActors,
		const FString& SubjectMode,
		int32 W,
		int32 H,
		const FString& PngPath,
		const FString& JsonPath)
	{
		if (!World || W <= 0 || H <= 0 ||
			CaptureOrthoWidth <= 1e-6 ||
			!FromLZIsUsableOrthographicProjectionMatrix(CaptureProjectionMatrix))
		{
			return false;
		}

		TArray<FColor> Pixels;
		Pixels.Init(FColor(0, 0, 0, 0), W * H);
		TArray<double> ZBuffer;
		ZBuffer.Init(TNumericLimits<double>::Max(), W * H);
		TMap<FString, int32> EntryIdByKey;
		TArray<TSharedPtr<FJsonValue>> Entries;

		auto RasterizeActor = [&] (AActor* Actor)
		{
			if (!IsValid(Actor) || Actor->IsHidden())
			{
				return;
			}

			TArray<UStaticMeshComponent*> StaticComponents;
			Actor->GetComponents<UStaticMeshComponent>(StaticComponents);
			for (UStaticMeshComponent* Component : StaticComponents)
			{
				RasterizeStaticMeshComponent(Component, CameraTransform, CaptureProjectionMatrix, W, H, EntryIdByKey, Entries, ZBuffer, Pixels);
			}

			TArray<UProceduralMeshComponent*> ProceduralComponents;
			Actor->GetComponents<UProceduralMeshComponent>(ProceduralComponents);
			for (UProceduralMeshComponent* Component : ProceduralComponents)
			{
				RasterizeProceduralMeshComponent(Component, CameraTransform, CaptureProjectionMatrix, W, H, EntryIdByKey, Entries, ZBuffer, Pixels);
			}
		};

		for (AActor* Actor : SubjectActors)
		{
			RasterizeActor(Actor);
		}

		IImageWrapperModule& IWM = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
		TSharedPtr<IImageWrapper> IW = IWM.CreateImageWrapper(EImageFormat::PNG);
		if (!IW.IsValid())
		{
			return false;
		}
		IW->SetRaw(Pixels.GetData(), Pixels.Num() * sizeof(FColor), W, H, ERGBFormat::BGRA, 8);
		const TArray64<uint8>& Compressed = IW->GetCompressed();
		const bool bSavedPng = FFileHelper::SaveArrayToFile(
			TArrayView<const uint8>(Compressed.GetData(), static_cast<int32>(Compressed.Num())),
			*PngPath);

		TSharedRef<FJsonObject> Root = MakeShared<FJsonObject>();
		Root->SetNumberField(TEXT("version"), 1);
		TSharedRef<FJsonObject> ImageObject = MakeShared<FJsonObject>();
		ImageObject->SetNumberField(TEXT("w"), W);
		ImageObject->SetNumberField(TEXT("h"), H);
		Root->SetObjectField(TEXT("image"), ImageObject);
		Root->SetStringField(TEXT("encoding"), TEXT("rgb24_id_background_zero"));
		Root->SetStringField(TEXT("projection_mode"), TEXT("Orthographic"));
		Root->SetNumberField(TEXT("ortho_width"), CaptureOrthoWidth);
		Root->SetStringField(TEXT("projection_source"), TEXT("FMinimalViewInfo::CalculateProjectionMatrix"));
		Root->SetObjectField(TEXT("projection_matrix"), SerializeMatrix(CaptureProjectionMatrix));
		Root->SetStringField(TEXT("subject_mode"), SubjectMode);
		Root->SetNumberField(TEXT("subject_actor_count"), SubjectActors.Num());
		Root->SetArrayField(TEXT("entries"), Entries);

		FString JsonText;
		const TSharedRef<TJsonWriter<TCHAR, TPrettyJsonPrintPolicy<TCHAR>>> Writer =
			TJsonWriterFactory<TCHAR, TPrettyJsonPrintPolicy<TCHAR>>::Create(&JsonText);
		const bool bSerialized = FJsonSerializer::Serialize(Root, Writer);
		const bool bSavedJson = bSerialized && FFileHelper::SaveStringToFile(JsonText, *JsonPath, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);

		if (bSavedPng && bSavedJson)
		{
			UE_LOG(LogTemp, Log, TEXT("CaptureFromLZ: actor/material id buffer entries=%d -> %s"), Entries.Num(), *JsonPath);
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ: failed to save actor/material id buffer png=%d json=%d (%s / %s)."), bSavedPng ? 1 : 0, bSavedJson ? 1 : 0, *PngPath, *JsonPath);
		}
		return bSavedPng && bSavedJson;
	}
}

static bool SaveFColorPng(const TArray<FColor>& Pixels, int32 W, int32 H, const FString& OutputPath, const TCHAR* DebugName)
{
	if (W <= 0 || H <= 0 || Pixels.Num() != W * H)
	{
		UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ: invalid %s debug png buffer (%dx%d, pixels=%d)."), DebugName, W, H, Pixels.Num());
		return false;
	}

	IImageWrapperModule& IWM = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
	TSharedPtr<IImageWrapper> IW = IWM.CreateImageWrapper(EImageFormat::PNG);
	if (!IW.IsValid())
	{
		UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ: failed to create image wrapper for %s debug png."), DebugName);
		return false;
	}

	IW->SetRaw(Pixels.GetData(), Pixels.Num() * sizeof(FColor), W, H, ERGBFormat::BGRA, 8);
	const TArray64<uint8>& Compressed = IW->GetCompressed();
	const bool bSaved = FFileHelper::SaveArrayToFile(
		TArrayView<const uint8>(Compressed.GetData(), static_cast<int32>(Compressed.Num())),
		*OutputPath);

	if (bSaved)
	{
		UE_LOG(LogTemp, Log, TEXT("CaptureFromLZ: saved %s debug png to %s"), DebugName, *OutputPath);
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ: failed to save %s debug png to %s"), DebugName, *OutputPath);
	}
	return bSaved;
}

static APlayerController* GetPawnPlayerController(const APawn* Pawn)
{
	return Pawn ? Cast<APlayerController>(Pawn->GetController()) : nullptr;
}

static bool CalculateAspectFittedCaptureSize(
	const FIntPoint& MaxSize,
	double AspectRatio,
	FIntPoint& OutSize)
{
	OutSize = FIntPoint::ZeroValue;
	if (MaxSize.X <= 0 || MaxSize.Y <= 0 ||
		!FMath::IsFinite(AspectRatio) || AspectRatio <= 1e-6)
	{
		return false;
	}

	const double MaxAspectRatio = double(MaxSize.X) / double(MaxSize.Y);
	int32 Width = MaxSize.X;
	int32 Height = MaxSize.Y;
	if (MaxAspectRatio > AspectRatio)
	{
		Width = FMath::FloorToInt(double(MaxSize.Y) * AspectRatio);
	}
	else
	{
		Height = FMath::FloorToInt(double(MaxSize.X) / AspectRatio);
	}

	Width = FMath::Clamp(Width, 1, MaxSize.X);
	Height = FMath::Clamp(Height, 1, MaxSize.Y);
	OutSize = FIntPoint(FMath::Max(Width, 1), FMath::Max(Height, 1));
	return true;
}

static bool BuildOffscreenOrthographicCaptureView(
	const APawn* Pawn,
	UCameraComponent* Camera,
	FViewport* Viewport,
	const TArray<AActor*>& FramingActors,
	bool bAllowSubjectBoundsCenterOrthoWidth,
	FIntPoint CaptureResolutionOverride,
	FFromLZCaptureView& OutView,
	FFromLZCameraProjectionSnapshot& OutSourceProjection,
	bool& bOutUsedDefaultOrthoWidth,
	bool& bOutUsedSubjectBoundsCenterOrthoWidth,
	FVector& OutSubjectBoundsCenter,
	double& OutSubjectFocusDepth)
{
	if (!Pawn || !Camera || !Viewport)
	{
		return false;
	}

	const FIntPoint MaxSize = (CaptureResolutionOverride.X > 0 && CaptureResolutionOverride.Y > 0)
		? CaptureResolutionOverride
		: Viewport->GetSizeXY();
	OutSourceProjection = MakeCameraProjectionSnapshot(Camera);

	const FTransform SourceTransform = Camera->GetComponentTransform();
	FMinimalViewInfo ViewInfo;
	ViewInfo.Location = SourceTransform.GetLocation();
	ViewInfo.Rotation = SourceTransform.Rotator();
	ViewInfo.FOV = Camera->FieldOfView;
	ViewInfo.DesiredFOV = Camera->FieldOfView;
	ViewInfo.AspectRatio = Camera->AspectRatio;
	ViewInfo.OrthoWidth = Camera->OrthoWidth;
	if (!FMath::IsFinite(static_cast<double>(ViewInfo.AspectRatio)) || ViewInfo.AspectRatio <= 1e-6f)
	{
		UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ: selected camera %s has an invalid AspectRatio %.9g."), *Camera->GetName(), ViewInfo.AspectRatio);
		return false;
	}

	FIntPoint CaptureSize;
	if (!CalculateAspectFittedCaptureSize(MaxSize, ViewInfo.AspectRatio, CaptureSize))
	{
		UE_LOG(
			LogTemp,
			Warning,
			TEXT("CaptureFromLZ: could not fit camera aspect ratio %.9g inside viewport %dx%d."),
			ViewInfo.AspectRatio,
			MaxSize.X,
			MaxSize.Y);
		return false;
	}

	const double SourceOrthoWidth = static_cast<double>(ViewInfo.OrthoWidth);
	bOutUsedDefaultOrthoWidth =
		!FMath::IsFinite(SourceOrthoWidth) || SourceOrthoWidth <= 1e-6;
	bOutUsedSubjectBoundsCenterOrthoWidth = false;
	OutSubjectBoundsCenter = FVector::ZeroVector;
	OutSubjectFocusDepth = 0.0;
	double SubjectBoundsOrthoWidth = 0.0;
	if (bAllowSubjectBoundsCenterOrthoWidth &&
		FFromLZCaptureUtils::CalculateSubjectBoundsCenterOrthoWidth(
		Camera,
		FramingActors,
		SubjectBoundsOrthoWidth,
		OutSubjectBoundsCenter,
		OutSubjectFocusDepth))
	{
		ViewInfo.OrthoWidth = static_cast<float>(SubjectBoundsOrthoWidth);
		bOutUsedDefaultOrthoWidth = false;
		bOutUsedSubjectBoundsCenterOrthoWidth = true;
	}
	ViewInfo.ProjectionMode = ECameraProjectionMode::Orthographic;
	if (!bOutUsedSubjectBoundsCenterOrthoWidth)
	{
		ViewInfo.OrthoWidth = static_cast<float>(
			bOutUsedDefaultOrthoWidth ? FromLZDefaultOrthoWidth : SourceOrthoWidth);
	}
	ViewInfo.OrthoNearClipPlane = FromLZCaptureOrthoNearPlane;
	ViewInfo.OrthoFarClipPlane = FromLZCaptureOrthoFarPlane;
	ViewInfo.bAutoCalculateOrthoPlanes = false;
	ViewInfo.AutoPlaneShift = 0.0f;
	ViewInfo.bUpdateOrthoPlanes = false;
	ViewInfo.bUseCameraHeightAsViewTarget = false;
	ViewInfo.bConstrainAspectRatio = true;

	OutView = FFromLZCaptureView();
	OutView.SourceViewTransform = SourceTransform;
	OutView.Transform = OutView.SourceViewTransform;
	OutView.Fov = ViewInfo.FOV;
	OutView.OrthoWidth = ViewInfo.OrthoWidth;
	OutView.OrthoBackoff = 0.0;
	OutView.ProjectionMatrix = ViewInfo.CalculateProjectionMatrix();
	OutView.bUseCustomProjectionMatrix = FromLZIsUsableOrthographicProjectionMatrix(OutView.ProjectionMatrix);
	OutView.SourceViewTransformSource = TEXT("selected_camera_component_transform_snapshot");
	OutView.TransformSource = TEXT("selected_camera_component_transform_snapshot");
	OutView.FramingMode = TEXT("camera_aspect_ratio_max_inscribed_offscreen");
	OutView.OrthoWidthMode = bOutUsedDefaultOrthoWidth
		? TEXT("default_ortho_width_fallback")
		: (bOutUsedSubjectBoundsCenterOrthoWidth
			? TEXT("subject_bounds_center_from_perspective_fov")
			: TEXT("selected_camera_ortho_width"));
	OutView.MaxViewportSize = MaxSize;
	OutView.ViewportSize = CaptureSize;
	OutView.ViewportAspectRatio = double(CaptureSize.X) / double(CaptureSize.Y);

	APlayerController* PlayerController = GetPawnPlayerController(Pawn);
	OutView.bHasPlayerCameraManager = PlayerController && PlayerController->PlayerCameraManager;

	const FVector SourceViewLocation = OutView.SourceViewTransform.GetLocation();
	const FVector SourceViewForward = OutView.SourceViewTransform.GetUnitAxis(EAxis::X).GetSafeNormal();
	OutView.FocusDepth = OutView.OrthoWidth * 0.5;
	OutView.FocusPoint = SourceViewLocation + SourceViewForward * OutView.FocusDepth;
	OutView.FocusSource = TEXT("selected camera orthographic forward reference");
	OutView.ReferencePoint = OutView.FocusPoint;
	OutView.ReferenceDepth = OutView.FocusDepth;
	OutView.ReferenceSource = OutView.FocusSource;

	UE_LOG(
		LogTemp,
		Log,
		TEXT("CaptureFromLZ: built offscreen orthographic view camera=%s width=%.3f aspect=%.9g max=%dx%d output=%dx%d mode=%s."),
		*Camera->GetName(),
		OutView.OrthoWidth,
		ViewInfo.AspectRatio,
		MaxSize.X,
		MaxSize.Y,
		OutView.ViewportSize.X,
		OutView.ViewportSize.Y,
		*OutView.OrthoWidthMode);

	return
		OutView.OrthoWidth > 1e-6 &&
		!OutView.Transform.ContainsNaN() &&
		OutView.bUseCustomProjectionMatrix;
}

static void AddVectorFields(const TSharedRef<FJsonObject>& Object, const TCHAR* Prefix, const FVector& Value)
{
	Object->SetNumberField(FString::Printf(TEXT("%s_x"), Prefix), Value.X);
	Object->SetNumberField(FString::Printf(TEXT("%s_y"), Prefix), Value.Y);
	Object->SetNumberField(FString::Printf(TEXT("%s_z"), Prefix), Value.Z);
}

static TSharedRef<FJsonObject> SerializeVector(const FVector& Value)
{
	TSharedRef<FJsonObject> Object = MakeShared<FJsonObject>();
	AddVectorFields(Object, TEXT("value"), Value);
	return Object;
}

static TSharedRef<FJsonObject> SerializeMatrix(const FMatrix& Matrix)
{
	TSharedRef<FJsonObject> Object = MakeShared<FJsonObject>();
	for (int32 Row = 0; Row < 4; ++Row)
	{
		TArray<TSharedPtr<FJsonValue>> Values;
		for (int32 Column = 0; Column < 4; ++Column)
		{
			Values.Add(MakeShared<FJsonValueNumber>(Matrix.M[Row][Column]));
		}
		Object->SetArrayField(FString::Printf(TEXT("row_%d"), Row), Values);
	}
	return Object;
}

static void ConfigureSceneCaptureOrthoWithoutViewOriginShift(USceneCaptureComponent2D* SceneCapture)
{
	if (!SceneCapture)
	{
		return;
	}

	SceneCapture->bAutoCalculateOrthoPlanes = false;
	SceneCapture->AutoPlaneShift = 0.0f;
	SceneCapture->bUpdateOrthoPlanes = false;
	SceneCapture->bUseCameraHeightAsViewTarget = false;
}

static bool ConfigureSceneCaptureFromCaptureView(
	USceneCaptureComponent2D* SceneCapture,
	const FFromLZCaptureView& CaptureView)
{
	if (!SceneCapture ||
		!CaptureView.bUseCustomProjectionMatrix ||
		!FromLZIsUsableOrthographicProjectionMatrix(CaptureView.ProjectionMatrix))
	{
		return false;
	}

	SceneCapture->ProjectionType = ECameraProjectionMode::Orthographic;
	SceneCapture->FOVAngle = CaptureView.Fov;
	SceneCapture->OrthoWidth = CaptureView.OrthoWidth;
	ConfigureSceneCaptureOrthoWithoutViewOriginShift(SceneCapture);
	SceneCapture->bUseCustomProjectionMatrix = true;
	SceneCapture->CustomProjectionMatrix = CaptureView.ProjectionMatrix;
	return true;
}

static bool ActorHasTag(const AActor* Actor, FName Tag)
{
	return Actor && Actor->ActorHasTag(Tag);
}

static bool ActorIsTaggedBaseCaptureSubject(const AActor* Actor)
{
	return ActorHasTag(Actor, FromLZCaptureSubjectTag);
}

static bool ActorIsTaggedCapturePlane(const AActor* Actor)
{
	return ActorHasTag(Actor, FromLZCapturePlaneTag);
}

static bool ActorHasVisibleCaptureMeshComponent(AActor* Actor)
{
	if (!Actor)
	{
		return false;
	}

	TArray<UStaticMeshComponent*> StaticComponents;
	Actor->GetComponents<UStaticMeshComponent>(StaticComponents);
	for (UStaticMeshComponent* Component : StaticComponents)
	{
		if (Component && Component->IsRegistered() && Component->IsVisible() && Component->GetStaticMesh())
		{
			return true;
		}
	}

	TArray<UProceduralMeshComponent*> ProceduralComponents;
	Actor->GetComponents<UProceduralMeshComponent>(ProceduralComponents);
	for (UProceduralMeshComponent* Component : ProceduralComponents)
	{
		if (!Component || !Component->IsRegistered() || !Component->IsVisible())
		{
			continue;
		}

		for (int32 SectionIndex = 0; SectionIndex < Component->GetNumSections(); ++SectionIndex)
		{
			if (Component->IsMeshSectionVisible(SectionIndex))
			{
				return true;
			}
		}
	}

	return false;
}

static bool IsCaptureSubjectCandidate(AActor* Actor, const APawn* Pawn, const AActor* CameraActor)
{
	return Actor &&
		Actor != Pawn &&
		Actor != CameraActor &&
		!Actor->IsHidden() &&
		ActorHasVisibleCaptureMeshComponent(Actor);
}

static void BuildCaptureSubjectActors(
	UWorld* World,
	const APawn* Pawn,
	const AActor* CameraActor,
	const FVector& CameraLocation,
	TArray<AActor*>& OutActors,
	FString& OutMode)
{
	OutActors.Reset();
	OutMode = TEXT("none");
	if (!World)
	{
		return;
	}

	const bool bHasCaptureCamera =
		FMath::IsFinite(CameraLocation.X) &&
		FMath::IsFinite(CameraLocation.Y) &&
		FMath::IsFinite(CameraLocation.Z);

	int32 Step11RuntimeSubjectCount = 0;
	int32 BboxAtOrBelowCameraKeptCount = 0;
	int32 BboxAboveOutsideXYKeptCount = 0;
	int32 BboxAboveInsideXYExcludedCount = 0;
	int32 BboxCameraUnavailableKeptCount = 0;
	int32 BaseSubjectCount = 0;
	int32 CapturePlaneCount = 0;
	for (TActorIterator<AActor> It(World); It; ++It)
	{
		AActor* Actor = *It;
		if (!IsCaptureSubjectCandidate(Actor, Pawn, CameraActor))
		{
			continue;
		}

		const bool bStep11RuntimeActor = FFromLZFaceReconstructor::IsStep11RuntimeActor(Actor);
		const bool bBaseSubject = !bStep11RuntimeActor && ActorIsTaggedBaseCaptureSubject(Actor);
		const bool bCapturePlane = !bStep11RuntimeActor && ActorIsTaggedCapturePlane(Actor);
		const bool bActiveStep11RuntimeSubject = FFromLZFaceReconstructor::IsStep11RuntimeActorActiveForCapture(Actor);
		if (bBaseSubject || bCapturePlane || bActiveStep11RuntimeSubject)
		{
			if (bHasCaptureCamera)
			{
				FVector BoundsOrigin;
				FVector BoundsExtent;
				Actor->GetActorBounds(false, BoundsOrigin, BoundsExtent);
				const FVector BoundsMin = BoundsOrigin - BoundsExtent;
				const FVector BoundsMax = BoundsOrigin + BoundsExtent;
				const bool bEntireActorAboveCamera = BoundsMin.Z > CameraLocation.Z;
				const bool bCameraInsideBoundsXY =
					CameraLocation.X >= BoundsMin.X &&
					CameraLocation.X <= BoundsMax.X &&
					CameraLocation.Y >= BoundsMin.Y &&
					CameraLocation.Y <= BoundsMax.Y;

				if (bEntireActorAboveCamera && bCameraInsideBoundsXY)
				{
					++BboxAboveInsideXYExcludedCount;
					continue;
				}

				if (bEntireActorAboveCamera)
				{
					++BboxAboveOutsideXYKeptCount;
				}
				else
				{
					++BboxAtOrBelowCameraKeptCount;
				}
			}
			else
			{
				++BboxCameraUnavailableKeptCount;
			}

			OutActors.Add(Actor);
			if (bBaseSubject)
			{
				++BaseSubjectCount;
			}
			if (bCapturePlane)
			{
				++CapturePlaneCount;
			}
			if (bActiveStep11RuntimeSubject)
			{
				++Step11RuntimeSubjectCount;
			}
		}
	}

	if (OutActors.Num() > 0)
	{
		OutMode = FString::Printf(
			TEXT("tagged_scene_or_active_step11_runtime(subject=%d,plane=%d,step11=%d,bbox_at_or_below_kept=%d,bbox_above_outside_xy_kept=%d,bbox_above_inside_xy_excluded=%d,bbox_camera_unavailable_kept=%d)"),
			BaseSubjectCount,
			CapturePlaneCount,
			Step11RuntimeSubjectCount,
			BboxAtOrBelowCameraKeptCount,
			BboxAboveOutsideXYKeptCount,
			BboxAboveInsideXYExcludedCount,
			BboxCameraUnavailableKeptCount);
	}
}

static void ApplyCaptureSubjectActors(USceneCaptureComponent2D* SceneCapture, const TArray<AActor*>& SubjectActors)
{
	if (!SceneCapture)
	{
		return;
	}

	SceneCapture->ShowOnlyActors.Empty();
	for (AActor* Actor : SubjectActors)
	{
		if (IsValid(Actor))
		{
			SceneCapture->ShowOnlyActors.Add(Actor);
		}
	}

	SceneCapture->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_UseShowOnlyList;
}

static void BuildOrthoFramingActorsFromSubjects(const TArray<AActor*>& SubjectActors, TArray<AActor*>& OutFramingActors)
{
	OutFramingActors.Reset();
	for (AActor* Actor : SubjectActors)
	{
		if (!IsValid(Actor))
		{
			continue;
		}

		const bool bActiveStep11RuntimeSubject = FFromLZFaceReconstructor::IsStep11RuntimeActorActiveForCapture(Actor);
		const bool bPlaneSubject =
			!bActiveStep11RuntimeSubject &&
			ActorIsTaggedCapturePlane(Actor);
		if (!bPlaneSubject)
		{
			OutFramingActors.Add(Actor);
		}
	}
}

static bool CaptureViewportDebugPng(FViewport* Viewport, const FString& OutputPath)
{
	if (!Viewport)
	{
		UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ: viewport debug png failed because viewport is unavailable."));
		return false;
	}

	const FIntPoint Size = Viewport->GetSizeXY();
	if (Size.X <= 0 || Size.Y <= 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ: viewport debug png failed because viewport size is invalid (%dx%d)."), Size.X, Size.Y);
		return false;
	}

	TArray<FColor> Pixels;
	FReadSurfaceDataFlags ReadFlags(RCM_UNorm);
	FlushRenderingCommands();
	if (!Viewport->ReadPixels(Pixels, ReadFlags) || Pixels.Num() != Size.X * Size.Y)
	{
		UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ: viewport debug png failed to read pixels (%dx%d, pixels=%d)."), Size.X, Size.Y, Pixels.Num());
		return false;
	}

	for (FColor& Pixel : Pixels)
	{
		Pixel.A = 255;
	}
	return SaveFColorPng(Pixels, Size.X, Size.Y, OutputPath, TEXT("viewport"));
}

static bool CaptureFromLZCameraDebugPng(const APawn* Pawn, const UCameraComponent* Camera, const FFromLZCaptureView& CaptureView, const FString& OutputPath)
{
	if (!Pawn || !Camera)
	{
		UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ: FromLZ camera debug png failed because pawn or camera is null."));
		return false;
	}

	UWorld* World = Pawn->GetWorld();
	if (!World)
	{
		UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ: FromLZ camera debug png failed because world is null."));
		return false;
	}

	FIntPoint Size = CaptureView.ViewportSize;
	if (Size.X <= 0 || Size.Y <= 0)
	{
		UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ: FromLZ camera debug png failed because capture size is invalid (%dx%d)."), Size.X, Size.Y);
		return false;
	}

	UTextureRenderTarget2D* ColorRT = NewObject<UTextureRenderTarget2D>(GetTransientPackage(), NAME_None, RF_Transient);
	ColorRT->RenderTargetFormat = RTF_RGBA8;
	ColorRT->ClearColor = FLinearColor::Black;
	ColorRT->bAutoGenerateMips = false;
	ColorRT->InitAutoFormat(Size.X, Size.Y);
	ColorRT->UpdateResourceImmediate(true);
	ColorRT->AddToRoot();

	USceneCaptureComponent2D* SCC = NewObject<USceneCaptureComponent2D>(const_cast<APawn*>(Pawn), NAME_None, RF_Transient);
	SCC->bCaptureEveryFrame = false;
	SCC->bCaptureOnMovement = false;
	SCC->bAlwaysPersistRenderingState = true;
	SCC->SetWorldTransform(CaptureView.Transform);
	if (!ConfigureSceneCaptureFromCaptureView(SCC, CaptureView))
	{
		UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ: selected-camera debug png failed because the offscreen projection matrix is invalid."));
		ColorRT->RemoveFromRoot();
		return false;
	}
	SCC->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
	SCC->TextureTarget = ColorRT;
	SCC->RegisterComponentWithWorld(World);
	SCC->CaptureScene();

	FlushRenderingCommands();

	TArray<FColor> Pixels;
	FTextureRenderTargetResource* Resource = ColorRT->GameThread_GetRenderTargetResource();
	const bool bReadOk = Resource && Resource->ReadPixels(Pixels) && Pixels.Num() == Size.X * Size.Y;
	for (FColor& Pixel : Pixels)
	{
		Pixel.A = 255;
	}

	const bool bSaved = bReadOk && SaveFColorPng(Pixels, Size.X, Size.Y, OutputPath, TEXT("orthographic capture view"));
	if (!bReadOk)
	{
		UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ: FromLZ camera debug png failed to read render target (%dx%d, pixels=%d)."), Size.X, Size.Y, Pixels.Num());
	}

	SCC->UnregisterComponent();
	SCC->DestroyComponent();
	ColorRT->RemoveFromRoot();
	return bSaved;
}

static TSharedRef<FJsonObject> SerializeTransformDelta(const FTransform& From, const FTransform& To)
{
	TSharedRef<FJsonObject> DeltaObject = MakeShared<FJsonObject>();

	const FVector LocationDelta = To.GetLocation() - From.GetLocation();
	const FRotator RotationDelta = (To.Rotator() - From.Rotator()).GetNormalized();
	const FVector ScaleDelta = To.GetScale3D() - From.GetScale3D();

	DeltaObject->SetNumberField(TEXT("location_delta_x"), LocationDelta.X);
	DeltaObject->SetNumberField(TEXT("location_delta_y"), LocationDelta.Y);
	DeltaObject->SetNumberField(TEXT("location_delta_z"), LocationDelta.Z);
	DeltaObject->SetNumberField(TEXT("location_distance"), LocationDelta.Size());
	DeltaObject->SetNumberField(TEXT("pitch_delta"), RotationDelta.Pitch);
	DeltaObject->SetNumberField(TEXT("yaw_delta"), RotationDelta.Yaw);
	DeltaObject->SetNumberField(TEXT("roll_delta"), RotationDelta.Roll);
	DeltaObject->SetNumberField(TEXT("scale_delta_x"), ScaleDelta.X);
	DeltaObject->SetNumberField(TEXT("scale_delta_y"), ScaleDelta.Y);
	DeltaObject->SetNumberField(TEXT("scale_delta_z"), ScaleDelta.Z);

	return DeltaObject;
}

static bool AreViewPosesNearlyEqual(const FTransform& A, const FTransform& B)
{
	const FVector LocationDelta = B.GetLocation() - A.GetLocation();
	const FRotator RotationDelta = (B.Rotator() - A.Rotator()).GetNormalized();
	constexpr double LocationTolerance = 0.01;
	constexpr double RotationToleranceDeg = 0.01;

	return LocationDelta.Size() <= LocationTolerance &&
		FMath::Abs(RotationDelta.Pitch) <= RotationToleranceDeg &&
		FMath::Abs(RotationDelta.Yaw) <= RotationToleranceDeg &&
		FMath::Abs(RotationDelta.Roll) <= RotationToleranceDeg;
}

static bool TryGetPlayerCameraManagerTransform(const APawn* Pawn, FTransform& OutTransform)
{
	if (!Pawn)
	{
		return false;
	}

	const APlayerController* PlayerController = Cast<APlayerController>(Pawn->GetController());
	if (!PlayerController || !PlayerController->PlayerCameraManager)
	{
		return false;
	}

	OutTransform = FTransform(
		PlayerController->PlayerCameraManager->GetCameraRotation(),
		PlayerController->PlayerCameraManager->GetCameraLocation(),
		FVector::OneVector);
	return true;
}

static bool SaveDebugViewTransforms(
	const APawn* Pawn,
	const UCameraComponent* Camera,
	const USceneCaptureComponent2D* SceneCapture,
	const FFromLZCaptureView& CaptureView,
	const FString& OutputPath)
{
	if (!Pawn || !Camera || !SceneCapture)
	{
		UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ: debug transform json failed because pawn/camera/scene capture is null."));
		return false;
	}

	TSharedRef<FJsonObject> RootObject = MakeShared<FJsonObject>();
	RootObject->SetStringField(TEXT("debug_type"), TEXT("offscreen_capture_view_transforms"));
	RootObject->SetStringField(TEXT("json_path"), OutputPath);
	RootObject->SetStringField(TEXT("scene_capture_context"), TEXT("CaptureLineArtPng"));
	RootObject->SetStringField(TEXT("player_camera_manager_source"), TEXT("PlayerCameraManager->GetCameraLocation/Rotation"));
	RootObject->SetStringField(TEXT("selected_camera_component_source"), TEXT("UCameraComponent snapshot"));
	RootObject->SetStringField(TEXT("source_view_source"), CaptureView.SourceViewTransformSource);
	RootObject->SetStringField(TEXT("orthographic_capture_view_source"), CaptureView.TransformSource);
	RootObject->SetStringField(TEXT("orthographic_framing_mode"), CaptureView.FramingMode);
	RootObject->SetStringField(TEXT("focus_source"), CaptureView.FocusSource);
	RootObject->SetStringField(TEXT("scene_capture_source"), TEXT("SceneCapture->GetComponentTransform()"));
	RootObject->SetStringField(TEXT("source_view_projection_type"), StaticEnum<ECameraProjectionMode::Type>()->GetValueAsString(Camera->ProjectionMode));
	RootObject->SetStringField(TEXT("scene_capture_projection_type"), StaticEnum<ECameraProjectionMode::Type>()->GetValueAsString(SceneCapture->ProjectionType));
	RootObject->SetNumberField(TEXT("scene_capture_fov"), SceneCapture->FOVAngle);
	RootObject->SetNumberField(TEXT("scene_capture_ortho_width"), SceneCapture->OrthoWidth);
	RootObject->SetBoolField(TEXT("scene_capture_auto_calculate_ortho_planes"), SceneCapture->bAutoCalculateOrthoPlanes);
	RootObject->SetNumberField(TEXT("scene_capture_auto_plane_shift"), SceneCapture->AutoPlaneShift);
	RootObject->SetBoolField(TEXT("scene_capture_update_ortho_planes"), SceneCapture->bUpdateOrthoPlanes);
	RootObject->SetBoolField(TEXT("scene_capture_use_camera_height_as_view_target"), SceneCapture->bUseCameraHeightAsViewTarget);
	RootObject->SetBoolField(TEXT("scene_capture_uses_custom_projection_matrix"), SceneCapture->bUseCustomProjectionMatrix);
	RootObject->SetStringField(
		TEXT("scene_capture_projection_matrix_source"),
		TEXT("FMinimalViewInfo::CalculateProjectionMatrix"));
	RootObject->SetObjectField(
		TEXT("offscreen_projection_matrix"),
		SerializeMatrix(CaptureView.ProjectionMatrix));
	RootObject->SetObjectField(
		TEXT("scene_capture_custom_projection_matrix"),
		SerializeMatrix(SceneCapture->CustomProjectionMatrix));
	RootObject->SetBoolField(
		TEXT("scene_capture_projection_matrix_matches_offscreen_view"),
		SceneCapture->bUseCustomProjectionMatrix &&
		AreProjectionMatricesNearlyEqual(
			CaptureView.ProjectionMatrix,
			SceneCapture->CustomProjectionMatrix));
	RootObject->SetNumberField(TEXT("orthographic_focus_depth"), CaptureView.FocusDepth);
	RootObject->SetNumberField(TEXT("orthographic_backoff"), CaptureView.OrthoBackoff);
	AddVectorFields(RootObject, TEXT("orthographic_focus_point"), CaptureView.FocusPoint);

	FTransform PlayerCameraManagerTransform;
	const bool bHasPlayerCameraManager = TryGetPlayerCameraManagerTransform(Pawn, PlayerCameraManagerTransform);
	const FTransform FromLZTransform = Camera->GetComponentTransform();
	const FTransform SceneCaptureTransform = SceneCapture->GetComponentTransform();

	RootObject->SetBoolField(TEXT("player_camera_manager_available"), bHasPlayerCameraManager);
	if (bHasPlayerCameraManager)
	{
		RootObject->SetObjectField(TEXT("player_camera_manager_transform"), FFromLZCaptureUtils::SerializeTransform(PlayerCameraManagerTransform));
	}
	RootObject->SetObjectField(TEXT("fromlz_component_transform"), FFromLZCaptureUtils::SerializeTransform(FromLZTransform));
	RootObject->SetObjectField(TEXT("source_view_transform"), FFromLZCaptureUtils::SerializeTransform(CaptureView.SourceViewTransform));
	RootObject->SetObjectField(TEXT("orthographic_capture_view_transform"), FFromLZCaptureUtils::SerializeTransform(CaptureView.Transform));
	RootObject->SetObjectField(TEXT("scene_capture_transform"), FFromLZCaptureUtils::SerializeTransform(SceneCaptureTransform));

	RootObject->SetBoolField(TEXT("scene_capture_matches_fromlz_component_pose"), AreViewPosesNearlyEqual(FromLZTransform, SceneCaptureTransform));
	RootObject->SetBoolField(TEXT("scene_capture_matches_orthographic_capture_view_pose"), AreViewPosesNearlyEqual(CaptureView.Transform, SceneCaptureTransform));
	RootObject->SetObjectField(TEXT("scene_capture_minus_fromlz_component"), SerializeTransformDelta(FromLZTransform, SceneCaptureTransform));
	RootObject->SetObjectField(TEXT("scene_capture_minus_orthographic_capture_view"), SerializeTransformDelta(CaptureView.Transform, SceneCaptureTransform));

	if (bHasPlayerCameraManager)
	{
		RootObject->SetBoolField(TEXT("player_camera_manager_matches_fromlz_component_pose"), AreViewPosesNearlyEqual(PlayerCameraManagerTransform, FromLZTransform));
		RootObject->SetBoolField(TEXT("scene_capture_matches_player_camera_manager_pose"), AreViewPosesNearlyEqual(PlayerCameraManagerTransform, SceneCaptureTransform));
		RootObject->SetObjectField(TEXT("fromlz_component_minus_player_camera_manager"), SerializeTransformDelta(PlayerCameraManagerTransform, FromLZTransform));
		RootObject->SetObjectField(TEXT("scene_capture_minus_player_camera_manager"), SerializeTransformDelta(PlayerCameraManagerTransform, SceneCaptureTransform));
	}

	const bool bSaved = FFromLZCaptureUtils::SaveJsonToFile(RootObject, OutputPath);
	if (bSaved)
	{
		UE_LOG(LogTemp, Log, TEXT("CaptureFromLZ: saved debug view transforms to %s"), *OutputPath);
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ: failed to save debug view transforms to %s"), *OutputPath);
	}
	return bSaved;
}

static TSharedPtr<FJsonObject> LoadJsonObjectFromFile(const FString& FilePath)
{
	FString JsonText;
	if (!FFileHelper::LoadFileToString(JsonText, *FilePath))
	{
		return nullptr;
	}

	TSharedPtr<FJsonObject> Object;
	const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(JsonText);
	return FJsonSerializer::Deserialize(Reader, Object) ? Object : nullptr;
}

static bool FinalizeProjectionDebugFiles(const FPendingFromLZCapture& Pending)
{
	TSharedPtr<FJsonObject> DebugObject = LoadJsonObjectFromFile(Pending.OutputPaths.DebugTransformsJsonPath);
	if (!DebugObject)
	{
		DebugObject = MakeShared<FJsonObject>();
		DebugObject->SetStringField(TEXT("debug_type"), TEXT("offscreen_projection_validation"));
		DebugObject->SetStringField(TEXT("json_path"), Pending.OutputPaths.DebugTransformsJsonPath);
	}

	DebugObject->SetNumberField(TEXT("debug_version"), 3);
	DebugObject->SetStringField(TEXT("capture_camera_source"), Pending.CameraSource);
	DebugObject->SetStringField(
		TEXT("requested_camera_tag"),
		Pending.RequestedCameraTag.IsNone() ? FString() : Pending.RequestedCameraTag.ToString());
	DebugObject->SetBoolField(TEXT("player_view_target_changed"), false);
	DebugObject->SetBoolField(TEXT("source_camera_modified"), false);
	DebugObject->SetBoolField(TEXT("viewport_reference_saved"), Pending.bViewportReferenceSaved);
	DebugObject->SetNumberField(TEXT("max_viewport_width"), Pending.CaptureView.MaxViewportSize.X);
	DebugObject->SetNumberField(TEXT("max_viewport_height"), Pending.CaptureView.MaxViewportSize.Y);
	DebugObject->SetNumberField(TEXT("capture_width"), Pending.CaptureView.ViewportSize.X);
	DebugObject->SetNumberField(TEXT("capture_height"), Pending.CaptureView.ViewportSize.Y);
	DebugObject->SetNumberField(TEXT("capture_aspect_ratio"), Pending.CaptureView.ViewportAspectRatio);
	DebugObject->SetNumberField(TEXT("capture_ortho_width"), Pending.CaptureView.OrthoWidth);
	DebugObject->SetStringField(
		TEXT("projection_matrix_source"),
		TEXT("FMinimalViewInfo::CalculateProjectionMatrix"));
	DebugObject->SetObjectField(
		TEXT("offscreen_projection_matrix"),
		SerializeMatrix(Pending.CaptureView.ProjectionMatrix));

	TSharedRef<FJsonObject> Prevention = MakeShared<FJsonObject>();
	Prevention->SetStringField(TEXT("policy"), TEXT("disable_ortho_auto_planes_and_view_origin_correction"));
	Prevention->SetBoolField(TEXT("auto_calculate_ortho_planes"), false);
	Prevention->SetNumberField(TEXT("auto_plane_shift"), 0.0);
	Prevention->SetBoolField(TEXT("update_ortho_planes"), false);
	Prevention->SetBoolField(TEXT("use_camera_height_as_view_target"), false);
	Prevention->SetNumberField(TEXT("ortho_near_clip_plane"), FromLZCaptureOrthoNearPlane);
	Prevention->SetNumberField(TEXT("ortho_far_clip_plane"), FromLZCaptureOrthoFarPlane);
	DebugObject->SetObjectField(TEXT("view_origin_shift_prevention"), Prevention);

	bool bSceneCaptureUsesCustomProjection = false;
	bool bSceneCaptureProjectionMatchesOffscreenView = false;
	DebugObject->TryGetBoolField(
		TEXT("scene_capture_uses_custom_projection_matrix"),
		bSceneCaptureUsesCustomProjection);
	DebugObject->TryGetBoolField(
		TEXT("scene_capture_projection_matrix_matches_offscreen_view"),
		bSceneCaptureProjectionMatchesOffscreenView);

	TSharedRef<FJsonObject> Validation = MakeShared<FJsonObject>();
	Validation->SetBoolField(TEXT("player_view_target_unchanged"), true);
	Validation->SetBoolField(TEXT("source_camera_unchanged"), true);
	Validation->SetBoolField(
		TEXT("offscreen_projection_matrix_usable"),
		FromLZIsUsableOrthographicProjectionMatrix(Pending.CaptureView.ProjectionMatrix));
	Validation->SetBoolField(
		TEXT("scene_capture_uses_offscreen_projection_matrix"),
		bSceneCaptureUsesCustomProjection && bSceneCaptureProjectionMatchesOffscreenView);
	DebugObject->SetObjectField(TEXT("validation"), Validation);

	const bool bDebugSaved =
		FFromLZCaptureUtils::SaveJsonToFile(DebugObject.ToSharedRef(), Pending.OutputPaths.DebugTransformsJsonPath);

	if (TSharedPtr<FJsonObject> MainObject = LoadJsonObjectFromFile(Pending.OutputPaths.JsonPath))
	{
		MainObject->SetObjectField(TEXT("projection_debug_validation"), Validation);
		FFromLZCaptureUtils::SaveJsonToFile(MainObject.ToSharedRef(), Pending.OutputPaths.JsonPath);
	}

	return bDebugSaved;
}

// Renders scene depth and world-normal to off-screen render targets on the GPU,
// then runs a CPU Sobel edge-detection pass to produce a white background with
// black contour/crease lines (depth discontinuity = silhouette/occlusion edges,
// normal discontinuity = surface creases). Occlusion is handled naturally because
// the detection works in screen space on the rendered buffers.
static bool CaptureLineArtPng(
	const APawn* Pawn,
	const UCameraComponent* Camera,
	const FFromLZCaptureView& CaptureView,
	const TArray<AActor*>& SubjectActors,
	const FString& SubjectMode,
	const FString& OutputPath,
	const FString& DebugTransformsJsonPath)
{
	UWorld* World = Pawn->GetWorld();
	if (!World)
	{
		return false;
	}

	FIntPoint Size = CaptureView.ViewportSize;
	if (Size.X <= 0 || Size.Y <= 0)
	{
		return false;
	}

	auto MakeRenderTarget = [&](ETextureRenderTargetFormat Format) -> UTextureRenderTarget2D*
	{
		UTextureRenderTarget2D* RT = NewObject<UTextureRenderTarget2D>(GetTransientPackage(), NAME_None, RF_Transient);
		RT->RenderTargetFormat = Format;
		RT->ClearColor = FLinearColor::Black;
		RT->bAutoGenerateMips = false;
		RT->InitAutoFormat(Size.X, Size.Y);
		RT->UpdateResourceImmediate(true);
		RT->AddToRoot();
		return RT;
	};

	// 16-bit float so normals can store negative components and depth keeps range.
	UTextureRenderTarget2D* DepthRT = MakeRenderTarget(RTF_RGBA16f);
	UTextureRenderTarget2D* NormalRT = MakeRenderTarget(RTF_RGBA16f);

	USceneCaptureComponent2D* SCC = NewObject<USceneCaptureComponent2D>(const_cast<APawn*>(Pawn), NAME_None, RF_Transient);
	SCC->bCaptureEveryFrame = false;
	SCC->bCaptureOnMovement = false;
	// Do NOT persist TAA / temporal history across the two CaptureScene() calls
	// below (depth then normal). Otherwise the normal pass blends with the depth
	// pass's stale history and smears silhouettes of newly added foreground
	// geometry over 2-3 pixels, diluting the 3x3 Sobel/Laplacian response below
	// the edge thresholds.
	SCC->bAlwaysPersistRenderingState = false;
	// Disable engine-level smoothing so SCS_SceneDepth and SCS_Normal readbacks
	// stay pixel-sharp at silhouettes. Without these, UE5's default scene capture
	// pipeline runs TAA / temporal upsampling and edge AA, widening 1-pixel depth
	// and normal discontinuities into 2-3 pixel ramps that the 3x3 edge kernels
	// in this function cannot recover.
	SCC->ShowFlags.SetTemporalAA(false);
	SCC->ShowFlags.SetAntiAliasing(false);
	SCC->ShowFlags.SetMotionBlur(false);
	SCC->SetWorldTransform(CaptureView.Transform);
	if (!ConfigureSceneCaptureFromCaptureView(SCC, CaptureView))
	{
		UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ: line-art capture failed because the offscreen projection matrix is invalid."));
		DepthRT->RemoveFromRoot();
		NormalRT->RemoveFromRoot();
		return false;
	}
	SCC->RegisterComponentWithWorld(World);
	SaveDebugViewTransforms(Pawn, Camera, SCC, CaptureView, DebugTransformsJsonPath);

	ApplyCaptureSubjectActors(SCC, SubjectActors);
	UE_LOG(LogTemp, Log, TEXT("CaptureFromLZ: ortho capture subjects mode=%s count=%d."), *SubjectMode, SubjectActors.Num());

	// Captures linear scene depth + world normal with the SCC's current config and
	// reads them back to CPU arrays. Reused for line art, planar faces and the
	// actor/material buffer, all with the same orthographic view and subject list.
	auto CaptureDepthNormal = [&](TArray<float>& OutDepth, TArray<FVector3f>& OutNormal) -> bool
	{
		SCC->CaptureSource = ESceneCaptureSource::SCS_SceneDepth;
		SCC->TextureTarget = DepthRT;
		SCC->CaptureScene();

		SCC->CaptureSource = ESceneCaptureSource::SCS_Normal;
		SCC->TextureTarget = NormalRT;
		SCC->CaptureScene();

		FlushRenderingCommands();

		TArray<FFloat16Color> DepthPixels;
		TArray<FFloat16Color> NormalPixels;
		FTextureRenderTargetResource* DRes = DepthRT->GameThread_GetRenderTargetResource();
		FTextureRenderTargetResource* NRes = NormalRT->GameThread_GetRenderTargetResource();
		if (!(DRes && DRes->ReadFloat16Pixels(DepthPixels) &&
			  NRes && NRes->ReadFloat16Pixels(NormalPixels) &&
			  DepthPixels.Num() == Size.X * Size.Y &&
			  NormalPixels.Num() == Size.X * Size.Y))
		{
			return false;
		}
		const int32 Num = Size.X * Size.Y;
		OutDepth.SetNumUninitialized(Num);
		OutNormal.SetNumUninitialized(Num);
		for (int32 i = 0; i < Num; ++i)
		{
			OutDepth[i] = DepthPixels[i].R.GetFloat();
			OutNormal[i] = FVector3f(NormalPixels[i].R.GetFloat(), NormalPixels[i].G.GetFloat(), NormalPixels[i].B.GetFloat());
		}
		return true;
	};

	// Pass set 1: Cube-restricted buffers -> line art.
	bool bSaved = false;
	TArray<float> Depth;
	TArray<FVector3f> Normal;
	const bool bReadOk = CaptureDepthNormal(Depth, Normal);

	if (bReadOk)
	{
		const int32 W = Size.X;
		const int32 H = Size.Y;

		// Tunable edge-detection thresholds.
		const float DepthRelThreshold = 0.0015f; // relative depth gradient (gradient / depth)
		const float NormalThreshold = 0.3f;    // normal gradient magnitude

		auto SampleDepth = [&](int32 x, int32 y) -> float
		{
			x = FMath::Clamp(x, 0, W - 1);
			y = FMath::Clamp(y, 0, H - 1);
			return Depth[y * W + x];
		};
		auto SampleNormal = [&](int32 x, int32 y) -> FVector3f
		{
			x = FMath::Clamp(x, 0, W - 1);
			y = FMath::Clamp(y, 0, H - 1);
			return Normal[y * W + x];
		};

		TArray<FColor> Out;
		Out.SetNumUninitialized(W * H);

		for (int32 y = 0; y < H; ++y)
		{
			for (int32 x = 0; x < W; ++x)
			{
				const float Dc = SampleDepth(x, y);

				// Slant-invariant depth discontinuity: compare the center depth against
				// the linear prediction from opposite neighbors (discrete 2nd derivative).
				// A planar surface is locally linear in depth even when viewed edge-on, so
				// this stays ~0 there; only true depth jumps (silhouettes/occlusions) and
				// creases fire. This avoids filling grazing faces solid black.
				const float DevX = FMath::Abs(2.f * Dc - SampleDepth(x - 1, y) - SampleDepth(x + 1, y));
				const float DevY = FMath::Abs(2.f * Dc - SampleDepth(x, y - 1) - SampleDepth(x, y + 1));
				const float DevD1 = FMath::Abs(2.f * Dc - SampleDepth(x - 1, y - 1) - SampleDepth(x + 1, y + 1));
				const float DevD2 = FMath::Abs(2.f * Dc - SampleDepth(x + 1, y - 1) - SampleDepth(x - 1, y + 1));
				const float DepthDev = FMath::Max(FMath::Max(DevX, DevY), FMath::Max(DevD1, DevD2));
				const bool bDepthEdge = (DepthDev / FMath::Max(Dc, 1.f)) > DepthRelThreshold;

				const FVector3f N00 = SampleNormal(x - 1, y - 1);
				const FVector3f N10 = SampleNormal(x, y - 1);
				const FVector3f N20 = SampleNormal(x + 1, y - 1);
				const FVector3f N01 = SampleNormal(x - 1, y);
				const FVector3f N21 = SampleNormal(x + 1, y);
				const FVector3f N02 = SampleNormal(x - 1, y + 1);
				const FVector3f N12 = SampleNormal(x, y + 1);
				const FVector3f N22 = SampleNormal(x + 1, y + 1);

				const FVector3f Ngx = -N00 - 2.f * N01 - N02 + N20 + 2.f * N21 + N22;
				const FVector3f Ngy = -N00 - 2.f * N10 - N20 + N02 + 2.f * N12 + N22;
				const float NormalGrad = FMath::Sqrt(Ngx.SizeSquared() + Ngy.SizeSquared());
				const bool bNormalEdge = NormalGrad > NormalThreshold;

				Out[y * W + x] = (bDepthEdge || bNormalEdge) ? FColor(0, 0, 0, 255) : FColor(255, 255, 255, 255);
			}
		}

		IImageWrapperModule& IWM = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
		TSharedPtr<IImageWrapper> IW = IWM.CreateImageWrapper(EImageFormat::PNG);
		if (IW.IsValid())
		{
			// FColor memory layout is B,G,R,A — use ERGBFormat::BGRA
			IW->SetRaw(Out.GetData(), Out.Num() * sizeof(FColor), W, H, ERGBFormat::BGRA, 8);
			const TArray64<uint8>& Compressed = IW->GetCompressed();
			bSaved = FFileHelper::SaveArrayToFile(
				TArrayView<const uint8>(Compressed.GetData(), static_cast<int32>(Compressed.Num())),
				*OutputPath
			);
		}

		// Debug visualisations of the raw G-buffer reads. These let us inspect
		// directly whether silhouettes are pixel-sharp or smeared by engine AA.
		//   _debug_depth.png  : grayscale, linearly normalised over finite depth.
		//   _debug_normal.png : world-space normal encoded as RGB = N*0.5 + 0.5.
		//   _debug_depth_lap.png : magnitude of the slant-invariant 2nd derivative
		//                        used by the line-art (red = above DepthRelThreshold).
		//   _debug_normal_grad.png : Sobel magnitude of the normal field used by
		//                        the line-art (red = above NormalThreshold).
		{
			const FString DebugBase = FPaths::Combine(FPaths::GetPath(OutputPath), FPaths::GetBaseFilename(OutputPath));
			const int32 Num = W * H;

			// --- Depth visualisation (auto-ranged grayscale) ---
			{
				float MinD = TNumericLimits<float>::Max();
				float MaxD = -TNumericLimits<float>::Max();
				for (int32 i = 0; i < Num; ++i)
				{
					const float D = Depth[i];
					if (FMath::IsFinite(D) && D > 0.f)
					{
						MinD = FMath::Min(MinD, D);
						MaxD = FMath::Max(MaxD, D);
					}
				}
				if (!(MaxD > MinD))
				{
					MinD = 0.f;
					MaxD = FMath::Max(MaxD, 1.f);
				}
				const float InvRange = 1.f / FMath::Max(MaxD - MinD, KINDA_SMALL_NUMBER);
				TArray<FColor> DepthVis;
				DepthVis.SetNumUninitialized(Num);
				for (int32 i = 0; i < Num; ++i)
				{
					const float D = Depth[i];
					float T = 0.f;
					if (FMath::IsFinite(D) && D > 0.f)
					{
						T = FMath::Clamp((D - MinD) * InvRange, 0.f, 1.f);
					}
					const uint8 G = static_cast<uint8>(FMath::RoundToInt(T * 255.f));
					DepthVis[i] = FColor(G, G, G, 255);
				}
				SaveFColorPng(DepthVis, W, H, DebugBase + TEXT("_debug_depth.png"), TEXT("debug depth"));
				UE_LOG(LogTemp, Log, TEXT("CaptureFromLZ: depth viz range [%.2f, %.2f] -> %s_debug_depth.png"), MinD, MaxD, *DebugBase);
			}

			// --- Normal visualisation (RGB = N*0.5 + 0.5) ---
			{
				TArray<FColor> NormalVis;
				NormalVis.SetNumUninitialized(Num);
				for (int32 i = 0; i < Num; ++i)
				{
					FVector3f Nn = Normal[i];
					if (!Nn.IsNearlyZero())
					{
						Nn = Nn.GetSafeNormal();
					}
					const int32 R = FMath::Clamp(FMath::RoundToInt((Nn.X * 0.5f + 0.5f) * 255.f), 0, 255);
					const int32 G = FMath::Clamp(FMath::RoundToInt((Nn.Y * 0.5f + 0.5f) * 255.f), 0, 255);
					const int32 B = FMath::Clamp(FMath::RoundToInt((Nn.Z * 0.5f + 0.5f) * 255.f), 0, 255);
					NormalVis[i] = FColor(static_cast<uint8>(R), static_cast<uint8>(G), static_cast<uint8>(B), 255);
				}
				SaveFColorPng(NormalVis, W, H, DebugBase + TEXT("_debug_normal.png"), TEXT("debug normal"));
			}

			// --- Depth-Laplacian magnitude (relative) -------------------------
			// Same operator as the line-art branch above; values >= threshold are
			// painted red so the firing locus is obvious.
			{
				TArray<FColor> LapVis;
				LapVis.SetNumUninitialized(Num);
				for (int32 y = 0; y < H; ++y)
				{
					for (int32 x = 0; x < W; ++x)
					{
						const float Dc = SampleDepth(x, y);
						const float DevX = FMath::Abs(2.f * Dc - SampleDepth(x - 1, y) - SampleDepth(x + 1, y));
						const float DevY = FMath::Abs(2.f * Dc - SampleDepth(x, y - 1) - SampleDepth(x, y + 1));
						const float DevD1 = FMath::Abs(2.f * Dc - SampleDepth(x - 1, y - 1) - SampleDepth(x + 1, y + 1));
						const float DevD2 = FMath::Abs(2.f * Dc - SampleDepth(x + 1, y - 1) - SampleDepth(x - 1, y + 1));
						const float Rel = FMath::Max(FMath::Max(DevX, DevY), FMath::Max(DevD1, DevD2)) / FMath::Max(Dc, 1.f);
						// Visualise on a logarithmic-ish ramp so subthreshold structure is visible.
						const float Vis = FMath::Clamp(Rel / FMath::Max(DepthRelThreshold * 2.f, KINDA_SMALL_NUMBER), 0.f, 1.f);
						const uint8 G = static_cast<uint8>(FMath::RoundToInt(Vis * 255.f));
						LapVis[y * W + x] = (Rel > DepthRelThreshold)
							? FColor(255, 0, 0, 255)
							: FColor(G, G, G, 255);
					}
				}
				SaveFColorPng(LapVis, W, H, DebugBase + TEXT("_debug_depth_lap.png"), TEXT("debug depth laplacian"));
			}

			// --- Normal-gradient (Sobel) magnitude ---------------------------
			{
				TArray<FColor> GradVis;
				GradVis.SetNumUninitialized(Num);
				for (int32 y = 0; y < H; ++y)
				{
					for (int32 x = 0; x < W; ++x)
					{
						const FVector3f N00 = SampleNormal(x - 1, y - 1);
						const FVector3f N10 = SampleNormal(x, y - 1);
						const FVector3f N20 = SampleNormal(x + 1, y - 1);
						const FVector3f N01 = SampleNormal(x - 1, y);
						const FVector3f N21 = SampleNormal(x + 1, y);
						const FVector3f N02 = SampleNormal(x - 1, y + 1);
						const FVector3f N12 = SampleNormal(x, y + 1);
						const FVector3f N22 = SampleNormal(x + 1, y + 1);
						const FVector3f Ngx = -N00 - 2.f * N01 - N02 + N20 + 2.f * N21 + N22;
						const FVector3f Ngy = -N00 - 2.f * N10 - N20 + N02 + 2.f * N12 + N22;
						const float NormalGrad = FMath::Sqrt(Ngx.SizeSquared() + Ngy.SizeSquared());
						const float Vis = FMath::Clamp(NormalGrad / FMath::Max(NormalThreshold * 2.f, KINDA_SMALL_NUMBER), 0.f, 1.f);
						const uint8 G = static_cast<uint8>(FMath::RoundToInt(Vis * 255.f));
						GradVis[y * W + x] = (NormalGrad > NormalThreshold)
							? FColor(255, 0, 0, 255)
							: FColor(G, G, G, 255);
					}
				}
				SaveFColorPng(GradVis, W, H, DebugBase + TEXT("_debug_normal_grad.png"), TEXT("debug normal gradient"));
			}
		}
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ: failed to read depth/normal render targets."));
	}

	// Pass set 2: same-subject buffers -> planar face segmentation and actor/material IDs.
	{
		ApplyCaptureSubjectActors(SCC, SubjectActors);
		TArray<float> FaceDepth;
		TArray<FVector3f> FaceNormal;
		if (CaptureDepthNormal(FaceDepth, FaceNormal))
		{
			const FString FacesBase = FPaths::Combine(FPaths::GetPath(OutputPath), FPaths::GetBaseFilename(OutputPath));
			if (!SaveNormalFaces(
				FaceDepth, FaceNormal, Size.X, Size.Y, CaptureView.Transform, CaptureView.Fov,
				/*bCaptureOrthographic*/ true, CaptureView.OrthoWidth,
				CaptureView.ProjectionMatrix,
				FacesBase + TEXT("_faces.png"), FacesBase + TEXT("_faces.json")))
			{
				UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ: failed to save unique-color planar face outputs for %s."), *FacesBase);
			}
			FromLZActorMaterialId::SaveActorMaterialIdBuffer(
				World,
				CaptureView.Transform,
				CaptureView.OrthoWidth,
				CaptureView.ProjectionMatrix,
				SubjectActors,
				SubjectMode,
				Size.X,
				Size.Y,
				FacesBase + TEXT("_actor_material_id.png"),
				FacesBase + TEXT("_actor_material_id.json"));
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ: failed to read whole-scene depth/normal for faces."));
		}
	}

	SCC->UnregisterComponent();
	SCC->DestroyComponent();
	DepthRT->RemoveFromRoot();
	NormalRT->RemoveFromRoot();

	return bSaved;
}

static bool CompleteCaptureFromTarget(
	const FPendingFromLZCapture& Pending)
{
	const APawn* Pawn = Pending.Pawn.Get();
	UCameraComponent* CameraComponent = Pending.Camera.Get();
	AActor* CameraActor = Pending.CameraActor.Get();
	const FPendingFromLZCapture::FOutputPaths& OutputPaths = Pending.OutputPaths;
	const FFromLZCaptureView& CaptureView = Pending.CaptureView;

	if (!Pawn)
	{
		UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ failed: controlled pawn is null."));
		return false;
	}

	if (!CameraComponent)
	{
		UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ failed: selected camera component is unavailable."));
		return false;
	}
	USpringArmComponent* CameraBoom = Cast<USpringArmComponent>(CameraComponent->GetAttachParent());

	if (!CaptureView.bUseCustomProjectionMatrix ||
		!FromLZIsUsableOrthographicProjectionMatrix(CaptureView.ProjectionMatrix))
	{
		UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ failed: offscreen orthographic capture view is invalid."));
		return false;
	}

	const bool bOrthographicViewportSaved =
		CaptureFromLZCameraDebugPng(Pawn, CameraComponent, CaptureView, OutputPaths.DebugViewportOrthographicPngPath);
	if (!bOrthographicViewportSaved)
	{
		UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ: failed to save the offscreen orthographic reference image."));
	}

	TArray<AActor*> SubjectActors;
	SubjectActors.Reserve(Pending.SubjectActors.Num());
	for (const TWeakObjectPtr<AActor>& SubjectActor : Pending.SubjectActors)
	{
		if (SubjectActor.IsValid())
		{
			SubjectActors.Add(SubjectActor.Get());
		}
	}
	const FString SubjectMode = Pending.SubjectMode;

	TSharedRef<FJsonObject> RootObject = MakeShared<FJsonObject>();
	RootObject->SetStringField(TEXT("capture_timestamp"), OutputPaths.Timestamp);
	RootObject->SetStringField(TEXT("json_path"), OutputPaths.JsonPath);
	RootObject->SetStringField(TEXT("png_path"), OutputPaths.PngPath);
	RootObject->SetStringField(TEXT("debug_viewport_png_path"), OutputPaths.DebugViewportOrthographicPngPath);
	RootObject->SetStringField(TEXT("debug_viewport_perspective_png_path"), OutputPaths.DebugViewportPerspectivePngPath);
	RootObject->SetStringField(TEXT("debug_viewport_orthographic_png_path"), OutputPaths.DebugViewportOrthographicPngPath);
	RootObject->SetStringField(TEXT("debug_fromlz_camera_png_path"), OutputPaths.DebugFromLZCameraPngPath);
	RootObject->SetStringField(TEXT("debug_transforms_json_path"), OutputPaths.DebugTransformsJsonPath);
	RootObject->SetStringField(TEXT("actor_material_png_path"), OutputPaths.ActorMaterialPngPath);
	RootObject->SetStringField(TEXT("actor_material_json_path"), OutputPaths.ActorMaterialJsonPath);
	RootObject->SetStringField(TEXT("capture_camera_source"), Pending.CameraSource);
	RootObject->SetStringField(TEXT("requested_camera_tag"), Pending.RequestedCameraTag.IsNone() ? FString() : Pending.RequestedCameraTag.ToString());
	RootObject->SetBoolField(TEXT("open_sketch_board_on_success"), Pending.bOpenSketchBoardOnSuccess);
	RootObject->SetStringField(TEXT("capture_camera_actor"), GetNameSafe(CameraActor));
	RootObject->SetStringField(TEXT("capture_camera_component"), GetNameSafe(CameraComponent));
	RootObject->SetBoolField(TEXT("view_target_was_changed"), false);
	RootObject->SetBoolField(TEXT("source_camera_was_modified"), false);
	RootObject->SetObjectField(TEXT("pawn"), FFromLZCaptureUtils::SerializeObjectProperties(Pawn));
	RootObject->SetObjectField(TEXT("pawn_transform"), FFromLZCaptureUtils::SerializeTransform(Pawn->GetActorTransform()));
	RootObject->SetObjectField(TEXT("camera_component"), FFromLZCaptureUtils::SerializeObjectProperties(CameraComponent));
	RootObject->SetObjectField(TEXT("camera_component_transform"), FFromLZCaptureUtils::SerializeTransform(CaptureView.SourceViewTransform));
	RootObject->SetBoolField(TEXT("camera_projection_temporarily_overridden"), false);
	RootObject->SetStringField(TEXT("camera_projection_restore_policy"), TEXT("source_camera_not_modified"));
	RootObject->SetBoolField(TEXT("capture_ortho_width_used_default"), Pending.bSourceOrthoWidthUsedDefault);
	RootObject->SetBoolField(TEXT("capture_ortho_width_from_subject_bounds_center"), Pending.bUsedSubjectBoundsCenterOrthoWidth);
	RootObject->SetNumberField(TEXT("subject_bounds_focus_depth"), Pending.SubjectFocusDepth);
	AddVectorFields(RootObject, TEXT("subject_bounds_center"), Pending.SubjectBoundsCenter);
	RootObject->SetNumberField(TEXT("orthographic_viewport_frames_rendered_before_capture"), 0);
	RootObject->SetNumberField(TEXT("orthographic_settle_seconds_required"), 0.0);
	RootObject->SetNumberField(TEXT("orthographic_settle_seconds_actual"), 0.0);
	RootObject->SetBoolField(TEXT("debug_viewport_perspective_saved"), Pending.bViewportReferenceSaved);
	RootObject->SetBoolField(TEXT("debug_viewport_orthographic_saved"), bOrthographicViewportSaved);
	RootObject->SetStringField(TEXT("debug_viewport_perspective_capture_source"), TEXT("unchanged_displayed_game_viewport_backbuffer"));
	RootObject->SetStringField(TEXT("debug_viewport_orthographic_capture_source"), TEXT("offscreen_scene_capture_from_selected_camera"));
	RootObject->SetObjectField(TEXT("original_camera_projection"), SerializeCameraProjectionSnapshot(Pending.SourceCameraProjection));
	FFromLZCameraProjectionSnapshot CaptureProjection = Pending.SourceCameraProjection;
	CaptureProjection.ProjectionMode = ECameraProjectionMode::Orthographic;
	CaptureProjection.OrthoWidth = static_cast<float>(CaptureView.OrthoWidth);
	CaptureProjection.OrthoNearClipPlane = FromLZCaptureOrthoNearPlane;
	CaptureProjection.OrthoFarClipPlane = FromLZCaptureOrthoFarPlane;
	CaptureProjection.bAutoCalculateOrthoPlanes = false;
	CaptureProjection.AutoPlaneShift = 0.0f;
	CaptureProjection.bUpdateOrthoPlanes = false;
	CaptureProjection.bUseCameraHeightAsViewTarget = false;
	CaptureProjection.bConstrainAspectRatio = true;
	RootObject->SetObjectField(TEXT("capture_camera_projection"), SerializeCameraProjectionSnapshot(CaptureProjection));
	RootObject->SetObjectField(TEXT("source_view_transform"), FFromLZCaptureUtils::SerializeTransform(CaptureView.SourceViewTransform));
	RootObject->SetStringField(TEXT("source_view_transform_source"), CaptureView.SourceViewTransformSource);
	RootObject->SetObjectField(TEXT("capture_view_transform"), FFromLZCaptureUtils::SerializeTransform(CaptureView.Transform));
	RootObject->SetStringField(TEXT("capture_view_transform_source"), CaptureView.TransformSource);
	RootObject->SetBoolField(TEXT("capture_view_has_player_camera_manager"), CaptureView.bHasPlayerCameraManager);
	if (CaptureView.bHasPlayerCameraManager)
	{
		FTransform PlayerCameraManagerTransform;
		if (TryGetPlayerCameraManagerTransform(Pawn, PlayerCameraManagerTransform))
		{
			RootObject->SetObjectField(TEXT("player_camera_manager_transform"), FFromLZCaptureUtils::SerializeTransform(PlayerCameraManagerTransform));
		}
	}

	if (CameraBoom)
	{
		RootObject->SetObjectField(TEXT("camera_boom"), FFromLZCaptureUtils::SerializeObjectProperties(CameraBoom));
		RootObject->SetObjectField(TEXT("camera_boom_transform"), FFromLZCaptureUtils::SerializeTransform(CameraBoom->GetComponentTransform()));
	}

	TSharedRef<FJsonObject> ViewObject = MakeShared<FJsonObject>();
	ViewObject->SetNumberField(TEXT("fov"), CaptureView.Fov);
	ViewObject->SetNumberField(TEXT("aspect_ratio"), Pending.SourceCameraProjection.AspectRatio);
	ViewObject->SetNumberField(TEXT("max_viewport_width"), CaptureView.MaxViewportSize.X);
	ViewObject->SetNumberField(TEXT("max_viewport_height"), CaptureView.MaxViewportSize.Y);
	ViewObject->SetNumberField(TEXT("viewport_width"), CaptureView.ViewportSize.X);
	ViewObject->SetNumberField(TEXT("viewport_height"), CaptureView.ViewportSize.Y);
	ViewObject->SetNumberField(TEXT("viewport_aspect_ratio"), CaptureView.ViewportAspectRatio);
	ViewObject->SetBoolField(TEXT("constrain_aspect_ratio"), true);
	ViewObject->SetStringField(TEXT("source_projection_mode"), ProjectionModeToString(Pending.SourceCameraProjection.ProjectionMode));
	ViewObject->SetStringField(TEXT("capture_projection_mode"), StaticEnum<ECameraProjectionMode::Type>()->GetValueAsString(ECameraProjectionMode::Orthographic));
	ViewObject->SetStringField(TEXT("projection_mode"), StaticEnum<ECameraProjectionMode::Type>()->GetValueAsString(ECameraProjectionMode::Orthographic));
	ViewObject->SetStringField(TEXT("framing_mode"), CaptureView.FramingMode);
	ViewObject->SetNumberField(TEXT("ortho_width"), CaptureView.OrthoWidth);
	ViewObject->SetBoolField(TEXT("uses_stable_viewport_projection_matrix"), false);
	ViewObject->SetBoolField(TEXT("uses_offscreen_custom_projection_matrix"), CaptureView.bUseCustomProjectionMatrix);
	ViewObject->SetStringField(TEXT("projection_matrix_source"), TEXT("FMinimalViewInfo::CalculateProjectionMatrix"));
	ViewObject->SetObjectField(TEXT("projection_matrix"), SerializeMatrix(CaptureView.ProjectionMatrix));
	ViewObject->SetNumberField(TEXT("default_ortho_width"), FromLZDefaultOrthoWidth);
	ViewObject->SetStringField(TEXT("ortho_width_mode"), CaptureView.OrthoWidthMode);
	ViewObject->SetBoolField(TEXT("ortho_width_from_subject_bounds_center"), Pending.bUsedSubjectBoundsCenterOrthoWidth);
	ViewObject->SetNumberField(TEXT("subject_bounds_focus_depth"), Pending.SubjectFocusDepth);
	AddVectorFields(ViewObject, TEXT("subject_bounds_center"), Pending.SubjectBoundsCenter);
	ViewObject->SetStringField(TEXT("focus_source"), CaptureView.FocusSource);
	ViewObject->SetNumberField(TEXT("focus_depth"), CaptureView.FocusDepth);
	ViewObject->SetNumberField(TEXT("ortho_backoff"), CaptureView.OrthoBackoff);
	AddVectorFields(ViewObject, TEXT("focus_point"), CaptureView.FocusPoint);
	ViewObject->SetStringField(TEXT("reference_source"), CaptureView.ReferenceSource);
	ViewObject->SetNumberField(TEXT("reference_depth"), CaptureView.ReferenceDepth);
	AddVectorFields(ViewObject, TEXT("reference_point"), CaptureView.ReferencePoint);
	ViewObject->SetBoolField(TEXT("used_deproject_width"), CaptureView.bUsedDeprojectWidth);
	ViewObject->SetBoolField(TEXT("used_formula_fallback"), CaptureView.bUsedFormulaFallback);
	ViewObject->SetBoolField(TEXT("used_depth_fallback"), CaptureView.bUsedDepthFallback);
	ViewObject->SetBoolField(TEXT("used_focus_trace"), CaptureView.bUsedFocusTrace);
	ViewObject->SetStringField(TEXT("subject_mode"), SubjectMode);
	ViewObject->SetNumberField(TEXT("subject_actor_count"), SubjectActors.Num());
	ViewObject->SetNumberField(TEXT("near_clip_plane"), FromLZCaptureOrthoNearPlane);
	ViewObject->SetNumberField(TEXT("far_clip_plane"), FromLZCaptureOrthoFarPlane);
	RootObject->SetObjectField(TEXT("camera_view"), ViewObject);

	if (!FFromLZCaptureUtils::SaveJsonToFile(RootObject, OutputPaths.JsonPath))
	{
		UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ failed: could not write json file %s"), *OutputPaths.JsonPath);
		return false;
	}

	CaptureFromLZCameraDebugPng(Pawn, CameraComponent, CaptureView, OutputPaths.DebugFromLZCameraPngPath);

	const bool bLineArtSaved = CaptureLineArtPng(Pawn, CameraComponent, CaptureView, SubjectActors, SubjectMode, OutputPaths.PngPath, OutputPaths.DebugTransformsJsonPath);
	const FString FacesBase = FPaths::Combine(FPaths::GetPath(OutputPaths.PngPath), FPaths::GetBaseFilename(OutputPaths.PngPath));
	const bool bRequiredOutputsAvailable =
		bLineArtSaved &&
		IFileManager::Get().FileExists(*OutputPaths.JsonPath) &&
		IFileManager::Get().FileExists(*(FacesBase + TEXT("_faces.png"))) &&
		IFileManager::Get().FileExists(*(FacesBase + TEXT("_faces.json")));
	if (bRequiredOutputsAvailable)
	{
		UE_LOG(LogTemp, Log, TEXT("CaptureFromLZ saved json to %s and line-art png to %s"), *OutputPaths.JsonPath, *OutputPaths.PngPath);
		if (Pending.bOpenSketchBoardOnSuccess && GEngine && GEngine->GameViewport)
		{
			FFromLZSketchBoard::ShowForCapture(Pawn->GetWorld(), GEngine->GameViewport, OutputPaths.PngPath);
		}
		else if (!Pending.bOpenSketchBoardOnSuccess)
		{
			UE_LOG(LogTemp, Log, TEXT("CaptureFromLZ: sketch board suppressed for source=%s."), *Pending.CameraSource);
		}
	}
	else
	{
		UE_LOG(
			LogTemp,
			Warning,
			TEXT("CaptureFromLZ did not open the sketch board because required capture outputs are incomplete: json=%d line=%d faces_png=%d faces_json=%d"),
			IFileManager::Get().FileExists(*OutputPaths.JsonPath) ? 1 : 0,
			bLineArtSaved ? 1 : 0,
			IFileManager::Get().FileExists(*(FacesBase + TEXT("_faces.png"))) ? 1 : 0,
			IFileManager::Get().FileExists(*(FacesBase + TEXT("_faces.json"))) ? 1 : 0);
	}
	return bRequiredOutputsAvailable;
}

static void ResetPendingOffscreenCapture(const TCHAR* Reason)
{
	if (!GPendingFromLZCapture)
	{
		return;
	}
	const FString ReasonString = Reason ? FString(Reason) : FString(TEXT("Unspecified capture reset."));
	FFromLZCaptureCompletionCallback CompletionCallback = MoveTemp(GPendingFromLZCapture->CompletionCallback);
	FFromLZCaptureResult Result = MakeCaptureResult(GPendingFromLZCapture->OutputPaths, false, ReasonString);
	UE_LOG(
		LogTemp,
		Log,
		TEXT("CaptureFromLZ: pending offscreen capture ended: %s"),
		Reason ? Reason : TEXT("unspecified"));
	GPendingFromLZCapture.Reset();
	DispatchCaptureCompletion(CompletionCallback, Result);
}

static bool PrepareForNewOffscreenCapture(const UWorld* World, FViewport* Viewport)
{
	if (!World || !Viewport)
	{
		UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ failed: world or viewport is null."));
		return false;
	}
	if (GPendingFromLZCapture)
	{
		UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ ignored: an offscreen capture is already pending."));
		return false;
	}
	return true;
}

static bool BeginResolvedOffscreenCapture(
	const UWorld* World,
	FViewport* Viewport,
	APawn* Pawn,
	AActor* CameraActor,
	UCameraComponent* Camera,
	const FString& CameraSource,
	FName RequestedCameraTag,
	bool bAllowSubjectBoundsCenterOrthoWidth,
	bool bOpenSketchBoardOnSuccess = true,
	FFromLZCaptureCompletionCallback CompletionCallback = nullptr,
	FIntPoint CaptureResolutionOverride = FIntPoint(0, 0))
{
	if (!World || !Viewport || !Pawn || !CameraActor || !Camera)
	{
		UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ failed: resolved offscreen capture target is incomplete."));
		DispatchCaptureCompletion(CompletionCallback, MakeCaptureFailureResult(TEXT("Resolved offscreen capture target is incomplete.")));
		return false;
	}

	TUniquePtr<FPendingFromLZCapture> Pending = MakeUnique<FPendingFromLZCapture>();
	Pending->Pawn = Pawn;
	Pending->CameraActor = CameraActor;
	Pending->Camera = Camera;
	Pending->World = const_cast<UWorld*>(World);
	Pending->OutputPaths = MakeCaptureOutputPaths();
	Pending->CameraSource = CameraSource;
	Pending->RequestedCameraTag = RequestedCameraTag;
	Pending->bOpenSketchBoardOnSuccess = bOpenSketchBoardOnSuccess;
	Pending->CompletionCallback = MoveTemp(CompletionCallback);

	TArray<AActor*> SubjectActors;
	BuildCaptureSubjectActors(
		const_cast<UWorld*>(World),
		Pawn,
		CameraActor,
		Camera->GetComponentLocation(),
		SubjectActors,
		Pending->SubjectMode);
	Pending->SubjectActors.Reserve(SubjectActors.Num());
	for (AActor* SubjectActor : SubjectActors)
	{
		Pending->SubjectActors.Add(SubjectActor);
	}
	TArray<AActor*> FramingActors;
	BuildOrthoFramingActorsFromSubjects(SubjectActors, FramingActors);

	if (!BuildOffscreenOrthographicCaptureView(
		Pawn,
		Camera,
		Viewport,
		FramingActors,
		bAllowSubjectBoundsCenterOrthoWidth,
		CaptureResolutionOverride,
		Pending->CaptureView,
		Pending->SourceCameraProjection,
		Pending->bSourceOrthoWidthUsedDefault,
		Pending->bUsedSubjectBoundsCenterOrthoWidth,
		Pending->SubjectBoundsCenter,
		Pending->SubjectFocusDepth))
	{
		DispatchCaptureCompletion(
			Pending->CompletionCallback,
			MakeCaptureResult(Pending->OutputPaths, false, TEXT("Failed to build the offscreen orthographic capture view.")));
		return false;
	}

	Pending->bViewportReferenceSaved =
		CaptureViewportDebugPng(Viewport, Pending->OutputPaths.DebugViewportPerspectivePngPath);
	if (!Pending->bViewportReferenceSaved)
	{
		UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ: current viewport reference image could not be saved; offscreen capture will continue."));
	}

	UE_LOG(
		LogTemp,
		Log,
		TEXT("CaptureFromLZ: queued offscreen capture source=%s camera=%s actor=%s tag=%s output=%dx%d."),
		*CameraSource,
		*Camera->GetName(),
		*CameraActor->GetName(),
		RequestedCameraTag.IsNone() ? TEXT("<none>") : *RequestedCameraTag.ToString(),
		Pending->CaptureView.ViewportSize.X,
		Pending->CaptureView.ViewportSize.Y);
	GPendingFromLZCapture = MoveTemp(Pending);
	return true;
}

bool FFromLZCaptureUtils::BeginCaptureFromWorld(const UWorld* World, FViewport* Viewport)
{
	if (!PrepareForNewOffscreenCapture(World, Viewport))
	{
		return false;
	}

	for (FConstPlayerControllerIterator It = World->GetPlayerControllerIterator(); It; ++It)
	{
		APlayerController* PlayerController = It->Get();
		APawn* Pawn = PlayerController ? PlayerController->GetPawn() : nullptr;
		UCameraComponent* Camera = Pawn ? FindFromLZCamera(Pawn) : nullptr;
		if (Pawn && Camera)
		{
			return BeginResolvedOffscreenCapture(
				World,
				Viewport,
				Pawn,
				Pawn,
				Camera,
				TEXT("pawn_fromlz_offscreen"),
				NAME_None,
				/*bAllowSubjectBoundsCenterOrthoWidth*/ true);
		}
	}

	UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ failed: no player pawn with a FromLZ camera was found."));
	return false;
}

bool FFromLZCaptureUtils::BeginCaptureFromTaggedCamera(const UWorld* World, FViewport* Viewport, FName CameraActorTag)
{
	if (!PrepareForNewOffscreenCapture(World, Viewport))
	{
		return false;
	}
	if (CameraActorTag.IsNone())
	{
		UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ failed: tagged-camera capture requested with an empty tag."));
		return false;
	}

	APawn* Pawn = nullptr;
	for (FConstPlayerControllerIterator It = World->GetPlayerControllerIterator(); It; ++It)
	{
		APlayerController* PlayerController = It->Get();
		if (PlayerController && PlayerController->GetPawn())
		{
			Pawn = PlayerController->GetPawn();
			break;
		}
	}
	if (!Pawn)
	{
		UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ failed: tagged-camera capture requires a player pawn."));
		return false;
	}

	TArray<AActor*> TaggedActors;
	for (TActorIterator<AActor> It(World); It; ++It)
	{
		AActor* Actor = *It;
		if (IsValid(Actor) && Actor->ActorHasTag(CameraActorTag))
		{
			TaggedActors.Add(Actor);
		}
	}
	if (TaggedActors.Num() != 1)
	{
		UE_LOG(
			LogTemp,
			Warning,
			TEXT("CaptureFromLZ failed: tag %s must resolve to exactly one actor, found %d."),
			*CameraActorTag.ToString(),
			TaggedActors.Num());
		return false;
	}

	AActor* CameraActor = TaggedActors[0];
	TArray<UCameraComponent*> CameraComponents;
	CameraActor->GetComponents(CameraComponents);
	TArray<UCameraComponent*> ActiveCameraComponents;
	for (UCameraComponent* CameraComponent : CameraComponents)
	{
		if (IsValid(CameraComponent) && CameraComponent->IsRegistered() && CameraComponent->IsActive())
		{
			ActiveCameraComponents.Add(CameraComponent);
		}
	}
	if (ActiveCameraComponents.Num() != 1)
	{
		UE_LOG(
			LogTemp,
			Warning,
			TEXT("CaptureFromLZ failed: tagged actor %s must have exactly one registered active camera component, found %d."),
			*CameraActor->GetName(),
			ActiveCameraComponents.Num());
		return false;
	}

	return BeginResolvedOffscreenCapture(
		World,
		Viewport,
		Pawn,
		CameraActor,
		ActiveCameraComponents[0],
		TEXT("tagged_actor_offscreen"),
		CameraActorTag,
		/*bAllowSubjectBoundsCenterOrthoWidth*/ false);
}

bool FFromLZCaptureUtils::BeginCaptureFromCameraComponent(
	const UWorld* World,
	FViewport* Viewport,
	UCameraComponent* CameraComponent,
	FIntPoint CaptureResolutionOverride,
	FFromLZCaptureCompletionCallback CompletionCallback)
{
	if (!PrepareForNewOffscreenCapture(World, Viewport))
	{
		DispatchCaptureCompletion(
			CompletionCallback,
			MakeCaptureFailureResult(TEXT("World/viewport is invalid, or another offscreen capture is already pending.")));
		return false;
	}
	if (!IsValid(CameraComponent))
	{
		UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ failed: blueprint camera component is null or invalid."));
		DispatchCaptureCompletion(CompletionCallback, MakeCaptureFailureResult(TEXT("Camera component is null or invalid.")));
		return false;
	}
	if (CameraComponent->GetWorld() != World)
	{
		UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ failed: blueprint camera component belongs to a different world."));
		DispatchCaptureCompletion(CompletionCallback, MakeCaptureFailureResult(TEXT("Camera component belongs to a different world.")));
		return false;
	}
	if (!CameraComponent->IsRegistered() || !CameraComponent->IsActive())
	{
		UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ failed: blueprint camera component must be registered and active."));
		DispatchCaptureCompletion(CompletionCallback, MakeCaptureFailureResult(TEXT("Camera component must be registered and active.")));
		return false;
	}

	AActor* CameraActor = CameraComponent->GetOwner();
	if (!IsValid(CameraActor))
	{
		UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ failed: blueprint camera component has no valid owning actor."));
		DispatchCaptureCompletion(CompletionCallback, MakeCaptureFailureResult(TEXT("Camera component has no valid owning actor.")));
		return false;
	}

	APawn* Pawn = nullptr;
	for (FConstPlayerControllerIterator It = World->GetPlayerControllerIterator(); It; ++It)
	{
		APlayerController* PlayerController = It->Get();
		if (PlayerController && PlayerController->GetPawn())
		{
			Pawn = PlayerController->GetPawn();
			break;
		}
	}
	if (!Pawn)
	{
		UE_LOG(LogTemp, Warning, TEXT("CaptureFromLZ failed: blueprint camera capture requires a player pawn."));
		DispatchCaptureCompletion(CompletionCallback, MakeCaptureFailureResult(TEXT("Blueprint camera capture requires a player pawn.")));
		return false;
	}

	return BeginResolvedOffscreenCapture(
		World,
		Viewport,
		Pawn,
		CameraActor,
		CameraComponent,
		TEXT("blueprint_camera_offscreen"),
		NAME_None,
		/*bAllowSubjectBoundsCenterOrthoWidth*/ false,
		/*bOpenSketchBoardOnSuccess*/ false,
		MoveTemp(CompletionCallback),
		CaptureResolutionOverride);
}

void FFromLZCaptureUtils::CancelPendingCapture()
{
	ResetPendingOffscreenCapture(TEXT("canceled"));
}

void FFromLZCaptureUtils::NotifyViewportDrawn(const UWorld* World, FViewport* Viewport)
{
	// Offscreen captures do not depend on a rendered player-view frame.
}

void FFromLZCaptureUtils::CompletePendingCapture(const UWorld* World, FViewport* Viewport)
{
	if (!GPendingFromLZCapture)
	{
		return;
	}
	if (!World || !Viewport || GPendingFromLZCapture->World.Get() != World ||
		!GPendingFromLZCapture->Pawn.IsValid() ||
		!GPendingFromLZCapture->CameraActor.IsValid() ||
		!GPendingFromLZCapture->Camera.IsValid())
	{
		ResetPendingOffscreenCapture(TEXT("resolved capture target became invalid"));
		return;
	}

	TUniquePtr<FPendingFromLZCapture> Pending = MoveTemp(GPendingFromLZCapture);
	const bool bCaptured = CompleteCaptureFromTarget(*Pending);
	const bool bDebugSaved = FinalizeProjectionDebugFiles(*Pending);
	const bool bSuccess = bCaptured && bDebugSaved;
	if (bCaptured && bDebugSaved)
	{
		UE_LOG(
			LogTemp,
			Log,
			TEXT("CaptureFromLZ: offscreen capture complete source=%s success=1 debug=1 output=%dx%d."),
			*Pending->CameraSource,
			Pending->CaptureView.ViewportSize.X,
			Pending->CaptureView.ViewportSize.Y);
	}
	else
	{
		UE_LOG(
			LogTemp,
			Warning,
			TEXT("CaptureFromLZ: offscreen capture complete source=%s success=%d debug=%d output=%dx%d."),
			*Pending->CameraSource,
			bCaptured ? 1 : 0,
			bDebugSaved ? 1 : 0,
			Pending->CaptureView.ViewportSize.X,
			Pending->CaptureView.ViewportSize.Y);
	}
	FFromLZCaptureCompletionCallback CompletionCallback = MoveTemp(Pending->CompletionCallback);
	DispatchCaptureCompletion(
		CompletionCallback,
		MakeCaptureResult(
			Pending->OutputPaths,
			bSuccess,
			bSuccess
				? FString(TEXT("Capture completed and output files are on disk."))
				: FString::Printf(TEXT("Capture finished with incomplete outputs. capture=%d debug=%d"), bCaptured ? 1 : 0, bDebugSaved ? 1 : 0)));
}

UCameraComponent* FFromLZCaptureUtils::FindFromLZCamera(const APawn* Pawn)
{
	if (!Pawn)
	{
		return nullptr;
	}

	TArray<UCameraComponent*> CameraComponents;
	Pawn->GetComponents(CameraComponents);

	for (UCameraComponent* CameraComponent : CameraComponents)
	{
		if (CameraComponent && CameraComponent->GetName() == TEXT("FromLZ"))
		{
			return CameraComponent;
		}
	}

	return nullptr;
}

USpringArmComponent* FFromLZCaptureUtils::FindCameraBoom(const APawn* Pawn)
{
	if (!Pawn)
	{
		return nullptr;
	}

	TArray<USpringArmComponent*> SpringArmComponents;
	Pawn->GetComponents(SpringArmComponents);

	for (USpringArmComponent* SpringArmComponent : SpringArmComponents)
	{
		if (SpringArmComponent && SpringArmComponent->GetName() == TEXT("CameraBoom"))
		{
			return SpringArmComponent;
		}
	}

	return nullptr;
}

void FFromLZCaptureUtils::BuildCaptureSubjectActors(
	UWorld* World,
	const APawn* Pawn,
	const AActor* CameraActor,
	const FVector& CameraLocation,
	TArray<AActor*>& OutActors,
	FString& OutMode)
{
	::BuildCaptureSubjectActors(World, Pawn, CameraActor, CameraLocation, OutActors, OutMode);
}

void FFromLZCaptureUtils::BuildOrthoFramingActors(const TArray<AActor*>& SubjectActors, TArray<AActor*>& OutFramingActors)
{
	::BuildOrthoFramingActorsFromSubjects(SubjectActors, OutFramingActors);
}

void FFromLZCaptureUtils::ApplyCaptureSubjectActors(USceneCaptureComponent2D* SceneCapture, const TArray<AActor*>& SubjectActors)
{
	::ApplyCaptureSubjectActors(SceneCapture, SubjectActors);
}

bool FFromLZCaptureUtils::CalculateSubjectBoundsCenterOrthoWidth(
	const UCameraComponent* Camera,
	const TArray<AActor*>& SubjectActors,
	double& OutOrthoWidth,
	FVector& OutBoundsCenter,
	double& OutFocusDepth)
{
	if (!Camera)
	{
		return false;
	}

	bool bHasBounds = false;
	FBox CombinedBounds(EForceInit::ForceInit);
	for (const AActor* Actor : SubjectActors)
	{
		if (!IsValid(Actor))
		{
			continue;
		}

		FVector Origin;
		FVector Extent;
		Actor->GetActorBounds(false, Origin, Extent);
		if (!Origin.ContainsNaN() && !Extent.ContainsNaN() && Extent.GetMax() > 0.0)
		{
			CombinedBounds += FBox::BuildAABB(Origin, Extent);
			bHasBounds = true;
		}
	}

	if (!bHasBounds || !CombinedBounds.IsValid)
	{
		return false;
	}

	const FTransform CameraTransform = Camera->GetComponentTransform();
	const FVector CameraLocation = CameraTransform.GetLocation();
	const FVector CameraForward = CameraTransform.GetUnitAxis(EAxis::X).GetSafeNormal();
	if (CameraLocation.ContainsNaN() || CameraForward.IsNearlyZero())
	{
		return false;
	}

	const double FovDegrees = static_cast<double>(Camera->FieldOfView);
	if (!FMath::IsFinite(FovDegrees) || FovDegrees <= 1e-3 || FovDegrees >= 179.0)
	{
		return false;
	}

	OutBoundsCenter = CombinedBounds.GetCenter();
	OutFocusDepth = FVector::DotProduct(OutBoundsCenter - CameraLocation, CameraForward);
	if (!FMath::IsFinite(OutFocusDepth) || OutFocusDepth <= 1e-3)
	{
		return false;
	}

	OutOrthoWidth = 2.0 * OutFocusDepth * FMath::Tan(FMath::DegreesToRadians(FovDegrees * 0.5));
	return FMath::IsFinite(OutOrthoWidth) && OutOrthoWidth > 1e-3;
}

TSharedRef<FJsonObject> FFromLZCaptureUtils::SerializeObjectProperties(const UObject* Object)
{
	TSharedRef<FJsonObject> RootObject = MakeShared<FJsonObject>();

	if (!Object)
	{
		RootObject->SetStringField(TEXT("error"), TEXT("Null object"));
		return RootObject;
	}

	RootObject->SetStringField(TEXT("object_name"), Object->GetName());
	RootObject->SetStringField(TEXT("class_name"), Object->GetClass()->GetName());

	TSharedRef<FJsonObject> PropertiesObject = MakeShared<FJsonObject>();

	for (TFieldIterator<FProperty> PropertyIt(Object->GetClass(), EFieldIterationFlags::IncludeSuper); PropertyIt; ++PropertyIt)
	{
		const FProperty* Property = *PropertyIt;
		FString ValueText;
		Property->ExportText_InContainer(0, ValueText, Object, Object, const_cast<UObject*>(Object), PPF_None);

		TSharedRef<FJsonObject> PropertyObject = MakeShared<FJsonObject>();
		PropertyObject->SetStringField(TEXT("cpp_type"), Property->GetCPPType());
		PropertyObject->SetStringField(TEXT("value_text"), ValueText);

		PropertiesObject->SetObjectField(Property->GetName(), PropertyObject);
	}

	RootObject->SetObjectField(TEXT("properties"), PropertiesObject);
	return RootObject;
}

TSharedRef<FJsonObject> FFromLZCaptureUtils::SerializeTransform(const FTransform& Transform)
{
	TSharedRef<FJsonObject> TransformObject = MakeShared<FJsonObject>();

	const FVector Location = Transform.GetLocation();
	const FRotator Rotation = Transform.Rotator();
	const FVector Scale = Transform.GetScale3D();

	TransformObject->SetNumberField(TEXT("location_x"), Location.X);
	TransformObject->SetNumberField(TEXT("location_y"), Location.Y);
	TransformObject->SetNumberField(TEXT("location_z"), Location.Z);
	TransformObject->SetNumberField(TEXT("pitch"), Rotation.Pitch);
	TransformObject->SetNumberField(TEXT("yaw"), Rotation.Yaw);
	TransformObject->SetNumberField(TEXT("roll"), Rotation.Roll);
	TransformObject->SetNumberField(TEXT("scale_x"), Scale.X);
	TransformObject->SetNumberField(TEXT("scale_y"), Scale.Y);
	TransformObject->SetNumberField(TEXT("scale_z"), Scale.Z);

	return TransformObject;
}

bool FFromLZCaptureUtils::SaveJsonToFile(const TSharedRef<FJsonObject>& JsonObject, const FString& FilePath)
{
	FString OutputString;
	const TSharedRef<TJsonWriter<TCHAR, TPrettyJsonPrintPolicy<TCHAR>>> Writer =
		TJsonWriterFactory<TCHAR, TPrettyJsonPrintPolicy<TCHAR>>::Create(&OutputString);

	if (!FJsonSerializer::Serialize(JsonObject, Writer))
	{
		return false;
	}

	return FFileHelper::SaveStringToFile(OutputString, *FilePath, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
}
