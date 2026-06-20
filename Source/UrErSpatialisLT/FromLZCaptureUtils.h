#pragma once

#include "CoreMinimal.h"

class APawn;
class AActor;
class FJsonObject;
class UCameraComponent;
class UObject;
class USceneCaptureComponent2D;
class USpringArmComponent;
class UWorld;
class FViewport;

struct FFromLZCaptureResult
{
	bool bSuccess = false;
	FString Message;
	FString CaptureDirectory;
	FString MainPngPath;
	FString MainJsonPath;
	TArray<FString> OutputFiles;
};

using FFromLZCaptureCompletionCallback = TFunction<void(const FFromLZCaptureResult& Result)>;

class FFromLZCaptureUtils
{
public:
	static bool BeginCaptureFromWorld(const UWorld* World, FViewport* Viewport);
	static bool BeginCaptureFromTaggedCamera(const UWorld* World, FViewport* Viewport, FName CameraActorTag);
	static bool BeginCaptureFromCameraComponent(
		const UWorld* World,
		FViewport* Viewport,
		UCameraComponent* CameraComponent,
		FIntPoint CaptureResolutionOverride = FIntPoint(0, 0),
		FFromLZCaptureCompletionCallback CompletionCallback = nullptr);
	static void CancelPendingCapture();
	static void NotifyViewportDrawn(const UWorld* World, FViewport* Viewport);
	static void CompletePendingCapture(const UWorld* World, FViewport* Viewport);
	static UCameraComponent* FindFromLZCamera(const APawn* Pawn);
	static USpringArmComponent* FindCameraBoom(const APawn* Pawn);
	static void BuildCaptureSubjectActors(UWorld* World, const APawn* Pawn, const AActor* CameraActor, const FVector& CameraLocation, TArray<AActor*>& OutActors, FString& OutMode);
	static void BuildOrthoFramingActors(const TArray<AActor*>& SubjectActors, TArray<AActor*>& OutFramingActors);
	static void ApplyCaptureSubjectActors(USceneCaptureComponent2D* SceneCapture, const TArray<AActor*>& SubjectActors);
	static bool CalculateSubjectBoundsCenterOrthoWidth(const UCameraComponent* Camera, const TArray<AActor*>& SubjectActors, double& OutOrthoWidth, FVector& OutBoundsCenter, double& OutFocusDepth);
	static TSharedRef<FJsonObject> SerializeObjectProperties(const UObject* Object);
	static TSharedRef<FJsonObject> SerializeTransform(const FTransform& Transform);
	static bool SaveJsonToFile(const TSharedRef<FJsonObject>& JsonObject, const FString& FilePath);
};
