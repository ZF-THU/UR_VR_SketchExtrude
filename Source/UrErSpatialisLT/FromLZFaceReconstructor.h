#pragma once

#include "CoreMinimal.h"

class AActor;
class UWorld;

struct FFromLZPressProcessResult
{
	bool bSuccess = false;
	FString Message;
	FString PressDir;
	FString ActionPressDir;
	TArray<FString> OutputFiles;
	int32 SpawnedRuntimeActorCount = 0;
};

using FFromLZPressCompletionCallback = TFunction<void(const FFromLZPressProcessResult& Result)>;

struct FFromLZStep11UndoResult
{
	bool bSuccess = false;
	FString Message;
	FString PressId;
	FString UndoDiagnosticPath;
	TArray<FString> OutputFiles;
	int32 DestroyedActorCount = 0;
	int32 DestroyedBooleanComponentCount = 0;
	int32 RestoredHiddenSourceCount = 0;
};

using FFromLZStep11UndoCompletionCallback = TFunction<void(const FFromLZStep11UndoResult& Result)>;

struct FFromLZCandidateFaceRequest
{
	FString CandidateSource;
	FString Action;
	TArray<FVector2D> CapPolygon;
	TArray<FVector2D> CapPolygonTranslated;
	TArray<FVector2D> SideVectors;
};

struct FFromLZCandidateFaceEvaluation
{
	bool bEvaluated = false;
	bool bValid = false;
	FString SourcePolygonKey;
	FString RejectReason;
	int32 CapMaskPixels = 0;
	int32 SelectedFaceId = -1;
	int32 SelectedFaceOverlapPixels = 0;
	double SelectedFaceOverlapRatio = 0.0;
	double SelectedFaceNormalSideAngleDegrees = -1.0;
	double SelectedFaceDistanceToCamera = 0.0;
	FVector SelectedPlaneHit = FVector::ZeroVector;
};

class FFromLZFaceReconstructor
{
public:
	static constexpr double CandidateFaceMinOverlapRatio = 0.25;
	static constexpr double CandidateFaceMaxNormalSideAngleDegrees = 30.0;
	static constexpr double CandidateFacePreferredNormalSideAngleDegrees = 10.0;

	static bool EvaluateCandidateFaces(
		const FString& PressDir,
		int32 SourceWidth,
		int32 SourceHeight,
		const TArray<FFromLZCandidateFaceRequest>& Requests,
		TArray<FFromLZCandidateFaceEvaluation>& OutEvaluations,
		FString& OutError);
	static void ProcessPress(
		const FString& PressDir,
		const FString& ActionPressDir,
		TWeakObjectPtr<UWorld> World,
		int32 SessionGeneration = INDEX_NONE,
		FFromLZPressCompletionCallback CompletionCallback = nullptr);
	static void RestoreStep11RuntimeBooleans(
		TWeakObjectPtr<UWorld> World,
		FFromLZStep11UndoCompletionCallback CompletionCallback = nullptr);
	static void ResetAllRuntimeState(TWeakObjectPtr<UWorld> World);
	static bool IsStep11RuntimeActor(const AActor* Actor);
	static bool IsStep11RuntimeActorActiveForCapture(const AActor* Actor);
};
