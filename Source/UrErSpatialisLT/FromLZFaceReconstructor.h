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

struct FFromLZFaceReconstructionParams
{
	float CandidateFaceMinOverlapRatio = 0.25f;
	float CandidateFaceMaxNormalSideAngleDegrees = 30.0f;
	float CandidateFacePreferredNormalSideAngleDegrees = 10.0f;

	bool bEnableAttachSupportPlaneFallback = true;
	int32 SupportFaceVoteRadiusPx = 2;
	float SupportPlanePolygonTolPx = 15.0f;
	float SupportFaceVoteSampleStepPx = 5.0f;
	float SupportFaceVoteMinCoverage = 0.20f;
	float NoPenetrationTolCm = 50.0f;
	float ContactAnchorTolPx = 20.0f;
	float AttachPathFrontDistanceTieTolCm = 5.0f;
	float AttachPathPlaneRelationAngleTolDeg = 10.0f;
	float AttachPathPlaneRelationDistanceTolCm = 10.0f;
	float SupportForceHardMinGreenChordCm = 1.0f;
	float SupportForcePreferredMinGreenChordCm = 5.0f;

	bool bWorldOrthoUsePerFaceCapture = true;
	float WorldOrthoPerFaceClipMarginPixels = 1.5f;
	bool bWorldOrthoPureRedAllowDiagonalRoot = false;
	bool bWorldOrthoAllowDiagonalSupports = false;
	float WorldOrthoBlackAxisToleranceDegrees = 5.0f;
	float WorldOrthoDiagThresholdDegrees = 40.0f;
	float WorldOrthoAngleComparisonEpsilonDegrees = 0.000001f;
	float WorldOrthoBlackNodeSnapTolerancePixels = 10.0f;
	float WorldOrthoRedMacroCorridorPixels = 5.0f;
	float WorldOrthoRedMacroGroupMinLengthPixels = 20.0f;
	float WorldOrthoRedPrimitiveRdpTolerancePixels = 1.0f;
	float WorldOrthoShortRedEdgeLengthPixels = 20.0f;
	float WorldOrthoMinAreaRatio = 0.4f;
	float WorldOrthoMaxWrapGapFraction = 0.1f;
	bool bWorldOrthoAllowTopologyRepair = true;
	int32 WorldOrthoPureRedMaxRootCandidates = 5;
};

struct FFromLZGreenChainCandidate2D
{
	int32 SeedStrokeId = -1;
	TArray<int32> StrokeIds;
	FVector2D Start = FVector2D::ZeroVector;
	FVector2D End = FVector2D::ZeroVector;
	FVector2D Vector = FVector2D::ZeroVector;
	FVector2D SeedDirection = FVector2D::ZeroVector;
	double ChordLength = 0.0;
	double PathLength = 0.0;
	double TotalGap = 0.0;
	FString StopReason;
};

struct FFromLZCandidateFaceRequest
{
	FString CandidateSource;
	FString Action;
	TArray<FVector2D> CapPolygon;
	TArray<FVector2D> CapPolygonTranslated;
	TArray<FVector2D> SideVectors;
	TArray<FFromLZGreenChainCandidate2D> GreenChains;
};

struct FFromLZSupportFaceVoteCandidate
{
	int32 FaceId = -1;
	int32 VotePixels = 0;
	int32 HitSampleCount = 0;
	int32 TotalSampleCount = 0;
	int32 TotalFaceVotePixels = 0;
	double SampleCoverage = 0.0;
	double VotePixelCoverage = 0.0;
	double WorldZMax = 0.0;
	double WorldZAverage = 0.0;
	double MinCameraDistance = 0.0;
	double WorstPolygonDistancePx = 0.0;
	bool bCoveragePass = false;
	bool bSelectedForChain = false;
};

struct FFromLZSupportFaceVoteAttempt
{
	int32 ChainIndex = -1;
	int32 SeedStrokeId = -1;
	FVector2D ChainStartFaceSpace = FVector2D::ZeroVector;
	FVector2D ChainEndFaceSpace = FVector2D::ZeroVector;
	double ChainChordLength = 0.0;
	double ChainPathLength = 0.0;
	bool bVoteFound = false;
	bool bCoveragePass = false;
	bool bSelected = false;
	int32 SupportFaceId = -1;
	int32 SupportVotePixels = 0;
	int32 SupportConsideredPixels = 0;
	int32 SupportHitSampleCount = 0;
	int32 SupportTotalSampleCount = 0;
	double SupportVoteCoverage = 0.0;
	double SupportVotePixelCoverage = 0.0;
	double SupportFaceWorldZMax = 0.0;
	double SupportFaceWorldZAverage = 0.0;
	double SupportMinCameraDistance = 0.0;
	double SupportWorstPolygonDistancePx = 0.0;
	FString RejectReason;
	TArray<FFromLZSupportFaceVoteCandidate> SupportFaceCandidates;
};

struct FFromLZCandidateFaceEvaluation
{
	bool bEvaluated = false;
	bool bValid = false;
	FString EvaluationMode;
	FString SourcePolygonKey;
	FString RejectReason;
	int32 CapMaskPixels = 0;
	int32 SelectedFaceId = -1;
	int32 SelectedFaceOverlapPixels = 0;
	double SelectedFaceOverlapRatio = 0.0;
	double SelectedFaceNormalSideAngleDegrees = -1.0;
	double SelectedFaceDistanceToCamera = 0.0;
	FVector SelectedPlaneHit = FVector::ZeroVector;
	bool bAttachSupportPlaneFallbackEligible = false;
	int32 SupportFaceId = -1;
	int32 SupportGreenChainIndex = -1;
	double SupportFaceVoteCoverage = 0.0;
	FString SupportFaceRejectReason;
	TArray<FFromLZSupportFaceVoteAttempt> SupportFaceVoteAttempts;
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
		const FFromLZFaceReconstructionParams& Params,
		TArray<FFromLZCandidateFaceEvaluation>& OutEvaluations,
		FString& OutError);
	static void ProcessPress(
		const FString& PressDir,
		const FString& ActionPressDir,
		TWeakObjectPtr<UWorld> World,
		int32 SessionGeneration = INDEX_NONE,
		FFromLZPressCompletionCallback CompletionCallback = nullptr);
	static void ProcessPress(
		const FString& PressDir,
		const FString& ActionPressDir,
		TWeakObjectPtr<UWorld> World,
		const FFromLZFaceReconstructionParams& Params,
		int32 SessionGeneration = INDEX_NONE,
		FFromLZPressCompletionCallback CompletionCallback = nullptr);
	static void RestoreStep11RuntimeBooleans(
		TWeakObjectPtr<UWorld> World,
		FFromLZStep11UndoCompletionCallback CompletionCallback = nullptr);
	static void ResetAllRuntimeState(TWeakObjectPtr<UWorld> World);
	static bool IsStep11RuntimeActor(const AActor* Actor);
	static bool IsStep11RuntimeActorActiveForCapture(const AActor* Actor);
};
