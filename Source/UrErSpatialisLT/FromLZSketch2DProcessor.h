#pragma once

#include "CoreMinimal.h"
#include "FromLZFaceReconstructor.h"

class UWorld;

struct FFromLZProcessParams
{
	uint8 Step1WhiteThreshold = 240;
	int32 Step1MinArea = 12;
	int32 Step2ThinningMaxIter = 100;
	int32 Step2SkeletonMinArea = 6;
	int32 ColorSampleRadius = 2;
	int32 ColorWhiteCutoff = 220;
	int32 ColorDominanceMargin = 30;
	float Step3GapTol = 10.0f;
	int32 Step3ConnectThickness = 1;
	float Step3SmallLoopBboxAreaThresh = 500.0f;
	float Step3BranchPruneMaxPixels = 0.0f;
	float Step4EndpointTol = 3.0f;
	float Step4ColorMinRunArc = 3.0f;
	int32 Step4TraceMinPixels = 3;
	float Step5AngleThresh = 25.0f;
	float Step5SegmentArc = 30.0f;
	float Step5SplitPeakMinDistance = 10.0f;
	int32 Step5MaxIters = 5;
	float Step6MaxGap = 3.0f;
	float Step6MaxAngle = 12.0f;
	int32 Step6MaxIters = 80;
	float Step6ProtectJunctionRadius = 3.0f;
	int32 Step8Thickness = 3;
	float Step9ConnectorTol = 20.0f;
	float Step9BlackSelectTol = 20.0f;

	float CapLoopGraphNodeSnapTol = 5.0f;
	float CapRedOnlyLoopMinBboxArea = 500.0f;
	float CapBorrowedLoopMinBboxArea = 500.0f;
	float CapMixedSelectionTimeBudgetSeconds = 5.0f;
	float InteriorGreenMinInsideLengthPx = 10.0f;
	float GreenTraceEndpointTolPx = 10.0f;
	float GreenTraceMaxChainDeviationDeg = 45.0f;

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

	int32 CompositeMaxWorkers = 1;
	int32 ParallelForMaxThreads = 8;

	FFromLZFaceReconstructionParams ToFaceReconstructionParams() const
	{
		FFromLZFaceReconstructionParams Out;
		Out.CandidateFaceMinOverlapRatio = FMath::Max(0.0f, CandidateFaceMinOverlapRatio);
		Out.CandidateFaceMaxNormalSideAngleDegrees = FMath::Clamp(CandidateFaceMaxNormalSideAngleDegrees, 0.0f, 180.0f);
		Out.CandidateFacePreferredNormalSideAngleDegrees = FMath::Clamp(CandidateFacePreferredNormalSideAngleDegrees, 0.0f, 180.0f);
		Out.bEnableAttachSupportPlaneFallback = bEnableAttachSupportPlaneFallback;
		Out.SupportFaceVoteRadiusPx = FMath::Max(0, SupportFaceVoteRadiusPx);
		Out.SupportPlanePolygonTolPx = FMath::Max(0.0f, SupportPlanePolygonTolPx);
		Out.SupportFaceVoteSampleStepPx = FMath::Max(1.0f, SupportFaceVoteSampleStepPx);
		Out.SupportFaceVoteMinCoverage = FMath::Clamp(SupportFaceVoteMinCoverage, 0.0f, 1.0f);
		Out.NoPenetrationTolCm = FMath::Max(0.0f, NoPenetrationTolCm);
		Out.ContactAnchorTolPx = FMath::Max(0.0f, ContactAnchorTolPx);
		Out.AttachPathFrontDistanceTieTolCm = FMath::Max(0.0f, AttachPathFrontDistanceTieTolCm);
		Out.AttachPathPlaneRelationAngleTolDeg = FMath::Max(0.0f, AttachPathPlaneRelationAngleTolDeg);
		Out.AttachPathPlaneRelationDistanceTolCm = FMath::Max(0.0f, AttachPathPlaneRelationDistanceTolCm);
		Out.SupportForceHardMinGreenChordCm = FMath::Max(0.0f, SupportForceHardMinGreenChordCm);
		Out.SupportForcePreferredMinGreenChordCm = FMath::Max(
			Out.SupportForceHardMinGreenChordCm,
			SupportForcePreferredMinGreenChordCm);
		Out.bWorldOrthoUsePerFaceCapture = bWorldOrthoUsePerFaceCapture;
		Out.WorldOrthoPerFaceClipMarginPixels = FMath::Max(0.0f, WorldOrthoPerFaceClipMarginPixels);
		Out.bWorldOrthoPureRedAllowDiagonalRoot = bWorldOrthoPureRedAllowDiagonalRoot;
		Out.bWorldOrthoAllowDiagonalSupports = bWorldOrthoAllowDiagonalSupports;
		Out.WorldOrthoBlackAxisToleranceDegrees = FMath::Clamp(WorldOrthoBlackAxisToleranceDegrees, 0.0f, 180.0f);
		Out.WorldOrthoDiagThresholdDegrees = FMath::Clamp(WorldOrthoDiagThresholdDegrees, 0.0f, 90.0f);
		Out.WorldOrthoAngleComparisonEpsilonDegrees = FMath::Max(0.0f, WorldOrthoAngleComparisonEpsilonDegrees);
		Out.WorldOrthoBlackNodeSnapTolerancePixels = FMath::Max(0.0f, WorldOrthoBlackNodeSnapTolerancePixels);
		Out.WorldOrthoRedMacroCorridorPixels = FMath::Max(0.0f, WorldOrthoRedMacroCorridorPixels);
		Out.WorldOrthoRedMacroGroupMinLengthPixels = FMath::Max(0.0f, WorldOrthoRedMacroGroupMinLengthPixels);
		Out.WorldOrthoRedPrimitiveRdpTolerancePixels = FMath::Max(0.0f, WorldOrthoRedPrimitiveRdpTolerancePixels);
		Out.WorldOrthoShortRedEdgeLengthPixels = FMath::Max(0.0f, WorldOrthoShortRedEdgeLengthPixels);
		Out.WorldOrthoMinAreaRatio = FMath::Max(0.0f, WorldOrthoMinAreaRatio);
		Out.WorldOrthoMaxWrapGapFraction = FMath::Clamp(WorldOrthoMaxWrapGapFraction, 0.0f, 1.0f);
		Out.bWorldOrthoAllowTopologyRepair = bWorldOrthoAllowTopologyRepair;
		Out.WorldOrthoPureRedMaxRootCandidates = FMath::Max(0, WorldOrthoPureRedMaxRootCandidates);
		return Out;
	}
};

using FFromLZCompositeStartedCallback = TFunction<void(const FFromLZPressProcessResult& Result)>;

// Migrated sketch pipeline: Steps 1-9 analyze the composite image in C++, then
// Step 10/11 are dispatched to the face reconstructor for runtime geometry.
//
// Triggered after a board Proceed/Space composite (wContextSketch_raw.png) is
// produced. Debug artifacts are written under <ProjectSaved>/2DDebug/ and
// Action.json files under <ProjectSaved>/FromAction/.
//
// Active steps:
//   [x] Step 1 preprocess (non-white binarize + remove small comps; no morphology)
//   [x] Step 2 skeletonize (Zhang-Suen + remove small skeleton comps)
//   [x] Step 3 skeleton gap repair (connect endpoints + small-loop / dangling prune)
//   [x] Step 4 stroke tracing (crossing-number nodes + 8-connected polylines)
//             + color classification (red/green/blue/black) and color-boundary split
//   [x] Step 5 corner split (PCA arc-window axis-angle + iterative NMS), color preserved
//   [x] Step 6 same-color merge (RGB/black classes never merged across colors)
//   [x] Step 7 stroke metrics (arc/chord/straightness/PCA errors/direction)
//   [x] Step 8 enclosed-region mask (endpoint-nearest-connect + flood)
//   [x] Step 9 per-component red cap-loops -> per-component longest-green side -> translate-copy
//       Each run writes to Saved/2DDebug/Press_##/, with one Component_%% subfolder per cap.
//   [x] Step 10/11 dispatch (face validation, solid rebuild, runtime spawn/Boolean)
//   Each stroke step emits a per-stroke palette PNG, a class-color PNG, and a JSON
//   (id / color / kind / metrics / endpoints / neighbors / points).
// Records which FromLZCaptures / FromSketch source files a Space press consumed.
// Paths are relative to the project's Saved/ folder (e.g. "FromLZCaptures/FromLZ_xx.png").
struct FSketchSourceInfo
{
	bool bHasCapture = false;
	FString CaptureStem;     // e.g. "FromLZ_20260531_131543"
	FString CapturePngRel;   // "FromLZCaptures/FromLZ_...png"
	FString CaptureJsonRel;  // "FromLZCaptures/FromLZ_...json"
	FString FacesPngRel;     // "FromLZCaptures/FromLZ_..._faces.png"
	FString FacesJsonRel;    // "FromLZCaptures/FromLZ_..._faces.json"
	FString ActorMaterialPngRel;  // "FromLZCaptures/FromLZ_..._actor_material_id.png"
	FString ActorMaterialJsonRel; // "FromLZCaptures/FromLZ_..._actor_material_id.json"
	FString SketchPngRel;    // "FromSketch/....png"
};

class FFromLZSketch2DProcessor
{
public:
	// Moves the RGBA buffer into the bounded background scheduler so the game
	// thread never blocks on the heavy 2D/3D processing path.
	static void ProcessCompositeAsync(TArray<uint8> RGBA, int32 Width, int32 Height, const FString& DebugDir, const FSketchSourceInfo& Source, UWorld* World);
	static void ProcessCompositeAsync(
		TArray<uint8> RGBA,
		int32 Width,
		int32 Height,
		const FString& DebugDir,
		const FSketchSourceInfo& Source,
		UWorld* World,
		const FFromLZProcessParams& Params,
		FFromLZCompositeStartedCallback StartedCallback,
		FFromLZPressCompletionCallback CompletionCallback);

	// Synchronous entry point (runs on the calling thread). Returns true on success.
	static bool ProcessComposite(const TArray<uint8>& RGBA, int32 Width, int32 Height, const FString& DebugDir, const FSketchSourceInfo& Source, TWeakObjectPtr<UWorld> World);
	static bool ProcessComposite(
		const TArray<uint8>& RGBA,
		int32 Width,
		int32 Height,
		const FString& DebugDir,
		const FSketchSourceInfo& Source,
		TWeakObjectPtr<UWorld> World,
		const FFromLZProcessParams& Params);

	static bool ProcessCompositeWithGeneration(const TArray<uint8>& RGBA, int32 Width, int32 Height, const FString& DebugDir, const FSketchSourceInfo& Source, int32 SessionGeneration, TWeakObjectPtr<UWorld> World);
	static bool ProcessCompositeWithGeneration(
		const TArray<uint8>& RGBA,
		int32 Width,
		int32 Height,
		const FString& DebugDir,
		const FSketchSourceInfo& Source,
		int32 SessionGeneration,
		TWeakObjectPtr<UWorld> World,
		const FFromLZProcessParams& Params,
		FFromLZCompositeStartedCallback StartedCallback = nullptr,
		FFromLZPressCompletionCallback CompletionCallback = nullptr);
};
