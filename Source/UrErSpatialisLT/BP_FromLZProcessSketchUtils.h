#pragma once

#include "CoreMinimal.h"
#include "FromLZSketch2DProcessor.h"
#include "Kismet/BlueprintAsyncActionBase.h"
#include "BP_FromLZProcessSketchUtils.generated.h"

USTRUCT(BlueprintType)
struct FBPFromLZProcessParams
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 1")
	int32 Step1WhiteThreshold = 240;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 1")
	int32 Step1MinArea = 12;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 2")
	int32 Step2ThinningMaxIter = 100;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 2")
	int32 Step2SkeletonMinArea = 6;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Color")
	int32 ColorSampleRadius = 2;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Color")
	int32 ColorWhiteCutoff = 220;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Color")
	int32 ColorDominanceMargin = 30;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 3")
	float Step3GapTol = 10.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 3")
	int32 Step3ConnectThickness = 1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 3")
	float Step3SmallLoopBboxAreaThresh = 500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 3")
	float Step3BranchPruneMaxPixels = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 4")
	float Step4EndpointTol = 3.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 4")
	float Step4ColorMinRunArc = 3.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 4")
	int32 Step4TraceMinPixels = 3;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 5")
	float Step5AngleThresh = 25.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 5")
	float Step5SegmentArc = 30.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 5")
	float Step5SplitPeakMinDistance = 10.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 5")
	int32 Step5MaxIters = 5;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 6")
	float Step6MaxGap = 3.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 6")
	float Step6MaxAngle = 12.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 6")
	int32 Step6MaxIters = 80;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 6")
	float Step6ProtectJunctionRadius = 3.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 8")
	int32 Step8Thickness = 3;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 9")
	float Step9ConnectorTol = 20.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 9")
	float Step9BlackSelectTol = 20.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 9")
	float CapLoopGraphNodeSnapTol = 5.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 9")
	float CapRedOnlyLoopMinBboxArea = 500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 9")
	float CapBorrowedLoopMinBboxArea = 500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 9")
	float CapMixedSelectionTimeBudgetSeconds = 5.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 9")
	float InteriorGreenMinInsideLengthPx = 10.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 9")
	float GreenTraceEndpointTolPx = 10.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 9")
	float GreenTraceMaxChainDeviationDeg = 45.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 10")
	float CandidateFaceMinOverlapRatio = 0.25f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 10")
	float CandidateFaceMaxNormalSideAngleDegrees = 30.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 10")
	float CandidateFacePreferredNormalSideAngleDegrees = 10.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 10|Attach Support Fallback")
	bool bEnableAttachSupportPlaneFallback = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 10|Attach Support Fallback", meta = (ClampMin = "0", UIMin = "0"))
	int32 SupportFaceVoteRadiusPx = 2;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 10|Attach Support Fallback", meta = (ClampMin = "0.0", UIMin = "0.0"))
	float SupportPlanePolygonTolPx = 15.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 10|Attach Support Fallback", meta = (ClampMin = "1.0", UIMin = "1.0"))
	float SupportFaceVoteSampleStepPx = 5.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 10|Attach Support Fallback", meta = (ClampMin = "0.0", ClampMax = "1.0", UIMin = "0.0", UIMax = "1.0"))
	float SupportFaceVoteMinCoverage = 0.20f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 10|Attach Support Fallback", meta = (ClampMin = "0.0", UIMin = "0.0"))
	float NoPenetrationTolCm = 50.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 10|Attach Support Fallback", meta = (ClampMin = "0.0", UIMin = "0.0"))
	float ContactAnchorTolPx = 20.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 10|Attach Support Fallback", meta = (ClampMin = "0.0", UIMin = "0.0"))
	float AttachPathFrontDistanceTieTolCm = 5.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 10|Attach Support Fallback", meta = (ClampMin = "0.0", UIMin = "0.0"))
	float AttachPathPlaneRelationAngleTolDeg = 10.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 10|Attach Support Fallback", meta = (ClampMin = "0.0", UIMin = "0.0"))
	float AttachPathPlaneRelationDistanceTolCm = 10.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 10|Attach Support Fallback", meta = (ClampMin = "0.0", UIMin = "0.0"))
	float SupportForceHardMinGreenChordCm = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 10|Attach Support Fallback", meta = (ClampMin = "0.0", UIMin = "0.0"))
	float SupportForcePreferredMinGreenChordCm = 5.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 10|World Orthogonal")
	bool bWorldOrthoUsePerFaceCapture = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 10|World Orthogonal", meta = (ClampMin = "0.0", UIMin = "0.0"))
	float WorldOrthoPerFaceClipMarginPixels = 1.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 10|World Orthogonal")
	bool bWorldOrthoPureRedAllowDiagonalRoot = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 10|World Orthogonal")
	bool bWorldOrthoAllowDiagonalSupports = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 10|World Orthogonal", meta = (ClampMin = "0.0", ClampMax = "1.0", UIMin = "0.0", UIMax = "1.0"))
	float WorldOrthoMaxWrapGapFraction = 0.1f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 10|World Orthogonal")
	bool bWorldOrthoAllowTopologyRepair = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 10|World Orthogonal", meta = (ClampMin = "0.0", ClampMax = "180.0", UIMin = "0.0", UIMax = "180.0"))
	float WorldOrthoBlackAxisToleranceDegrees = 5.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 10|World Orthogonal", meta = (ClampMin = "0.0", ClampMax = "90.0", UIMin = "0.0", UIMax = "90.0"))
	float WorldOrthoDiagThresholdDegrees = 40.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 10|World Orthogonal", meta = (ClampMin = "0.0", UIMin = "0.0"))
	float WorldOrthoAngleComparisonEpsilonDegrees = 0.000001f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 10|World Orthogonal", meta = (ClampMin = "0.0", UIMin = "0.0"))
	float WorldOrthoBlackNodeSnapTolerancePixels = 10.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 10|World Orthogonal", meta = (ClampMin = "0.0", UIMin = "0.0"))
	float WorldOrthoRedMacroCorridorPixels = 5.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 10|World Orthogonal", meta = (ClampMin = "0.0", UIMin = "0.0"))
	float WorldOrthoRedMacroGroupMinLengthPixels = 20.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 10|World Orthogonal", meta = (ClampMin = "0.0", UIMin = "0.0"))
	float WorldOrthoRedPrimitiveRdpTolerancePixels = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 10|World Orthogonal", meta = (ClampMin = "0.0", UIMin = "0.0"))
	float WorldOrthoShortRedEdgeLengthPixels = 20.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 10|World Orthogonal", meta = (ClampMin = "0.0", UIMin = "0.0"))
	float WorldOrthoMinAreaRatio = 0.4f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Step 10|World Orthogonal", meta = (ClampMin = "0", UIMin = "0"))
	int32 WorldOrthoPureRedMaxRootCandidates = 5;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Concurrency")
	int32 CompositeMaxWorkers = 1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Concurrency")
	int32 ParallelForMaxThreads = 8;

	FFromLZProcessParams ToCoreParams() const;
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(
	FBPFromLZProcessStarted,
	const FString&, PressDir,
	const FString&, ActionPressDir);

DECLARE_DYNAMIC_MULTICAST_DELEGATE_FourParams(
	FBPFromLZProcessCompleted,
	const TArray<FString>&, OutputFiles,
	const FString&, PressDir,
	const FString&, ActionPressDir,
	const FString&, MainCompositePath);

DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(
	FBPFromLZProcessFailed,
	const FString&, ErrorMessage,
	const TArray<FString>&, PartialOutputFiles);

UCLASS()
class URERSPATIALISLT_API UBP_FromLZProcessSketchUtils : public UBlueprintAsyncActionBase {
	GENERATED_BODY()

public:
	UPROPERTY(BlueprintAssignable)
	FBPFromLZProcessStarted OnStarted;

	UPROPERTY(BlueprintAssignable)
	FBPFromLZProcessCompleted OnCompleted;

	UPROPERTY(BlueprintAssignable)
	FBPFromLZProcessFailed OnFailed;

	UFUNCTION(BlueprintCallable, meta = (BlueprintInternalUseOnly = "true", WorldContext = "WorldContextObject", DisplayName = "BP FromLZ Process Latest Sketch From Capture", Category = "FromLZ|Process"))
	static UBP_FromLZProcessSketchUtils* ProcessLatestSketchFromCapture(
		UObject* WorldContextObject,
		const TArray<FString>& CaptureOutputFiles,
		const FString& SketchFolderPath,
		FBPFromLZProcessParams ProcessParams);

	virtual void Activate() override;

private:
	UPROPERTY()
	TObjectPtr<UObject> WorldContextObject = nullptr;

	UPROPERTY()
	TArray<FString> CaptureOutputFiles;

	UPROPERTY()
	FString SketchFolderPath;

	UPROPERTY()
	FBPFromLZProcessParams ProcessParams;

	UPROPERTY()
	FString MainCompositePath;

	UPROPERTY()
	FString ProcessDir;

	UPROPERTY()
	FString CompatibilityRedPath;

	UPROPERTY()
	FString CompatibilityGreenPath;

	UPROPERTY()
	FString CompatibilityBluePath;
};
