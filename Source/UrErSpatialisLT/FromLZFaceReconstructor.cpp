#include "FromLZFaceReconstructor.h"
#include "FromLZManifoldBoolean.h"
#include "FromLZProcessingLimits.h"
#include "FromLZSessionReset.h"

#include "Algo/Reverse.h"
#include "Async/Async.h"
#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "Components/PrimitiveComponent.h"
#include "Components/StaticMeshComponent.h"
#include "DynamicMesh/DynamicMesh3.h"
#include "Engine/Engine.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "HAL/FileManager.h"
#include "HAL/IConsoleManager.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Materials/Material.h"
#include "Materials/MaterialInterface.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Modules/ModuleManager.h"
#include "ProceduralMeshComponent.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "StaticMeshResources.h"

namespace
{
	const FName ReconstructedFaceTag(TEXT("FromLZ_ReconstructedFace"));
	const FName ReconstructedSolidTag(TEXT("FromLZ_ReconstructedSolid"));
	const FName FromLZCaptureSubjectTag(TEXT("FromLZCaptureSubject"));
	const FName FromLZCapturePlaneTag(TEXT("FromLZCapturePlane"));
	const FName Step11BooleanResultTag(TEXT("FromLZ_Step11BooleanResult"));
	const FName Step11HiddenSourceTag(TEXT("FromLZ_Step11HiddenSource"));
	const FName Step11ActionAttachTag(TEXT("FromLZ_Action_Attach"));
	const FName Step11ActionExcavateCutterTag(TEXT("FromLZ_Action_ExcavateCutter"));
	constexpr double NormalParallelThresholdDegrees = FFromLZFaceReconstructor::CandidateFaceMaxNormalSideAngleDegrees;
	constexpr double PreferredNormalAngleThresholdDegrees = FFromLZFaceReconstructor::CandidateFacePreferredNormalSideAngleDegrees;
	constexpr double AttachDirectedRayStepPx = 1.0;
	constexpr double AttachDirectedRayMinDistancePx = 1.0;
	constexpr int32 AttachDirectedRayHitRadiusPx = 2;
	constexpr double AttachDirectedRayAmbiguousDistancePx = 1.5;
	constexpr double MinProjectedNormalPixels = 1.0;
	constexpr double SolidCollinearTolerancePixels = 0.75;
	constexpr double SolidRdpTolerancePixels = 1.25;
	constexpr int32 SolidTargetMaxLoopPoints = 384;
	constexpr int32 MinSolidDepthSamples = 3;
	constexpr double CapBBoxRegularizationFillRatio = 0.70;
	constexpr double CapBBoxDegenerateAreaTolerance = 1e-6;
	constexpr double FaceBBoxSquareRelativeTolerance = 0.01;
	constexpr int32 CapBBoxDebugImageSize = 1024;

	// Per-face capture pipeline (Source A removal): when enabled, the face boundary
	// is reconstructed from the shared-camera contour by detecting image-clip corners
	// (those whose 2D pixel position lies on the captured image rectangle) and
	// snapping them onto the axis-aligned bbox in face UV. This eliminates the tilt
	// caused by the shared camera's image rectangle intersecting the face plane at
	// an oblique angle. Source B (cap polygon tilt from sketch via shared camera)
	// is intentionally NOT addressed here; it remains handled by pure_red_root_alignment.
	static int32 GFromLZUsePerFaceCapture = 1;
	static FAutoConsoleVariableRef CVarFromLZUsePerFaceCapture(
		TEXT("r.FromLZ.UsePerFaceCapture"),
		GFromLZUsePerFaceCapture,
		TEXT("If non-zero (default 1), derive face boundary by detecting image-clip corners in the shared-camera contour and snapping them to the axis-aligned bbox in face UV (eliminates Source A tilt). Set to 0 to use the legacy raw contour."),
		ECVF_Default);
	static constexpr double GFromLZPerFaceClipMarginPixels = 1.5;

	// Pure-red root selection: when 0 (default), the pure_red_root_alignment stage
	// only accepts root candidates targeting the U/V axes (0deg / 90deg) and skips
	// diagonal targets (Octi_DiagPlus = 45deg, Octi_DiagMinus = 135deg). Set to
	// non-zero to allow diagonal red roots (legacy behavior) so that a dominant
	// near-45deg / near-135deg red stroke can rotate the entire cap onto a
	// diagonal frame.
	static int32 GFromLZPureRedAllowDiagonalRoot = 0;
	static FAutoConsoleVariableRef CVarFromLZPureRedAllowDiagonalRoot(
		TEXT("r.FromLZ.PureRedAllowDiagonalRoot"),
		GFromLZPureRedAllowDiagonalRoot,
		TEXT("If 0 (default), pure-red root candidates targeting Octi_DiagPlus (45deg) or Octi_DiagMinus (135deg) are skipped, restricting pure-red root alignment to U/V (0deg/90deg). Set to non-zero to restore legacy behavior allowing diagonal roots."),
		ECVF_Default);

	constexpr double WorldOrthoBlackAxisToleranceDegrees = 5.0;
	constexpr double WorldOrthoDiagThresholdDegrees = 40.0;
	constexpr double WorldOrthoAngleComparisonEpsilonDegrees = 1e-6;
	constexpr double WorldOrthoBlackNodeSnapTolerancePixels = 10.0;
	constexpr double WorldOrthoRedMacroCorridorPixels = 5.0;
	constexpr double WorldOrthoRedMacroGroupMinLengthPixels = 20.0;
	constexpr double WorldOrthoRedPrimitiveRdpTolerancePixels = 1.0;
	constexpr double WorldOrthoShortRedEdgeLengthPixels = 20.0;
	constexpr double WorldOrthoMinAreaRatio = 0.4;
	constexpr double WorldOrthoMaxWrapGapFraction = 0.1;
	constexpr int32 WorldOrthoPureRedMaxRootCandidates = 5;
	constexpr double ExcavationCutterNormalScale = 1.2;
	constexpr double ExcavationCutterCapScale = 1.1;
	constexpr double Step11BooleanMinRenderableEdgeCm = 0.05;
	constexpr double Step11BooleanBoundsExpandCm = 1.0;
	const FColor ReconstructedDebugBlue(0, 120, 255, 255);
	static FString GActiveUndoPressId;
	static TArray<FString> GActiveStep11PressStack;

	struct FFaceInfo
	{
		int32 Id = -1;
		FColor Color = FColor::Black;
		FVector PlanePoint = FVector::ZeroVector;
		FVector Normal = FVector::UpVector;
		TArray<FVector2D> KeyPoints2D;
		TArray<FVector> KeyPoints3D;
	};

	struct FActorMaterialIdEntry
	{
		uint32 ColorKey = 0;
		FString ActorName;
		FString ActorPath;
		FString ComponentName;
		FString ComponentPath;
		FString ComponentType;
		int32 MaterialSlot = -1;
		FString MaterialName;
		FString MaterialPath;
	};

	struct FAttachMaterialIdSelection
	{
		bool bLookupAttempted = false;
		bool bFound = false;
		FString Error;
		FActorMaterialIdEntry Entry;
		int32 PixelCount = 0;
		int32 ConsideredPixelCount = 0;
		double Coverage = 0.0;
	};

	struct FOrthographicProjectionMetadata
	{
		bool bAvailable = false;
		FString Source;
		double M00 = 0.0;
		double M11 = 0.0;
		double M30 = 0.0;
		double M31 = 0.0;
	};

	struct FCameraInfo
	{
		FVector Location = FVector::ZeroVector;
		FVector Forward = FVector::ForwardVector;
		FVector Right = FVector::RightVector;
		FVector Up = FVector::UpVector;
		double Fov = 90.0;
		double OrthoWidth = 1536.0;
		bool bOrthographic = false;
		bool bHasProjectionMatrix = false;
		FMatrix ProjectionMatrix = FMatrix::Identity;
		FString ProjectionMatrixSource;
		int32 ViewportWidth = 0;
		int32 ViewportHeight = 0;
	};

	struct FOverlapAccum
	{
		int32 Pixels = 0;
		double SumX = 0.0;
		double SumY = 0.0;
	};

	struct FFaceCandidate
	{
		int32 FaceId = -1;
		int32 OverlapPixels = 0;
		double OverlapRatio = 0.0;
		FVector2D MaskCentroid = FVector2D::ZeroVector;
		bool bHasPlaneHit = false;
		FVector PlaneHit = FVector::ZeroVector;
		double DistanceToCamera = 0.0;
		bool bHasProjectedNormal = false;
		FVector2D ProjectedNormal2D = FVector2D::ZeroVector;
		double NormalGreenAngleDegrees = -1.0;
		bool bNormalParallelPass = false;
		bool bUsedDirectedAttachNormalCheck = false;
		bool bAttachNormalOrientationValid = false;
		int32 AttachDirectedGreenChainIndex = -1;
		FVector2D AttachGreenStart2D = FVector2D::ZeroVector;
		FVector2D AttachGreenEnd2D = FVector2D::ZeroVector;
		FVector2D AttachGreenDir2D = FVector2D::ZeroVector;
		FVector2D AttachExpectedNormalDir2D = FVector2D::ZeroVector;
		FVector2D RawProjectedNormal2D = FVector2D::ZeroVector;
		FVector2D OrientedProjectedNormal2D = FVector2D::ZeroVector;
		bool bAttachPlusRayHit = false;
		bool bAttachMinusRayHit = false;
		FVector2D AttachPlusRayHit2D = FVector2D::ZeroVector;
		FVector2D AttachMinusRayHit2D = FVector2D::ZeroVector;
		double AttachPlusRayDistancePx = -1.0;
		double AttachMinusRayDistancePx = -1.0;
		double UnorientedNormalGreenAngleDegrees = -1.0;
		FString AttachNormalOrientationReason;
	};

	struct FDirectedGreenChain2D
	{
		int32 ChainIndex = -1;
		FVector2D Start = FVector2D::ZeroVector;
		FVector2D End = FVector2D::ZeroVector;
		FVector2D Dir = FVector2D::ZeroVector;
		double Length = 0.0;
		double PathLength = 0.0;
	};

	struct FSupportFaceVoteResult
	{
		bool bFound = false;
		int32 FaceId = -1;
		int32 VotePixels = 0;
		int32 ConsideredPixels = 0;
		int32 HitSampleCount = 0;
		int32 TotalSampleCount = 0;
		double Coverage = 0.0;
		double VotePixelCoverage = 0.0;
		double WorldZMax = 0.0;
		double WorldZAverage = 0.0;
		double MinCameraDistance = 0.0;
		double WorstPolygonDistancePx = 0.0;
		FString Error;
		TMap<int32, int32> VotesByFaceId;
		TArray<FFromLZSupportFaceVoteCandidate> FaceCandidates;
	};

	struct FReconstructedMesh
	{
		FString ActorName;
		FName Tag;
		TArray<FVector> VerticesWorld;
		TArray<int32> Triangles;
		FVector Normal = FVector::UpVector;
		FColor Color = ReconstructedDebugBlue;
		bool bIsExcavateCutter = false;
		FString PressId;
		int32 SourceFaceId = -1;
		FVector SourcePlanePoint = FVector::ZeroVector;
		FVector SourcePlaneNormal = FVector::UpVector;
		TArray<FVector> SourceFaceVerticesWorld;
		TArray<FVector> SourceMaterialProbePointsWorld;
		FAttachMaterialIdSelection AttachMaterialId;
	};

	struct FSolidDepthSample
	{
		int32 Index = -1;
		bool bValid = false;
		FString Error;
		FVector2D SourcePixel = FVector2D::ZeroVector;
		FVector2D CopiedPixel = FVector2D::ZeroVector;
		FVector SourceWorld = FVector::ZeroVector;
		FVector PointOnExtrusion = FVector::ZeroVector;
		FVector PointOnRay = FVector::ZeroVector;
		double Depth = 0.0;
		double ClosestWorldDistance = 0.0;
		double ReprojectionErrorPixels = 0.0;
	};

	struct FWorldOrthogonalStageDebug
	{
		FString Name;
		FString Status = TEXT("not_reached");
		FString Message;
	};

	struct FWorldOrthogonalRunDebug
	{
		int32 RunIndex = INDEX_NONE;
		int32 StrokeId = INDEX_NONE;
		FString Type;
		bool bIgnoredConnector = false;
		bool bUnprojected = false;
		int32 StartNodeId = INDEX_NONE;
		int32 EndNodeId = INDEX_NONE;
		FVector2D StartNodeCapSpace = FVector2D::ZeroVector;
		FVector2D EndNodeCapSpace = FVector2D::ZeroVector;
		FVector2D StartNodeUV = FVector2D::ZeroVector;
		FVector2D EndNodeUV = FVector2D::ZeroVector;
		TArray<FVector2D> SourcePointsCapSpace;
		TArray<FVector2D> FacePixels;
		TArray<FVector2D> SamplesUV;
		TArray<int32> ProtectedPointIndices;
		TArray<FVector2D> ProtectedPointsUV;
		FString RedProtectionMode;
		double RedMaxChordDeviationPixels = 0.0;
		TArray<FString> RedDirectionalGroupAxisTypes;
		TArray<double> RedDirectionalGroupLengthsPixels;
		TArray<int32> RedDirectionalGroupStartIndices;
		TArray<int32> RedDirectionalGroupEndIndices;
	};

	struct FWorldOrthogonalEdgeDebug
	{
		int32 EdgeIndex = INDEX_NONE;
		FString Type;
		FString AxisTypeName;
		TArray<int32> SourceRunIndices;
		TArray<int32> SourceStrokeIds;
		TArray<int32> PrimitiveEdgeIndices;
		TArray<int32> GraphNodeIds;
		TArray<FVector2D> GraphNodeCapSpace;
		TArray<FVector2D> GraphNodeUV;
		TArray<FVector2D> SnappedGraphNodeUV;
		TArray<double> GraphNodeSnapDistancesPixels;
		TArray<FVector2D> SamplesUV;
		FVector2D StartUV = FVector2D::ZeroVector;
		FVector2D EndUV = FVector2D::ZeroVector;
		FVector2D DirectionUV = FVector2D::ZeroVector;
		FVector2D SolvedStartUV = FVector2D::ZeroVector;
		FVector2D SolvedEndUV = FVector2D::ZeroVector;
		int32 AxisType = 0; // EOctilinearAxis value for debug JSON
		double AngleToU = 180.0;
		double AngleToV = 180.0;
		double AngleToDiagPlus = 180.0;
		double AngleToDiagMinus = 180.0;
		double AngleToAssignedAxis = 0.0;
		double SupportCoordinate = 0.0;
		double PathLength = 0.0;
	};

	struct FWorldOrthogonalRootHypothesisDebug
	{
		int32 HypothesisIndex = INDEX_NONE;
		int32 RootPrimitiveEdgeIndex = INDEX_NONE;
		TArray<int32> RootStrokeIds;
		FString TargetAxisTypeName;
		FVector2D RootStartUV = FVector2D::ZeroVector;
		FVector2D RootEndUV = FVector2D::ZeroVector;
		FVector2D AlignedRootStartUV = FVector2D::ZeroVector;
		FVector2D AlignedRootEndUV = FVector2D::ZeroVector;
		double RootLengthPixels = 0.0;
		double RootMaxChordDeviationPixels = 0.0;
		double Reliability = 0.0;
		double RotationDegrees = 0.0;
		bool bValid = false;
		bool bSelected = false;
		FString RejectionReason;
		int32 GeometryEdgeCount = 0;
		double AngleCost = 0.0;
		double AreaRatio = 0.0;
		double MeanBoundaryDistance = 0.0;
		double MaxBoundaryDistance = 0.0;
	};

	struct FCapBBoxRegularizationResult
	{
		bool bAttempted = false;
		bool bApplied = false;
		bool bFallbackToOriginal = false;
		int32 SelectedFaceId = -1;
		FString Action;
		FString SourcePolygonKey;
		FString CopiedPolygonKey;
		FString SelectedGeometry = TEXT("original");
		FString RejectionReason;
		double FillRatioThreshold = CapBBoxRegularizationFillRatio;
		double CapArea = 0.0;
		double CapBaseBBoxArea = 0.0;
		double CapBaseBBoxRatio = 0.0;
		bool bCapBaseBBoxPassed = false;
		bool bCapMinimumBBoxComputed = false;
		double CapMinimumBBoxArea = 0.0;
		double CapMinimumBBoxRatio = 0.0;
		bool bCapMinimumBBoxPassed = false;
		bool bWorldOrthogonal = false;
		bool bWorldOrthoUsePerFaceCapture = true;
		double WorldOrthoPerFaceClipMarginPixels = 1.5;
		bool bWorldOrthoPureRedAllowDiagonalRoot = false;
		bool bWorldOrthoAllowDiagonalSupports = false;
		double WorldOrthoBlackAxisToleranceDegreesUsed = WorldOrthoBlackAxisToleranceDegrees;
		double WorldOrthoDiagThresholdDegreesUsed = WorldOrthoDiagThresholdDegrees;
		double WorldOrthoAngleComparisonEpsilonDegreesUsed = WorldOrthoAngleComparisonEpsilonDegrees;
		double WorldOrthoBlackNodeSnapTolerancePixelsUsed = WorldOrthoBlackNodeSnapTolerancePixels;
		double WorldOrthoRedMacroCorridorPixelsUsed = WorldOrthoRedMacroCorridorPixels;
		double WorldOrthoRedMacroGroupMinLengthPixelsUsed = WorldOrthoRedMacroGroupMinLengthPixels;
		double WorldOrthoRedPrimitiveRdpTolerancePixelsUsed = WorldOrthoRedPrimitiveRdpTolerancePixels;
		double WorldOrthoShortRedEdgeLengthPixelsUsed = WorldOrthoShortRedEdgeLengthPixels;
		double WorldOrthoMinAreaRatioUsed = WorldOrthoMinAreaRatio;
		double WorldOrthoMaxWrapGapFractionUsed = WorldOrthoMaxWrapGapFraction;
		bool bWorldOrthoAllowTopologyRepair = false;
		bool bTopologyRepairAttempted = false;
		bool bTopologyRepairApplied = false;
		int32 TopologyRepairInsertedCount = 0;
		double TopologyRepairMaxGapPixels = 0.0;
		FString TopologyRepairInsertedAxisTypeNames;
		int32 WorldOrthoPureRedMaxRootCandidatesUsed = WorldOrthoPureRedMaxRootCandidates;
		bool bContainsBlack = false;
		int32 BoundaryRunCount = 0;
		int32 PrimitiveGeometryEdgeCount = 0;
		int32 GeometryEdgeCount = 0;
		int32 IgnoredConnectorCount = 0;
		double WorldOrthogonalAngleCost = 0.0;
		double WorldOrthogonalAreaRatio = 0.0;
		double WorldOrthogonalMeanBoundaryDistance = 0.0;
		double WorldOrthogonalMaxBoundaryDistance = 0.0;
		double WorldOrthogonalCapDiagonal = 0.0;
		bool bPureRedRootAlignmentAttempted = false;
		int32 SelectedRootHypothesisIndex = INDEX_NONE;
		int32 SelectedRootPrimitiveEdgeIndex = INDEX_NONE;
		TArray<int32> SelectedRootStrokeIds;
		FString SelectedRootTargetAxisTypeName;
		double SelectedRootRotationDegrees = 0.0;
		FVector2D RootRotationCenterUV = FVector2D::ZeroVector;
		FVector2D SelectedRootStartUV = FVector2D::ZeroVector;
		FVector2D SelectedRootEndUV = FVector2D::ZeroVector;
		FVector2D SelectedAlignedRootStartUV = FVector2D::ZeroVector;
		FVector2D SelectedAlignedRootEndUV = FVector2D::ZeroVector;
		FVector FaceOriginWorld = FVector::ZeroVector;
		FVector FaceNormalWorld = FVector::UpVector;
		FVector FaceAxisUWorld = FVector::RightVector;
		FVector FaceAxisVWorld = FVector::ForwardVector;
		FVector2D OriginalSourceToCopiedDeltaPixels = FVector2D::ZeroVector;
		FString WorldOrthogonalActiveStage;
		TArray<FWorldOrthogonalStageDebug> WorldOrthogonalStages;
		TArray<FWorldOrthogonalRunDebug> WorldOrthogonalRuns;
		TArray<FWorldOrthogonalEdgeDebug> WorldOrthogonalPrimitiveEdges;
		TArray<FWorldOrthogonalEdgeDebug> WorldOrthogonalMergedEdges;
		TArray<FWorldOrthogonalRootHypothesisDebug> WorldOrthogonalRootHypotheses;
		TArray<FVector2D> WorldOrthogonalSolvedVerticesUV;
		TArray<FVector2D> WorldOrthogonalPhaseAlignedCapUV;
		TArray<FVector2D> FaceBoundaryUV;
		FString FaceBoundarySource = TEXT("shared_camera_legacy");
		TArray<FVector2D> FaceHullUV;
		TArray<FVector2D> FaceBBoxUV;
		TArray<FVector2D> OriginalCapUV;
		TArray<FVector2D> OriginalCapHullUV;
		TArray<FVector2D> CapBaseBBoxUV;
		TArray<FVector2D> CapMinimumBBoxUV;
		TArray<FVector2D> FinalCapUV;
		TArray<FVector> OriginalCapWorld;
		TArray<FVector> CapBaseBBoxWorld;
		TArray<FVector> CapMinimumBBoxWorld;
		TArray<FVector> FinalCapWorld;
	};

	enum class ECapBoundaryRunType : uint8
	{
		Red,
		Black,
		Connector
	};

	struct FCapBoundaryRunInput
	{
		int32 StrokeId = INDEX_NONE;
		ECapBoundaryRunType Type = ECapBoundaryRunType::Red;
		bool bSynthetic = false;
		bool bReversed = false;
		int32 StartNodeId = INDEX_NONE;
		int32 EndNodeId = INDEX_NONE;
		FVector2D StartNodePosition = FVector2D::ZeroVector;
		FVector2D EndNodePosition = FVector2D::ZeroVector;
		TArray<FVector2D> Points;
	};

	struct FSolidReconstructionResult
	{
		FString ComponentName;
		FString Action;
		FString SourcePolygonKey;
		FString CopiedPolygonKey;
		FString Error;
		FString Warning;
		FString ActorName;
		FString ReconstructionMethod = TEXT("direct_cap_face");
		bool bSuccess = false;
		bool bAttachSupportPlaneFallback = false;
		bool bForcedSupportPlaneOutput = false;
		FString ForcedSupportPlaneReason;
		int32 ForcedSupportPlaneAttemptIndex = INDEX_NONE;
		int32 CapWidth = 0;
		int32 CapHeight = 0;
		int32 FacesWidth = 0;
		int32 FacesHeight = 0;
		int32 SelectedFaceId = -1;
		int32 SupportFaceId = -1;
		int32 SupportGreenChainIndex = -1;
		FVector SourcePlanePoint = FVector::ZeroVector;
		FVector SourcePlaneNormal = FVector::UpVector;
		FVector SupportPlanePoint = FVector::ZeroVector;
		FVector SupportPlaneNormal = FVector::UpVector;
		FVector ExtrusionVectorWorld = FVector::ZeroVector;
		TArray<FVector> SourceFaceVerticesWorld;
		TArray<FVector> SourceMaterialProbePointsWorld;
		FAttachMaterialIdSelection AttachMaterialId;
		FVector OrientedNormal = FVector::UpVector;
		FVector2D SourceToCopiedVector2D = FVector2D::ZeroVector;
		FVector2D ProjectedNormal2D = FVector2D::ZeroVector;
		double ExtrusionDepth = 0.0;
		double MaxDepthSampleReprojectionErrorPixels = 0.0;
		double MeanSourceReprojectionErrorPixels = 0.0;
		double MaxSourceReprojectionErrorPixels = 0.0;
		double MeanCopiedReprojectionErrorPixels = 0.0;
		double MaxCopiedReprojectionErrorPixels = 0.0;
		FString ProjectionMatrixSource;
		FString ProjectionValidation;
		bool bHasProjectionMatrix = false;
		int32 ProjectionViewportWidth = 0;
		int32 ProjectionViewportHeight = 0;
		double ProjectionM00 = 0.0;
		double ProjectionM11 = 0.0;
		double ProjectionM30 = 0.0;
		double ProjectionM31 = 0.0;
		double ProjectionHorizontalSpan = 0.0;
		double ProjectionVerticalSpan = 0.0;
		TArray<FSolidDepthSample> DepthSamples;
		TArray<FVector2D> SourceLoop2D;
		TArray<FVector2D> CopiedTargetLoop2D;
		TArray<FVector2D> ReprojectedSourceLoop2D;
		TArray<FVector2D> ReprojectedCopiedLoop2D;
		TArray<FVector> SourceLoopWorld;
		TArray<FVector> CopiedLoopWorld;
		TArray<FVector> MeshVerticesWorld;
		TArray<int32> MeshTriangles;
		FVector MeshNormal = FVector::UpVector;
		FCapBBoxRegularizationResult CapBBoxRegularization;
	};

	struct FComponentResult
	{
		FString ComponentName;
		FString Action;
		FString PolygonKey;
		FString Error;
		FString ActorName;
		FString AttachPathMode;
		FString ChosenAttachPath;
		FString AttachPathSelectionReason;
		int32 CapWidth = 0;
		int32 CapHeight = 0;
		int32 FacesWidth = 0;
		int32 FacesHeight = 0;
		int32 CapMaskPixels = 0;
		int32 MinOverlapPixels = 0;
		int32 SelectedFaceId = -1;
		double CandidateFaceMinOverlapRatioUsed = FFromLZFaceReconstructor::CandidateFaceMinOverlapRatio;
		double NormalParallelThresholdDegreesUsed = NormalParallelThresholdDegrees;
		double PreferredNormalAngleThresholdDegreesUsed = PreferredNormalAngleThresholdDegrees;
		FVector2D GreenLineVector2D = FVector2D::ZeroVector;
		// Every cap-connected green line, mapped to faces image space. The candidate-face
		// normal passes the parallel filter if it is parallel to ANY of these.
		TArray<FVector2D> GreenLineVectors2D;    // normalized direction per green
		TArray<FVector2D> GreenSegStarts;        // chord start in faces space (debug)
		TArray<FVector2D> GreenSegEnds;          // chord end in faces space (debug)
		bool bSuccess = false;
		TArray<FFaceCandidate> Candidates;
		FVector SelectedPlaneHit = FVector::ZeroVector;
		TArray<FVector> MeshVerticesWorld;
		TArray<int32> MeshTriangles;
		FVector MeshNormal = FVector::UpVector;
		FSolidReconstructionResult Solid;
	};

	struct FSupportPlaneTopologyDebug
	{
		bool bChecked = false;
		bool bPass = false;
		FString Stage;
		FString Reason;
		int32 SourceVertexCount = 0;
		int32 WorldVertexCount = 0;
		double SourceAreaPx2 = 0.0;
		double ProjectedAreaCm2 = 0.0;
		double MinSourceEdgePx = 0.0;
		double MinProjectedEdgeCm = 0.0;
		TArray<FVector2D> ProjectedUVLoop;
		TArray<FIntPoint> SourceSelfIntersectionEdges;
		TArray<FIntPoint> ProjectedSelfIntersectionEdges;
	};

	struct FAttachSupportPlaneFallbackAttempt
	{
		int32 ChainIndex = -1;
		int32 SeedStrokeId = -1;
		int32 SupportFaceId = -1;
		bool bSuccess = false;
		bool bForceable = false;
		FString ForceableReason;
		FString RejectReason;
		FVector2D ChainStartFaceSpace = FVector2D::ZeroVector;
		FVector2D ChainEndFaceSpace = FVector2D::ZeroVector;
		double ChainPathLength = 0.0;
		double ChainChordLength = 0.0;
		double SupportVoteCoverage = 0.0;
		double SupportVotePixelCoverage = 0.0;
		int32 SupportVotePixels = 0;
		int32 SupportConsideredPixels = 0;
		int32 SupportHitSampleCount = 0;
		int32 SupportTotalSampleCount = 0;
		double SupportFaceWorldZMax = 0.0;
		double SupportFaceWorldZAverage = 0.0;
		double SupportMinCameraDistance = 0.0;
		double SupportWorstPolygonDistancePx = 0.0;
		TArray<FFromLZSupportFaceVoteCandidate> SupportFaceCandidates;
		FVector AnchorWorld = FVector::ZeroVector;
		FVector ChainEndWorld = FVector::ZeroVector;
		FVector ExtrusionVectorWorld = FVector::ZeroVector;
		FVector BasePlaneNormal = FVector::UpVector;
		FVector SupportPlaneNormal = FVector::UpVector;
		FVector SupportAwareAxisU = FVector::RightVector;
		FVector SupportAwareAxisV = FVector::UpVector;
		bool bHasBaseProjection = false;
		TArray<FVector2D> BaseLoop2D;
		TArray<FVector2D> CopiedTargetLoop2D;
		TArray<FVector2D> ReprojectedBaseLoop2D;
		TArray<FVector2D> ReprojectedCopiedLoop2D;
		TArray<FVector2D> VirtualBasePlaneQuad2D;
		TArray<FVector> BaseLoopWorld;
		TArray<FVector> CopiedLoopWorld;
		TArray<FVector> VirtualBasePlaneQuadWorld;
		FSupportPlaneTopologyDebug RawProjectionTopology;
		FSupportPlaneTopologyDebug RegularizedProjectionTopology;
		FSupportPlaneTopologyDebug FinalProjectionTopology;
		TArray<FVector2D> NoPenetrationSamplePixels;
		TArray<FVector> NoPenetrationBaseWorld;
		TArray<FVector> NoPenetrationSupportWorld;
		TArray<double> NoPenetrationViolationCm;
		bool bOrthogonalAttempted = false;
		bool bOrthogonalApplied = false;
		bool bOrthogonalFallbackToOriginal = false;
		double ContactDistancePixels = 0.0;
		bool bContactPass = false;
		double MaxNoPenetrationViolationCm = 0.0;
		bool bNoPenetrationPass = false;
		double MaxReprojectionErrorPixels = 0.0;
		bool bReprojectionPass = false;
	};

	struct FAttachSupportPlaneFallbackDebug
	{
		bool bAttempted = false;
		bool bSuccess = false;
		bool bEnabled = false;
		bool bForcedOutput = false;
		double SupportFaceVoteMinCoverage = 0.20;
		double SupportFaceVoteSampleStepPx = 5.0;
		FString Error;
		int32 BestForceableAttemptIndex = INDEX_NONE;
		FString BestForceableReason;
		int32 ForcedAttemptIndex = INDEX_NONE;
		FString ForcedReason;
		TArray<FAttachSupportPlaneFallbackAttempt> Attempts;
	};

	struct FCommonInputs
	{
		FString CaptureJsonRel;
		FString FacesPngRel;
		FString FacesJsonRel;
		FString ActorMaterialPngRel;
		FString ActorMaterialJsonRel;
		FString CaptureJsonPath;
		FString FacesPngPath;
		FString FacesJsonPath;
		FString ActorMaterialPngPath;
		FString ActorMaterialJsonPath;
		FCameraInfo Camera;
		TArray<FFaceInfo> Faces;
		TMap<int32, int32> FaceIndexById;
		TMap<uint32, int32> FaceIdByColorKey;
		TArray<uint8> FacesRGBA;
		int32 FacesWidth = 0;
		int32 FacesHeight = 0;
		TArray<uint8> ActorMaterialRGBA;
		int32 ActorMaterialWidth = 0;
		int32 ActorMaterialHeight = 0;
		TMap<uint32, FActorMaterialIdEntry> ActorMaterialEntryByColorKey;
		FOrthographicProjectionMetadata FacesProjection;
		FOrthographicProjectionMetadata ActorMaterialProjection;
		FString ProjectionValidation;
	};

	struct FStep11MeshDiagnostics
	{
		FString Label;
		FString SourceType;
		int32 VertexCount = 0;
		int32 TriangleCount = 0;
		int32 EdgeCount = 0;
		int32 BoundaryEdgeCount = 0;
		int32 NonManifoldEdgeCount = 0;
		int32 InconsistentOrientationEdgeCount = 0;
		int32 InvalidTriangleCount = 0;
		int32 DegenerateTriangleCount = 0;
		int32 TinyTriangleCount = 0;
		int32 DuplicateTriangleCount = 0;
		double SurfaceArea = 0.0;
		double MinTriangleArea = 0.0;
		double MaxTriangleArea = 0.0;
		double SignedVolume = 0.0;
		double AbsVolume = 0.0;
		double SignedVolumeBeforeOrientationFix = 0.0;
		double SignedVolumeAfterOrientationFix = 0.0;
		double MinEdgeLength = 0.0;
		double MaxEdgeLength = 0.0;
		double MeanEdgeLength = 0.0;
		FBox Bounds = FBox(ForceInit);
		bool bOrientationReversedForBoolean = false;
	};

	static uint32 ColorKey(uint8 R, uint8 G, uint8 B)
	{
		return (uint32(R) << 16) | (uint32(G) << 8) | uint32(B);
	}

	static FString ResolveSavedPath(const FString& RelativeOrAbsolute)
	{
		if (RelativeOrAbsolute.IsEmpty())
		{
			return FString();
		}
		if (FPaths::IsRelative(RelativeOrAbsolute))
		{
			return FPaths::ProjectSavedDir() / RelativeOrAbsolute;
		}
		return RelativeOrAbsolute;
	}

	static FString StripTrailingFacesSuffixes(FString Stem)
	{
		while (Stem.EndsWith(TEXT("_faces")))
		{
			Stem.LeftChopInline(6);
		}
		return Stem;
	}

	static FString StripTrailingCaptureAuxSuffixes(FString Stem)
	{
		bool bChanged = true;
		while (bChanged)
		{
			bChanged = false;
			if (Stem.EndsWith(TEXT("_faces")))
			{
				Stem.LeftChopInline(6);
				bChanged = true;
			}
			if (Stem.EndsWith(TEXT("_actor_material_id")))
			{
				Stem.LeftChopInline(18);
				bChanged = true;
			}
		}
		return Stem;
	}

	static FString BuildCaptureRelPath(const FString& CaptureStem, const FString& Extension)
	{
		if (CaptureStem.IsEmpty())
		{
			return FString();
		}
		return TEXT("FromLZCaptures/") + StripTrailingCaptureAuxSuffixes(CaptureStem) + Extension;
	}

	static FString BuildFacesRelPath(const FString& CaptureStem, const FString& Extension)
	{
		if (CaptureStem.IsEmpty())
		{
			return FString();
		}
		return TEXT("FromLZCaptures/") + StripTrailingCaptureAuxSuffixes(CaptureStem) + TEXT("_faces") + Extension;
	}

	static FString BuildActorMaterialRelPath(const FString& CaptureStem, const FString& Extension)
	{
		if (CaptureStem.IsEmpty())
		{
			return FString();
		}
		return TEXT("FromLZCaptures/") + StripTrailingCaptureAuxSuffixes(CaptureStem) + TEXT("_actor_material_id") + Extension;
	}

	static void NormalizeFacesRelPath(FString& RelPath, const FString& Extension)
	{
		if (RelPath.IsEmpty())
		{
			return;
		}

		const FString Dir = FPaths::GetPath(RelPath);
		const FString Stem = StripTrailingCaptureAuxSuffixes(FPaths::GetBaseFilename(RelPath));
		const FString Filename = Stem + TEXT("_faces") + Extension;
		RelPath = Dir.IsEmpty() ? Filename : Dir / Filename;
	}

	static void NormalizeActorMaterialRelPath(FString& RelPath, const FString& Extension)
	{
		if (RelPath.IsEmpty())
		{
			return;
		}

		const FString Dir = FPaths::GetPath(RelPath);
		const FString Stem = StripTrailingCaptureAuxSuffixes(FPaths::GetBaseFilename(RelPath));
		const FString Filename = Stem + TEXT("_actor_material_id") + Extension;
		RelPath = Dir.IsEmpty() ? Filename : Dir / Filename;
	}

	static void NormalizeCaptureRelPath(FString& RelPath, const FString& Extension)
	{
		if (RelPath.IsEmpty())
		{
			return;
		}

		const FString Dir = FPaths::GetPath(RelPath);
		const FString Stem = StripTrailingCaptureAuxSuffixes(FPaths::GetBaseFilename(RelPath));
		const FString Filename = Stem + Extension;
		RelPath = Dir.IsEmpty() ? Filename : Dir / Filename;
	}

	static bool LoadJsonObject(const FString& Path, TSharedPtr<FJsonObject>& OutObject)
	{
		FString Text;
		if (!FFileHelper::LoadFileToString(Text, *Path))
		{
			return false;
		}

		const TSharedRef<TJsonReader<TCHAR>> Reader = TJsonReaderFactory<TCHAR>::Create(Text);
		return FJsonSerializer::Deserialize(Reader, OutObject) && OutObject.IsValid();
	}

	static bool SaveJsonObject(const TSharedRef<FJsonObject>& Object, const FString& Path)
	{
		FString Text;
		const TSharedRef<TJsonWriter<TCHAR, TPrettyJsonPrintPolicy<TCHAR>>> Writer =
			TJsonWriterFactory<TCHAR, TPrettyJsonPrintPolicy<TCHAR>>::Create(&Text);
		if (!FJsonSerializer::Serialize(Object, Writer))
		{
			return false;
		}
		return FFileHelper::SaveStringToFile(Text, *Path, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
	}

	static bool DecodePngToRGBA(const FString& Path, TArray<uint8>& OutPixels, int32& OutWidth, int32& OutHeight)
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
		return OutWidth > 0 && OutHeight > 0 && OutPixels.Num() >= OutWidth * OutHeight * 4;
	}

	static bool SaveRGBAToPng(const TArray<uint8>& RGBA, int32 Width, int32 Height, const FString& Path)
	{
		if (Width <= 0 || Height <= 0 || RGBA.Num() < Width * Height * 4)
		{
			return false;
		}

		IImageWrapperModule& ImageWrapperModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
		TSharedPtr<IImageWrapper> Wrapper = ImageWrapperModule.CreateImageWrapper(EImageFormat::PNG);
		if (!Wrapper.IsValid())
		{
			return false;
		}

		Wrapper->SetRaw(RGBA.GetData(), RGBA.Num(), Width, Height, ERGBFormat::RGBA, 8);
		const TArray64<uint8>& Compressed = Wrapper->GetCompressed();
		return FFileHelper::SaveArrayToFile(
			TArrayView<const uint8>(Compressed.GetData(), static_cast<int32>(Compressed.Num())),
			*Path);
	}

	static bool ParseVector2DField(const TSharedPtr<FJsonObject>& Object, const TCHAR* Key, FVector2D& Out);

	static bool ParseVector2DArray(const TSharedPtr<FJsonObject>& Object, const TCHAR* Key, TArray<FVector2D>& Out)
	{
		Out.Reset();
		const TArray<TSharedPtr<FJsonValue>>* Outer = nullptr;
		if (!Object.IsValid() || !Object->TryGetArrayField(Key, Outer))
		{
			return false;
		}

		for (const TSharedPtr<FJsonValue>& Value : *Outer)
		{
			if (!Value.IsValid() || Value->Type != EJson::Array)
			{
				continue;
			}
			const TArray<TSharedPtr<FJsonValue>>& Pair = Value->AsArray();
			if (Pair.Num() < 2)
			{
				continue;
			}
			Out.Emplace(Pair[0]->AsNumber(), Pair[1]->AsNumber());
		}
		return Out.Num() > 0;
	}

	static bool ParseOrderedBoundaryRuns(
		const TSharedPtr<FJsonObject>& Object,
		TArray<FCapBoundaryRunInput>& OutRuns)
	{
		OutRuns.Reset();
		const TArray<TSharedPtr<FJsonValue>>* Values = nullptr;
		if (!Object.IsValid() || !Object->TryGetArrayField(TEXT("ordered_boundary_runs"), Values))
		{
			return false;
		}

		for (const TSharedPtr<FJsonValue>& Value : *Values)
		{
			const TSharedPtr<FJsonObject> RunObject =
				Value.IsValid() && Value->Type == EJson::Object ? Value->AsObject() : nullptr;
			if (!RunObject.IsValid())
			{
				continue;
			}

			FCapBoundaryRunInput Run;
			double Number = -1.0;
			RunObject->TryGetNumberField(TEXT("stroke_id"), Number);
			Run.StrokeId = FMath::RoundToInt(Number);
			RunObject->TryGetBoolField(TEXT("synthetic"), Run.bSynthetic);
			RunObject->TryGetBoolField(TEXT("reversed"), Run.bReversed);
			Number = -1.0;
			RunObject->TryGetNumberField(TEXT("start_node_id"), Number);
			Run.StartNodeId = FMath::RoundToInt(Number);
			Number = -1.0;
			RunObject->TryGetNumberField(TEXT("end_node_id"), Number);
			Run.EndNodeId = FMath::RoundToInt(Number);
			ParseVector2DField(RunObject, TEXT("start_node_position"), Run.StartNodePosition);
			ParseVector2DField(RunObject, TEXT("end_node_position"), Run.EndNodePosition);

			FString Type;
			RunObject->TryGetStringField(TEXT("type"), Type);
			if (Run.bSynthetic || Type.Equals(TEXT("connector"), ESearchCase::IgnoreCase))
			{
				Run.Type = ECapBoundaryRunType::Connector;
			}
			else if (Type.Equals(TEXT("black"), ESearchCase::IgnoreCase))
			{
				Run.Type = ECapBoundaryRunType::Black;
			}
			else
			{
				Run.Type = ECapBoundaryRunType::Red;
			}

			if (!ParseVector2DArray(RunObject, TEXT("points"), Run.Points) || Run.Points.Num() < 2)
			{
				continue;
			}
			OutRuns.Add(MoveTemp(Run));
		}
		return OutRuns.Num() > 0;
	}

	// Parse an array of 2-point segments: [[[x0,y0],[x1,y1]], ...].
	static bool ParseSegment2DArray(const TSharedPtr<FJsonObject>& Object, const TCHAR* Key, TArray<FVector2D>& OutStarts, TArray<FVector2D>& OutEnds)
	{
		OutStarts.Reset();
		OutEnds.Reset();
		const TArray<TSharedPtr<FJsonValue>>* Outer = nullptr;
		if (!Object.IsValid() || !Object->TryGetArrayField(Key, Outer))
		{
			return false;
		}
		for (const TSharedPtr<FJsonValue>& Value : *Outer)
		{
			if (!Value.IsValid() || Value->Type != EJson::Array)
			{
				continue;
			}
			const TArray<TSharedPtr<FJsonValue>>& Seg = Value->AsArray();
			if (Seg.Num() < 2 || !Seg[0].IsValid() || !Seg[1].IsValid() ||
				Seg[0]->Type != EJson::Array || Seg[1]->Type != EJson::Array)
			{
				continue;
			}
			const TArray<TSharedPtr<FJsonValue>>& A = Seg[0]->AsArray();
			const TArray<TSharedPtr<FJsonValue>>& B = Seg[1]->AsArray();
			if (A.Num() < 2 || B.Num() < 2)
			{
				continue;
			}
			OutStarts.Emplace(A[0]->AsNumber(), A[1]->AsNumber());
			OutEnds.Emplace(B[0]->AsNumber(), B[1]->AsNumber());
		}
		return OutStarts.Num() > 0;
	}

	static bool ParseIntArrayField(const TSharedPtr<FJsonObject>& Object, const TCHAR* Key, TArray<int32>& OutValues)
	{
		OutValues.Reset();
		const TArray<TSharedPtr<FJsonValue>>* Values = nullptr;
		if (!Object.IsValid() || !Object->TryGetArrayField(Key, Values))
		{
			return false;
		}
		for (const TSharedPtr<FJsonValue>& Value : *Values)
		{
			if (Value.IsValid() && Value->Type == EJson::Number)
			{
				OutValues.Add(FMath::RoundToInt(Value->AsNumber()));
			}
		}
		return OutValues.Num() > 0;
	}

	static bool ParseGreenChainCandidates(
		const TSharedPtr<FJsonObject>& Object,
		TArray<FFromLZGreenChainCandidate2D>& OutChains)
	{
		OutChains.Reset();
		const TArray<TSharedPtr<FJsonValue>>* Values = nullptr;
		if (!Object.IsValid() || !Object->TryGetArrayField(TEXT("green_chain_candidates"), Values))
		{
			return false;
		}

		for (const TSharedPtr<FJsonValue>& Value : *Values)
		{
			const TSharedPtr<FJsonObject> ChainObject =
				Value.IsValid() && Value->Type == EJson::Object ? Value->AsObject() : nullptr;
			if (!ChainObject.IsValid())
			{
				continue;
			}

			FFromLZGreenChainCandidate2D Chain;
			double Number = -1.0;
			ChainObject->TryGetNumberField(TEXT("seed_stroke_id"), Number);
			Chain.SeedStrokeId = FMath::RoundToInt(Number);
			ParseIntArrayField(ChainObject, TEXT("stroke_ids"), Chain.StrokeIds);
			ParseVector2DField(ChainObject, TEXT("start"), Chain.Start);
			ParseVector2DField(ChainObject, TEXT("end"), Chain.End);
			if (!ParseVector2DField(ChainObject, TEXT("vector"), Chain.Vector))
			{
				Chain.Vector = Chain.End - Chain.Start;
			}
			ParseVector2DField(ChainObject, TEXT("seed_direction"), Chain.SeedDirection);
			Number = 0.0;
			ChainObject->TryGetNumberField(TEXT("chord_length"), Number);
			Chain.ChordLength = Number;
			Number = 0.0;
			ChainObject->TryGetNumberField(TEXT("path_length"), Number);
			Chain.PathLength = Number;
			Number = 0.0;
			ChainObject->TryGetNumberField(TEXT("total_gap"), Number);
			Chain.TotalGap = Number;
			ChainObject->TryGetStringField(TEXT("stop_reason"), Chain.StopReason);
			if ((Chain.End - Chain.Start).SizeSquared() > 1e-8)
			{
				if (Chain.ChordLength <= 0.0)
				{
					Chain.ChordLength = (Chain.End - Chain.Start).Size();
				}
				OutChains.Add(MoveTemp(Chain));
			}
		}

		OutChains.Sort([](const FFromLZGreenChainCandidate2D& A, const FFromLZGreenChainCandidate2D& B)
		{
			if (!FMath::IsNearlyEqual(A.PathLength, B.PathLength, 1e-6))
			{
				return A.PathLength > B.PathLength;
			}
			if (!FMath::IsNearlyEqual(A.ChordLength, B.ChordLength, 1e-6))
			{
				return A.ChordLength > B.ChordLength;
			}
			return A.SeedStrokeId < B.SeedStrokeId;
		});
		return OutChains.Num() > 0;
	}

	static bool ParseVector2DField(const TSharedPtr<FJsonObject>& Object, const TCHAR* Key, FVector2D& Out)
	{
		const TArray<TSharedPtr<FJsonValue>>* Values = nullptr;
		if (!Object.IsValid() || !Object->TryGetArrayField(Key, Values) || Values->Num() < 2)
		{
			return false;
		}
		Out = FVector2D((*Values)[0]->AsNumber(), (*Values)[1]->AsNumber());
		return true;
	}

	static bool ParseVectorArrayField(const TSharedPtr<FJsonObject>& Object, const TCHAR* Key, TArray<FVector>& Out)
	{
		Out.Reset();
		const TArray<TSharedPtr<FJsonValue>>* Outer = nullptr;
		if (!Object.IsValid() || !Object->TryGetArrayField(Key, Outer))
		{
			return false;
		}

		for (const TSharedPtr<FJsonValue>& Value : *Outer)
		{
			if (!Value.IsValid() || Value->Type != EJson::Array)
			{
				continue;
			}
			const TArray<TSharedPtr<FJsonValue>>& Triple = Value->AsArray();
			if (Triple.Num() < 3)
			{
				continue;
			}
			Out.Emplace(Triple[0]->AsNumber(), Triple[1]->AsNumber(), Triple[2]->AsNumber());
		}
		return Out.Num() > 0;
	}

	static bool ParseColorField(const TSharedPtr<FJsonObject>& Object, FColor& OutColor)
	{
		const TArray<TSharedPtr<FJsonValue>>* ColorArray = nullptr;
		if (!Object.IsValid() || !Object->TryGetArrayField(TEXT("color_rgb"), ColorArray) || ColorArray->Num() < 3)
		{
			return false;
		}

		OutColor = FColor(
			uint8(FMath::Clamp(FMath::RoundToInt((*ColorArray)[0]->AsNumber()), 0, 255)),
			uint8(FMath::Clamp(FMath::RoundToInt((*ColorArray)[1]->AsNumber()), 0, 255)),
			uint8(FMath::Clamp(FMath::RoundToInt((*ColorArray)[2]->AsNumber()), 0, 255)),
			255);
		return true;
	}

	static bool ParseVectorField(const TSharedPtr<FJsonObject>& Object, const TCHAR* Key, FVector& Out)
	{
		const TArray<TSharedPtr<FJsonValue>>* Values = nullptr;
		if (!Object.IsValid() || !Object->TryGetArrayField(Key, Values) || Values->Num() < 3)
		{
			return false;
		}
		Out = FVector((*Values)[0]->AsNumber(), (*Values)[1]->AsNumber(), (*Values)[2]->AsNumber());
		return true;
	}

	static bool ParseMatrixObject(const TSharedPtr<FJsonObject>& Object, FMatrix& OutMatrix)
	{
		if (!Object.IsValid())
		{
			return false;
		}

		FMatrix Matrix = FMatrix::Identity;
		for (int32 Row = 0; Row < 4; ++Row)
		{
			const TArray<TSharedPtr<FJsonValue>>* Values = nullptr;
			if (!Object->TryGetArrayField(FString::Printf(TEXT("row_%d"), Row), Values) || Values->Num() < 4)
			{
				return false;
			}
			for (int32 Column = 0; Column < 4; ++Column)
			{
				const double Value = (*Values)[Column]->AsNumber();
				if (!FMath::IsFinite(Value))
				{
					return false;
				}
				Matrix.M[Row][Column] = Value;
			}
		}

		OutMatrix = Matrix;
		return true;
	}

	static bool IsUsableOrthographicProjectionMatrix(const FMatrix& Matrix)
	{
		return FMath::IsFinite(Matrix.M[0][0]) &&
			FMath::IsFinite(Matrix.M[1][1]) &&
			FMath::IsFinite(Matrix.M[3][0]) &&
			FMath::IsFinite(Matrix.M[3][1]) &&
			FMath::Abs(Matrix.M[0][0]) > 1e-12 &&
			FMath::Abs(Matrix.M[1][1]) > 1e-12;
	}

	static FOrthographicProjectionMetadata MakeProjectionMetadata(
		const FMatrix& Matrix, const FString& Source)
	{
		FOrthographicProjectionMetadata Metadata;
		Metadata.bAvailable = IsUsableOrthographicProjectionMatrix(Matrix);
		Metadata.Source = Source;
		Metadata.M00 = Matrix.M[0][0];
		Metadata.M11 = Matrix.M[1][1];
		Metadata.M30 = Matrix.M[3][0];
		Metadata.M31 = Matrix.M[3][1];
		return Metadata;
	}

	static bool ParseProjectionScaleMetadata(
		const TSharedPtr<FJsonObject>& Root,
		FOrthographicProjectionMetadata& OutMetadata)
	{
		OutMetadata = FOrthographicProjectionMetadata();
		if (!Root.IsValid())
		{
			return false;
		}

		const TSharedPtr<FJsonObject>* ScaleObject = nullptr;
		if (!Root->TryGetObjectField(TEXT("projection_scale"), ScaleObject) ||
			!ScaleObject || !ScaleObject->IsValid())
		{
			return false;
		}

		(*ScaleObject)->TryGetNumberField(TEXT("m00"), OutMetadata.M00);
		(*ScaleObject)->TryGetNumberField(TEXT("m11"), OutMetadata.M11);
		(*ScaleObject)->TryGetNumberField(TEXT("m30"), OutMetadata.M30);
		(*ScaleObject)->TryGetNumberField(TEXT("m31"), OutMetadata.M31);
		Root->TryGetStringField(TEXT("projection_source"), OutMetadata.Source);
		OutMetadata.bAvailable =
			FMath::IsFinite(OutMetadata.M00) &&
			FMath::IsFinite(OutMetadata.M11) &&
			FMath::IsFinite(OutMetadata.M30) &&
			FMath::IsFinite(OutMetadata.M31) &&
			FMath::Abs(OutMetadata.M00) > 1e-12 &&
			FMath::Abs(OutMetadata.M11) > 1e-12;
		return OutMetadata.bAvailable;
	}

	static bool LoadFacesJson(
		const FString& Path,
		TArray<FFaceInfo>& OutFaces,
		FOrthographicProjectionMetadata& OutProjection)
	{
		OutFaces.Reset();
		OutProjection = FOrthographicProjectionMetadata();

		TSharedPtr<FJsonObject> Root;
		if (!LoadJsonObject(Path, Root))
		{
			return false;
		}
		ParseProjectionScaleMetadata(Root, OutProjection);

		const TArray<TSharedPtr<FJsonValue>>* FacesArray = nullptr;
		if (!Root->TryGetArrayField(TEXT("faces"), FacesArray))
		{
			return false;
		}

		for (const TSharedPtr<FJsonValue>& FaceValue : *FacesArray)
		{
			const TSharedPtr<FJsonObject> FaceObject = FaceValue.IsValid() ? FaceValue->AsObject() : nullptr;
			if (!FaceObject.IsValid())
			{
				continue;
			}

			FFaceInfo Face;
			double IdNumber = -1.0;
			FaceObject->TryGetNumberField(TEXT("id"), IdNumber);
			Face.Id = FMath::RoundToInt(IdNumber);
			ParseColorField(FaceObject, Face.Color);
			ParseVectorField(FaceObject, TEXT("plane_point"), Face.PlanePoint);
			ParseVectorField(FaceObject, TEXT("normal_world"), Face.Normal);
			Face.Normal = Face.Normal.GetSafeNormal();
			ParseVector2DArray(FaceObject, TEXT("key_points_2d"), Face.KeyPoints2D);
			ParseVectorArrayField(FaceObject, TEXT("key_points_3d"), Face.KeyPoints3D);

			if (Face.Id >= 0 && Face.KeyPoints2D.Num() >= 3 && Face.KeyPoints3D.Num() == Face.KeyPoints2D.Num() && !Face.Normal.IsNearlyZero())
			{
				OutFaces.Add(MoveTemp(Face));
			}
		}

		return OutFaces.Num() > 0;
	}

	static bool LoadActorMaterialIdJson(
		const FString& Path,
		TMap<uint32, FActorMaterialIdEntry>& OutEntriesByColorKey,
		FOrthographicProjectionMetadata& OutProjection)
	{
		OutEntriesByColorKey.Reset();
		OutProjection = FOrthographicProjectionMetadata();

		TSharedPtr<FJsonObject> Root;
		if (!LoadJsonObject(Path, Root))
		{
			return false;
		}

		const TSharedPtr<FJsonObject>* ProjectionObject = nullptr;
		FMatrix ProjectionMatrix = FMatrix::Identity;
		if (Root->TryGetObjectField(TEXT("projection_matrix"), ProjectionObject) &&
			ProjectionObject && ProjectionObject->IsValid() &&
			ParseMatrixObject(*ProjectionObject, ProjectionMatrix))
		{
			FString ProjectionSource;
			Root->TryGetStringField(TEXT("projection_source"), ProjectionSource);
			OutProjection = MakeProjectionMetadata(ProjectionMatrix, ProjectionSource);
		}

		const TArray<TSharedPtr<FJsonValue>>* EntriesArray = nullptr;
		if (!Root->TryGetArrayField(TEXT("entries"), EntriesArray))
		{
			return false;
		}

		for (const TSharedPtr<FJsonValue>& EntryValue : *EntriesArray)
		{
			const TSharedPtr<FJsonObject> EntryObject = EntryValue.IsValid() ? EntryValue->AsObject() : nullptr;
			if (!EntryObject.IsValid())
			{
				continue;
			}

			FActorMaterialIdEntry Entry;
			FColor Color = FColor::Black;
			if (!ParseColorField(EntryObject, Color))
			{
				continue;
			}

			Entry.ColorKey = ColorKey(Color.R, Color.G, Color.B);
			EntryObject->TryGetStringField(TEXT("actor_name"), Entry.ActorName);
			EntryObject->TryGetStringField(TEXT("actor_path"), Entry.ActorPath);
			EntryObject->TryGetStringField(TEXT("component_name"), Entry.ComponentName);
			EntryObject->TryGetStringField(TEXT("component_path"), Entry.ComponentPath);
			EntryObject->TryGetStringField(TEXT("component_type"), Entry.ComponentType);
			EntryObject->TryGetStringField(TEXT("material_name"), Entry.MaterialName);
			EntryObject->TryGetStringField(TEXT("material_path"), Entry.MaterialPath);
			double MaterialSlotNumber = -1.0;
			EntryObject->TryGetNumberField(TEXT("material_slot"), MaterialSlotNumber);
			Entry.MaterialSlot = FMath::RoundToInt(MaterialSlotNumber);
			if (Entry.ColorKey != 0 && Entry.MaterialSlot >= 0)
			{
				OutEntriesByColorKey.Add(Entry.ColorKey, Entry);
			}
		}

		return OutEntriesByColorKey.Num() > 0;
	}

	static bool LoadCameraJson(const FString& Path, FCameraInfo& OutCamera, FString& OutError)
	{
		OutCamera = FCameraInfo();
		OutError.Reset();

		TSharedPtr<FJsonObject> Root;
		if (!LoadJsonObject(Path, Root))
		{
			OutError = FString::Printf(TEXT("Failed to parse capture camera json: %s"), *Path);
			return false;
		}

		const TSharedPtr<FJsonObject>* TransformObject = nullptr;
		if (!Root->TryGetObjectField(TEXT("capture_view_transform"), TransformObject) || !TransformObject || !TransformObject->IsValid())
		{
			if (!Root->TryGetObjectField(TEXT("camera_component_transform"), TransformObject) || !TransformObject || !TransformObject->IsValid())
			{
				OutError = TEXT("capture json has no valid capture_view_transform or camera_component_transform");
				return false;
			}
		}

		double X = 0.0, Y = 0.0, Z = 0.0;
		double Pitch = 0.0, Yaw = 0.0, Roll = 0.0;
		(*TransformObject)->TryGetNumberField(TEXT("location_x"), X);
		(*TransformObject)->TryGetNumberField(TEXT("location_y"), Y);
		(*TransformObject)->TryGetNumberField(TEXT("location_z"), Z);
		(*TransformObject)->TryGetNumberField(TEXT("pitch"), Pitch);
		(*TransformObject)->TryGetNumberField(TEXT("yaw"), Yaw);
		(*TransformObject)->TryGetNumberField(TEXT("roll"), Roll);

		OutCamera.Location = FVector(X, Y, Z);
		const FRotator Rot(Pitch, Yaw, Roll);
		const FRotationMatrix RotMatrix(Rot);
		OutCamera.Forward = RotMatrix.GetScaledAxis(EAxis::X).GetSafeNormal();
		OutCamera.Right = RotMatrix.GetScaledAxis(EAxis::Y).GetSafeNormal();
		OutCamera.Up = RotMatrix.GetScaledAxis(EAxis::Z).GetSafeNormal();

		const TSharedPtr<FJsonObject>* ViewObject = nullptr;
		if (!Root->TryGetObjectField(TEXT("camera_view"), ViewObject) || !ViewObject || !ViewObject->IsValid())
		{
			OutError = TEXT("capture json has no valid camera_view");
			return false;
		}

		(*ViewObject)->TryGetNumberField(TEXT("fov"), OutCamera.Fov);
		(*ViewObject)->TryGetNumberField(TEXT("ortho_width"), OutCamera.OrthoWidth);
		double ViewportWidth = 0.0;
		double ViewportHeight = 0.0;
		(*ViewObject)->TryGetNumberField(TEXT("viewport_width"), ViewportWidth);
		(*ViewObject)->TryGetNumberField(TEXT("viewport_height"), ViewportHeight);
		OutCamera.ViewportWidth = FMath::RoundToInt(ViewportWidth);
		OutCamera.ViewportHeight = FMath::RoundToInt(ViewportHeight);

		FString ProjectionMode;
		if ((*ViewObject)->TryGetStringField(TEXT("projection_mode"), ProjectionMode))
		{
			OutCamera.bOrthographic = ProjectionMode.Contains(TEXT("Orthographic"));
		}
		(*ViewObject)->TryGetStringField(TEXT("projection_matrix_source"), OutCamera.ProjectionMatrixSource);

		const TSharedPtr<FJsonObject>* ProjectionObject = nullptr;
		OutCamera.bHasProjectionMatrix =
			(*ViewObject)->TryGetObjectField(TEXT("projection_matrix"), ProjectionObject) &&
			ProjectionObject && ProjectionObject->IsValid() &&
			ParseMatrixObject(*ProjectionObject, OutCamera.ProjectionMatrix) &&
			IsUsableOrthographicProjectionMatrix(OutCamera.ProjectionMatrix);

		if (OutCamera.Forward.IsNearlyZero())
		{
			OutError = TEXT("capture camera forward vector is invalid");
			return false;
		}
		if (!OutCamera.bOrthographic)
		{
			OutError = TEXT("Step 10 requires an orthographic capture camera");
			return false;
		}
		if (!OutCamera.bHasProjectionMatrix)
		{
			OutError = TEXT("camera_view.projection_matrix is missing or unusable; Step 10 will not fall back to ortho_width");
			return false;
		}
		return true;
	}

	static bool ProjectionTermsNearlyEqual(double A, double B)
	{
		const double Scale = FMath::Max3(1.0, FMath::Abs(A), FMath::Abs(B));
		return FMath::Abs(A - B) <= Scale * 1e-8;
	}

	static bool ProjectionMetadataMatches(
		const FOrthographicProjectionMetadata& A,
		const FOrthographicProjectionMetadata& B)
	{
		return A.bAvailable && B.bAvailable &&
			ProjectionTermsNearlyEqual(A.M00, B.M00) &&
			ProjectionTermsNearlyEqual(A.M11, B.M11) &&
			ProjectionTermsNearlyEqual(A.M30, B.M30) &&
			ProjectionTermsNearlyEqual(A.M31, B.M31);
	}

	static bool ValidateProjectionMetadata(FCommonInputs& Inputs, FString& OutError)
	{
		OutError.Reset();
		const FOrthographicProjectionMetadata CaptureProjection =
			MakeProjectionMetadata(Inputs.Camera.ProjectionMatrix, Inputs.Camera.ProjectionMatrixSource);

		if (!Inputs.FacesProjection.bAvailable)
		{
			OutError = TEXT("faces json is missing usable projection_scale metadata");
			return false;
		}
		if (!ProjectionMetadataMatches(CaptureProjection, Inputs.FacesProjection))
		{
			OutError = FString::Printf(
				TEXT("capture/faces projection mismatch: capture=(%.17g, %.17g, %.17g, %.17g) faces=(%.17g, %.17g, %.17g, %.17g)"),
				CaptureProjection.M00, CaptureProjection.M11, CaptureProjection.M30, CaptureProjection.M31,
				Inputs.FacesProjection.M00, Inputs.FacesProjection.M11,
				Inputs.FacesProjection.M30, Inputs.FacesProjection.M31);
			return false;
		}

		if (Inputs.ActorMaterialEntryByColorKey.Num() > 0)
		{
			if (!Inputs.ActorMaterialProjection.bAvailable)
			{
				OutError = TEXT("actor/material id json is missing a usable projection_matrix");
				return false;
			}
			if (!ProjectionMetadataMatches(CaptureProjection, Inputs.ActorMaterialProjection))
			{
				OutError = TEXT("capture and actor/material id projection matrices do not match");
				return false;
			}
		}

		Inputs.ProjectionValidation = Inputs.ActorMaterialEntryByColorKey.Num() > 0
			? TEXT("capture camera, faces, and actor/material projection metadata match")
			: TEXT("capture camera and faces projection metadata match; actor/material buffer unavailable");
		return true;
	}

	static bool LoadCaptureRef(const FString& PressDir, FCommonInputs& OutInputs)
	{
		TSharedPtr<FJsonObject> Root;
		if (!LoadJsonObject(PressDir / TEXT("capture_ref.json"), Root))
		{
			return false;
		}

		FString CaptureStem;
		Root->TryGetStringField(TEXT("capture_stem"), CaptureStem);
		CaptureStem = StripTrailingCaptureAuxSuffixes(CaptureStem);
		Root->TryGetStringField(TEXT("capture_json"), OutInputs.CaptureJsonRel);
		Root->TryGetStringField(TEXT("faces_png"), OutInputs.FacesPngRel);
		Root->TryGetStringField(TEXT("faces_json"), OutInputs.FacesJsonRel);
		Root->TryGetStringField(TEXT("actor_material_png"), OutInputs.ActorMaterialPngRel);
		Root->TryGetStringField(TEXT("actor_material_json"), OutInputs.ActorMaterialJsonRel);

		if (OutInputs.CaptureJsonRel.IsEmpty() && !CaptureStem.IsEmpty())
		{
			OutInputs.CaptureJsonRel = BuildCaptureRelPath(CaptureStem, TEXT(".json"));
		}
		if (OutInputs.FacesPngRel.IsEmpty() && !CaptureStem.IsEmpty())
		{
			OutInputs.FacesPngRel = BuildFacesRelPath(CaptureStem, TEXT(".png"));
		}
		if (OutInputs.FacesJsonRel.IsEmpty() && !CaptureStem.IsEmpty())
		{
			OutInputs.FacesJsonRel = BuildFacesRelPath(CaptureStem, TEXT(".json"));
		}
		if (OutInputs.ActorMaterialPngRel.IsEmpty() && !CaptureStem.IsEmpty())
		{
			OutInputs.ActorMaterialPngRel = BuildActorMaterialRelPath(CaptureStem, TEXT(".png"));
		}
		if (OutInputs.ActorMaterialJsonRel.IsEmpty() && !CaptureStem.IsEmpty())
		{
			OutInputs.ActorMaterialJsonRel = BuildActorMaterialRelPath(CaptureStem, TEXT(".json"));
		}
		NormalizeCaptureRelPath(OutInputs.CaptureJsonRel, TEXT(".json"));
		NormalizeFacesRelPath(OutInputs.FacesPngRel, TEXT(".png"));
		NormalizeFacesRelPath(OutInputs.FacesJsonRel, TEXT(".json"));
		NormalizeActorMaterialRelPath(OutInputs.ActorMaterialPngRel, TEXT(".png"));
		NormalizeActorMaterialRelPath(OutInputs.ActorMaterialJsonRel, TEXT(".json"));

		OutInputs.CaptureJsonPath = ResolveSavedPath(OutInputs.CaptureJsonRel);
		OutInputs.FacesPngPath = ResolveSavedPath(OutInputs.FacesPngRel);
		OutInputs.FacesJsonPath = ResolveSavedPath(OutInputs.FacesJsonRel);
		OutInputs.ActorMaterialPngPath = ResolveSavedPath(OutInputs.ActorMaterialPngRel);
		OutInputs.ActorMaterialJsonPath = ResolveSavedPath(OutInputs.ActorMaterialJsonRel);
		return !OutInputs.CaptureJsonPath.IsEmpty() && !OutInputs.FacesPngPath.IsEmpty() && !OutInputs.FacesJsonPath.IsEmpty();
	}

	static bool BuildFaceLookups(FCommonInputs& Inputs, FString& OutError)
	{
		Inputs.FaceIndexById.Reset();
		Inputs.FaceIdByColorKey.Reset();
		for (int32 i = 0; i < Inputs.Faces.Num(); ++i)
		{
			const FFaceInfo& Face = Inputs.Faces[i];
			Inputs.FaceIndexById.Add(Face.Id, i);
			const uint32 Key = ColorKey(Face.Color.R, Face.Color.G, Face.Color.B);
			if (const int32* ExistingFaceId = Inputs.FaceIdByColorKey.Find(Key))
			{
				OutError = FString::Printf(
					TEXT("Duplicate face color in faces.json/faces.png; recapture required (color_rgb=[%d,%d,%d], face_id=%d conflicts with face_id=%d)."),
					Face.Color.R, Face.Color.G, Face.Color.B, Face.Id, *ExistingFaceId);
				return false;
			}
			Inputs.FaceIdByColorKey.Add(Key, Face.Id);
		}
		return true;
	}

	static double PolygonArea2D(const TArray<FVector2D>& Poly)
	{
		double Area = 0.0;
		for (int32 i = 0, j = Poly.Num() - 1; i < Poly.Num(); j = i++)
		{
			Area += double(Poly[j].X) * double(Poly[i].Y) - double(Poly[i].X) * double(Poly[j].Y);
		}
		return Area * 0.5;
	}

	static bool PointInPolygon(const TArray<FVector2D>& Poly, const FVector2D& P)
	{
		bool bInside = false;
		for (int32 i = 0, j = Poly.Num() - 1; i < Poly.Num(); j = i++)
		{
			const FVector2D& A = Poly[i];
			const FVector2D& B = Poly[j];
			const double Denom = double(B.Y) - double(A.Y);
			if (((A.Y > P.Y) != (B.Y > P.Y)) &&
				FMath::Abs(Denom) > 1e-12)
			{
				const double XIntersect = (double(B.X) - double(A.X)) * (double(P.Y) - double(A.Y)) / Denom + double(A.X);
				if (double(P.X) < XIntersect)
				{
					bInside = !bInside;
				}
			}
		}
		return bInside;
	}

	static void RasterizePolygonMask(const TArray<FVector2D>& Poly, int32 Width, int32 Height, TArray<uint8>& OutMask, int32& OutPixelCount)
	{
		OutMask.Init(0, Width * Height);
		OutPixelCount = 0;
		if (Poly.Num() < 3 || Width <= 0 || Height <= 0)
		{
			return;
		}

		double MinX = Width - 1;
		double MinY = Height - 1;
		double MaxX = 0;
		double MaxY = 0;
		for (const FVector2D& P : Poly)
		{
			MinX = FMath::Min(MinX, P.X);
			MinY = FMath::Min(MinY, P.Y);
			MaxX = FMath::Max(MaxX, P.X);
			MaxY = FMath::Max(MaxY, P.Y);
		}

		const int32 X0 = FMath::Clamp(FMath::FloorToInt(MinX), 0, Width - 1);
		const int32 Y0 = FMath::Clamp(FMath::FloorToInt(MinY), 0, Height - 1);
		const int32 X1 = FMath::Clamp(FMath::CeilToInt(MaxX), 0, Width - 1);
		const int32 Y1 = FMath::Clamp(FMath::CeilToInt(MaxY), 0, Height - 1);

		for (int32 y = Y0; y <= Y1; ++y)
		{
			for (int32 x = X0; x <= X1; ++x)
			{
				if (PointInPolygon(Poly, FVector2D(double(x) + 0.5, double(y) + 0.5)))
				{
					OutMask[y * Width + x] = 255;
					++OutPixelCount;
				}
			}
		}
	}

	static bool HasActorMaterialIdBuffer(const FCommonInputs& Inputs)
	{
		return Inputs.ActorMaterialWidth > 0 &&
			Inputs.ActorMaterialHeight > 0 &&
			Inputs.ActorMaterialRGBA.Num() >= Inputs.ActorMaterialWidth * Inputs.ActorMaterialHeight * 4 &&
			Inputs.ActorMaterialEntryByColorKey.Num() > 0;
	}

	static bool SelectAttachMaterialFromIdBuffer(
		const FCommonInputs& Inputs,
		const FSolidReconstructionResult& Solid,
		FAttachMaterialIdSelection& OutSelection)
	{
		OutSelection = FAttachMaterialIdSelection();
		OutSelection.bLookupAttempted = HasActorMaterialIdBuffer(Inputs);
		if (!OutSelection.bLookupAttempted)
		{
			OutSelection.Error = TEXT("actor/material id buffer is unavailable");
			return false;
		}
		if (!Solid.Action.Equals(TEXT("attach"), ESearchCase::IgnoreCase))
		{
			OutSelection.Error = TEXT("solid is not an attach mesh");
			return false;
		}
		if (Solid.SourceLoop2D.Num() < 3)
		{
			OutSelection.Error = TEXT("attach source loop has fewer than three points");
			return false;
		}

		TArray<FVector2D> IdSpaceLoop;
		IdSpaceLoop.Reserve(Solid.SourceLoop2D.Num());
		const double ScaleX = Inputs.FacesWidth > 0 ? double(Inputs.ActorMaterialWidth) / double(Inputs.FacesWidth) : 1.0;
		const double ScaleY = Inputs.FacesHeight > 0 ? double(Inputs.ActorMaterialHeight) / double(Inputs.FacesHeight) : 1.0;
		for (const FVector2D& P : Solid.SourceLoop2D)
		{
			IdSpaceLoop.Emplace(P.X * ScaleX, P.Y * ScaleY);
		}

		TArray<uint8> Mask;
		int32 MaskPixels = 0;
		RasterizePolygonMask(IdSpaceLoop, Inputs.ActorMaterialWidth, Inputs.ActorMaterialHeight, Mask, MaskPixels);
		if (MaskPixels <= 0)
		{
			OutSelection.Error = TEXT("attach source loop rasterized to an empty actor/material id mask");
			return false;
		}

		auto Accumulate = [&](bool bRequireSelectedFace, TMap<uint32, int32>& Counts, int32& ConsideredPixels)
		{
			Counts.Reset();
			ConsideredPixels = 0;
			for (int32 y = 0; y < Inputs.ActorMaterialHeight; ++y)
			{
				for (int32 x = 0; x < Inputs.ActorMaterialWidth; ++x)
				{
					const int32 PixelIndex = y * Inputs.ActorMaterialWidth + x;
					if (Mask[PixelIndex] == 0)
					{
						continue;
					}

					if (bRequireSelectedFace &&
						Inputs.FacesWidth > 0 &&
						Inputs.FacesHeight > 0 &&
						Inputs.FacesRGBA.Num() >= Inputs.FacesWidth * Inputs.FacesHeight * 4 &&
						Solid.SelectedFaceId >= 0)
					{
						const int32 FaceX = FMath::Clamp(FMath::FloorToInt((double(x) + 0.5) * double(Inputs.FacesWidth) / double(Inputs.ActorMaterialWidth)), 0, Inputs.FacesWidth - 1);
						const int32 FaceY = FMath::Clamp(FMath::FloorToInt((double(y) + 0.5) * double(Inputs.FacesHeight) / double(Inputs.ActorMaterialHeight)), 0, Inputs.FacesHeight - 1);
						const int32 FaceOff = (FaceY * Inputs.FacesWidth + FaceX) * 4;
						const uint32 FaceKey = ColorKey(Inputs.FacesRGBA[FaceOff + 0], Inputs.FacesRGBA[FaceOff + 1], Inputs.FacesRGBA[FaceOff + 2]);
						const int32* FaceId = Inputs.FaceIdByColorKey.Find(FaceKey);
						if (!FaceId || *FaceId != Solid.SelectedFaceId)
						{
							continue;
						}
					}

					const int32 Off = PixelIndex * 4;
					if (Inputs.ActorMaterialRGBA[Off + 3] == 0)
					{
						continue;
					}

					const uint32 Key = ColorKey(
						Inputs.ActorMaterialRGBA[Off + 0],
						Inputs.ActorMaterialRGBA[Off + 1],
						Inputs.ActorMaterialRGBA[Off + 2]);
					if (Key == 0 || !Inputs.ActorMaterialEntryByColorKey.Contains(Key))
					{
						continue;
					}

					++ConsideredPixels;
					Counts.FindOrAdd(Key) += 1;
				}
			}
		};

		TMap<uint32, int32> Counts;
		int32 ConsideredPixels = 0;
		Accumulate(/*bRequireSelectedFace*/ true, Counts, ConsideredPixels);
		if (Counts.Num() == 0)
		{
			Accumulate(/*bRequireSelectedFace*/ false, Counts, ConsideredPixels);
		}

		uint32 BestKey = 0;
		int32 BestCount = 0;
		for (const TPair<uint32, int32>& Pair : Counts)
		{
			if (Pair.Value > BestCount)
			{
				BestKey = Pair.Key;
				BestCount = Pair.Value;
			}
		}

		const FActorMaterialIdEntry* Entry = Inputs.ActorMaterialEntryByColorKey.Find(BestKey);
		if (!Entry || BestCount <= 0)
		{
			OutSelection.Error = FString::Printf(TEXT("no actor/material id won the attach source mask vote (mask_pixels=%d considered=%d)"), MaskPixels, ConsideredPixels);
			return false;
		}

		OutSelection.bFound = true;
		OutSelection.Entry = *Entry;
		OutSelection.PixelCount = BestCount;
		OutSelection.ConsideredPixelCount = ConsideredPixels;
		OutSelection.Coverage = double(BestCount) / double(FMath::Max(1, ConsideredPixels));
		return true;
	}

	static bool SaveMaskPng(const TArray<uint8>& Mask, int32 Width, int32 Height, const FString& Path)
	{
		TArray<uint8> RGBA;
		RGBA.SetNumUninitialized(Width * Height * 4);
		for (int32 i = 0; i < Width * Height; ++i)
		{
			const uint8 V = Mask[i] > 0 ? 0 : 255;
			const int32 Off = i * 4;
			RGBA[Off + 0] = V;
			RGBA[Off + 1] = V;
			RGBA[Off + 2] = V;
			RGBA[Off + 3] = 255;
		}
		return SaveRGBAToPng(RGBA, Width, Height, Path);
	}

	static uint8 BlendChannel(uint8 A, uint8 B, double T)
	{
		return uint8(FMath::Clamp(FMath::RoundToInt(double(A) * (1.0 - T) + double(B) * T), 0, 255));
	}

	static bool SaveOverlapPng(
		const TArray<uint8>& FacesRGBA, const TArray<uint8>& Mask,
		const TMap<uint32, int32>& FaceIdByColorKey, const TSet<int32>& CandidateFaceIds,
		const TSet<int32>& ParallelFaceIds,
		int32 SelectedFaceId, int32 Width, int32 Height, const FString& Path)
	{
		TArray<uint8> RGBA = FacesRGBA;
		if (RGBA.Num() < Width * Height * 4 || Mask.Num() < Width * Height)
		{
			return false;
		}

		for (int32 i = 0; i < Width * Height; ++i)
		{
			if (Mask[i] == 0)
			{
				continue;
			}

			const int32 Off = i * 4;
			const uint32 Key = ColorKey(RGBA[Off + 0], RGBA[Off + 1], RGBA[Off + 2]);
			const int32* FaceId = FaceIdByColorKey.Find(Key);
			FColor Overlay = FColor(255, 60, 60, 255);
			if (FaceId)
			{
				if (*FaceId == SelectedFaceId)
				{
					Overlay = FColor(40, 230, 80, 255);
				}
				else if (ParallelFaceIds.Contains(*FaceId))
				{
					Overlay = FColor(255, 220, 40, 255);
				}
				else if (CandidateFaceIds.Contains(*FaceId))
				{
					Overlay = FColor(255, 140, 40, 255);
				}
				else
				{
					continue;
				}
			}

			RGBA[Off + 0] = BlendChannel(RGBA[Off + 0], Overlay.R, 0.65);
			RGBA[Off + 1] = BlendChannel(RGBA[Off + 1], Overlay.G, 0.65);
			RGBA[Off + 2] = BlendChannel(RGBA[Off + 2], Overlay.B, 0.65);
			RGBA[Off + 3] = 255;
		}

		return SaveRGBAToPng(RGBA, Width, Height, Path);
	}

	static FVector CameraRayDirection(const FCameraInfo& Camera, int32 Width, int32 Height, const FVector2D& Pixel)
	{
		const double NdcX = 2.0 * ((Pixel.X + 0.5) / double(Width)) - 1.0;
		const double NdcY = 1.0 - 2.0 * ((Pixel.Y + 0.5) / double(Height));
		const double TanX = FMath::Tan(FMath::DegreesToRadians(Camera.Fov * 0.5));
		const double TanY = TanX * (double(Height) / double(Width));
		return (Camera.Forward + Camera.Right * (NdcX * TanX) + Camera.Up * (NdcY * TanY)).GetSafeNormal();
	}

	static bool TryGetOrthographicViewPlanePosition(
		const FCameraInfo& Camera,
		double NdcX,
		double NdcY,
		double& OutRight,
		double& OutUp)
	{
		if (!Camera.bHasProjectionMatrix ||
			!IsUsableOrthographicProjectionMatrix(Camera.ProjectionMatrix))
		{
			return false;
		}

		OutRight = (NdcX - Camera.ProjectionMatrix.M[3][0]) / Camera.ProjectionMatrix.M[0][0];
		OutUp = (NdcY - Camera.ProjectionMatrix.M[3][1]) / Camera.ProjectionMatrix.M[1][1];
		return FMath::IsFinite(OutRight) && FMath::IsFinite(OutUp);
	}

	static bool CameraOrthoRayOrigin(
		const FCameraInfo& Camera,
		int32 Width,
		int32 Height,
		const FVector2D& Pixel,
		FVector& OutOrigin)
	{
		if (Width <= 0 || Height <= 0)
		{
			return false;
		}

		const double NdcX = 2.0 * ((Pixel.X + 0.5) / double(Width)) - 1.0;
		const double NdcY = 1.0 - 2.0 * ((Pixel.Y + 0.5) / double(Height));
		double ViewRight = 0.0;
		double ViewUp = 0.0;
		if (!TryGetOrthographicViewPlanePosition(Camera, NdcX, NdcY, ViewRight, ViewUp))
		{
			return false;
		}

		OutOrigin = Camera.Location + Camera.Right * ViewRight + Camera.Up * ViewUp;
		return FMath::IsFinite(OutOrigin.X) && FMath::IsFinite(OutOrigin.Y) && FMath::IsFinite(OutOrigin.Z);
	}

	static bool IntersectPixelWithPlaneOrthographic(
		const FCameraInfo& Camera, int32 Width, int32 Height, const FVector2D& Pixel,
		const FVector& PlanePoint, const FVector& PlaneNormal,
		FVector& OutHit, double* OutDistance = nullptr)
	{
		FVector RayOrigin;
		if (!CameraOrthoRayOrigin(Camera, Width, Height, Pixel, RayOrigin))
		{
			return false;
		}

		const FVector RayDir = Camera.Forward.GetSafeNormal();
		const FVector Normal = PlaneNormal.GetSafeNormal();
		const double Denom = FVector::DotProduct(RayDir, Normal);
		if (FMath::Abs(Denom) < 1e-8)
		{
			return false;
		}

		const double T = FVector::DotProduct(PlanePoint - RayOrigin, Normal) / Denom;
		if (!FMath::IsFinite(T))
		{
			return false;
		}

		OutHit = RayOrigin + RayDir * T;
		if (OutDistance)
		{
			*OutDistance = FVector::Distance(Camera.Location, OutHit);
		}
		return FMath::IsFinite(OutHit.X) && FMath::IsFinite(OutHit.Y) && FMath::IsFinite(OutHit.Z);
	}

	static bool ProjectWorldToImageOrthographic(
		const FCameraInfo& Camera, int32 Width, int32 Height, const FVector& World,
		FVector2D& OutPixel)
	{
		if (Width <= 0 || Height <= 0 ||
			!Camera.bHasProjectionMatrix ||
			!IsUsableOrthographicProjectionMatrix(Camera.ProjectionMatrix))
		{
			return false;
		}

		const FVector Delta = World - Camera.Location;
		const double Depth = FVector::DotProduct(Delta, Camera.Forward);
		if (!FMath::IsFinite(Depth) || Depth <= 0.0)
		{
			return false;
		}

		const double ViewRight = FVector::DotProduct(Delta, Camera.Right);
		const double ViewUp = FVector::DotProduct(Delta, Camera.Up);
		const double NdcX =
			ViewRight * Camera.ProjectionMatrix.M[0][0] +
			Camera.ProjectionMatrix.M[3][0];
		const double NdcY =
			ViewUp * Camera.ProjectionMatrix.M[1][1] +
			Camera.ProjectionMatrix.M[3][1];
		OutPixel = FVector2D(
			((NdcX + 1.0) * 0.5 * double(Width)) - 0.5,
			((1.0 - NdcY) * 0.5 * double(Height)) - 0.5);
		return FMath::IsFinite(OutPixel.X) && FMath::IsFinite(OutPixel.Y);
	}

	static bool OrthographicPixelsPerWorldAlongDirection(
		const FCameraInfo& Camera, int32 Width, int32 Height,
		const FVector& DirectionWorld, FVector2D& OutPixelsPerWorld)
	{
		if (Width <= 0 || Height <= 0 ||
			!Camera.bHasProjectionMatrix ||
			!IsUsableOrthographicProjectionMatrix(Camera.ProjectionMatrix))
		{
			return false;
		}

		const FVector Direction = DirectionWorld.GetSafeNormal();
		if (Direction.IsNearlyZero())
		{
			return false;
		}

		OutPixelsPerWorld = FVector2D(
			FVector::DotProduct(Direction, Camera.Right) *
				(0.5 * double(Width) * Camera.ProjectionMatrix.M[0][0]),
			-FVector::DotProduct(Direction, Camera.Up) *
				(0.5 * double(Height) * Camera.ProjectionMatrix.M[1][1]));
		return OutPixelsPerWorld.SizeSquared() > 1e-12 &&
			FMath::IsFinite(OutPixelsPerWorld.X) &&
			FMath::IsFinite(OutPixelsPerWorld.Y);
	}

	static bool ProjectWorldDirectionToImageOrthographic(
		const FCameraInfo& Camera, int32 Width, int32 Height,
		const FVector& DirectionWorld, FVector2D& OutDirection)
	{
		FVector2D PixelsPerWorld;
		if (!OrthographicPixelsPerWorldAlongDirection(Camera, Width, Height, DirectionWorld, PixelsPerWorld))
		{
			return false;
		}
		OutDirection = PixelsPerWorld.GetSafeNormal();
		return true;
	}

	static bool CameraRayForPixel(
		const FCameraInfo& Camera, int32 Width, int32 Height, const FVector2D& Pixel,
		FVector& OutOrigin, FVector& OutDirection)
	{
		if (Camera.bOrthographic)
		{
			if (!CameraOrthoRayOrigin(Camera, Width, Height, Pixel, OutOrigin))
			{
				return false;
			}
		}
		else
		{
			OutOrigin = Camera.Location;
		}
		OutDirection = Camera.bOrthographic ? Camera.Forward : CameraRayDirection(Camera, Width, Height, Pixel);
		OutDirection = OutDirection.GetSafeNormal();
		return !OutDirection.IsNearlyZero();
	}

	static bool IntersectPixelWithPlane(
		const FCameraInfo& Camera, int32 Width, int32 Height,
		const FVector2D& Pixel, const FVector& PlanePoint, const FVector& PlaneNormal,
		FVector& OutHit, double* OutDistance = nullptr)
	{
		FVector RayOrigin;
		FVector RayDir;
		if (!CameraRayForPixel(Camera, Width, Height, Pixel, RayOrigin, RayDir))
		{
			return false;
		}
		const FVector Normal = PlaneNormal.GetSafeNormal();
		const double Denom = FVector::DotProduct(RayDir, Normal);
		if (FMath::Abs(Denom) < 1e-8)
		{
			return false;
		}

		const double T = FVector::DotProduct(PlanePoint - RayOrigin, Normal) / Denom;
		if (!FMath::IsFinite(T))
		{
			return false;
		}
		OutHit = RayOrigin + RayDir * T;
		if (OutDistance)
		{
			*OutDistance = FVector::Distance(Camera.Location, OutHit);
		}
		return FMath::IsFinite(OutHit.X) && FMath::IsFinite(OutHit.Y) && FMath::IsFinite(OutHit.Z);
	}

	static bool ProjectWorldToImage(const FCameraInfo& Camera, int32 Width, int32 Height, const FVector& World, FVector2D& OutPixel)
	{
		if (Width <= 0 || Height <= 0)
		{
			return false;
		}

		const FVector Delta = World - Camera.Location;
		double NdcX = 0.0;
		double NdcY = 0.0;
		if (Camera.bOrthographic)
		{
			return ProjectWorldToImageOrthographic(Camera, Width, Height, World, OutPixel);
		}
		else
		{
			const double Depth = FVector::DotProduct(Delta, Camera.Forward);
			if (Depth <= 1e-6)
			{
				return false;
			}

			const double TanX = FMath::Tan(FMath::DegreesToRadians(Camera.Fov * 0.5));
			const double TanY = TanX * (double(Height) / double(Width));
			if (FMath::Abs(TanX) < 1e-8 || FMath::Abs(TanY) < 1e-8)
			{
				return false;
			}

			NdcX = FVector::DotProduct(Delta, Camera.Right) / (Depth * TanX);
			NdcY = FVector::DotProduct(Delta, Camera.Up) / (Depth * TanY);
		}

		OutPixel = FVector2D(
			((NdcX + 1.0) * 0.5 * double(Width)) - 0.5,
			((1.0 - NdcY) * 0.5 * double(Height)) - 0.5);
		return FMath::IsFinite(OutPixel.X) && FMath::IsFinite(OutPixel.Y);
	}

	static bool IntersectMaskCentroidWithFacePlane(
		const FCameraInfo& Camera, int32 Width, int32 Height,
		const FVector2D& Pixel, const FFaceInfo& Face, FVector& OutHit, double& OutDistance)
	{
		return IntersectPixelWithPlaneOrthographic(
			Camera, Width, Height, Pixel, Face.PlanePoint, Face.Normal, OutHit, &OutDistance);
	}

	static bool ProjectFaceNormalToImage(
		const FCameraInfo& Camera, int32 Width, int32 Height,
		const FFaceInfo& Face, FVector2D& OutDirection)
	{
		return ProjectWorldDirectionToImageOrthographic(Camera, Width, Height, Face.Normal, OutDirection);
	}

	static void MapPolygonToFacesSpace(
		const TArray<FVector2D>& InPoly, double ScaleX, double ScaleY, TArray<FVector2D>& OutPoly);
	static void SaveCandidateFaceValidationDebug(
		const FFromLZCandidateFaceRequest& Request,
		const FFromLZCandidateFaceEvaluation& Evaluation,
		const FFromLZFaceReconstructionParams& Params,
		const FCommonInputs& Inputs,
		int32 SourceWidth,
		int32 SourceHeight,
		const TArray<uint8>& Mask,
		const TArray<FFaceCandidate>& Candidates,
		const FString& OutputDir);

	static bool IsCandidateOverlapPixel(
		const FCommonInputs& Inputs,
		const TArray<uint8>& Mask,
		int32 FaceId,
		int32 X,
		int32 Y)
	{
		if (X < 0 || Y < 0 || X >= Inputs.FacesWidth || Y >= Inputs.FacesHeight ||
			Mask.Num() < Inputs.FacesWidth * Inputs.FacesHeight ||
			Inputs.FacesRGBA.Num() < Inputs.FacesWidth * Inputs.FacesHeight * 4)
		{
			return false;
		}

		const int32 Pixel = Y * Inputs.FacesWidth + X;
		if (Mask[Pixel] == 0)
		{
			return false;
		}

		const int32 Offset = Pixel * 4;
		const uint32 Key = ColorKey(
			Inputs.FacesRGBA[Offset + 0],
			Inputs.FacesRGBA[Offset + 1],
			Inputs.FacesRGBA[Offset + 2]);
		const int32* FoundFaceId = Inputs.FaceIdByColorKey.Find(Key);
		return FoundFaceId && *FoundFaceId == FaceId;
	}

	static bool IsCandidateOverlapNearPixel(
		const FCommonInputs& Inputs,
		const TArray<uint8>& Mask,
		int32 FaceId,
		const FVector2D& Pixel,
		int32 RadiusPx)
	{
		const int32 CenterX = FMath::RoundToInt(Pixel.X);
		const int32 CenterY = FMath::RoundToInt(Pixel.Y);
		const int32 Radius = FMath::Max(0, RadiusPx);
		for (int32 Dy = -Radius; Dy <= Radius; ++Dy)
		{
			for (int32 Dx = -Radius; Dx <= Radius; ++Dx)
			{
				if (IsCandidateOverlapPixel(Inputs, Mask, FaceId, CenterX + Dx, CenterY + Dy))
				{
					return true;
				}
			}
		}
		return false;
	}

	static bool FindFirstCandidateOverlapRayHit(
		const FCommonInputs& Inputs,
		const TArray<uint8>& Mask,
		int32 FaceId,
		const FVector2D& Origin,
		const FVector2D& Direction,
		FVector2D& OutHit,
		double& OutDistance)
	{
		OutHit = FVector2D::ZeroVector;
		OutDistance = -1.0;
		const FVector2D Unit = Direction.GetSafeNormal();
		if (Unit.IsNearlyZero() ||
			Inputs.FacesWidth <= 0 ||
			Inputs.FacesHeight <= 0 ||
			Mask.Num() < Inputs.FacesWidth * Inputs.FacesHeight)
		{
			return false;
		}

		const double MaxDistance =
			FMath::Sqrt(double(Inputs.FacesWidth) * double(Inputs.FacesWidth) +
				double(Inputs.FacesHeight) * double(Inputs.FacesHeight)) * 1.5;
		const double ExpandedMinX = -double(AttachDirectedRayHitRadiusPx);
		const double ExpandedMinY = -double(AttachDirectedRayHitRadiusPx);
		const double ExpandedMaxX = double(Inputs.FacesWidth - 1 + AttachDirectedRayHitRadiusPx);
		const double ExpandedMaxY = double(Inputs.FacesHeight - 1 + AttachDirectedRayHitRadiusPx);
		bool bWasInsideExpandedImage = false;
		for (double Distance = AttachDirectedRayMinDistancePx;
			Distance <= MaxDistance;
			Distance += AttachDirectedRayStepPx)
		{
			const FVector2D P = Origin + Unit * Distance;
			const bool bInsideExpandedImage =
				P.X >= ExpandedMinX &&
				P.Y >= ExpandedMinY &&
				P.X <= ExpandedMaxX &&
				P.Y <= ExpandedMaxY;
			if (!bInsideExpandedImage)
			{
				if (bWasInsideExpandedImage)
				{
					break;
				}
				continue;
			}
			bWasInsideExpandedImage = true;

			if (IsCandidateOverlapNearPixel(
				Inputs,
				Mask,
				FaceId,
				P,
				AttachDirectedRayHitRadiusPx))
			{
				OutHit = P;
				OutDistance = Distance;
				return true;
			}
		}
		return false;
	}

	static bool EvaluateAttachDirectedNormalAgainstGreenChains(
		const FCommonInputs& Inputs,
		const TArray<uint8>& Mask,
		const TArray<FDirectedGreenChain2D>& GreenChains,
		const FVector2D& RawNormalDir,
		FFaceCandidate& Candidate)
	{
		Candidate.bUsedDirectedAttachNormalCheck = true;
		Candidate.RawProjectedNormal2D = RawNormalDir;
		Candidate.ProjectedNormal2D = RawNormalDir;
		Candidate.NormalGreenAngleDegrees = 180.0;
		Candidate.AttachNormalOrientationReason = TEXT("no directed green chain produced a candidate-overlap ray hit");
		if (RawNormalDir.IsNearlyZero() || GreenChains.Num() == 0)
		{
			Candidate.AttachNormalOrientationReason = TEXT("missing raw projected normal or directed green chain");
			return false;
		}

		bool bFound = false;
		double BestAngle = 180.0;
		double BestChosenRayDistance = TNumericLimits<double>::Max();
		FFaceCandidate BestDebug = Candidate;
		for (const FDirectedGreenChain2D& Chain : GreenChains)
		{
			FVector2D PlusHit;
			FVector2D MinusHit;
			double PlusDistance = -1.0;
			double MinusDistance = -1.0;
			const bool bPlusHit = FindFirstCandidateOverlapRayHit(
				Inputs,
				Mask,
				Candidate.FaceId,
				Chain.Start,
				RawNormalDir,
				PlusHit,
				PlusDistance);
			const bool bMinusHit = FindFirstCandidateOverlapRayHit(
				Inputs,
				Mask,
				Candidate.FaceId,
				Chain.Start,
				-RawNormalDir,
				MinusHit,
				MinusDistance);

			FVector2D OrientedNormal = FVector2D::ZeroVector;
			FVector2D ExpectedNormalDir = Chain.Dir;
			double ChosenRayDistance = -1.0;
			FString Reason;
			bool bOrientationValid = false;
			if (bPlusHit && bMinusHit)
			{
				const double DistanceDelta = FMath::Abs(PlusDistance - MinusDistance);
				if (DistanceDelta <= AttachDirectedRayAmbiguousDistancePx)
				{
					Reason = FString::Printf(
						TEXT("chain %d ambiguous: plus/minus ray distances %.3f/%.3f differ by <= %.3f px"),
						Chain.ChainIndex,
						PlusDistance,
						MinusDistance,
						AttachDirectedRayAmbiguousDistancePx);
				}
				else if (PlusDistance < MinusDistance)
				{
					OrientedNormal = -RawNormalDir;
					ChosenRayDistance = PlusDistance;
					bOrientationValid = true;
					Reason = TEXT("plus_ray_hit_candidate_overlap_first; attach_oriented_normal_is_negative_projected_face_normal");
				}
				else
				{
					OrientedNormal = -RawNormalDir;
					ChosenRayDistance = MinusDistance;
					bOrientationValid = true;
					Reason = TEXT("minus_ray_hit_candidate_overlap_first; attach_oriented_normal_is_negative_projected_face_normal");
				}
			}
			else if (bPlusHit)
			{
				OrientedNormal = -RawNormalDir;
				ChosenRayDistance = PlusDistance;
				bOrientationValid = true;
				Reason = TEXT("plus_ray_hit_candidate_overlap; attach_oriented_normal_is_negative_projected_face_normal");
			}
			else if (bMinusHit)
			{
				OrientedNormal = -RawNormalDir;
				ChosenRayDistance = MinusDistance;
				bOrientationValid = true;
				Reason = TEXT("minus_ray_hit_candidate_overlap; attach_oriented_normal_is_negative_projected_face_normal");
			}
			else
			{
				Reason = FString::Printf(
					TEXT("chain %d found no +/- normal ray hit on candidate overlap mask"),
					Chain.ChainIndex);
			}

			const double UnorientedDot = FMath::Clamp(
				FMath::Abs(FVector2D::DotProduct(RawNormalDir, ExpectedNormalDir)),
				0.0,
				1.0);
			const double UnorientedAngle = FMath::RadiansToDegrees(FMath::Acos(UnorientedDot));
			if (!bOrientationValid)
			{
				if (Candidate.AttachDirectedGreenChainIndex < 0)
				{
					Candidate.AttachDirectedGreenChainIndex = Chain.ChainIndex;
					Candidate.AttachGreenStart2D = Chain.Start;
					Candidate.AttachGreenEnd2D = Chain.End;
					Candidate.AttachGreenDir2D = Chain.Dir;
					Candidate.AttachExpectedNormalDir2D = ExpectedNormalDir;
					Candidate.bAttachPlusRayHit = bPlusHit;
					Candidate.bAttachMinusRayHit = bMinusHit;
					Candidate.AttachPlusRayHit2D = PlusHit;
					Candidate.AttachMinusRayHit2D = MinusHit;
					Candidate.AttachPlusRayDistancePx = PlusDistance;
					Candidate.AttachMinusRayDistancePx = MinusDistance;
					Candidate.UnorientedNormalGreenAngleDegrees = UnorientedAngle;
					Candidate.AttachNormalOrientationReason = Reason;
				}
				continue;
			}

			const double DirectedDot = FMath::Clamp(
				FVector2D::DotProduct(OrientedNormal.GetSafeNormal(), ExpectedNormalDir),
				-1.0,
				1.0);
			const double DirectedAngle = FMath::RadiansToDegrees(FMath::Acos(DirectedDot));
			if (!bFound ||
				DirectedAngle < BestAngle - 1e-6 ||
				(FMath::IsNearlyEqual(DirectedAngle, BestAngle, 1e-6) &&
					ChosenRayDistance < BestChosenRayDistance))
			{
				bFound = true;
				BestAngle = DirectedAngle;
				BestChosenRayDistance = ChosenRayDistance;
				BestDebug = Candidate;
				BestDebug.bAttachNormalOrientationValid = true;
				BestDebug.AttachDirectedGreenChainIndex = Chain.ChainIndex;
				BestDebug.AttachGreenStart2D = Chain.Start;
				BestDebug.AttachGreenEnd2D = Chain.End;
				BestDebug.AttachGreenDir2D = Chain.Dir;
				BestDebug.AttachExpectedNormalDir2D = ExpectedNormalDir;
				BestDebug.OrientedProjectedNormal2D = OrientedNormal.GetSafeNormal();
				BestDebug.ProjectedNormal2D = BestDebug.OrientedProjectedNormal2D;
				BestDebug.bAttachPlusRayHit = bPlusHit;
				BestDebug.bAttachMinusRayHit = bMinusHit;
				BestDebug.AttachPlusRayHit2D = PlusHit;
				BestDebug.AttachMinusRayHit2D = MinusHit;
				BestDebug.AttachPlusRayDistancePx = PlusDistance;
				BestDebug.AttachMinusRayDistancePx = MinusDistance;
				BestDebug.UnorientedNormalGreenAngleDegrees = UnorientedAngle;
				BestDebug.NormalGreenAngleDegrees = DirectedAngle;
				BestDebug.AttachNormalOrientationReason = Reason;
			}
		}

		if (bFound)
		{
			Candidate = BestDebug;
			return true;
		}
		return false;
	}

	static FFromLZCandidateFaceEvaluation EvaluateSourcePolygonForFaces(
		const TArray<FVector2D>& SourcePolygon,
		const TArray<FVector2D>& SideVectors,
		const TArray<FFromLZGreenChainCandidate2D>& GreenChains,
		bool bUseAttachDirectedNormalCheck,
		int32 SourceWidth,
		int32 SourceHeight,
		const FString& SourcePolygonKey,
		const FCommonInputs& Inputs,
		TArray<uint8>* OutMask = nullptr,
		TArray<FFaceCandidate>* OutCandidates = nullptr)
	{
		FFromLZCandidateFaceEvaluation Evaluation;
		Evaluation.bEvaluated = true;
		Evaluation.EvaluationMode = TEXT("direct_cap_face");
		Evaluation.SourcePolygonKey = SourcePolygonKey;

		if (SourceWidth <= 0 || SourceHeight <= 0 || Inputs.FacesWidth <= 0 || Inputs.FacesHeight <= 0)
		{
			Evaluation.RejectReason = TEXT("invalid source or faces image dimensions");
			return Evaluation;
		}
		if (SourcePolygon.Num() < 3)
		{
			Evaluation.RejectReason = TEXT("source polygon has fewer than three points");
			return Evaluation;
		}

		const double ScaleX = double(Inputs.FacesWidth) / double(SourceWidth);
		const double ScaleY = double(Inputs.FacesHeight) / double(SourceHeight);
		TArray<FVector2D> FaceSpacePoly;
		MapPolygonToFacesSpace(SourcePolygon, ScaleX, ScaleY, FaceSpacePoly);

		TArray<FVector2D> FaceSpaceSideVectors;
		for (const FVector2D& SideVector : SideVectors)
		{
			const FVector2D Scaled(SideVector.X * ScaleX, SideVector.Y * ScaleY);
			if (Scaled.SizeSquared() >= 1e-8)
			{
				FaceSpaceSideVectors.Add(Scaled.GetSafeNormal());
			}
		}
		TArray<FDirectedGreenChain2D> FaceSpaceGreenChains;
		for (int32 ChainIndex = 0; ChainIndex < GreenChains.Num(); ++ChainIndex)
		{
			const FFromLZGreenChainCandidate2D& Chain = GreenChains[ChainIndex];
			FDirectedGreenChain2D DirectedChain;
			DirectedChain.ChainIndex = ChainIndex;
			DirectedChain.Start = FVector2D(Chain.Start.X * ScaleX, Chain.Start.Y * ScaleY);
			DirectedChain.End = FVector2D(Chain.End.X * ScaleX, Chain.End.Y * ScaleY);
			const FVector2D Delta = DirectedChain.End - DirectedChain.Start;
			DirectedChain.Length = Delta.Size();
			DirectedChain.PathLength = Chain.PathLength > 0.0 ? Chain.PathLength : DirectedChain.Length;
			if (DirectedChain.Length >= 1e-6)
			{
				DirectedChain.Dir = Delta / DirectedChain.Length;
				FaceSpaceGreenChains.Add(DirectedChain);
			}
		}
		if (bUseAttachDirectedNormalCheck && FaceSpaceGreenChains.Num() > 1)
		{
			FaceSpaceGreenChains.Sort([](const FDirectedGreenChain2D& A, const FDirectedGreenChain2D& B)
			{
				if (!FMath::IsNearlyEqual(A.PathLength, B.PathLength, 1e-6))
				{
					return A.PathLength > B.PathLength;
				}
				if (!FMath::IsNearlyEqual(A.Length, B.Length, 1e-6))
				{
					return A.Length > B.Length;
				}
				return A.ChainIndex < B.ChainIndex;
			});
			FaceSpaceGreenChains.SetNum(1, EAllowShrinking::No);
		}
		if (bUseAttachDirectedNormalCheck && FaceSpaceGreenChains.Num() == 0)
		{
			Evaluation.RejectReason = TEXT("attach candidate has no non-zero directed green chain in faces image space");
			return Evaluation;
		}
		if (!bUseAttachDirectedNormalCheck && FaceSpaceSideVectors.Num() == 0)
		{
			Evaluation.RejectReason = TEXT("candidate has no non-zero side vector in faces image space");
			return Evaluation;
		}

		TArray<uint8> Mask;
		RasterizePolygonMask(FaceSpacePoly, Inputs.FacesWidth, Inputs.FacesHeight, Mask, Evaluation.CapMaskPixels);
		if (OutMask)
		{
			*OutMask = Mask;
		}
		if (Evaluation.CapMaskPixels <= 0)
		{
			Evaluation.RejectReason = TEXT("source polygon mask is empty in faces image space");
			return Evaluation;
		}

		TMap<int32, FOverlapAccum> AccumByFace;
		for (int32 y = 0; y < Inputs.FacesHeight; ++y)
		{
			for (int32 x = 0; x < Inputs.FacesWidth; ++x)
			{
				const int32 PixIdx = y * Inputs.FacesWidth + x;
				if (Mask[PixIdx] == 0)
				{
					continue;
				}
				const int32 Off = PixIdx * 4;
				const uint32 Key = ColorKey(Inputs.FacesRGBA[Off + 0], Inputs.FacesRGBA[Off + 1], Inputs.FacesRGBA[Off + 2]);
				if (const int32* FaceId = Inputs.FaceIdByColorKey.Find(Key))
				{
					FOverlapAccum& Acc = AccumByFace.FindOrAdd(*FaceId);
					++Acc.Pixels;
					Acc.SumX += double(x) + 0.5;
					Acc.SumY += double(y) + 0.5;
				}
			}
		}

		TArray<FFaceCandidate> Candidates;
		bool bAnyOverlapPass = false;
		bool bAnyAnglePass = false;
		bool bAnyPlaneHit = false;
		for (const TPair<int32, FOverlapAccum>& Pair : AccumByFace)
		{
			const double OverlapRatio = double(Pair.Value.Pixels) / double(Evaluation.CapMaskPixels);
			const int32* FaceIndex = Inputs.FaceIndexById.Find(Pair.Key);
			if (!FaceIndex)
			{
				continue;
			}
			const bool bOverlapPass =
				OverlapRatio >= FFromLZFaceReconstructor::CandidateFaceMinOverlapRatio;
			bAnyOverlapPass |= bOverlapPass;
			const FFaceInfo& Face = Inputs.Faces[*FaceIndex];
			FFaceCandidate Candidate;
			Candidate.FaceId = Pair.Key;
			Candidate.OverlapPixels = Pair.Value.Pixels;
			Candidate.OverlapRatio = OverlapRatio;
			Candidate.MaskCentroid = FVector2D(
				Pair.Value.SumX / double(Pair.Value.Pixels),
				Pair.Value.SumY / double(Pair.Value.Pixels));
			Candidate.bHasPlaneHit = IntersectMaskCentroidWithFacePlane(
				Inputs.Camera, Inputs.FacesWidth, Inputs.FacesHeight,
				Candidate.MaskCentroid, Face, Candidate.PlaneHit, Candidate.DistanceToCamera);
			bAnyPlaneHit |= bOverlapPass && Candidate.bHasPlaneHit;
			if (Candidate.bHasPlaneHit)
			{
				Candidate.bHasProjectedNormal = ProjectFaceNormalToImage(
					Inputs.Camera, Inputs.FacesWidth, Inputs.FacesHeight,
					Face, Candidate.ProjectedNormal2D);
				if (Candidate.bHasProjectedNormal)
				{
					const FVector2D NormalDir = Candidate.ProjectedNormal2D.GetSafeNormal();
					if (bUseAttachDirectedNormalCheck)
					{
						const bool bDirectedAngleValid = EvaluateAttachDirectedNormalAgainstGreenChains(
							Inputs,
							Mask,
							FaceSpaceGreenChains,
							NormalDir,
							Candidate);
						Candidate.bNormalParallelPass =
							bDirectedAngleValid &&
							Candidate.NormalGreenAngleDegrees <= NormalParallelThresholdDegrees;
					}
					else
					{
						double BestAngle = 180.0;
						FVector2D BestOrientedNormalDir = NormalDir;
						for (const FVector2D& GreenDir : FaceSpaceSideVectors)
						{
							const double SignedDot = FVector2D::DotProduct(NormalDir, GreenDir);
							const double Dot = FMath::Clamp(FMath::Abs(SignedDot), 0.0, 1.0);
							const double Angle = FMath::RadiansToDegrees(FMath::Acos(Dot));
							if (Angle < BestAngle)
							{
								BestAngle = Angle;
								BestOrientedNormalDir = SignedDot >= 0.0 ? NormalDir : -NormalDir;
							}
						}
						Candidate.ProjectedNormal2D = BestOrientedNormalDir;
						Candidate.NormalGreenAngleDegrees = BestAngle;
						Candidate.bNormalParallelPass = BestAngle <= NormalParallelThresholdDegrees;
					}
					bAnyAnglePass |= bOverlapPass && Candidate.bNormalParallelPass;
				}
			}
			Candidates.Add(Candidate);
		}

		Candidates.Sort([](const FFaceCandidate& A, const FFaceCandidate& B)
		{
			return A.FaceId < B.FaceId;
		});
		if (OutCandidates)
		{
			*OutCandidates = Candidates;
		}

		const FFaceCandidate* Preferred = nullptr;
		const FFaceCandidate* Fallback = nullptr;
		for (const FFaceCandidate& Candidate : Candidates)
		{
			if (Candidate.OverlapRatio < FFromLZFaceReconstructor::CandidateFaceMinOverlapRatio ||
				!Candidate.bHasPlaneHit ||
				!Candidate.bNormalParallelPass)
			{
				continue;
			}
			if (!Fallback || Candidate.DistanceToCamera < Fallback->DistanceToCamera)
			{
				Fallback = &Candidate;
			}
			if (Candidate.NormalGreenAngleDegrees < PreferredNormalAngleThresholdDegrees &&
				(!Preferred || Candidate.DistanceToCamera < Preferred->DistanceToCamera))
			{
				Preferred = &Candidate;
			}
		}

		const FFaceCandidate* Selected = Preferred ? Preferred : Fallback;
		if (!Selected)
		{
			if (!bAnyOverlapPass)
			{
				Evaluation.RejectReason = FString::Printf(
					TEXT("no face reaches candidate overlap threshold %.3f"),
					FFromLZFaceReconstructor::CandidateFaceMinOverlapRatio);
			}
			else if (!bAnyPlaneHit)
			{
				Evaluation.RejectReason = TEXT("overlap passed but no face had a valid camera-to-plane intersection");
			}
			else if (!bAnyAnglePass)
			{
				Evaluation.RejectReason = FString::Printf(
					TEXT("overlap passed but no face normal is within %.1f degrees of the side vector"),
					NormalParallelThresholdDegrees);
			}
			else
			{
				Evaluation.RejectReason = TEXT("no face passed all candidate face checks");
			}
			Evaluation.EvaluationMode = TEXT("direct_cap_face_rejected");
			return Evaluation;
		}

		Evaluation.bValid = true;
		Evaluation.EvaluationMode = TEXT("direct_cap_face");
		Evaluation.RejectReason = TEXT("selected");
		Evaluation.SelectedFaceId = Selected->FaceId;
		Evaluation.SelectedFaceOverlapPixels = Selected->OverlapPixels;
		Evaluation.SelectedFaceOverlapRatio = Selected->OverlapRatio;
		Evaluation.SelectedFaceNormalSideAngleDegrees = Selected->NormalGreenAngleDegrees;
		Evaluation.SelectedFaceDistanceToCamera = Selected->DistanceToCamera;
		Evaluation.SelectedPlaneHit = Selected->PlaneHit;
		return Evaluation;
	}

	static bool ProjectSignedWorldDirectionToImage(
		const FCameraInfo& Camera, int32 Width, int32 Height,
		const FVector& DirectionWorld, FVector2D& OutDirection)
	{
		return ProjectWorldDirectionToImageOrthographic(Camera, Width, Height, DirectionWorld, OutDirection);
	}

	static void MapPolygonToFacesSpace(
		const TArray<FVector2D>& InPoly, double ScaleX, double ScaleY, TArray<FVector2D>& OutPoly)
	{
		OutPoly.Reset();
		OutPoly.Reserve(InPoly.Num());
		for (const FVector2D& P : InPoly)
		{
			OutPoly.Emplace(P.X * ScaleX, P.Y * ScaleY);
		}
	}

	static void RemoveClosingDuplicatePair(TArray<FVector2D>& Source, TArray<FVector2D>& Copied)
	{
		if (Source.Num() < 2 || Source.Num() != Copied.Num())
		{
			return;
		}
		if ((Source[0] - Source.Last()).SizeSquared() <= 1e-6 &&
			(Copied[0] - Copied.Last()).SizeSquared() <= 1e-6)
		{
			Source.RemoveAt(Source.Num() - 1);
			Copied.RemoveAt(Copied.Num() - 1);
		}
	}

	static bool IsNearlyCollinear2D(const FVector2D& Prev, const FVector2D& Cur, const FVector2D& Next)
	{
		const FVector2D A = Cur - Prev;
		const FVector2D B = Next - Prev;
		const double BLen = B.Size();
		if (BLen < 1e-8)
		{
			return true;
		}

		const double PerpDist = FMath::Abs(FVector2D::CrossProduct(B, A)) / BLen;
		const double Dot = FVector2D::DotProduct(Cur - Prev, Cur - Next);
		return PerpDist <= SolidCollinearTolerancePixels && Dot <= SolidCollinearTolerancePixels * SolidCollinearTolerancePixels;
	}

	static double DistancePointToSegmentSquared2D(const FVector2D& P, const FVector2D& A, const FVector2D& B)
	{
		const FVector2D AB = B - A;
		const double LenSq = AB.SizeSquared();
		if (LenSq < 1e-12)
		{
			return (P - A).SizeSquared();
		}
		const double T = FMath::Clamp(FVector2D::DotProduct(P - A, AB) / LenSq, 0.0, 1.0);
		return (P - (A + AB * T)).SizeSquared();
	}

	static double DistancePointToPolylineSquared2D(const FVector2D& P, const TArray<FVector2D>& Poly, bool bClosed)
	{
		if (Poly.Num() == 0)
		{
			return TNumericLimits<double>::Max();
		}
		if (Poly.Num() == 1)
		{
			return (P - Poly[0]).SizeSquared();
		}

		double Best = TNumericLimits<double>::Max();
		for (int32 i = 0; i + 1 < Poly.Num(); ++i)
		{
			Best = FMath::Min(Best, DistancePointToSegmentSquared2D(P, Poly[i], Poly[i + 1]));
		}
		if (bClosed)
		{
			Best = FMath::Min(Best, DistancePointToSegmentSquared2D(P, Poly.Last(), Poly[0]));
		}
		return Best;
	}

	static bool IsPointInsideOrNearPolygon2D(const FVector2D& P, const TArray<FVector2D>& Poly, double TolPx)
	{
		if (Poly.Num() < 3)
		{
			return false;
		}
		if (PointInPolygon(Poly, P))
		{
			return true;
		}
		return DistancePointToPolylineSquared2D(P, Poly, true) <= TolPx * TolPx;
	}

	static bool AccumulateFaceVotesNearPixel(
		const FCommonInputs& Inputs,
		const FVector2D& Pixel,
		int32 RadiusPx,
		TMap<int32, int32>& InOutVotesByFaceId,
		int32& InOutConsideredPixels,
		TSet<int32>* OutFaceIdsHit = nullptr)
	{
		if (Inputs.FacesWidth <= 0 || Inputs.FacesHeight <= 0 ||
			Inputs.FacesRGBA.Num() < Inputs.FacesWidth * Inputs.FacesHeight * 4)
		{
			return false;
		}

		const int32 CenterX = FMath::RoundToInt(Pixel.X);
		const int32 CenterY = FMath::RoundToInt(Pixel.Y);
		const int32 Radius = FMath::Max(0, RadiusPx);
		bool bAny = false;
		for (int32 Dy = -Radius; Dy <= Radius; ++Dy)
		{
			for (int32 Dx = -Radius; Dx <= Radius; ++Dx)
			{
				const int32 X = CenterX + Dx;
				const int32 Y = CenterY + Dy;
				if (X < 0 || Y < 0 || X >= Inputs.FacesWidth || Y >= Inputs.FacesHeight)
				{
					continue;
				}

				const int32 Off = (Y * Inputs.FacesWidth + X) * 4;
				const uint32 Key = ColorKey(
					Inputs.FacesRGBA[Off + 0],
					Inputs.FacesRGBA[Off + 1],
					Inputs.FacesRGBA[Off + 2]);
				if (const int32* FaceId = Inputs.FaceIdByColorKey.Find(Key))
				{
					++InOutConsideredPixels;
					++InOutVotesByFaceId.FindOrAdd(*FaceId);
					if (OutFaceIdsHit)
					{
						OutFaceIdsHit->Add(*FaceId);
					}
					bAny = true;
				}
			}
		}
		return bAny;
	}

	static double MaxFaceWorldZ(const FFaceInfo& Face)
	{
		double MaxZ = -TNumericLimits<double>::Max();
		for (const FVector& Point : Face.KeyPoints3D)
		{
			MaxZ = FMath::Max(MaxZ, double(Point.Z));
		}
		return MaxZ > -TNumericLimits<double>::Max() ? MaxZ : double(Face.PlanePoint.Z);
	}

	static double AverageFaceWorldZ(const FFaceInfo& Face)
	{
		if (Face.KeyPoints3D.Num() == 0)
		{
			return double(Face.PlanePoint.Z);
		}
		double SumZ = 0.0;
		for (const FVector& Point : Face.KeyPoints3D)
		{
			SumZ += Point.Z;
		}
		return SumZ / double(Face.KeyPoints3D.Num());
	}

	static FVector AverageFaceWorldPoint(const FFaceInfo& Face)
	{
		if (Face.KeyPoints3D.Num() == 0)
		{
			return Face.PlanePoint;
		}
		FVector Sum = FVector::ZeroVector;
		for (const FVector& Point : Face.KeyPoints3D)
		{
			Sum += Point;
		}
		return Sum / double(Face.KeyPoints3D.Num());
	}

	static double ComputeWorstPolygonDistancePx(
		const FFaceInfo& Face,
		const FVector2D& StartFaceSpace,
		const FVector2D& Delta,
		int32 SampleCount)
	{
		double WorstDistancePx = 0.0;
		if (Face.KeyPoints2D.Num() < 3)
		{
			return WorstDistancePx;
		}
		for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
		{
			const double T = SampleCount > 1 ? double(SampleIndex) / double(SampleCount - 1) : 0.0;
			const FVector2D P = StartFaceSpace + Delta * T;
			if (!PointInPolygon(Face.KeyPoints2D, P))
			{
				WorstDistancePx = FMath::Max(
					WorstDistancePx,
					FMath::Sqrt(DistancePointToPolylineSquared2D(P, Face.KeyPoints2D, true)));
			}
		}
		return WorstDistancePx;
	}

	static double ComputeMinCameraDistanceForFaceSamples(
		const FCommonInputs& Inputs,
		const FFaceInfo& Face,
		const TArray<FVector2D>& SamplePixels)
	{
		double BestDistance = TNumericLimits<double>::Max();
		for (const FVector2D& Pixel : SamplePixels)
		{
			FVector HitWorld;
			if (IntersectPixelWithPlaneOrthographic(
				Inputs.Camera,
				Inputs.FacesWidth,
				Inputs.FacesHeight,
				Pixel,
				Face.PlanePoint,
				Face.Normal,
				HitWorld))
			{
				BestDistance = FMath::Min(BestDistance, FVector::Dist(Inputs.Camera.Location, HitWorld));
			}
		}
		if (BestDistance < TNumericLimits<double>::Max())
		{
			return BestDistance;
		}
		return FVector::Dist(Inputs.Camera.Location, AverageFaceWorldPoint(Face));
	}

	static bool VoteSupportFaceForSegment(
		const FCommonInputs& Inputs,
		const FVector2D& StartFaceSpace,
		const FVector2D& EndFaceSpace,
		int32 RadiusPx,
		double SampleStepPx,
		double MinSampleCoverage,
		FSupportFaceVoteResult& OutVote)
	{
		OutVote = FSupportFaceVoteResult();
		const FVector2D Delta = EndFaceSpace - StartFaceSpace;
		const double Length = Delta.Size();
		if (!FMath::IsFinite(Length) || Length < 1e-6)
		{
			OutVote.Error = TEXT("green chain chord is too short");
			return false;
		}

		const double SafeSampleStepPx = FMath::Max(1.0, SampleStepPx);
		const int32 SampleCount = FMath::Clamp(FMath::CeilToInt(Length / SafeSampleStepPx) + 1, 3, 512);
		OutVote.TotalSampleCount = SampleCount;
		TMap<int32, int32> HitSampleCountByFaceId;
		TMap<int32, TArray<FVector2D>> HitSamplePixelsByFaceId;
		for (int32 SampleIndex = 0; SampleIndex < SampleCount; ++SampleIndex)
		{
			const double T = SampleCount > 1 ? double(SampleIndex) / double(SampleCount - 1) : 0.0;
			const FVector2D P = StartFaceSpace + Delta * T;
			TSet<int32> FaceIdsHitThisSample;
			AccumulateFaceVotesNearPixel(
				Inputs,
				P,
				RadiusPx,
				OutVote.VotesByFaceId,
				OutVote.ConsideredPixels,
				&FaceIdsHitThisSample);
			for (int32 FaceId : FaceIdsHitThisSample)
			{
				++HitSampleCountByFaceId.FindOrAdd(FaceId);
				HitSamplePixelsByFaceId.FindOrAdd(FaceId).Add(P);
			}
		}

		if (OutVote.VotesByFaceId.Num() == 0 || OutVote.ConsideredPixels <= 0)
		{
			OutVote.Error = TEXT("green chain did not vote for any captured face");
			return false;
		}

		for (const TPair<int32, int32>& Pair : OutVote.VotesByFaceId)
		{
			const int32 FaceId = Pair.Key;
			const int32* FaceIndex = Inputs.FaceIndexById.Find(FaceId);
			if (!FaceIndex)
			{
				continue;
			}
			const FFaceInfo& Face = Inputs.Faces[*FaceIndex];
			FFromLZSupportFaceVoteCandidate Candidate;
			Candidate.FaceId = FaceId;
			Candidate.VotePixels = Pair.Value;
			Candidate.HitSampleCount = HitSampleCountByFaceId.FindRef(FaceId);
			Candidate.TotalSampleCount = SampleCount;
			Candidate.TotalFaceVotePixels = OutVote.ConsideredPixels;
			Candidate.SampleCoverage = double(Candidate.HitSampleCount) / double(FMath::Max(1, SampleCount));
			Candidate.VotePixelCoverage = double(Candidate.VotePixels) / double(FMath::Max(1, OutVote.ConsideredPixels));
			Candidate.WorldZMax = MaxFaceWorldZ(Face);
			Candidate.WorldZAverage = AverageFaceWorldZ(Face);
			Candidate.MinCameraDistance = ComputeMinCameraDistanceForFaceSamples(
				Inputs,
				Face,
				HitSamplePixelsByFaceId.FindOrAdd(FaceId));
			Candidate.WorstPolygonDistancePx = ComputeWorstPolygonDistancePx(Face, StartFaceSpace, Delta, SampleCount);
			Candidate.bCoveragePass = Candidate.SampleCoverage >= MinSampleCoverage;
			OutVote.FaceCandidates.Add(Candidate);
		}

		if (OutVote.FaceCandidates.Num() == 0)
		{
			OutVote.Error = TEXT("voted support face ids were not in the face table");
			return false;
		}

		int32 BestCandidateIndex = INDEX_NONE;
		for (int32 CandidateIndex = 0; CandidateIndex < OutVote.FaceCandidates.Num(); ++CandidateIndex)
		{
			const FFromLZSupportFaceVoteCandidate& Candidate = OutVote.FaceCandidates[CandidateIndex];
			if (!Candidate.bCoveragePass)
			{
				continue;
			}
			if (BestCandidateIndex == INDEX_NONE)
			{
				BestCandidateIndex = CandidateIndex;
				continue;
			}
			const FFromLZSupportFaceVoteCandidate& Best = OutVote.FaceCandidates[BestCandidateIndex];
			const bool bHigher =
				Candidate.WorldZMax > Best.WorldZMax + 1e-6;
			const bool bTieCloser =
				FMath::IsNearlyEqual(Candidate.WorldZMax, Best.WorldZMax, 1e-6) &&
				Candidate.MinCameraDistance < Best.MinCameraDistance - 1e-6;
			const bool bTieBetterCoverage =
				FMath::IsNearlyEqual(Candidate.WorldZMax, Best.WorldZMax, 1e-6) &&
				FMath::IsNearlyEqual(Candidate.MinCameraDistance, Best.MinCameraDistance, 1e-6) &&
				Candidate.SampleCoverage > Best.SampleCoverage + 1e-6;
			const bool bTieMoreSamples =
				FMath::IsNearlyEqual(Candidate.WorldZMax, Best.WorldZMax, 1e-6) &&
				FMath::IsNearlyEqual(Candidate.MinCameraDistance, Best.MinCameraDistance, 1e-6) &&
				FMath::IsNearlyEqual(Candidate.SampleCoverage, Best.SampleCoverage, 1e-6) &&
				Candidate.HitSampleCount > Best.HitSampleCount;
			if (bHigher || bTieCloser || bTieBetterCoverage || bTieMoreSamples)
			{
				BestCandidateIndex = CandidateIndex;
			}
		}

		if (!OutVote.FaceCandidates.IsValidIndex(BestCandidateIndex))
		{
			OutVote.Error = FString::Printf(
				TEXT("no captured face reached sample coverage threshold %.3f"),
				MinSampleCoverage);
			return false;
		}

		FFromLZSupportFaceVoteCandidate& BestCandidate = OutVote.FaceCandidates[BestCandidateIndex];
		BestCandidate.bSelectedForChain = true;
		OutVote.bFound = true;
		OutVote.FaceId = BestCandidate.FaceId;
		OutVote.VotePixels = BestCandidate.VotePixels;
		OutVote.HitSampleCount = BestCandidate.HitSampleCount;
		OutVote.Coverage = BestCandidate.SampleCoverage;
		OutVote.VotePixelCoverage = BestCandidate.VotePixelCoverage;
		OutVote.WorldZMax = BestCandidate.WorldZMax;
		OutVote.WorldZAverage = BestCandidate.WorldZAverage;
		OutVote.MinCameraDistance = BestCandidate.MinCameraDistance;
		OutVote.WorstPolygonDistancePx = BestCandidate.WorstPolygonDistancePx;
		return true;
	}

	static void MarkRdpSegment(const TArray<FVector2D>& Points, int32 Start, int32 End, double ToleranceSq, TArray<bool>& Keep)
	{
		TArray<TPair<int32, int32>> Stack;
		Stack.Emplace(Start, End);
		Keep[Start] = true;
		Keep[End] = true;

		while (Stack.Num() > 0)
		{
			const TPair<int32, int32> Range = Stack.Pop(EAllowShrinking::No);
			double BestDistSq = -1.0;
			int32 BestIndex = INDEX_NONE;
			for (int32 i = Range.Key + 1; i < Range.Value; ++i)
			{
				const double DistSq = DistancePointToSegmentSquared2D(Points[i], Points[Range.Key], Points[Range.Value]);
				if (DistSq > BestDistSq)
				{
					BestDistSq = DistSq;
					BestIndex = i;
				}
			}

			if (BestIndex != INDEX_NONE && BestDistSq > ToleranceSq)
			{
				Keep[BestIndex] = true;
				Stack.Emplace(Range.Key, BestIndex);
				Stack.Emplace(BestIndex, Range.Value);
			}
		}
	}

	static void SimplifyClosedLoopRdpPairs(TArray<FVector2D>& Source, TArray<FVector2D>& Copied)
	{
		const int32 N = Source.Num();
		if (N <= SolidTargetMaxLoopPoints || N != Copied.Num())
		{
			return;
		}

		int32 Split = 0;
		double BestDistSq = -1.0;
		for (int32 i = 1; i < N; ++i)
		{
			const double DistSq = (Source[i] - Source[0]).SizeSquared();
			if (DistSq > BestDistSq)
			{
				BestDistSq = DistSq;
				Split = i;
			}
		}
		if (Split <= 0 || Split >= N - 1)
		{
			return;
		}

		TArray<bool> Keep;
		Keep.Init(false, N);
		const double ToleranceSq = SolidRdpTolerancePixels * SolidRdpTolerancePixels;
		MarkRdpSegment(Source, 0, Split, ToleranceSq, Keep);
		MarkRdpSegment(Source, Split, N - 1, ToleranceSq, Keep);
		Keep[0] = true;
		Keep[Split] = true;

		TArray<FVector2D> NewSource;
		TArray<FVector2D> NewCopied;
		NewSource.Reserve(N);
		NewCopied.Reserve(N);
		for (int32 i = 0; i < N; ++i)
		{
			if (Keep[i])
			{
				NewSource.Add(Source[i]);
				NewCopied.Add(Copied[i]);
			}
		}

		if (NewSource.Num() >= 3)
		{
			Source = MoveTemp(NewSource);
			Copied = MoveTemp(NewCopied);
		}
	}

	static void DecimateLoopPairsToTarget(TArray<FVector2D>& Source, TArray<FVector2D>& Copied)
	{
		if (Source.Num() <= SolidTargetMaxLoopPoints || Source.Num() != Copied.Num())
		{
			return;
		}

		const int32 N = Source.Num();
		TArray<FVector2D> NewSource;
		TArray<FVector2D> NewCopied;
		NewSource.Reserve(SolidTargetMaxLoopPoints);
		NewCopied.Reserve(SolidTargetMaxLoopPoints);
		int32 LastIndex = INDEX_NONE;
		for (int32 k = 0; k < SolidTargetMaxLoopPoints; ++k)
		{
			const int32 Index = FMath::Clamp(FMath::RoundToInt(double(k) * double(N) / double(SolidTargetMaxLoopPoints)), 0, N - 1);
			if (Index == LastIndex)
			{
				continue;
			}
			NewSource.Add(Source[Index]);
			NewCopied.Add(Copied[Index]);
			LastIndex = Index;
		}
		if (NewSource.Num() >= 3)
		{
			Source = MoveTemp(NewSource);
			Copied = MoveTemp(NewCopied);
		}
	}

	static void SimplifyLoopPairs(TArray<FVector2D>& Source, TArray<FVector2D>& Copied)
	{
		if (Source.Num() != Copied.Num())
		{
			return;
		}

		RemoveClosingDuplicatePair(Source, Copied);

		for (int32 i = Source.Num() - 1; i >= 0 && Source.Num() > 3; --i)
		{
			const int32 Prev = (i + Source.Num() - 1) % Source.Num();
			if ((Source[i] - Source[Prev]).SizeSquared() <= 1e-6 &&
				(Copied[i] - Copied[Prev]).SizeSquared() <= 1e-6)
			{
				Source.RemoveAt(i);
				Copied.RemoveAt(i);
			}
		}

		bool bRemoved = true;
		int32 Safety = 0;
		while (bRemoved && Source.Num() > 3 && ++Safety < 64)
		{
			bRemoved = false;
			for (int32 i = 0; i < Source.Num() && Source.Num() > 3; ++i)
			{
				const int32 Prev = (i + Source.Num() - 1) % Source.Num();
				const int32 Next = (i + 1) % Source.Num();
				if (IsNearlyCollinear2D(Source[Prev], Source[i], Source[Next]) &&
					IsNearlyCollinear2D(Copied[Prev], Copied[i], Copied[Next]))
				{
					Source.RemoveAt(i);
					Copied.RemoveAt(i);
					bRemoved = true;
					--i;
				}
			}
		}

		SimplifyClosedLoopRdpPairs(Source, Copied);
		DecimateLoopPairsToTarget(Source, Copied);
	}

	// Orthographic closed-form extrusion depth:
	// source_to_copied_2d = depth * projected_normal_pixels_per_world.
	static bool SolveExtrusionDepthOrthographic(
		const FCameraInfo& Camera, int32 Width, int32 Height,
		const FVector& OrientedNormal,
		const FVector2D& SourceToCopiedVector2D,
		double& OutDepth,
		FString& OutError)
	{
		FVector2D PixelsPerWorld;
		if (!OrthographicPixelsPerWorldAlongDirection(Camera, Width, Height, OrientedNormal, PixelsPerWorld))
		{
			OutError = TEXT("base face normal has near-zero orthographic image projection; extrusion depth is not observable");
			return false;
		}

		const double ScaleDenom = FVector2D::DotProduct(PixelsPerWorld, PixelsPerWorld);
		if (ScaleDenom < 1e-12)
		{
			OutError = TEXT("orthographic normal image scale is too small");
			return false;
		}

		const double Depth = FVector2D::DotProduct(PixelsPerWorld, SourceToCopiedVector2D) / ScaleDenom;
		if (!FMath::IsFinite(Depth) || Depth <= 1e-4)
		{
			OutError = TEXT("orthographic extrusion depth is not positive");
			return false;
		}

		OutDepth = Depth;
		return true;
	}

	static bool PointInTriangle2D(const FVector2D& P, const FVector2D& A, const FVector2D& B, const FVector2D& C)
	{
		const double D1 = (P.X - B.X) * (A.Y - B.Y) - (A.X - B.X) * (P.Y - B.Y);
		const double D2 = (P.X - C.X) * (B.Y - C.Y) - (B.X - C.X) * (P.Y - C.Y);
		const double D3 = (P.X - A.X) * (C.Y - A.Y) - (C.X - A.X) * (P.Y - A.Y);
		const bool bHasNeg = (D1 < -1e-8) || (D2 < -1e-8) || (D3 < -1e-8);
		const bool bHasPos = (D1 > 1e-8) || (D2 > 1e-8) || (D3 > 1e-8);
		return !(bHasNeg && bHasPos);
	}

	static bool TriangulatePolygon2D(const TArray<FVector2D>& Poly, TArray<int32>& OutTriangles)
	{
		OutTriangles.Reset();
		const int32 N = Poly.Num();
		if (N < 3)
		{
			return false;
		}
		if (N == 3)
		{
			OutTriangles = { 0, 1, 2 };
			return true;
		}

		TArray<int32> Indices;
		Indices.Reserve(N);
		for (int32 i = 0; i < N; ++i)
		{
			Indices.Add(i);
		}

		const double Orient = PolygonArea2D(Poly) >= 0.0 ? 1.0 : -1.0;
		int32 Safety = 0;
		while (Indices.Num() > 3 && ++Safety < N * N)
		{
			bool bClipped = false;
			for (int32 i = 0; i < Indices.Num(); ++i)
			{
				const int32 Prev = Indices[(i + Indices.Num() - 1) % Indices.Num()];
				const int32 Cur = Indices[i];
				const int32 Next = Indices[(i + 1) % Indices.Num()];

				const FVector2D A = Poly[Prev];
				const FVector2D B = Poly[Cur];
				const FVector2D C = Poly[Next];
				const double Cross = FVector2D::CrossProduct(B - A, C - B);
				if (Cross * Orient <= 1e-8)
				{
					continue;
				}

				bool bContainsOther = false;
				for (int32 TestIdx : Indices)
				{
					if (TestIdx == Prev || TestIdx == Cur || TestIdx == Next)
					{
						continue;
					}
					if (PointInTriangle2D(Poly[TestIdx], A, B, C))
					{
						bContainsOther = true;
						break;
					}
				}
				if (bContainsOther)
				{
					continue;
				}

				OutTriangles.Add(Prev);
				OutTriangles.Add(Cur);
				OutTriangles.Add(Next);
				Indices.RemoveAt(i);
				bClipped = true;
				break;
			}

			if (!bClipped)
			{
				OutTriangles.Reset();
				for (int32 i = 1; i + 1 < N; ++i)
				{
					OutTriangles.Add(0);
					OutTriangles.Add(i);
					OutTriangles.Add(i + 1);
				}
				return true;
			}
		}

		if (Indices.Num() == 3)
		{
			OutTriangles.Add(Indices[0]);
			OutTriangles.Add(Indices[1]);
			OutTriangles.Add(Indices[2]);
		}

		return OutTriangles.Num() >= 3;
	}

	static FVector ComputeTriangleNormal(const TArray<FVector>& Vertices, const TArray<int32>& Triangles)
	{
		FVector N = FVector::ZeroVector;
		for (int32 i = 0; i + 2 < Triangles.Num(); i += 3)
		{
			const FVector& A = Vertices[Triangles[i]];
			const FVector& B = Vertices[Triangles[i + 1]];
			const FVector& C = Vertices[Triangles[i + 2]];
			N += FVector::CrossProduct(B - A, C - A);
		}
		return N.GetSafeNormal();
	}

	static FVector AverageVector(const TArray<FVector>& Points)
	{
		FVector Sum = FVector::ZeroVector;
		for (const FVector& P : Points)
		{
			Sum += P;
		}
		return Points.Num() > 0 ? Sum / double(Points.Num()) : FVector::ZeroVector;
	}

	static void ScaleVerticesPerpendicularToAxis(TArray<FVector>& Vertices, const FVector& Axis, double Scale)
	{
		const FVector UnitAxis = Axis.GetSafeNormal();
		if (Vertices.Num() == 0 || UnitAxis.IsNearlyZero() || !FMath::IsFinite(Scale) || FMath::IsNearlyEqual(Scale, 1.0))
		{
			return;
		}

		const FVector Anchor = AverageVector(Vertices);
		const double DeltaScale = Scale - 1.0;
		for (FVector& Vertex : Vertices)
		{
			const FVector Offset = Vertex - Anchor;
			const double ParallelDist = FVector::DotProduct(Offset, UnitAxis);
			const FVector Parallel = UnitAxis * ParallelDist;
			const FVector Perp = Offset - Parallel;
			Vertex = Anchor + Parallel + Perp * (1.0 + DeltaScale);
		}
	}

	static void ScaleVerticesAlongAxis(TArray<FVector>& Vertices, const FVector& Axis, double Scale)
	{
		const FVector UnitAxis = Axis.GetSafeNormal();
		if (Vertices.Num() == 0 || UnitAxis.IsNearlyZero() || !FMath::IsFinite(Scale) || FMath::IsNearlyEqual(Scale, 1.0))
		{
			return;
		}

		const FVector Anchor = AverageVector(Vertices);
		const double DeltaScale = Scale - 1.0;
		for (FVector& Vertex : Vertices)
		{
			const double AxisDistance = FVector::DotProduct(Vertex - Anchor, UnitAxis);
			Vertex += UnitAxis * (AxisDistance * DeltaScale);
		}
	}

	static FVector2D AverageVector2DDelta(const TArray<FVector2D>& Source, const TArray<FVector2D>& Copied)
	{
		FVector2D Sum = FVector2D::ZeroVector;
		const int32 Count = FMath::Min(Source.Num(), Copied.Num());
		for (int32 i = 0; i < Count; ++i)
		{
			Sum += Copied[i] - Source[i];
		}
		return Count > 0 ? Sum / double(Count) : FVector2D::ZeroVector;
	}

	static void DrawPointRGBA(TArray<uint8>& RGBA, int32 Width, int32 Height, int32 X, int32 Y, const FColor& Color, int32 Radius)
	{
		for (int32 Dy = -Radius; Dy <= Radius; ++Dy)
		{
			for (int32 Dx = -Radius; Dx <= Radius; ++Dx)
			{
				const int32 PX = X + Dx;
				const int32 PY = Y + Dy;
				if (PX < 0 || PY < 0 || PX >= Width || PY >= Height)
				{
					continue;
				}
				const int32 Off = (PY * Width + PX) * 4;
				RGBA[Off + 0] = Color.R;
				RGBA[Off + 1] = Color.G;
				RGBA[Off + 2] = Color.B;
				RGBA[Off + 3] = 255;
			}
		}
	}

	static void DrawLineRGBA(TArray<uint8>& RGBA, int32 Width, int32 Height, const FVector2D& A, const FVector2D& B, const FColor& Color, int32 Radius)
	{
		const double Dx = B.X - A.X;
		const double Dy = B.Y - A.Y;
		const int32 Steps = FMath::Max(1, FMath::CeilToInt(FMath::Max(FMath::Abs(Dx), FMath::Abs(Dy))));
		for (int32 i = 0; i <= Steps; ++i)
		{
			const double T = double(i) / double(Steps);
			const int32 X = FMath::RoundToInt(A.X + Dx * T);
			const int32 Y = FMath::RoundToInt(A.Y + Dy * T);
			DrawPointRGBA(RGBA, Width, Height, X, Y, Color, Radius);
		}
	}

	static void DrawClosedPolylineRGBA(TArray<uint8>& RGBA, int32 Width, int32 Height, const TArray<FVector2D>& Points, const FColor& Color, int32 Radius)
	{
		if (Points.Num() < 2)
		{
			return;
		}
		for (int32 i = 0; i < Points.Num(); ++i)
		{
			DrawLineRGBA(RGBA, Width, Height, Points[i], Points[(i + 1) % Points.Num()], Color, Radius);
		}
	}

	static void DrawOpenPolylineRGBA(TArray<uint8>& RGBA, int32 Width, int32 Height, const TArray<FVector2D>& Points, const FColor& Color, int32 Radius)
	{
		for (int32 Index = 1; Index < Points.Num(); ++Index)
		{
			DrawLineRGBA(RGBA, Width, Height, Points[Index - 1], Points[Index], Color, Radius);
		}
	}

	static TSharedPtr<FJsonValue> JsonVector2D(const FVector2D& V)
	{
		TArray<TSharedPtr<FJsonValue>> Arr;
		Arr.Add(MakeShared<FJsonValueNumber>(V.X));
		Arr.Add(MakeShared<FJsonValueNumber>(V.Y));
		return MakeShared<FJsonValueArray>(Arr);
	}

	static TSharedPtr<FJsonValue> JsonVector(const FVector& V)
	{
		TArray<TSharedPtr<FJsonValue>> Arr;
		Arr.Add(MakeShared<FJsonValueNumber>(V.X));
		Arr.Add(MakeShared<FJsonValueNumber>(V.Y));
		Arr.Add(MakeShared<FJsonValueNumber>(V.Z));
		return MakeShared<FJsonValueArray>(Arr);
	}

	static TSharedPtr<FJsonValue> JsonIntTriple(int32 A, int32 B, int32 C)
	{
		TArray<TSharedPtr<FJsonValue>> Arr;
		Arr.Add(MakeShared<FJsonValueNumber>(A));
		Arr.Add(MakeShared<FJsonValueNumber>(B));
		Arr.Add(MakeShared<FJsonValueNumber>(C));
		return MakeShared<FJsonValueArray>(Arr);
	}

	static void SetVectorArrayField(TSharedRef<FJsonObject> Object, const TCHAR* Key, const TArray<FVector>& Values)
	{
		TArray<TSharedPtr<FJsonValue>> JsonValues;
		JsonValues.Reserve(Values.Num());
		for (const FVector& V : Values)
		{
			JsonValues.Add(JsonVector(V));
		}
		Object->SetArrayField(Key, JsonValues);
	}

	static void SetVector2DArrayField(TSharedRef<FJsonObject> Object, const TCHAR* Key, const TArray<FVector2D>& Values)
	{
		TArray<TSharedPtr<FJsonValue>> JsonValues;
		JsonValues.Reserve(Values.Num());
		for (const FVector2D& V : Values)
		{
			JsonValues.Add(JsonVector2D(V));
		}
		Object->SetArrayField(Key, JsonValues);
	}

	static void SetIntArrayField(TSharedRef<FJsonObject> Object, const TCHAR* Key, const TArray<int32>& Values)
	{
		TArray<TSharedPtr<FJsonValue>> JsonValues;
		JsonValues.Reserve(Values.Num());
		for (int32 Value : Values)
		{
			JsonValues.Add(MakeShared<FJsonValueNumber>(Value));
		}
		Object->SetArrayField(Key, JsonValues);
	}

	static void SetDoubleArrayField(
		TSharedRef<FJsonObject> Object,
		const TCHAR* Key,
		const TArray<double>& Values)
	{
		TArray<TSharedPtr<FJsonValue>> JsonValues;
		JsonValues.Reserve(Values.Num());
		for (double Value : Values)
		{
			JsonValues.Add(MakeShared<FJsonValueNumber>(Value));
		}
		Object->SetArrayField(Key, JsonValues);
	}

	static void SetIntPointArrayField(TSharedRef<FJsonObject> Object, const TCHAR* Key, const TArray<FIntPoint>& Values)
	{
		TArray<TSharedPtr<FJsonValue>> JsonValues;
		JsonValues.Reserve(Values.Num());
		for (const FIntPoint& Value : Values)
		{
			TArray<TSharedPtr<FJsonValue>> Pair;
			Pair.Add(MakeShared<FJsonValueNumber>(Value.X));
			Pair.Add(MakeShared<FJsonValueNumber>(Value.Y));
			JsonValues.Add(MakeShared<FJsonValueArray>(Pair));
		}
		Object->SetArrayField(Key, JsonValues);
	}

	static TSharedRef<FJsonObject> MakeSupportTopologyDebugJson(const FSupportPlaneTopologyDebug& Debug)
	{
		TSharedRef<FJsonObject> Obj = MakeShared<FJsonObject>();
		Obj->SetBoolField(TEXT("checked"), Debug.bChecked);
		Obj->SetBoolField(TEXT("pass"), Debug.bPass);
		Obj->SetStringField(TEXT("stage"), Debug.Stage);
		Obj->SetStringField(TEXT("reason"), Debug.Reason);
		Obj->SetNumberField(TEXT("source_vertex_count"), Debug.SourceVertexCount);
		Obj->SetNumberField(TEXT("world_vertex_count"), Debug.WorldVertexCount);
		Obj->SetNumberField(TEXT("source_area_px2"), Debug.SourceAreaPx2);
		Obj->SetNumberField(TEXT("projected_area_cm2"), Debug.ProjectedAreaCm2);
		Obj->SetNumberField(TEXT("min_source_edge_px"), Debug.MinSourceEdgePx);
		Obj->SetNumberField(TEXT("min_projected_edge_cm"), Debug.MinProjectedEdgeCm);
		SetVector2DArrayField(Obj, TEXT("projected_uv_loop"), Debug.ProjectedUVLoop);
		SetIntPointArrayField(Obj, TEXT("source_self_intersection_edges"), Debug.SourceSelfIntersectionEdges);
		SetIntPointArrayField(Obj, TEXT("projected_self_intersection_edges"), Debug.ProjectedSelfIntersectionEdges);
		return Obj;
	}

	static TSharedRef<FJsonObject> MakeSupportFaceVoteCandidateJson(const FFromLZSupportFaceVoteCandidate& Candidate)
	{
		TSharedRef<FJsonObject> Obj = MakeShared<FJsonObject>();
		Obj->SetNumberField(TEXT("face_id"), Candidate.FaceId);
		Obj->SetNumberField(TEXT("vote_pixels"), Candidate.VotePixels);
		Obj->SetNumberField(TEXT("hit_sample_count"), Candidate.HitSampleCount);
		Obj->SetNumberField(TEXT("total_sample_count"), Candidate.TotalSampleCount);
		Obj->SetNumberField(TEXT("total_face_vote_pixels"), Candidate.TotalFaceVotePixels);
		Obj->SetNumberField(TEXT("sample_coverage"), Candidate.SampleCoverage);
		Obj->SetNumberField(TEXT("vote_pixel_coverage"), Candidate.VotePixelCoverage);
		Obj->SetNumberField(TEXT("world_z_max"), Candidate.WorldZMax);
		Obj->SetNumberField(TEXT("world_z_average"), Candidate.WorldZAverage);
		Obj->SetNumberField(TEXT("min_camera_distance"), Candidate.MinCameraDistance);
		Obj->SetNumberField(TEXT("worst_polygon_distance_px"), Candidate.WorstPolygonDistancePx);
		Obj->SetBoolField(TEXT("coverage_pass"), Candidate.bCoveragePass);
		Obj->SetBoolField(TEXT("selected_for_chain"), Candidate.bSelectedForChain);
		return Obj;
	}

	static void SetSupportFaceVoteCandidatesArray(
		TSharedRef<FJsonObject> Object,
		const TCHAR* Key,
		const TArray<FFromLZSupportFaceVoteCandidate>& Candidates)
	{
		TArray<TSharedPtr<FJsonValue>> Values;
		Values.Reserve(Candidates.Num());
		for (const FFromLZSupportFaceVoteCandidate& Candidate : Candidates)
		{
			Values.Add(MakeShared<FJsonValueObject>(MakeSupportFaceVoteCandidateJson(Candidate)));
		}
		Object->SetArrayField(Key, Values);
	}

	static void SetSupportFaceVoteAttemptDebugFields(
		TSharedRef<FJsonObject> Object,
		int32 VotePixels,
		int32 ConsideredPixels,
		int32 HitSampleCount,
		int32 TotalSampleCount,
		double SampleCoverage,
		double VotePixelCoverage,
		double WorldZMax,
		double WorldZAverage,
		double MinCameraDistance,
		double WorstPolygonDistancePx,
		const TArray<FFromLZSupportFaceVoteCandidate>& Candidates)
	{
		Object->SetNumberField(TEXT("support_vote_pixels"), VotePixels);
		Object->SetNumberField(TEXT("support_considered_pixels"), ConsideredPixels);
		Object->SetNumberField(TEXT("support_hit_sample_count"), HitSampleCount);
		Object->SetNumberField(TEXT("support_total_sample_count"), TotalSampleCount);
		Object->SetNumberField(TEXT("support_vote_coverage"), SampleCoverage);
		Object->SetNumberField(TEXT("support_sample_coverage"), SampleCoverage);
		Object->SetNumberField(TEXT("support_vote_pixel_coverage"), VotePixelCoverage);
		Object->SetNumberField(TEXT("support_face_world_z_max"), WorldZMax);
		Object->SetNumberField(TEXT("support_face_world_z_average"), WorldZAverage);
		Object->SetNumberField(TEXT("support_min_camera_distance"), MinCameraDistance);
		Object->SetNumberField(TEXT("support_worst_polygon_distance_px"), WorstPolygonDistancePx);
		SetSupportFaceVoteCandidatesArray(Object, TEXT("support_face_candidates"), Candidates);
	}

	static void SetTriangleArrayField(TSharedRef<FJsonObject> Object, const TCHAR* Key, const TArray<int32>& Triangles)
	{
		TArray<TSharedPtr<FJsonValue>> JsonValues;
		for (int32 i = 0; i + 2 < Triangles.Num(); i += 3)
		{
			JsonValues.Add(JsonIntTriple(Triangles[i], Triangles[i + 1], Triangles[i + 2]));
		}
		Object->SetArrayField(Key, JsonValues);
	}

	static void SaveSolidResultJson(const FSolidReconstructionResult& Result, const FString& Path)
	{
		TSharedRef<FJsonObject> Root = MakeShared<FJsonObject>();
		Root->SetStringField(TEXT("component"), Result.ComponentName);
		Root->SetStringField(TEXT("action"), Result.Action);
		Root->SetStringField(TEXT("source_polygon_key"), Result.SourcePolygonKey);
		Root->SetStringField(TEXT("copied_polygon_key"), Result.CopiedPolygonKey);
		Root->SetBoolField(TEXT("success"), Result.bSuccess);
		Root->SetStringField(TEXT("error"), Result.Error);
		Root->SetStringField(TEXT("warning"), Result.Warning);
		Root->SetStringField(TEXT("actor_name"), Result.ActorName);
		Root->SetStringField(TEXT("reconstruction_method"), Result.ReconstructionMethod);
		Root->SetBoolField(TEXT("attach_support_plane_fallback"), Result.bAttachSupportPlaneFallback);
		Root->SetBoolField(TEXT("forced_support_plane_output"), Result.bForcedSupportPlaneOutput);
		Root->SetStringField(TEXT("forced_support_plane_reason"), Result.ForcedSupportPlaneReason);
		Root->SetNumberField(TEXT("forced_support_plane_attempt_index"), Result.ForcedSupportPlaneAttemptIndex);
		Root->SetNumberField(TEXT("cap_width"), Result.CapWidth);
		Root->SetNumberField(TEXT("cap_height"), Result.CapHeight);
		Root->SetNumberField(TEXT("faces_width"), Result.FacesWidth);
		Root->SetNumberField(TEXT("faces_height"), Result.FacesHeight);
		Root->SetNumberField(TEXT("selected_source_face_id"), Result.SelectedFaceId);
		Root->SetNumberField(TEXT("support_face_id"), Result.SupportFaceId);
		Root->SetNumberField(TEXT("support_green_chain_index"), Result.SupportGreenChainIndex);
		Root->SetBoolField(TEXT("projection_matrix_available"), Result.bHasProjectionMatrix);
		Root->SetStringField(TEXT("projection_matrix_source"), Result.ProjectionMatrixSource);
		Root->SetStringField(TEXT("projection_validation"), Result.ProjectionValidation);
		Root->SetNumberField(TEXT("projection_viewport_width"), Result.ProjectionViewportWidth);
		Root->SetNumberField(TEXT("projection_viewport_height"), Result.ProjectionViewportHeight);
		Root->SetNumberField(TEXT("projection_m00"), Result.ProjectionM00);
		Root->SetNumberField(TEXT("projection_m11"), Result.ProjectionM11);
		Root->SetNumberField(TEXT("projection_m30"), Result.ProjectionM30);
		Root->SetNumberField(TEXT("projection_m31"), Result.ProjectionM31);
		Root->SetNumberField(TEXT("projection_horizontal_span"), Result.ProjectionHorizontalSpan);
		Root->SetNumberField(TEXT("projection_vertical_span"), Result.ProjectionVerticalSpan);
		Root->SetBoolField(TEXT("attach_material_id_lookup_attempted"), Result.AttachMaterialId.bLookupAttempted);
		Root->SetBoolField(TEXT("attach_material_id_found"), Result.AttachMaterialId.bFound);
		Root->SetStringField(TEXT("attach_material_id_error"), Result.AttachMaterialId.Error);
		if (Result.AttachMaterialId.bFound)
		{
			Root->SetNumberField(TEXT("attach_material_id_color_key"), double(Result.AttachMaterialId.Entry.ColorKey));
			Root->SetStringField(TEXT("attach_material_actor_name"), Result.AttachMaterialId.Entry.ActorName);
			Root->SetStringField(TEXT("attach_material_actor_path"), Result.AttachMaterialId.Entry.ActorPath);
			Root->SetStringField(TEXT("attach_material_component_name"), Result.AttachMaterialId.Entry.ComponentName);
			Root->SetStringField(TEXT("attach_material_component_path"), Result.AttachMaterialId.Entry.ComponentPath);
			Root->SetNumberField(TEXT("attach_material_slot"), Result.AttachMaterialId.Entry.MaterialSlot);
			Root->SetStringField(TEXT("attach_material_name"), Result.AttachMaterialId.Entry.MaterialName);
			Root->SetStringField(TEXT("attach_material_path"), Result.AttachMaterialId.Entry.MaterialPath);
			Root->SetNumberField(TEXT("attach_material_vote_pixels"), Result.AttachMaterialId.PixelCount);
			Root->SetNumberField(TEXT("attach_material_considered_pixels"), Result.AttachMaterialId.ConsideredPixelCount);
			Root->SetNumberField(TEXT("attach_material_vote_coverage"), Result.AttachMaterialId.Coverage);
		}
		Root->SetArrayField(TEXT("source_plane_point"), JsonVector(Result.SourcePlanePoint)->AsArray());
		Root->SetArrayField(TEXT("source_plane_normal"), JsonVector(Result.SourcePlaneNormal)->AsArray());
		Root->SetArrayField(TEXT("support_plane_point"), JsonVector(Result.SupportPlanePoint)->AsArray());
		Root->SetArrayField(TEXT("support_plane_normal"), JsonVector(Result.SupportPlaneNormal)->AsArray());
		Root->SetArrayField(TEXT("extrusion_vector_world"), JsonVector(Result.ExtrusionVectorWorld)->AsArray());
		Root->SetArrayField(TEXT("oriented_normal_source_to_copied"), JsonVector(Result.OrientedNormal)->AsArray());
		Root->SetArrayField(TEXT("source_to_copied_vector_2d"), JsonVector2D(Result.SourceToCopiedVector2D)->AsArray());
		Root->SetArrayField(TEXT("projected_oriented_normal_2d"), JsonVector2D(Result.ProjectedNormal2D)->AsArray());
		Root->SetNumberField(TEXT("extrusion_depth"), Result.ExtrusionDepth);
		Root->SetNumberField(TEXT("max_depth_sample_reprojection_error_pixels"), Result.MaxDepthSampleReprojectionErrorPixels);
		Root->SetNumberField(TEXT("mean_source_reprojection_error_pixels"), Result.MeanSourceReprojectionErrorPixels);
		Root->SetNumberField(TEXT("max_source_reprojection_error_pixels"), Result.MaxSourceReprojectionErrorPixels);
		Root->SetNumberField(TEXT("mean_copied_reprojection_error_pixels"), Result.MeanCopiedReprojectionErrorPixels);
		Root->SetNumberField(TEXT("max_copied_reprojection_error_pixels"), Result.MaxCopiedReprojectionErrorPixels);

		TArray<TSharedPtr<FJsonValue>> SampleValues;
		for (const FSolidDepthSample& Sample : Result.DepthSamples)
		{
			TSharedRef<FJsonObject> Obj = MakeShared<FJsonObject>();
			Obj->SetNumberField(TEXT("index"), Sample.Index);
			Obj->SetBoolField(TEXT("valid"), Sample.bValid);
			Obj->SetStringField(TEXT("error"), Sample.Error);
			Obj->SetArrayField(TEXT("source_pixel_2d"), JsonVector2D(Sample.SourcePixel)->AsArray());
			Obj->SetArrayField(TEXT("copied_pixel_2d"), JsonVector2D(Sample.CopiedPixel)->AsArray());
			Obj->SetArrayField(TEXT("source_world"), JsonVector(Sample.SourceWorld)->AsArray());
			Obj->SetArrayField(TEXT("point_on_extrusion"), JsonVector(Sample.PointOnExtrusion)->AsArray());
			Obj->SetArrayField(TEXT("point_on_copied_ray"), JsonVector(Sample.PointOnRay)->AsArray());
			Obj->SetNumberField(TEXT("depth"), Sample.Depth);
			Obj->SetNumberField(TEXT("closest_world_distance"), Sample.ClosestWorldDistance);
			Obj->SetNumberField(TEXT("reprojection_error_pixels"), Sample.ReprojectionErrorPixels);
			SampleValues.Add(MakeShared<FJsonValueObject>(Obj));
		}
		Root->SetArrayField(TEXT("depth_samples"), SampleValues);

		SetVector2DArrayField(Root, TEXT("source_loop_2d"), Result.SourceLoop2D);
		SetVector2DArrayField(Root, TEXT("copied_target_loop_2d"), Result.CopiedTargetLoop2D);
		SetVector2DArrayField(Root, TEXT("reprojected_source_loop_2d"), Result.ReprojectedSourceLoop2D);
		SetVector2DArrayField(Root, TEXT("reprojected_copied_loop_2d"), Result.ReprojectedCopiedLoop2D);
		SetVectorArrayField(Root, TEXT("source_loop_world"), Result.SourceLoopWorld);
		SetVectorArrayField(Root, TEXT("copied_loop_world"), Result.CopiedLoopWorld);
		SetVectorArrayField(Root, TEXT("mesh_vertices_world"), Result.MeshVerticesWorld);
		SetTriangleArrayField(Root, TEXT("mesh_triangles"), Result.MeshTriangles);
		Root->SetBoolField(TEXT("cap_bbox_regularization_attempted"), Result.CapBBoxRegularization.bAttempted);
		Root->SetBoolField(TEXT("cap_bbox_regularization_applied"), Result.CapBBoxRegularization.bApplied);
		Root->SetBoolField(TEXT("cap_bbox_regularization_fallback_to_original"), Result.CapBBoxRegularization.bFallbackToOriginal);
		Root->SetStringField(TEXT("cap_bbox_regularization_reason"), Result.CapBBoxRegularization.RejectionReason);
		Root->SetStringField(TEXT("cap_bbox_selected_geometry"), Result.CapBBoxRegularization.SelectedGeometry);
		Root->SetNumberField(TEXT("cap_bbox_fill_ratio"), Result.CapBBoxRegularization.CapBaseBBoxRatio);
		Root->SetNumberField(TEXT("cap_bbox_fill_ratio_threshold"), Result.CapBBoxRegularization.FillRatioThreshold);
		Root->SetNumberField(TEXT("cap_base_bbox_ratio"), Result.CapBBoxRegularization.CapBaseBBoxRatio);
		Root->SetNumberField(TEXT("cap_minimum_bbox_ratio"), Result.CapBBoxRegularization.CapMinimumBBoxRatio);
		Root->SetBoolField(TEXT("cap_world_orthogonal_attempted"), Result.CapBBoxRegularization.bAttempted);
		Root->SetBoolField(TEXT("cap_world_orthogonal_applied"), Result.CapBBoxRegularization.bApplied);
		Root->SetBoolField(TEXT("cap_world_orthogonal_fallback_to_original"), Result.CapBBoxRegularization.bFallbackToOriginal);
		Root->SetBoolField(TEXT("cap_world_orthogonal_contains_black"), Result.CapBBoxRegularization.bContainsBlack);
		Root->SetStringField(TEXT("cap_world_orthogonal_reason"), Result.CapBBoxRegularization.RejectionReason);
		Root->SetStringField(TEXT("cap_world_orthogonal_selected_geometry"), Result.CapBBoxRegularization.SelectedGeometry);
		Root->SetNumberField(TEXT("cap_world_orthogonal_boundary_run_count"), Result.CapBBoxRegularization.BoundaryRunCount);
		Root->SetNumberField(TEXT("cap_world_orthogonal_primitive_geometry_edge_count"), Result.CapBBoxRegularization.PrimitiveGeometryEdgeCount);
		Root->SetNumberField(TEXT("cap_world_orthogonal_geometry_edge_count"), Result.CapBBoxRegularization.GeometryEdgeCount);
		Root->SetNumberField(TEXT("cap_world_orthogonal_ignored_connector_count"), Result.CapBBoxRegularization.IgnoredConnectorCount);
		Root->SetNumberField(TEXT("cap_world_orthogonal_area_ratio"), Result.CapBBoxRegularization.WorldOrthogonalAreaRatio);
		Root->SetNumberField(TEXT("cap_world_orthogonal_mean_boundary_distance"), Result.CapBBoxRegularization.WorldOrthogonalMeanBoundaryDistance);
		Root->SetNumberField(TEXT("cap_world_orthogonal_max_boundary_distance"), Result.CapBBoxRegularization.WorldOrthogonalMaxBoundaryDistance);
		Root->SetBoolField(
			TEXT("cap_world_orthogonal_pure_red_root_alignment_attempted"),
			Result.CapBBoxRegularization.bPureRedRootAlignmentAttempted);
		Root->SetNumberField(
			TEXT("cap_world_orthogonal_root_hypothesis_count"),
			Result.CapBBoxRegularization.WorldOrthogonalRootHypotheses.Num());
		Root->SetNumberField(
			TEXT("cap_world_orthogonal_selected_root_hypothesis_index"),
			Result.CapBBoxRegularization.SelectedRootHypothesisIndex);
		Root->SetNumberField(
			TEXT("cap_world_orthogonal_selected_root_primitive_edge_index"),
			Result.CapBBoxRegularization.SelectedRootPrimitiveEdgeIndex);
		SetIntArrayField(
			Root,
			TEXT("cap_world_orthogonal_selected_root_stroke_ids"),
			Result.CapBBoxRegularization.SelectedRootStrokeIds);
		Root->SetStringField(
			TEXT("cap_world_orthogonal_selected_root_target_axis"),
			Result.CapBBoxRegularization.SelectedRootTargetAxisTypeName);
		Root->SetNumberField(
			TEXT("cap_world_orthogonal_selected_root_rotation_degrees"),
			Result.CapBBoxRegularization.SelectedRootRotationDegrees);

		SaveJsonObject(Root, Path);
	}

	static bool SaveSolidProjectionCheckPng(
		const FSolidReconstructionResult& Result,
		const TArray<uint8>& FacesRGBA, int32 Width, int32 Height, const FString& Path)
	{
		if (FacesRGBA.Num() < Width * Height * 4 || !Result.bSuccess)
		{
			return false;
		}

		TArray<uint8> RGBA = FacesRGBA;
		DrawClosedPolylineRGBA(RGBA, Width, Height, Result.SourceLoop2D, FColor(0, 180, 255, 255), 1);
		DrawClosedPolylineRGBA(RGBA, Width, Height, Result.CopiedTargetLoop2D, FColor(255, 140, 0, 255), 1);
		DrawClosedPolylineRGBA(RGBA, Width, Height, Result.ReprojectedSourceLoop2D, FColor(0, 255, 80, 255), 2);
		DrawClosedPolylineRGBA(RGBA, Width, Height, Result.ReprojectedCopiedLoop2D, FColor(255, 0, 180, 255), 2);
		return SaveRGBAToPng(RGBA, Width, Height, Path);
	}

	static void DrawArrowRGBA(TArray<uint8>& RGBA, int32 Width, int32 Height, const FVector2D& A, const FVector2D& B, const FColor& Color, int32 Radius)
	{
		DrawLineRGBA(RGBA, Width, Height, A, B, Color, Radius);
		FVector2D Dir = B - A;
		if (Dir.SizeSquared() < 1e-6)
		{
			return;
		}
		Dir.Normalize();
		const FVector2D Perp(-Dir.Y, Dir.X);
		const double HeadLen = 20.0;
		const double HeadHalf = 10.0;
		const FVector2D Base = B - Dir * HeadLen;
		DrawLineRGBA(RGBA, Width, Height, B, Base + Perp * HeadHalf, Color, Radius);
		DrawLineRGBA(RGBA, Width, Height, B, Base - Perp * HeadHalf, Color, Radius);
	}

	static void SaveCandidateFaceValidationDebug(
		const FFromLZCandidateFaceRequest& Request,
		const FFromLZCandidateFaceEvaluation& Evaluation,
		const FFromLZFaceReconstructionParams& Params,
		const FCommonInputs& Inputs,
		int32 SourceWidth,
		int32 SourceHeight,
		const TArray<uint8>& Mask,
		const TArray<FFaceCandidate>& Candidates,
		const FString& OutputDir)
	{
		IFileManager::Get().MakeDirectory(*OutputDir, true);

		const TArray<FVector2D>& SourcePolygon =
			Evaluation.SourcePolygonKey == TEXT("cap_polygon_translated")
				? Request.CapPolygonTranslated
				: Request.CapPolygon;
		const double ScaleX = SourceWidth > 0 ? double(Inputs.FacesWidth) / double(SourceWidth) : 1.0;
		const double ScaleY = SourceHeight > 0 ? double(Inputs.FacesHeight) / double(SourceHeight) : 1.0;
		TArray<FVector2D> FaceSpacePolygon;
		MapPolygonToFacesSpace(SourcePolygon, ScaleX, ScaleY, FaceSpacePolygon);

		if (Inputs.FacesRGBA.Num() >= Inputs.FacesWidth * Inputs.FacesHeight * 4)
		{
			TArray<uint8> RGBA = Inputs.FacesRGBA;
			TMap<int32, const FFaceCandidate*> CandidateByFaceId;
			for (const FFaceCandidate& Candidate : Candidates)
			{
				CandidateByFaceId.Add(Candidate.FaceId, &Candidate);
			}

			if (Mask.Num() >= Inputs.FacesWidth * Inputs.FacesHeight)
			{
				for (int32 Pixel = 0; Pixel < Inputs.FacesWidth * Inputs.FacesHeight; ++Pixel)
				{
					if (Mask[Pixel] == 0)
					{
						continue;
					}

					const int32 Offset = Pixel * 4;
					const uint32 Key = ColorKey(RGBA[Offset + 0], RGBA[Offset + 1], RGBA[Offset + 2]);
					const int32* FaceId = Inputs.FaceIdByColorKey.Find(Key);
					FColor Overlay(230, 30, 50, 255);
					if (FaceId)
					{
						if (const FFaceCandidate* const* Found = CandidateByFaceId.Find(*FaceId))
						{
							const FFaceCandidate& Candidate = **Found;
							if (Candidate.FaceId == Evaluation.SelectedFaceId)
							{
								Overlay = FColor(40, 230, 80, 255);
							}
							else if (Candidate.OverlapRatio < FFromLZFaceReconstructor::CandidateFaceMinOverlapRatio)
							{
								Overlay = FColor(230, 30, 50, 255);
							}
							else if (!Candidate.bHasPlaneHit || !Candidate.bNormalParallelPass)
							{
								Overlay = FColor(255, 205, 30, 255);
							}
							else
							{
								Overlay = FColor(0, 200, 255, 255);
							}
						}
					}

					RGBA[Offset + 0] = BlendChannel(RGBA[Offset + 0], Overlay.R, 0.70);
					RGBA[Offset + 1] = BlendChannel(RGBA[Offset + 1], Overlay.G, 0.70);
					RGBA[Offset + 2] = BlendChannel(RGBA[Offset + 2], Overlay.B, 0.70);
					RGBA[Offset + 3] = 255;
				}
			}

			DrawClosedPolylineRGBA(
				RGBA, Inputs.FacesWidth, Inputs.FacesHeight,
				FaceSpacePolygon, FColor::White, 2);

			FVector2D PolygonCentroid = FVector2D::ZeroVector;
			for (const FVector2D& Point : FaceSpacePolygon)
			{
				PolygonCentroid += Point;
			}
			if (FaceSpacePolygon.Num() > 0)
			{
				PolygonCentroid /= double(FaceSpacePolygon.Num());
			}

			for (const FVector2D& SideVector : Request.SideVectors)
			{
				const FVector2D ScaledSide(SideVector.X * ScaleX, SideVector.Y * ScaleY);
				if (ScaledSide.SizeSquared() > 1e-8)
				{
					DrawArrowRGBA(
						RGBA, Inputs.FacesWidth, Inputs.FacesHeight,
						PolygonCentroid, PolygonCentroid + ScaledSide,
						FColor(0, 255, 80, 255), 2);
				}
			}
			for (int32 ChainIndex = 0; ChainIndex < Request.GreenChains.Num(); ++ChainIndex)
			{
				const FFromLZGreenChainCandidate2D& Chain = Request.GreenChains[ChainIndex];
				const FVector2D ChainStart(Chain.Start.X * ScaleX, Chain.Start.Y * ScaleY);
				const FVector2D ChainEnd(Chain.End.X * ScaleX, Chain.End.Y * ScaleY);
				if ((ChainEnd - ChainStart).SizeSquared() > 1e-8)
				{
					const bool bSelectedSupportChain = ChainIndex == Evaluation.SupportGreenChainIndex;
					DrawArrowRGBA(
						RGBA, Inputs.FacesWidth, Inputs.FacesHeight,
						ChainStart,
						ChainEnd,
						bSelectedSupportChain ? FColor(0, 255, 255, 255) : FColor(30, 180, 30, 255),
						bSelectedSupportChain ? 3 : 1);
				}
			}

			const double NormalArrowLength = 110.0;
			for (const FFaceCandidate& Candidate : Candidates)
			{
				if (!Candidate.bHasProjectedNormal)
				{
					DrawPointRGBA(
						RGBA, Inputs.FacesWidth, Inputs.FacesHeight,
						FMath::RoundToInt(Candidate.MaskCentroid.X),
						FMath::RoundToInt(Candidate.MaskCentroid.Y),
						FColor(255, 205, 30, 255), 4);
					continue;
				}

				FColor ArrowColor(230, 30, 50, 255);
				if (Candidate.FaceId == Evaluation.SelectedFaceId)
				{
					ArrowColor = FColor(40, 230, 80, 255);
				}
				else if (Candidate.OverlapRatio >= FFromLZFaceReconstructor::CandidateFaceMinOverlapRatio &&
					Candidate.bHasPlaneHit &&
					Candidate.bNormalParallelPass)
				{
					ArrowColor = FColor(0, 200, 255, 255);
				}
				else if (Candidate.OverlapRatio >= FFromLZFaceReconstructor::CandidateFaceMinOverlapRatio)
				{
					ArrowColor = FColor(255, 205, 30, 255);
				}
				const FVector2D Tip =
					Candidate.MaskCentroid +
					Candidate.ProjectedNormal2D.GetSafeNormal() * NormalArrowLength;
				DrawArrowRGBA(
					RGBA, Inputs.FacesWidth, Inputs.FacesHeight,
					Candidate.MaskCentroid, Tip, ArrowColor, 2);

				if (Candidate.bUsedDirectedAttachNormalCheck)
				{
					DrawPointRGBA(
						RGBA,
						Inputs.FacesWidth,
						Inputs.FacesHeight,
						FMath::RoundToInt(Candidate.AttachGreenStart2D.X),
						FMath::RoundToInt(Candidate.AttachGreenStart2D.Y),
						FColor(255, 255, 255, 255),
						3);
					if (Candidate.bAttachPlusRayHit)
					{
						DrawLineRGBA(
							RGBA,
							Inputs.FacesWidth,
							Inputs.FacesHeight,
							Candidate.AttachGreenStart2D,
							Candidate.AttachPlusRayHit2D,
							FColor(0, 220, 255, 255),
							1);
						DrawPointRGBA(
							RGBA,
							Inputs.FacesWidth,
							Inputs.FacesHeight,
							FMath::RoundToInt(Candidate.AttachPlusRayHit2D.X),
							FMath::RoundToInt(Candidate.AttachPlusRayHit2D.Y),
							FColor(0, 220, 255, 255),
							3);
					}
					if (Candidate.bAttachMinusRayHit)
					{
						DrawLineRGBA(
							RGBA,
							Inputs.FacesWidth,
							Inputs.FacesHeight,
							Candidate.AttachGreenStart2D,
							Candidate.AttachMinusRayHit2D,
							FColor(255, 150, 0, 255),
							1);
						DrawPointRGBA(
							RGBA,
							Inputs.FacesWidth,
							Inputs.FacesHeight,
							FMath::RoundToInt(Candidate.AttachMinusRayHit2D.X),
							FMath::RoundToInt(Candidate.AttachMinusRayHit2D.Y),
							FColor(255, 150, 0, 255),
							3);
					}
				}
			}

			SaveRGBAToPng(
				RGBA, Inputs.FacesWidth, Inputs.FacesHeight,
				OutputDir / TEXT("face_validation.png"));
		}

		TSharedRef<FJsonObject> Root = MakeShared<FJsonObject>();
		Root->SetStringField(TEXT("candidate_source"), Request.CandidateSource);
		Root->SetStringField(TEXT("action"), Request.Action);
		Root->SetStringField(TEXT("source_polygon"), Evaluation.SourcePolygonKey);
		Root->SetBoolField(TEXT("valid"), Evaluation.bValid);
		Root->SetStringField(TEXT("evaluation_mode"), Evaluation.EvaluationMode);
		Root->SetStringField(TEXT("reason"), Evaluation.RejectReason);
		Root->SetNumberField(TEXT("cap_mask_pixels"), Evaluation.CapMaskPixels);
		Root->SetNumberField(TEXT("overlap_threshold"), FFromLZFaceReconstructor::CandidateFaceMinOverlapRatio);
		Root->SetNumberField(TEXT("max_normal_side_angle_degrees"), FFromLZFaceReconstructor::CandidateFaceMaxNormalSideAngleDegrees);
		Root->SetNumberField(TEXT("preferred_normal_side_angle_degrees"), FFromLZFaceReconstructor::CandidateFacePreferredNormalSideAngleDegrees);
		Root->SetStringField(
			TEXT("normal_green_angle_mode"),
			Request.Action.Equals(TEXT("attach"), ESearchCase::IgnoreCase)
				? TEXT("attach_directed_2d_ray_to_candidate_overlap_mask")
				: TEXT("unoriented_2d_parallel"));
		Root->SetNumberField(TEXT("attach_directed_ray_step_px"), AttachDirectedRayStepPx);
		Root->SetNumberField(TEXT("attach_directed_ray_min_distance_px"), AttachDirectedRayMinDistancePx);
		Root->SetNumberField(TEXT("attach_directed_ray_hit_radius_px"), AttachDirectedRayHitRadiusPx);
		Root->SetNumberField(TEXT("attach_directed_ray_ambiguous_distance_px"), AttachDirectedRayAmbiguousDistancePx);
		Root->SetBoolField(TEXT("attach_support_plane_fallback_enabled"), Params.bEnableAttachSupportPlaneFallback);
		Root->SetNumberField(TEXT("support_face_vote_radius_px"), Params.SupportFaceVoteRadiusPx);
		Root->SetNumberField(TEXT("support_plane_polygon_tol_px"), Params.SupportPlanePolygonTolPx);
		Root->SetBoolField(TEXT("support_plane_polygon_check_enforced"), false);
		Root->SetNumberField(TEXT("support_face_vote_sample_step_px"), Params.SupportFaceVoteSampleStepPx);
		Root->SetNumberField(TEXT("support_face_vote_min_coverage"), Params.SupportFaceVoteMinCoverage);
		Root->SetNumberField(TEXT("no_penetration_tol_cm"), Params.NoPenetrationTolCm);
		Root->SetNumberField(TEXT("contact_anchor_tol_px"), Params.ContactAnchorTolPx);
		Root->SetNumberField(TEXT("attach_path_front_distance_tie_tol_cm"), Params.AttachPathFrontDistanceTieTolCm);
		Root->SetNumberField(TEXT("attach_path_plane_relation_angle_tol_deg"), Params.AttachPathPlaneRelationAngleTolDeg);
		Root->SetNumberField(TEXT("attach_path_plane_relation_distance_tol_cm"), Params.AttachPathPlaneRelationDistanceTolCm);
		Root->SetNumberField(TEXT("support_force_hard_min_green_chord_cm"), Params.SupportForceHardMinGreenChordCm);
		Root->SetNumberField(TEXT("support_force_preferred_min_green_chord_cm"), Params.SupportForcePreferredMinGreenChordCm);
		Root->SetNumberField(TEXT("selected_face_id"), Evaluation.SelectedFaceId);
		Root->SetNumberField(TEXT("selected_face_overlap_pixels"), Evaluation.SelectedFaceOverlapPixels);
		Root->SetNumberField(TEXT("selected_face_overlap_ratio"), Evaluation.SelectedFaceOverlapRatio);
		Root->SetNumberField(TEXT("selected_face_normal_side_angle_degrees"), Evaluation.SelectedFaceNormalSideAngleDegrees);
		Root->SetNumberField(TEXT("selected_face_distance_to_camera"), Evaluation.SelectedFaceDistanceToCamera);
		Root->SetBoolField(TEXT("attach_support_plane_fallback_eligible"), Evaluation.bAttachSupportPlaneFallbackEligible);
		Root->SetNumberField(TEXT("support_face_id"), Evaluation.SupportFaceId);
		Root->SetNumberField(TEXT("support_green_chain_index"), Evaluation.SupportGreenChainIndex);
		Root->SetNumberField(TEXT("support_face_vote_coverage"), Evaluation.SupportFaceVoteCoverage);
		Root->SetStringField(TEXT("support_face_reject_reason"), Evaluation.SupportFaceRejectReason);
		TSharedRef<FJsonObject> Legend = MakeShared<FJsonObject>();
		Legend->SetStringField(TEXT("white_outline"), TEXT("candidate source polygon"));
		Legend->SetStringField(TEXT("green_arrow"), TEXT("local-green side vector"));
		Legend->SetStringField(TEXT("thin_green_arrows"), TEXT("support fallback green chain candidates"));
		Legend->SetStringField(TEXT("cyan_thick_arrow"), TEXT("selected support fallback green chain in Step9 precheck"));
		Legend->SetStringField(TEXT("red_mask"), TEXT("background or face overlap below threshold"));
		Legend->SetStringField(TEXT("yellow_mask_or_arrow"), TEXT("overlap passed but plane intersection or normal-angle validation failed"));
		Legend->SetStringField(TEXT("cyan_mask_or_arrow"), TEXT("face passed all hard conditions but was not selected"));
		Legend->SetStringField(TEXT("green_mask_or_arrow"), TEXT("preselected base face"));
		Root->SetObjectField(TEXT("legend"), Legend);

		TArray<TSharedPtr<FJsonValue>> CandidateValues;
		for (const FFaceCandidate& Candidate : Candidates)
		{
			TSharedRef<FJsonObject> CandidateObject = MakeShared<FJsonObject>();
			CandidateObject->SetNumberField(TEXT("face_id"), Candidate.FaceId);
			CandidateObject->SetNumberField(TEXT("overlap_pixels"), Candidate.OverlapPixels);
			CandidateObject->SetNumberField(TEXT("overlap_ratio"), Candidate.OverlapRatio);
			CandidateObject->SetBoolField(
				TEXT("overlap_pass"),
				Candidate.OverlapRatio >= FFromLZFaceReconstructor::CandidateFaceMinOverlapRatio);
			CandidateObject->SetBoolField(TEXT("plane_intersection_valid"), Candidate.bHasPlaneHit);
			CandidateObject->SetBoolField(TEXT("projected_normal_valid"), Candidate.bHasProjectedNormal);
			CandidateObject->SetNumberField(TEXT("normal_side_angle_degrees"), Candidate.NormalGreenAngleDegrees);
			CandidateObject->SetBoolField(TEXT("normal_side_angle_pass"), Candidate.bNormalParallelPass);
			CandidateObject->SetNumberField(TEXT("distance_to_camera"), Candidate.DistanceToCamera);
			CandidateObject->SetBoolField(TEXT("selected"), Candidate.FaceId == Evaluation.SelectedFaceId);
			CandidateObject->SetBoolField(TEXT("used_directed_attach_normal_check"), Candidate.bUsedDirectedAttachNormalCheck);
			CandidateObject->SetBoolField(TEXT("attach_normal_orientation_valid"), Candidate.bAttachNormalOrientationValid);
			CandidateObject->SetNumberField(TEXT("attach_directed_green_chain_index"), Candidate.AttachDirectedGreenChainIndex);
			CandidateObject->SetArrayField(TEXT("attach_green_start_2d"), JsonVector2D(Candidate.AttachGreenStart2D)->AsArray());
			CandidateObject->SetArrayField(TEXT("attach_green_end_2d"), JsonVector2D(Candidate.AttachGreenEnd2D)->AsArray());
			CandidateObject->SetArrayField(TEXT("attach_green_dir_2d"), JsonVector2D(Candidate.AttachGreenDir2D)->AsArray());
			CandidateObject->SetArrayField(TEXT("attach_expected_normal_dir_2d"), JsonVector2D(Candidate.AttachExpectedNormalDir2D)->AsArray());
			CandidateObject->SetArrayField(TEXT("raw_projected_normal_2d"), JsonVector2D(Candidate.RawProjectedNormal2D)->AsArray());
			CandidateObject->SetArrayField(TEXT("oriented_projected_normal_2d"), JsonVector2D(Candidate.OrientedProjectedNormal2D)->AsArray());
			CandidateObject->SetNumberField(TEXT("unoriented_normal_side_angle_degrees"), Candidate.UnorientedNormalGreenAngleDegrees);
			CandidateObject->SetNumberField(TEXT("directed_normal_side_angle_degrees"), Candidate.NormalGreenAngleDegrees);
			CandidateObject->SetBoolField(TEXT("plus_ray_hit"), Candidate.bAttachPlusRayHit);
			CandidateObject->SetBoolField(TEXT("minus_ray_hit"), Candidate.bAttachMinusRayHit);
			CandidateObject->SetArrayField(TEXT("plus_ray_hit_2d"), JsonVector2D(Candidate.AttachPlusRayHit2D)->AsArray());
			CandidateObject->SetArrayField(TEXT("minus_ray_hit_2d"), JsonVector2D(Candidate.AttachMinusRayHit2D)->AsArray());
			CandidateObject->SetNumberField(TEXT("plus_ray_distance_px"), Candidate.AttachPlusRayDistancePx);
			CandidateObject->SetNumberField(TEXT("minus_ray_distance_px"), Candidate.AttachMinusRayDistancePx);
			CandidateObject->SetStringField(TEXT("attach_normal_orientation_reason"), Candidate.AttachNormalOrientationReason);
			CandidateValues.Add(MakeShared<FJsonValueObject>(CandidateObject));
		}
		Root->SetArrayField(TEXT("faces"), CandidateValues);

		TMap<int32, const FFromLZSupportFaceVoteAttempt*> SupportVoteAttemptByChainIndex;
		TArray<TSharedPtr<FJsonValue>> SupportVoteAttemptValues;
		for (const FFromLZSupportFaceVoteAttempt& Attempt : Evaluation.SupportFaceVoteAttempts)
		{
			SupportVoteAttemptByChainIndex.Add(Attempt.ChainIndex, &Attempt);
			TSharedRef<FJsonObject> AttemptObject = MakeShared<FJsonObject>();
			AttemptObject->SetNumberField(TEXT("chain_index"), Attempt.ChainIndex);
			AttemptObject->SetNumberField(TEXT("seed_stroke_id"), Attempt.SeedStrokeId);
			AttemptObject->SetArrayField(TEXT("chain_start_face_space"), JsonVector2D(Attempt.ChainStartFaceSpace)->AsArray());
			AttemptObject->SetArrayField(TEXT("chain_end_face_space"), JsonVector2D(Attempt.ChainEndFaceSpace)->AsArray());
			AttemptObject->SetNumberField(TEXT("chain_chord_length"), Attempt.ChainChordLength);
			AttemptObject->SetNumberField(TEXT("chain_path_length"), Attempt.ChainPathLength);
			AttemptObject->SetBoolField(TEXT("vote_found"), Attempt.bVoteFound);
			AttemptObject->SetBoolField(TEXT("coverage_pass"), Attempt.bCoveragePass);
			AttemptObject->SetBoolField(TEXT("selected"), Attempt.bSelected);
			AttemptObject->SetNumberField(TEXT("support_face_id"), Attempt.SupportFaceId);
			SetSupportFaceVoteAttemptDebugFields(
				AttemptObject,
				Attempt.SupportVotePixels,
				Attempt.SupportConsideredPixels,
				Attempt.SupportHitSampleCount,
				Attempt.SupportTotalSampleCount,
				Attempt.SupportVoteCoverage,
				Attempt.SupportVotePixelCoverage,
				Attempt.SupportFaceWorldZMax,
				Attempt.SupportFaceWorldZAverage,
				Attempt.SupportMinCameraDistance,
				Attempt.SupportWorstPolygonDistancePx,
				Attempt.SupportFaceCandidates);
			AttemptObject->SetStringField(TEXT("reject_reason"), Attempt.RejectReason);
			SupportVoteAttemptValues.Add(MakeShared<FJsonValueObject>(AttemptObject));
		}
		Root->SetArrayField(TEXT("support_face_vote_attempts"), SupportVoteAttemptValues);

		TArray<TSharedPtr<FJsonValue>> GreenChainValues;
		for (int32 ChainIndex = 0; ChainIndex < Request.GreenChains.Num(); ++ChainIndex)
		{
			const FFromLZGreenChainCandidate2D& Chain = Request.GreenChains[ChainIndex];
			TSharedRef<FJsonObject> ChainObject = MakeShared<FJsonObject>();
			ChainObject->SetNumberField(TEXT("index"), ChainIndex);
			ChainObject->SetNumberField(TEXT("seed_stroke_id"), Chain.SeedStrokeId);
			SetIntArrayField(ChainObject, TEXT("stroke_ids"), Chain.StrokeIds);
			ChainObject->SetArrayField(TEXT("start"), JsonVector2D(Chain.Start)->AsArray());
			ChainObject->SetArrayField(TEXT("end"), JsonVector2D(Chain.End)->AsArray());
			ChainObject->SetArrayField(TEXT("vector"), JsonVector2D(Chain.Vector)->AsArray());
			ChainObject->SetArrayField(TEXT("seed_direction"), JsonVector2D(Chain.SeedDirection)->AsArray());
			ChainObject->SetNumberField(TEXT("chord_length"), Chain.ChordLength);
			ChainObject->SetNumberField(TEXT("path_length"), Chain.PathLength);
			ChainObject->SetNumberField(TEXT("total_gap"), Chain.TotalGap);
			ChainObject->SetStringField(TEXT("stop_reason"), Chain.StopReason);
			if (const FFromLZSupportFaceVoteAttempt* const* FoundAttempt = SupportVoteAttemptByChainIndex.Find(ChainIndex))
			{
				const FFromLZSupportFaceVoteAttempt& Attempt = **FoundAttempt;
				ChainObject->SetBoolField(TEXT("support_vote_attempted"), true);
				ChainObject->SetBoolField(TEXT("support_vote_found"), Attempt.bVoteFound);
				ChainObject->SetBoolField(TEXT("support_vote_coverage_pass"), Attempt.bCoveragePass);
				ChainObject->SetBoolField(TEXT("support_vote_selected"), Attempt.bSelected);
				ChainObject->SetNumberField(TEXT("support_face_id"), Attempt.SupportFaceId);
				SetSupportFaceVoteAttemptDebugFields(
					ChainObject,
					Attempt.SupportVotePixels,
					Attempt.SupportConsideredPixels,
					Attempt.SupportHitSampleCount,
					Attempt.SupportTotalSampleCount,
					Attempt.SupportVoteCoverage,
					Attempt.SupportVotePixelCoverage,
					Attempt.SupportFaceWorldZMax,
					Attempt.SupportFaceWorldZAverage,
					Attempt.SupportMinCameraDistance,
					Attempt.SupportWorstPolygonDistancePx,
					Attempt.SupportFaceCandidates);
				ChainObject->SetStringField(TEXT("support_vote_reject_reason"), Attempt.RejectReason);
			}
			else
			{
				ChainObject->SetBoolField(TEXT("support_vote_attempted"), false);
			}
			GreenChainValues.Add(MakeShared<FJsonValueObject>(ChainObject));
		}
		Root->SetArrayField(TEXT("green_chain_candidates"), GreenChainValues);
		SaveJsonObject(Root, OutputDir / TEXT("face_validation.json"));
	}

	static bool IsFiniteVector2D(const FVector2D& Value)
	{
		return FMath::IsFinite(Value.X) && FMath::IsFinite(Value.Y);
	}

	static bool IsFiniteVector(const FVector& Value)
	{
		return FMath::IsFinite(Value.X) && FMath::IsFinite(Value.Y) && FMath::IsFinite(Value.Z);
	}

	static double Cross2D(const FVector2D& A, const FVector2D& B, const FVector2D& C)
	{
		const FVector2D AB = B - A;
		const FVector2D AC = C - A;
		return AB.X * AC.Y - AB.Y * AC.X;
	}

	static double SignedPolygonAreaForRegularization(const TArray<FVector2D>& Polygon)
	{
		if (Polygon.Num() < 3)
		{
			return 0.0;
		}

		double TwiceArea = 0.0;
		for (int32 Index = 0; Index < Polygon.Num(); ++Index)
		{
			const FVector2D& A = Polygon[Index];
			const FVector2D& B = Polygon[(Index + 1) % Polygon.Num()];
			TwiceArea += A.X * B.Y - A.Y * B.X;
		}
		return 0.5 * TwiceArea;
	}

	static bool BuildConvexHull2D(const TArray<FVector2D>& Points, TArray<FVector2D>& OutHull)
	{
		OutHull.Reset();
		TArray<FVector2D> Sorted;
		for (const FVector2D& Point : Points)
		{
			if (IsFiniteVector2D(Point))
			{
				Sorted.Add(Point);
			}
		}
		Sorted.Sort([](const FVector2D& A, const FVector2D& B)
		{
			if (!FMath::IsNearlyEqual(A.X, B.X, 1e-9))
			{
				return A.X < B.X;
			}
			return A.Y < B.Y;
		});

		TArray<FVector2D> Unique;
		for (const FVector2D& Point : Sorted)
		{
			if (Unique.Num() == 0 || FVector2D::Distance(Unique.Last(), Point) > 1e-7)
			{
				Unique.Add(Point);
			}
		}
		if (Unique.Num() < 3)
		{
			return false;
		}

		TArray<FVector2D> Lower;
		for (const FVector2D& Point : Unique)
		{
			while (Lower.Num() >= 2 && Cross2D(Lower[Lower.Num() - 2], Lower.Last(), Point) <= 1e-9)
			{
				Lower.Pop();
			}
			Lower.Add(Point);
		}

		TArray<FVector2D> Upper;
		for (int32 Index = Unique.Num() - 1; Index >= 0; --Index)
		{
			const FVector2D& Point = Unique[Index];
			while (Upper.Num() >= 2 && Cross2D(Upper[Upper.Num() - 2], Upper.Last(), Point) <= 1e-9)
			{
				Upper.Pop();
			}
			Upper.Add(Point);
		}

		Lower.Pop();
		Upper.Pop();
		OutHull = MoveTemp(Lower);
		OutHull.Append(Upper);
		return OutHull.Num() >= 3 && FMath::Abs(SignedPolygonAreaForRegularization(OutHull)) > CapBBoxDegenerateAreaTolerance;
	}

	struct FMinimumAreaBBox2D
	{
		bool bValid = false;
		FVector2D AxisX = FVector2D(1.0, 0.0);
		FVector2D AxisY = FVector2D(0.0, 1.0);
		double MinX = 0.0;
		double MaxX = 0.0;
		double MinY = 0.0;
		double MaxY = 0.0;
		double Area = 0.0;
	};

	static bool ComputeMinimumAreaBBox2D(const TArray<FVector2D>& Hull, FMinimumAreaBBox2D& OutBBox)
	{
		OutBBox = FMinimumAreaBBox2D();
		double BestArea = TNumericLimits<double>::Max();
		for (int32 Index = 0; Index < Hull.Num(); ++Index)
		{
			FVector2D AxisX = Hull[(Index + 1) % Hull.Num()] - Hull[Index];
			if (!AxisX.Normalize())
			{
				continue;
			}
			const FVector2D AxisY(-AxisX.Y, AxisX.X);
			double MinX = TNumericLimits<double>::Max();
			double MaxX = -TNumericLimits<double>::Max();
			double MinY = TNumericLimits<double>::Max();
			double MaxY = -TNumericLimits<double>::Max();
			for (const FVector2D& Point : Hull)
			{
				const double X = FVector2D::DotProduct(Point, AxisX);
				const double Y = FVector2D::DotProduct(Point, AxisY);
				MinX = FMath::Min(MinX, X);
				MaxX = FMath::Max(MaxX, X);
				MinY = FMath::Min(MinY, Y);
				MaxY = FMath::Max(MaxY, Y);
			}
			const double Area = (MaxX - MinX) * (MaxY - MinY);
			if (Area < BestArea)
			{
				BestArea = Area;
				OutBBox.bValid = Area > CapBBoxDegenerateAreaTolerance;
				OutBBox.AxisX = AxisX;
				OutBBox.AxisY = AxisY;
				OutBBox.MinX = MinX;
				OutBBox.MaxX = MaxX;
				OutBBox.MinY = MinY;
				OutBBox.MaxY = MaxY;
				OutBBox.Area = Area;
			}
		}
		return OutBBox.bValid;
	}

	static bool BuildTemporaryFaceBasis(
		const FFaceInfo& Face,
		const FCameraInfo& Camera,
		FVector& OutNormal,
		FVector& OutAxisX,
		FVector& OutAxisY)
	{
		OutNormal = Face.Normal.GetSafeNormal();
		if (OutNormal.IsNearlyZero())
		{
			return false;
		}

		OutAxisX = Camera.Right - OutNormal * FVector::DotProduct(Camera.Right, OutNormal);
		if (!OutAxisX.Normalize())
		{
			const FVector Fallback = FMath::Abs(OutNormal.Z) < 0.9 ? FVector::UpVector : FVector::ForwardVector;
			OutAxisX = Fallback - OutNormal * FVector::DotProduct(Fallback, OutNormal);
			if (!OutAxisX.Normalize())
			{
				return false;
			}
		}
		OutAxisY = FVector::CrossProduct(OutNormal, OutAxisX).GetSafeNormal();
		return !OutAxisY.IsNearlyZero();
	}

	static FVector2D WorldPointToFaceUV(
		const FVector& Point,
		const FVector& Origin,
		const FVector& AxisU,
		const FVector& AxisV)
	{
		const FVector Delta = Point - Origin;
		return FVector2D(
			FVector::DotProduct(Delta, AxisU),
			FVector::DotProduct(Delta, AxisV));
	}

	static FVector FaceUVToWorld(
		const FVector2D& UV,
		const FVector& Origin,
		const FVector& AxisU,
		const FVector& AxisV)
	{
		return Origin + AxisU * UV.X + AxisV * UV.Y;
	}

	static void ComputeAxisAlignedBounds(
		const TArray<FVector2D>& Points,
		double& OutMinU,
		double& OutMaxU,
		double& OutMinV,
		double& OutMaxV)
	{
		OutMinU = TNumericLimits<double>::Max();
		OutMaxU = -TNumericLimits<double>::Max();
		OutMinV = TNumericLimits<double>::Max();
		OutMaxV = -TNumericLimits<double>::Max();
		for (const FVector2D& Point : Points)
		{
			OutMinU = FMath::Min(OutMinU, Point.X);
			OutMaxU = FMath::Max(OutMaxU, Point.X);
			OutMinV = FMath::Min(OutMinV, Point.Y);
			OutMaxV = FMath::Max(OutMaxV, Point.Y);
		}
	}

	static TArray<FVector2D> MakeBBoxCorners(double MinU, double MaxU, double MinV, double MaxV)
	{
		TArray<FVector2D> Corners;
		Corners.Reserve(4);
		Corners.Emplace(MinU, MinV);
		Corners.Emplace(MaxU, MinV);
		Corners.Emplace(MaxU, MaxV);
		Corners.Emplace(MinU, MaxV);
		return Corners;
	}

	static TArray<FVector2D> MakeOrientedBBoxCorners(const FMinimumAreaBBox2D& BBox)
	{
		TArray<FVector2D> Corners;
		if (!BBox.bValid)
		{
			return Corners;
		}
		Corners.Reserve(4);
		Corners.Add(BBox.AxisX * BBox.MinX + BBox.AxisY * BBox.MinY);
		Corners.Add(BBox.AxisX * BBox.MaxX + BBox.AxisY * BBox.MinY);
		Corners.Add(BBox.AxisX * BBox.MaxX + BBox.AxisY * BBox.MaxY);
		Corners.Add(BBox.AxisX * BBox.MinX + BBox.AxisY * BBox.MaxY);
		return Corners;
	}

	static bool ConvertFaceUVLoopToWorld(
		const TArray<FVector2D>& UVLoop,
		const FVector& Origin,
		const FVector& AxisU,
		const FVector& AxisV,
		TArray<FVector>& OutWorld)
	{
		OutWorld.Reset();
		OutWorld.Reserve(UVLoop.Num());
		for (const FVector2D& UV : UVLoop)
		{
			const FVector World = FaceUVToWorld(UV, Origin, AxisU, AxisV);
			if (!IsFiniteVector(World))
			{
				OutWorld.Reset();
				return false;
			}
			OutWorld.Add(World);
		}
		return OutWorld.Num() == UVLoop.Num();
	}

	static void SetRegularizationFallback(FCapBBoxRegularizationResult& Debug, const FString& Reason)
	{
		Debug.bApplied = false;
		Debug.bFallbackToOriginal = true;
		Debug.SelectedGeometry = TEXT("original");
		Debug.RejectionReason = Reason;
		Debug.FinalCapUV = Debug.OriginalCapUV;
		Debug.FinalCapWorld = Debug.OriginalCapWorld;
		for (FWorldOrthogonalStageDebug& Stage : Debug.WorldOrthogonalStages)
		{
			if (Stage.Name == Debug.WorldOrthogonalActiveStage)
			{
				Stage.Status = TEXT("failed");
				Stage.Message = Reason;
				break;
			}
		}
	}

	static const TCHAR* WorldOrthogonalStageNames[] =
	{
		TEXT("input_validation"),
		TEXT("world_aligned_face_uv"),
		TEXT("boundary_run_unprojection"),
		TEXT("red_corner_protection"),
		TEXT("primitive_edge_classification"),
		TEXT("pure_red_root_alignment"),
		TEXT("same_axis_merge"),
		TEXT("topology_repair"),
		TEXT("adjacency_validation"),
		TEXT("support_line_solve"),
		TEXT("vertex_intersections"),
		TEXT("topology_validation"),
		TEXT("metric_validation"),
		TEXT("world_conversion"),
		TEXT("orthographic_reprojection")
	};

	static void InitializeWorldOrthogonalStages(FCapBBoxRegularizationResult& Debug)
	{
		Debug.WorldOrthogonalStages.Reset();
		for (const TCHAR* Name : WorldOrthogonalStageNames)
		{
			FWorldOrthogonalStageDebug Stage;
			Stage.Name = Name;
			Debug.WorldOrthogonalStages.Add(MoveTemp(Stage));
		}
	}

	static void SetWorldOrthogonalStage(
		FCapBBoxRegularizationResult& Debug,
		const TCHAR* Name,
		const TCHAR* Status,
		const FString& Message = FString())
	{
		for (FWorldOrthogonalStageDebug& Stage : Debug.WorldOrthogonalStages)
		{
			if (Stage.Name == Name)
			{
				Stage.Status = Status;
				Stage.Message = Message;
				return;
			}
		}
		FWorldOrthogonalStageDebug Stage;
		Stage.Name = Name;
		Stage.Status = Status;
		Stage.Message = Message;
		Debug.WorldOrthogonalStages.Add(MoveTemp(Stage));
	}

	static void BeginWorldOrthogonalStage(
		FCapBBoxRegularizationResult& Debug,
		const TCHAR* Name)
	{
		Debug.WorldOrthogonalActiveStage = Name;
		SetWorldOrthogonalStage(Debug, Name, TEXT("in_progress"));
	}

	static void PassWorldOrthogonalStage(
		FCapBBoxRegularizationResult& Debug,
		const TCHAR* Name,
		const FString& Message = FString())
	{
		SetWorldOrthogonalStage(Debug, Name, TEXT("passed"), Message);
		if (Debug.WorldOrthogonalActiveStage == Name)
		{
			Debug.WorldOrthogonalActiveStage.Reset();
		}
	}

	static void ApplyWorldOrthogonalParamsToDebug(
		FCapBBoxRegularizationResult& Debug,
		const FFromLZFaceReconstructionParams& Params)
	{
		Debug.bWorldOrthoUsePerFaceCapture = Params.bWorldOrthoUsePerFaceCapture;
		Debug.WorldOrthoPerFaceClipMarginPixels = FMath::Max(
			0.0,
			double(Params.WorldOrthoPerFaceClipMarginPixels));
		Debug.bWorldOrthoPureRedAllowDiagonalRoot =
			Params.bWorldOrthoPureRedAllowDiagonalRoot;
		Debug.bWorldOrthoAllowDiagonalSupports =
			Params.bWorldOrthoAllowDiagonalSupports;
		Debug.WorldOrthoBlackAxisToleranceDegreesUsed = FMath::Clamp(
			double(Params.WorldOrthoBlackAxisToleranceDegrees),
			0.0,
			180.0);
		Debug.WorldOrthoDiagThresholdDegreesUsed = FMath::Clamp(
			double(Params.WorldOrthoDiagThresholdDegrees),
			0.0,
			90.0);
		Debug.WorldOrthoAngleComparisonEpsilonDegreesUsed = FMath::Max(
			0.0,
			double(Params.WorldOrthoAngleComparisonEpsilonDegrees));
		Debug.WorldOrthoBlackNodeSnapTolerancePixelsUsed = FMath::Max(
			0.0,
			double(Params.WorldOrthoBlackNodeSnapTolerancePixels));
		Debug.WorldOrthoRedMacroCorridorPixelsUsed = FMath::Max(
			0.0,
			double(Params.WorldOrthoRedMacroCorridorPixels));
		Debug.WorldOrthoRedMacroGroupMinLengthPixelsUsed = FMath::Max(
			0.0,
			double(Params.WorldOrthoRedMacroGroupMinLengthPixels));
		Debug.WorldOrthoRedPrimitiveRdpTolerancePixelsUsed = FMath::Max(
			0.0,
			double(Params.WorldOrthoRedPrimitiveRdpTolerancePixels));
		Debug.WorldOrthoShortRedEdgeLengthPixelsUsed = FMath::Max(
			0.0,
			double(Params.WorldOrthoShortRedEdgeLengthPixels));
		Debug.WorldOrthoMinAreaRatioUsed = FMath::Max(
			0.0,
			double(Params.WorldOrthoMinAreaRatio));
		Debug.WorldOrthoMaxWrapGapFractionUsed = FMath::Clamp(
			double(Params.WorldOrthoMaxWrapGapFraction),
			0.0,
			1.0);
		Debug.bWorldOrthoAllowTopologyRepair =
			Params.bWorldOrthoAllowTopologyRepair;
		Debug.WorldOrthoPureRedMaxRootCandidatesUsed =
			FMath::Max(0, Params.WorldOrthoPureRedMaxRootCandidates);
	}

	// Octilinear axis types: 0=U(horizontal), 1=V(vertical), 2=Diag+(45deg), 3=Diag-(135deg)
	enum EOctilinearAxis : int32
	{
		Octi_U = 0,
		Octi_V = 1,
		Octi_DiagPlus = 2,
		Octi_DiagMinus = 3
	};

	static EOctilinearAxis ClassifyOctilinearAxisByThreshold(
		double AngleToU,
		double AngleToV,
		double AngleToDiagPlus,
		double AngleToDiagMinus,
		const FFromLZFaceReconstructionParams& Params)
	{
		if (!Params.bWorldOrthoAllowDiagonalSupports)
		{
			return AngleToU <= AngleToV ? Octi_U : Octi_V;
		}
		const double MinAxisAngle = FMath::Min(AngleToU, AngleToV);
		if (MinAxisAngle <=
			FMath::Clamp(double(Params.WorldOrthoDiagThresholdDegrees), 0.0, 90.0) +
				FMath::Max(0.0, double(Params.WorldOrthoAngleComparisonEpsilonDegrees)))
		{
			return AngleToU <= AngleToV ? Octi_U : Octi_V;
		}
		return AngleToDiagPlus <= AngleToDiagMinus
			? Octi_DiagPlus
			: Octi_DiagMinus;
	}

	struct FWorldOrthogonalEdge
	{
		ECapBoundaryRunType Type = ECapBoundaryRunType::Red;
		TArray<int32> SourceRunIndices;
		TArray<int32> SourceStrokeIds;
		TArray<int32> PrimitiveEdgeIndices;
		TArray<int32> GraphNodeIds;
		TArray<FVector2D> GraphNodeCapSpace;
		TArray<FVector2D> GraphNodeUV;
		TArray<FVector2D> SnappedGraphNodeUV;
		TArray<double> GraphNodeSnapDistancesPixels;
		TArray<FVector2D> SamplesUV;
		TArray<FVector2D> SamplesCapSpace;
		FVector2D DirectionUV = FVector2D::ZeroVector;
		FVector2D StartUV = FVector2D::ZeroVector;
		FVector2D EndUV = FVector2D::ZeroVector;
		FVector2D StartCapSpace = FVector2D::ZeroVector;
		FVector2D EndCapSpace = FVector2D::ZeroVector;
		EOctilinearAxis AxisType = Octi_U;
		bool bUseMajorChordDirection = false;
		double AngleToU = 180.0;
		double AngleToV = 180.0;
		double AngleToDiagPlus = 180.0; // angle to (1,1) normalized = 45deg direction
		double AngleToDiagMinus = 180.0; // angle to (1,-1) normalized = 135deg direction
		double AngleToAssignedAxis = 0.0; // angle to whichever axis the edge was assigned
		double SupportCoordinate = 0.0;
		double PathLength = 0.0;
		double CapPathLengthPixels = 0.0;
	};

	// Per-face capture: detect contour corners whose 2D pixel position lies on the
	// shared-camera image rectangle, then snap their face-UV onto the axis-aligned
	// bbox so image-clip edges become axis-aligned in face UV. Real geometry corners
	// (interior to the image) are kept as-is. Returns false on degenerate input.
	static bool BuildPerFaceCleanBoundaryUV(
		const TArray<FVector2D>& KeyPoints2D,
		const TArray<FVector>& KeyPoints3D,
		const FVector& PlanePoint,
		const FVector& AxisU,
		const FVector& AxisV,
		int32 ImageWidth,
		int32 ImageHeight,
		double ClipMarginPixels,
		TArray<FVector2D>& OutCleanBoundaryUV)
	{
		OutCleanBoundaryUV.Reset();
		const int32 N = KeyPoints3D.Num();
		if (N < 3 || N != KeyPoints2D.Num() || ImageWidth <= 1 || ImageHeight <= 1)
		{
			return false;
		}

		TArray<FVector2D> RawUV;
		RawUV.SetNum(N);
		for (int32 i = 0; i < N; ++i)
		{
			RawUV[i] = WorldPointToFaceUV(KeyPoints3D[i], PlanePoint, AxisU, AxisV);
		}

		const double MaxX = double(ImageWidth - 1);
		const double MaxY = double(ImageHeight - 1);
		TArray<bool> IsClipped;
		IsClipped.SetNum(N);
		int32 ClippedCount = 0;
		for (int32 i = 0; i < N; ++i)
		{
			const FVector2D& P = KeyPoints2D[i];
			const bool bClipped =
				P.X <= ClipMarginPixels ||
				P.X >= MaxX - ClipMarginPixels ||
				P.Y <= ClipMarginPixels ||
				P.Y >= MaxY - ClipMarginPixels;
			IsClipped[i] = bClipped;
			if (bClipped) { ++ClippedCount; }
		}

		if (ClippedCount == 0)
		{
			OutCleanBoundaryUV = MoveTemp(RawUV);
			return OutCleanBoundaryUV.Num() >= 3;
		}

		double MinU = TNumericLimits<double>::Max();
		double MaxU = -TNumericLimits<double>::Max();
		double MinV = TNumericLimits<double>::Max();
		double MaxV = -TNumericLimits<double>::Max();
		for (int32 i = 0; i < N; ++i)
		{
			MinU = FMath::Min(MinU, RawUV[i].X);
			MaxU = FMath::Max(MaxU, RawUV[i].X);
			MinV = FMath::Min(MinV, RawUV[i].Y);
			MaxV = FMath::Max(MaxV, RawUV[i].Y);
		}
		if (!FMath::IsFinite(MinU) || !FMath::IsFinite(MaxU) ||
			!FMath::IsFinite(MinV) || !FMath::IsFinite(MaxV) ||
			MaxU - MinU <= 1e-9 || MaxV - MinV <= 1e-9)
		{
			return false;
		}

		if (ClippedCount == N)
		{
			OutCleanBoundaryUV.Add(FVector2D(MinU, MinV));
			OutCleanBoundaryUV.Add(FVector2D(MaxU, MinV));
			OutCleanBoundaryUV.Add(FVector2D(MaxU, MaxV));
			OutCleanBoundaryUV.Add(FVector2D(MinU, MaxV));
			return true;
		}

		auto SnapToBBoxEdge = [&](int32 Index) -> FVector2D
		{
			const FVector2D& UV = RawUV[Index];
			const double DistMinU = FMath::Abs(UV.X - MinU);
			const double DistMaxU = FMath::Abs(UV.X - MaxU);
			const double DistMinV = FMath::Abs(UV.Y - MinV);
			const double DistMaxV = FMath::Abs(UV.Y - MaxV);
			double Best = DistMinU;
			int32 Side = 0;
			if (DistMaxU < Best) { Best = DistMaxU; Side = 1; }
			if (DistMinV < Best) { Best = DistMinV; Side = 2; }
			if (DistMaxV < Best) { Best = DistMaxV; Side = 3; }

			FVector2D Snapped(
				FMath::Clamp(UV.X, MinU, MaxU),
				FMath::Clamp(UV.Y, MinV, MaxV));
			switch (Side)
			{
				case 0: Snapped.X = MinU; break;
				case 1: Snapped.X = MaxU; break;
				case 2: Snapped.Y = MinV; break;
				case 3: Snapped.Y = MaxV; break;
				default: break;
			}
			return Snapped;
		};

		TArray<FVector2D> Built;
		Built.Reserve(N);
		for (int32 i = 0; i < N; ++i)
		{
			Built.Add(IsClipped[i] ? SnapToBBoxEdge(i) : RawUV[i]);
		}

		constexpr double DupTolUV = 1e-3;
		TArray<FVector2D> Deduped;
		Deduped.Reserve(Built.Num());
		for (int32 i = 0; i < Built.Num(); ++i)
		{
			const FVector2D& Curr = Built[i];
			if (Deduped.Num() == 0 ||
				FVector2D::Distance(Curr, Deduped.Last()) > DupTolUV)
			{
				Deduped.Add(Curr);
			}
		}
		while (Deduped.Num() >= 2 &&
			FVector2D::Distance(Deduped[0], Deduped.Last()) <= DupTolUV)
		{
			Deduped.Pop(EAllowShrinking::No);
		}

		OutCleanBoundaryUV = MoveTemp(Deduped);
		return OutCleanBoundaryUV.Num() >= 3;
	}

	static bool BuildWorldAlignedFaceBasis(
		const FVector& PlaneNormal,
		FVector& OutAxisU,
		FVector& OutAxisV,
		int32& OutExcludedWorldAxis)
	{
		const FVector Normal = PlaneNormal.GetSafeNormal();
		if (Normal.IsNearlyZero())
		{
			return false;
		}

		const FVector WorldAxes[3] =
		{
			FVector::XAxisVector,
			FVector::YAxisVector,
			FVector::ZAxisVector
		};
		const double Alignment[3] =
		{
			FMath::Abs(FVector::DotProduct(Normal, WorldAxes[0])),
			FMath::Abs(FVector::DotProduct(Normal, WorldAxes[1])),
			FMath::Abs(FVector::DotProduct(Normal, WorldAxes[2]))
		};

		OutExcludedWorldAxis = 0;
		if (Alignment[1] > Alignment[OutExcludedWorldAxis])
		{
			OutExcludedWorldAxis = 1;
		}
		if (Alignment[2] > Alignment[OutExcludedWorldAxis])
		{
			OutExcludedWorldAxis = 2;
		}

		int32 PrimaryAxis = 0;
		int32 SecondaryAxis = 1;
		if (OutExcludedWorldAxis == 0)
		{
			PrimaryAxis = 1;
			SecondaryAxis = 2;
		}
		else if (OutExcludedWorldAxis == 1)
		{
			PrimaryAxis = 0;
			SecondaryAxis = 2;
		}

		OutAxisU =
			WorldAxes[PrimaryAxis] -
			Normal * FVector::DotProduct(WorldAxes[PrimaryAxis], Normal);
		if (!OutAxisU.Normalize())
		{
			return false;
		}

		OutAxisV = FVector::CrossProduct(Normal, OutAxisU).GetSafeNormal();
		if (OutAxisV.IsNearlyZero())
		{
			return false;
		}

		FVector ProjectedSecondary =
			WorldAxes[SecondaryAxis] -
			Normal * FVector::DotProduct(WorldAxes[SecondaryAxis], Normal);
		if (ProjectedSecondary.Normalize() &&
			FVector::DotProduct(OutAxisV, ProjectedSecondary) < 0.0)
		{
			OutAxisV *= -1.0;
		}
		if (FVector::DotProduct(FVector::CrossProduct(OutAxisU, OutAxisV), Normal) < 0.0)
		{
			OutAxisV *= -1.0;
		}
		return true;
	}

	static bool ComputePrincipalDirection2D(
		const TArray<FVector2D>& Points,
		FVector2D& OutDirection,
		double& OutPathLength)
	{
		OutDirection = FVector2D::ZeroVector;
		OutPathLength = 0.0;
		if (Points.Num() < 2)
		{
			return false;
		}

		FVector2D Mean = FVector2D::ZeroVector;
		for (const FVector2D& Point : Points)
		{
			Mean += Point;
		}
		Mean /= double(Points.Num());

		double XX = 0.0;
		double XY = 0.0;
		double YY = 0.0;
		for (int32 Index = 0; Index < Points.Num(); ++Index)
		{
			const FVector2D Delta = Points[Index] - Mean;
			XX += Delta.X * Delta.X;
			XY += Delta.X * Delta.Y;
			YY += Delta.Y * Delta.Y;
			if (Index > 0)
			{
				OutPathLength += FVector2D::Distance(Points[Index - 1], Points[Index]);
			}
		}

		const double Angle = 0.5 * FMath::Atan2(2.0 * XY, XX - YY);
		OutDirection = FVector2D(FMath::Cos(Angle), FMath::Sin(Angle)).GetSafeNormal();
		const FVector2D Chord = Points.Last() - Points[0];
		if (Chord.SizeSquared() > 1e-12 &&
			FVector2D::DotProduct(OutDirection, Chord) < 0.0)
		{
			OutDirection *= -1.0;
		}
		return !OutDirection.IsNearlyZero() && OutPathLength > 1e-8;
	}

	struct FRedDirectionalGroup
	{
		int32 StartIndex = INDEX_NONE;
		int32 EndIndex = INDEX_NONE;
		EOctilinearAxis AxisType = Octi_U;
		double LengthPixels = 0.0;
	};

	static FString OctilinearAxisName(EOctilinearAxis Axis);

	static EOctilinearAxis ClassifyRedMacroOctilinearAxis(
		const FVector2D& Direction,
		const FFromLZFaceReconstructionParams& Params)
	{
		const FVector2D Unit = Direction.GetSafeNormal();
		if (Unit.IsNearlyZero())
		{
			return Octi_U;
		}
		if (!Params.bWorldOrthoAllowDiagonalSupports)
		{
			const double AngleU = FMath::RadiansToDegrees(FMath::Acos(
				FMath::Clamp(FMath::Abs(Unit.X), 0.0, 1.0)));
			const double AngleV = FMath::RadiansToDegrees(FMath::Acos(
				FMath::Clamp(FMath::Abs(Unit.Y), 0.0, 1.0)));
			return AngleU <= AngleV ? Octi_U : Octi_V;
		}
		const FVector2D DiagPlus =
			FVector2D(1.0, 1.0).GetSafeNormal();
		const FVector2D DiagMinus =
			FVector2D(1.0, -1.0).GetSafeNormal();
		const double AngleU = FMath::RadiansToDegrees(FMath::Acos(
			FMath::Clamp(FMath::Abs(Unit.X), 0.0, 1.0)));
		const double AngleV = FMath::RadiansToDegrees(FMath::Acos(
			FMath::Clamp(FMath::Abs(Unit.Y), 0.0, 1.0)));
		const double AngleDiagPlus = FMath::RadiansToDegrees(FMath::Acos(
			FMath::Clamp(
				FMath::Abs(FVector2D::DotProduct(Unit, DiagPlus)),
				0.0,
				1.0)));
		const double AngleDiagMinus = FMath::RadiansToDegrees(FMath::Acos(
			FMath::Clamp(
				FMath::Abs(FVector2D::DotProduct(Unit, DiagMinus)),
				0.0,
				1.0)));
		return ClassifyOctilinearAxisByThreshold(
			AngleU,
			AngleV,
			AngleDiagPlus,
			AngleDiagMinus,
			Params);
	}

	static void FindProtectedRedRunIndices(
		const TArray<FVector2D>& PointsUV,
		const TArray<FVector2D>& PointsCapSpace,
		TArray<int32>& OutIndices,
		FWorldOrthogonalRunDebug& OutDebug,
		const FFromLZFaceReconstructionParams& Params)
	{
		OutIndices.Reset();
		OutDebug.RedProtectionMode.Reset();
		OutDebug.RedMaxChordDeviationPixels = 0.0;
		OutDebug.RedDirectionalGroupAxisTypes.Reset();
		OutDebug.RedDirectionalGroupLengthsPixels.Reset();
		OutDebug.RedDirectionalGroupStartIndices.Reset();
		OutDebug.RedDirectionalGroupEndIndices.Reset();
		if (PointsUV.Num() < 2 ||
			PointsCapSpace.Num() != PointsUV.Num())
		{
			return;
		}

		const int32 LastIndex = PointsCapSpace.Num() - 1;
		const FVector2D ChordStart = PointsCapSpace[0];
		const FVector2D ChordEnd = PointsCapSpace[LastIndex];
		for (const FVector2D& Point : PointsCapSpace)
		{
			OutDebug.RedMaxChordDeviationPixels = FMath::Max(
				OutDebug.RedMaxChordDeviationPixels,
				FMath::Sqrt(DistancePointToSegmentSquared2D(
					Point,
					ChordStart,
					ChordEnd)));
		}
		if (FVector2D::Distance(ChordStart, ChordEnd) > 1e-6 &&
			OutDebug.RedMaxChordDeviationPixels <=
				FMath::Max(0.0, double(Params.WorldOrthoRedMacroCorridorPixels)))
		{
			OutIndices = { 0, LastIndex };
			OutDebug.RedProtectionMode = TEXT("straight_corridor_macro_edge");
			FVector2D DirectionUV;
			double PathLengthUV = 0.0;
			const EOctilinearAxis AxisType =
				ComputePrincipalDirection2D(PointsUV, DirectionUV, PathLengthUV)
					? ClassifyRedMacroOctilinearAxis(DirectionUV, Params)
					: Octi_U;
			double LengthPixels = 0.0;
			for (int32 Index = 1; Index < PointsCapSpace.Num(); ++Index)
			{
				LengthPixels += FVector2D::Distance(
					PointsCapSpace[Index - 1],
					PointsCapSpace[Index]);
			}
			OutDebug.RedDirectionalGroupAxisTypes.Add(
				OctilinearAxisName(AxisType));
			OutDebug.RedDirectionalGroupLengthsPixels.Add(LengthPixels);
			OutDebug.RedDirectionalGroupStartIndices.Add(0);
			OutDebug.RedDirectionalGroupEndIndices.Add(LastIndex);
			return;
		}

		TArray<bool> Keep;
		Keep.Init(false, PointsCapSpace.Num());
		MarkRdpSegment(
			PointsCapSpace,
			0,
			LastIndex,
			FMath::Square(FMath::Max(
				0.0,
				double(Params.WorldOrthoRedPrimitiveRdpTolerancePixels))),
			Keep);
		TArray<int32> SimplifiedIndices;
		for (int32 Index = 0; Index < PointsCapSpace.Num(); ++Index)
		{
			if (Keep[Index])
			{
				SimplifiedIndices.Add(Index);
			}
		}
		if (SimplifiedIndices.Num() < 2)
		{
			OutIndices = { 0, LastIndex };
			OutDebug.RedProtectionMode = TEXT("rdp_macro_edge");
			return;
		}

		TArray<double> PrefixLengthPixels;
		PrefixLengthPixels.SetNum(PointsCapSpace.Num());
		PrefixLengthPixels[0] = 0.0;
		for (int32 Index = 1; Index < PointsCapSpace.Num(); ++Index)
		{
			PrefixLengthPixels[Index] =
				PrefixLengthPixels[Index - 1] +
				FVector2D::Distance(
					PointsCapSpace[Index - 1],
					PointsCapSpace[Index]);
		}

		TArray<FRedDirectionalGroup> Groups;
		for (int32 SegmentIndex = 0;
			SegmentIndex + 1 < SimplifiedIndices.Num();
			++SegmentIndex)
		{
			const int32 StartIndex = SimplifiedIndices[SegmentIndex];
			const int32 EndIndex = SimplifiedIndices[SegmentIndex + 1];
			if (EndIndex <= StartIndex)
			{
				continue;
			}
			TArray<FVector2D> SegmentUV;
			SegmentUV.Reserve(EndIndex - StartIndex + 1);
			for (int32 PointIndex = StartIndex;
				PointIndex <= EndIndex;
				++PointIndex)
			{
				SegmentUV.Add(PointsUV[PointIndex]);
			}
			FVector2D DirectionUV;
			double PathLengthUV = 0.0;
			if (!ComputePrincipalDirection2D(
				SegmentUV,
				DirectionUV,
				PathLengthUV))
			{
				continue;
			}

			FRedDirectionalGroup Group;
			Group.StartIndex = StartIndex;
			Group.EndIndex = EndIndex;
			Group.AxisType = ClassifyRedMacroOctilinearAxis(DirectionUV, Params);
			Group.LengthPixels =
				PrefixLengthPixels[EndIndex] -
				PrefixLengthPixels[StartIndex];
			if (Groups.Num() > 0 &&
				Groups.Last().AxisType == Group.AxisType)
			{
				Groups.Last().EndIndex = Group.EndIndex;
				Groups.Last().LengthPixels += Group.LengthPixels;
			}
			else
			{
				Groups.Add(Group);
			}
		}

		if (Groups.Num() == 0)
		{
			OutIndices = { 0, LastIndex };
			OutDebug.RedProtectionMode = TEXT("degenerate_groups_macro_edge");
			return;
		}

		for (const FRedDirectionalGroup& Group : Groups)
		{
			OutDebug.RedDirectionalGroupAxisTypes.Add(
				OctilinearAxisName(Group.AxisType));
			OutDebug.RedDirectionalGroupLengthsPixels.Add(Group.LengthPixels);
			OutDebug.RedDirectionalGroupStartIndices.Add(Group.StartIndex);
			OutDebug.RedDirectionalGroupEndIndices.Add(Group.EndIndex);
		}

		OutIndices.Add(0);
		for (int32 GroupIndex = 0; GroupIndex + 1 < Groups.Num(); ++GroupIndex)
		{
			const FRedDirectionalGroup& Incoming = Groups[GroupIndex];
			const FRedDirectionalGroup& Outgoing = Groups[GroupIndex + 1];
			if (Incoming.LengthPixels >=
					FMath::Max(0.0, double(Params.WorldOrthoRedMacroGroupMinLengthPixels)) &&
				Outgoing.LengthPixels >=
					FMath::Max(0.0, double(Params.WorldOrthoRedMacroGroupMinLengthPixels)))
			{
				OutIndices.Add(Incoming.EndIndex);
			}
		}
		OutIndices.Add(LastIndex);
		OutDebug.RedProtectionMode =
			OutIndices.Num() > 2
				? TEXT("macro_directional_corners")
				: TEXT("short_groups_absorbed_macro_edge");
	}

	static double DirectionAngleToWorldUVAxis(const FVector2D& Direction, int32 Axis);

	static double DirectionAngleToOctilinearAxis(const FVector2D& Direction, EOctilinearAxis Axis)
	{
		const FVector2D Unit = Direction.GetSafeNormal();
		FVector2D TargetAxis;
		switch (Axis)
		{
		case Octi_U:        TargetAxis = FVector2D(1.0, 0.0); break;
		case Octi_V:        TargetAxis = FVector2D(0.0, 1.0); break;
		case Octi_DiagPlus: TargetAxis = FVector2D(1.0, 1.0).GetSafeNormal(); break;
		case Octi_DiagMinus:TargetAxis = FVector2D(1.0,-1.0).GetSafeNormal(); break;
		default:            TargetAxis = FVector2D(1.0, 0.0); break;
		}
		const double Dot = FMath::Abs(FVector2D::DotProduct(Unit, TargetAxis));
		return FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(Dot, 0.0, 1.0)));
	}

	static EOctilinearAxis ClassifyEdgeAxisType(
		const FWorldOrthogonalEdge& Edge,
		const FFromLZFaceReconstructionParams& Params)
	{
		return ClassifyOctilinearAxisByThreshold(
			Edge.AngleToU,
			Edge.AngleToV,
			Edge.AngleToDiagPlus,
			Edge.AngleToDiagMinus,
			Params);
	}

	static FString OctilinearAxisName(EOctilinearAxis Axis)
	{
		switch (Axis)
		{
		case Octi_U:         return TEXT("U");
		case Octi_V:         return TEXT("V");
		case Octi_DiagPlus:  return TEXT("Diag+");
		case Octi_DiagMinus: return TEXT("Diag-");
		default:             return TEXT("Unknown");
		}
	}

	static double OctilinearAxisAngleDegrees(EOctilinearAxis Axis)
	{
		switch (Axis)
		{
		case Octi_U:         return 0.0;
		case Octi_DiagPlus:  return 45.0;
		case Octi_V:         return 90.0;
		case Octi_DiagMinus: return 135.0;
		default:             return 0.0;
		}
	}

	static double DirectionAngleDegrees180(const FVector2D& Direction)
	{
		double Angle = FMath::RadiansToDegrees(
			FMath::Atan2(Direction.Y, Direction.X));
		while (Angle < 0.0)
		{
			Angle += 180.0;
		}
		while (Angle >= 180.0)
		{
			Angle -= 180.0;
		}
		return Angle;
	}

	static double RotationToOctilinearAxisDegrees(
		const FVector2D& Direction,
		EOctilinearAxis Axis)
	{
		const double SourceAngle = DirectionAngleDegrees180(Direction);
		const double BaseTarget = OctilinearAxisAngleDegrees(Axis);
		double BestRotation = BaseTarget - SourceAngle;
		for (int32 Offset = -2; Offset <= 2; ++Offset)
		{
			const double Candidate =
				BaseTarget + 180.0 * double(Offset) - SourceAngle;
			if (FMath::Abs(Candidate) < FMath::Abs(BestRotation))
			{
				BestRotation = Candidate;
			}
		}
		return BestRotation;
	}

	static FVector2D RotatePointAround2D(
		const FVector2D& Point,
		const FVector2D& Center,
		double RotationDegrees)
	{
		const double Angle = FMath::DegreesToRadians(RotationDegrees);
		const double CosAngle = FMath::Cos(Angle);
		const double SinAngle = FMath::Sin(Angle);
		const FVector2D Delta = Point - Center;
		return Center + FVector2D(
			CosAngle * Delta.X - SinAngle * Delta.Y,
			SinAngle * Delta.X + CosAngle * Delta.Y);
	}

	static void RotatePointsAround2D(
		TArray<FVector2D>& Points,
		const FVector2D& Center,
		double RotationDegrees)
	{
		for (FVector2D& Point : Points)
		{
			Point = RotatePointAround2D(Point, Center, RotationDegrees);
		}
	}

	static bool InitializeWorldOrthogonalEdge(
		FWorldOrthogonalEdge& Edge,
		const FFromLZFaceReconstructionParams& Params);

	static bool RotateAndReclassifyPureRedEdge(
		FWorldOrthogonalEdge& Edge,
		const FVector2D& Center,
		double RotationDegrees,
		const FFromLZFaceReconstructionParams& Params)
	{
		RotatePointsAround2D(Edge.SamplesUV, Center, RotationDegrees);
		Edge.StartUV = RotatePointAround2D(
			Edge.StartUV,
			Center,
			RotationDegrees);
		Edge.EndUV = RotatePointAround2D(
			Edge.EndUV,
			Center,
			RotationDegrees);
		RotatePointsAround2D(Edge.GraphNodeUV, Center, RotationDegrees);
		if (!InitializeWorldOrthogonalEdge(Edge, Params))
		{
			return false;
		}
		return true;
	}

	static double OctilinearSupportValue(
		const FVector2D& Point,
		EOctilinearAxis Axis)
	{
		switch (Axis)
		{
		case Octi_U:         return Point.Y;
		case Octi_V:         return Point.X;
		case Octi_DiagPlus:  return Point.X - Point.Y;
		case Octi_DiagMinus: return Point.X + Point.Y;
		default:             return Point.Y;
		}
	}

	static FVector2D ProjectPointToOctilinearSupport(
		const FVector2D& Point,
		EOctilinearAxis Axis,
		double SupportCoordinate)
	{
		switch (Axis)
		{
		case Octi_U:
			return FVector2D(Point.X, SupportCoordinate);
		case Octi_V:
			return FVector2D(SupportCoordinate, Point.Y);
		case Octi_DiagPlus:
		{
			const double Offset =
				0.5 * (OctilinearSupportValue(Point, Axis) - SupportCoordinate);
			return FVector2D(Point.X - Offset, Point.Y + Offset);
		}
		case Octi_DiagMinus:
		{
			const double Offset =
				0.5 * (OctilinearSupportValue(Point, Axis) - SupportCoordinate);
			return FVector2D(Point.X - Offset, Point.Y - Offset);
		}
		default:
			return Point;
		}
	}

	static bool InitializeWorldOrthogonalEdge(
		FWorldOrthogonalEdge& Edge,
		const FFromLZFaceReconstructionParams& Params)
	{
		if (Edge.SamplesUV.Num() < 2)
		{
			return false;
		}
		FVector2D PrincipalDirection;
		if (!ComputePrincipalDirection2D(
				Edge.SamplesUV,
				PrincipalDirection,
				Edge.PathLength))
		{
			return false;
		}
		const FVector2D MajorChord = Edge.EndUV - Edge.StartUV;
		Edge.DirectionUV =
			Edge.bUseMajorChordDirection && !MajorChord.IsNearlyZero()
				? MajorChord.GetSafeNormal()
				: PrincipalDirection;
		Edge.CapPathLengthPixels = 0.0;
		for (int32 Index = 1; Index < Edge.SamplesCapSpace.Num(); ++Index)
		{
			Edge.CapPathLengthPixels += FVector2D::Distance(
				Edge.SamplesCapSpace[Index - 1],
				Edge.SamplesCapSpace[Index]);
		}
		Edge.AngleToU = DirectionAngleToWorldUVAxis(Edge.DirectionUV, 0);
		Edge.AngleToV = DirectionAngleToWorldUVAxis(Edge.DirectionUV, 1);
		Edge.AngleToDiagPlus = DirectionAngleToOctilinearAxis(Edge.DirectionUV, Octi_DiagPlus);
		Edge.AngleToDiagMinus = DirectionAngleToOctilinearAxis(Edge.DirectionUV, Octi_DiagMinus);
		Edge.AxisType = ClassifyEdgeAxisType(Edge, Params);
		Edge.AngleToAssignedAxis = DirectionAngleToOctilinearAxis(Edge.DirectionUV, Edge.AxisType);
		return true;
	}

	static double UnorientedDirectionAngleDegrees(
		const FVector2D& A,
		const FVector2D& B)
	{
		const FVector2D UnitA = A.GetSafeNormal();
		const FVector2D UnitB = B.GetSafeNormal();
		if (UnitA.IsNearlyZero() || UnitB.IsNearlyZero())
		{
			return 90.0;
		}
		const double Dot = FMath::Abs(FVector2D::DotProduct(UnitA, UnitB));
		return FMath::RadiansToDegrees(
			FMath::Acos(FMath::Clamp(Dot, 0.0, 1.0)));
	}

	static bool MergeOrderedWorldOrthogonalEdges(
		const FWorldOrthogonalEdge& A,
		const FWorldOrthogonalEdge& B,
		FWorldOrthogonalEdge& Out,
		const FFromLZFaceReconstructionParams& Params)
	{
		if (A.Type != B.Type)
		{
			return false;
		}
		Out = A;
		auto AppendDistinctPoints = [](
			const TArray<FVector2D>& Source,
			TArray<FVector2D>& Target)
		{
			for (const FVector2D& Point : Source)
			{
				if (Target.Num() == 0 ||
					FVector2D::Distance(Target.Last(), Point) > 1e-7)
				{
					Target.Add(Point);
				}
			}
		};
		AppendDistinctPoints(B.SamplesUV, Out.SamplesUV);
		AppendDistinctPoints(B.SamplesCapSpace, Out.SamplesCapSpace);
		Out.EndUV = B.EndUV;
		Out.EndCapSpace = B.EndCapSpace;
		Out.SourceRunIndices.Append(B.SourceRunIndices);
		Out.SourceStrokeIds.Append(B.SourceStrokeIds);
		Out.PrimitiveEdgeIndices.Append(B.PrimitiveEdgeIndices);
		Out.GraphNodeIds.Append(B.GraphNodeIds);
		Out.GraphNodeCapSpace.Append(B.GraphNodeCapSpace);
		Out.GraphNodeUV.Append(B.GraphNodeUV);
		Out.SnappedGraphNodeUV.Reset();
		Out.GraphNodeSnapDistancesPixels.Reset();
		Out.bUseMajorChordDirection = true;
		return InitializeWorldOrthogonalEdge(Out, Params);
	}

	static int32 CollapseShortRedPrimitiveEdges(
		TArray<FWorldOrthogonalEdge>& Edges,
		const FFromLZFaceReconstructionParams& Params)
	{
		int32 CollapseCount = 0;
		bool bChanged = true;
		while (bChanged && Edges.Num() > 3)
		{
			bChanged = false;
			int32 ShortIndex = INDEX_NONE;
			double ShortestLength = TNumericLimits<double>::Max();
			for (int32 EdgeIndex = 0; EdgeIndex < Edges.Num(); ++EdgeIndex)
			{
				const FWorldOrthogonalEdge& Edge = Edges[EdgeIndex];
				if (Edge.Type == ECapBoundaryRunType::Red &&
					Edge.CapPathLengthPixels <
						FMath::Max(0.0, double(Params.WorldOrthoShortRedEdgeLengthPixels)) &&
					Edge.CapPathLengthPixels < ShortestLength)
				{
					const FWorldOrthogonalEdge& Previous =
						Edges[(EdgeIndex + Edges.Num() - 1) % Edges.Num()];
					const FWorldOrthogonalEdge& Next =
						Edges[(EdgeIndex + 1) % Edges.Num()];
					if (Previous.Type == ECapBoundaryRunType::Red &&
						Next.Type == ECapBoundaryRunType::Red &&
						FVector2D::Distance(
							Previous.EndCapSpace,
							Edge.StartCapSpace) <= 1e-6 &&
						FVector2D::Distance(
							Edge.EndCapSpace,
							Next.StartCapSpace) <= 1e-6)
					{
						ShortIndex = EdgeIndex;
						ShortestLength = Edge.CapPathLengthPixels;
					}
				}
			}
			if (ShortIndex == INDEX_NONE)
			{
				break;
			}

			const int32 PreviousIndex =
				(ShortIndex + Edges.Num() - 1) % Edges.Num();
			const int32 NextIndex = (ShortIndex + 1) % Edges.Num();
			const FWorldOrthogonalEdge& Short = Edges[ShortIndex];
			const FWorldOrthogonalEdge& Previous = Edges[PreviousIndex];
			const FWorldOrthogonalEdge& Next = Edges[NextIndex];
			const FVector2D ShortChord =
				Short.EndCapSpace - Short.StartCapSpace;
			const double PreviousAngle = UnorientedDirectionAngleDegrees(
				ShortChord,
				Previous.EndCapSpace - Previous.StartCapSpace);
			const double NextAngle = UnorientedDirectionAngleDegrees(
				ShortChord,
				Next.EndCapSpace - Next.StartCapSpace);
			const bool bMergeIntoPrevious =
				PreviousAngle < NextAngle ||
				(FMath::IsNearlyEqual(PreviousAngle, NextAngle, 1e-6) &&
				 Previous.CapPathLengthPixels >= Next.CapPathLengthPixels);

			FWorldOrthogonalEdge Merged;
			if (bMergeIntoPrevious)
			{
				if (!MergeOrderedWorldOrthogonalEdges(
					Previous,
					Short,
					Merged,
					Params))
				{
					break;
				}
				if (ShortIndex > 0)
				{
					Edges[PreviousIndex] = MoveTemp(Merged);
					Edges.RemoveAt(ShortIndex);
				}
				else
				{
					Edges[PreviousIndex] = MoveTemp(Merged);
					Edges.RemoveAt(0);
				}
			}
			else
			{
				if (!MergeOrderedWorldOrthogonalEdges(
					Short,
					Next,
					Merged,
					Params))
				{
					break;
				}
				if (NextIndex > ShortIndex)
				{
					Edges[ShortIndex] = MoveTemp(Merged);
					Edges.RemoveAt(NextIndex);
				}
				else
				{
					Edges[ShortIndex] = MoveTemp(Merged);
					Edges.RemoveAt(0);
				}
			}
			++CollapseCount;
			bChanged = true;
		}
		return CollapseCount;
	}

	static FWorldOrthogonalEdgeDebug MakeWorldOrthogonalEdgeDebug(
		const FWorldOrthogonalEdge& Edge,
		int32 EdgeIndex)
	{
		FWorldOrthogonalEdgeDebug Debug;
		Debug.EdgeIndex = EdgeIndex;
		Debug.Type = Edge.Type == ECapBoundaryRunType::Black ? TEXT("black") : TEXT("red");
		Debug.AxisTypeName = OctilinearAxisName(Edge.AxisType);
		Debug.SourceRunIndices = Edge.SourceRunIndices;
		Debug.SourceStrokeIds = Edge.SourceStrokeIds;
		Debug.PrimitiveEdgeIndices = Edge.PrimitiveEdgeIndices;
		Debug.GraphNodeIds = Edge.GraphNodeIds;
		Debug.GraphNodeCapSpace = Edge.GraphNodeCapSpace;
		Debug.GraphNodeUV = Edge.GraphNodeUV;
		Debug.SnappedGraphNodeUV = Edge.SnappedGraphNodeUV;
		Debug.GraphNodeSnapDistancesPixels = Edge.GraphNodeSnapDistancesPixels;
		Debug.SamplesUV = Edge.SamplesUV;
		Debug.StartUV = Edge.StartUV;
		Debug.EndUV = Edge.EndUV;
		Debug.DirectionUV = Edge.DirectionUV;
		Debug.AxisType = static_cast<int32>(Edge.AxisType);
		Debug.AngleToU = Edge.AngleToU;
		Debug.AngleToV = Edge.AngleToV;
		Debug.AngleToDiagPlus = Edge.AngleToDiagPlus;
		Debug.AngleToDiagMinus = Edge.AngleToDiagMinus;
		Debug.AngleToAssignedAxis = Edge.AngleToAssignedAxis;
		Debug.SupportCoordinate = Edge.SupportCoordinate;
		Debug.PathLength = Edge.PathLength;
		return Debug;
	}

	static void CaptureWorldOrthogonalEdgeDebug(
		const TArray<FWorldOrthogonalEdge>& Edges,
		TArray<FWorldOrthogonalEdgeDebug>& OutDebug)
	{
		OutDebug.Reset();
		OutDebug.Reserve(Edges.Num());
		for (int32 Index = 0; Index < Edges.Num(); ++Index)
		{
			OutDebug.Add(MakeWorldOrthogonalEdgeDebug(Edges[Index], Index));
		}
	}

	static double WeightedMedianForEdgeOctilinear(
		const TArray<FVector2D>& Points,
		EOctilinearAxis Axis);

	struct FAdjacencyViolation
	{
		int32 EdgeIndex = INDEX_NONE;
		int32 NextIndex = INDEX_NONE;
	};

	static bool TryTopologyRepair(
		TArray<FWorldOrthogonalEdge>& Edges,
		TFunctionRef<double(const FVector2D&, const FVector2D&)> DistancePixels,
		const FFromLZFaceReconstructionParams& Params,
		FCapBBoxRegularizationResult& OutDebug,
		double CapDiagonal)
	{
		OutDebug.bTopologyRepairAttempted = false;
		OutDebug.bTopologyRepairApplied = false;
		OutDebug.TopologyRepairInsertedCount = 0;
		OutDebug.TopologyRepairMaxGapPixels = 0.0;
		OutDebug.TopologyRepairInsertedAxisTypeNames.Empty();

		if (!Params.bWorldOrthoAllowTopologyRepair || Edges.Num() < 3)
		{
			return true;
		}

		// Scan all cyclic pairs for same-axis violations
		TArray<FAdjacencyViolation> Violations;
		for (int32 i = 0; i < Edges.Num(); ++i)
		{
			const int32 Next = (i + 1) % Edges.Num();
			if (Edges[i].AxisType == Edges[Next].AxisType)
			{
				Violations.Add({ i, Next });
			}
		}

		if (Violations.Num() == 0)
		{
			return true;  // no violations, pass
		}

		OutDebug.bTopologyRepairAttempted = true;

		if (Violations.Num() > 2)
		{
			SetRegularizationFallback(
				OutDebug,
				FString::Printf(
					TEXT("%d same-axis violations detected; topology-repair supports at most 2"),
					Violations.Num()));
			return false;
		}

		const double GapThreshold = FMath::Max(0.0, double(Params.WorldOrthoMaxWrapGapFraction)) * CapDiagonal;

		// Sort violations in reverse index order so insertions don't shift later indices
		Violations.Sort([](const FAdjacencyViolation& A, const FAdjacencyViolation& B)
		{
			return A.EdgeIndex > B.EdgeIndex;
		});

		int32 InsertedCount = 0;
		double MaxGapPixels = 0.0;
		FString InsertedAxes;

		for (const FAdjacencyViolation& V : Violations)
		{
			const FWorldOrthogonalEdge& EdgeA = Edges[V.EdgeIndex];
			const FWorldOrthogonalEdge& EdgeB = Edges[V.NextIndex];

			// Guard: at least one edge must be red
			if (EdgeA.Type != ECapBoundaryRunType::Red &&
				EdgeB.Type != ECapBoundaryRunType::Red)
			{
				SetRegularizationFallback(
					OutDebug,
					FString::Printf(
						TEXT("violation at pair (%d,%d): both edges are black; topology-repair requires at least one red edge"),
						V.EdgeIndex, V.NextIndex));
				return false;
			}

			// Guard: restricted to U/V pairs only
			if (EdgeA.AxisType != Octi_U && EdgeA.AxisType != Octi_V)
			{
				SetRegularizationFallback(
					OutDebug,
					FString::Printf(
						TEXT("violation at pair (%d,%d) involves diagonal axis (%s); topology-repair restricted to U/V pairs only"),
						V.EdgeIndex, V.NextIndex, *OctilinearAxisName(EdgeA.AxisType)));
				return false;
			}

			// Compute gap
			const double GapPixels = DistancePixels(EdgeA.EndUV, EdgeB.StartUV);
			if (GapPixels > GapThreshold)
			{
				SetRegularizationFallback(
					OutDebug,
					FString::Printf(
						TEXT("violation at pair (%d,%d): gap %.1fpx exceeds threshold %.1fpx (%.1f%% of cap diagonal %.1fpx); repair skipped"),
						V.EdgeIndex, V.NextIndex, GapPixels, GapThreshold,
						(CapDiagonal > 1e-6) ? (100.0 * GapThreshold / CapDiagonal) : 0.0, CapDiagonal));
				return false;
			}

			// Determine complementary axis
			const EOctilinearAxis NewAxis = (EdgeA.AxisType == Octi_U) ? Octi_V : Octi_U;

			// Compute support coordinate from midpoint
			const FVector2D MidpointUV = 0.5 * (EdgeA.EndUV + EdgeB.StartUV);
			const double NewSupport = (NewAxis == Octi_V) ? MidpointUV.X : MidpointUV.Y;

			// Direction and angles
			const FVector2D DirectionUV = EdgeB.StartUV - EdgeA.EndUV;
			const double AngleToU = DirectionAngleToWorldUVAxis(DirectionUV, 0);
			const double AngleToV = DirectionAngleToWorldUVAxis(DirectionUV, 1);
			const double AngleToAssigned = (NewAxis == Octi_U) ? AngleToU : AngleToV;

			// Construct synthetic edge
			FWorldOrthogonalEdge Synthetic;
			Synthetic.Type = ECapBoundaryRunType::Red;
			Synthetic.AxisType = NewAxis;
			Synthetic.SupportCoordinate = NewSupport;
			Synthetic.SamplesUV = { EdgeA.EndUV, EdgeB.StartUV };
			Synthetic.SamplesCapSpace = { EdgeA.EndCapSpace, EdgeB.StartCapSpace };
			Synthetic.StartUV = EdgeA.EndUV;
			Synthetic.EndUV = EdgeB.StartUV;
			Synthetic.StartCapSpace = EdgeA.EndCapSpace;
			Synthetic.EndCapSpace = EdgeB.StartCapSpace;
			Synthetic.DirectionUV = DirectionUV;
			// PathLength is in UV-space units; CapPathLengthPixels is direct cap-space pixels.
			Synthetic.PathLength = FVector2D::Distance(EdgeA.EndUV, EdgeB.StartUV);
			Synthetic.CapPathLengthPixels = FVector2D::Distance(EdgeA.EndCapSpace, EdgeB.StartCapSpace);
			Synthetic.AngleToU = AngleToU;
			Synthetic.AngleToV = AngleToV;
			Synthetic.AngleToAssignedAxis = AngleToAssigned;

			Edges.Insert(Synthetic, V.EdgeIndex + 1);
			++InsertedCount;
			MaxGapPixels = FMath::Max(MaxGapPixels, GapPixels);
			if (!InsertedAxes.IsEmpty()) InsertedAxes += TEXT(",");
			InsertedAxes += OctilinearAxisName(NewAxis);
		}

		// Post-insertion validation: re-check all pairs
		for (int32 i = 0; i < Edges.Num(); ++i)
		{
			const int32 Next = (i + 1) % Edges.Num();
			if (Edges[i].AxisType == Edges[Next].AxisType)
			{
				SetRegularizationFallback(
					OutDebug,
					TEXT("synthetic topology-repair edges did not resolve all adjacency violations"));
				return false;
			}
		}

		// Success
		OutDebug.bTopologyRepairApplied = true;
		OutDebug.TopologyRepairInsertedCount = InsertedCount;
		OutDebug.TopologyRepairMaxGapPixels = MaxGapPixels;
		OutDebug.TopologyRepairInsertedAxisTypeNames = InsertedAxes;
		return true;
	}

	static bool MergeAdjacentWorldOrthogonalEdges(
		TArray<FWorldOrthogonalEdge>& Edges,
		TFunctionRef<double(const FVector2D&, const FVector2D&)> DistancePixels,
		const FFromLZFaceReconstructionParams& Params)
	{
		auto AppendGraphNodes = [](
			const FWorldOrthogonalEdge& Source,
			FWorldOrthogonalEdge& Target)
		{
			for (int32 NodeIndex = 0; NodeIndex < Source.GraphNodeIds.Num(); ++NodeIndex)
			{
				const int32 NodeId = Source.GraphNodeIds[NodeIndex];
				const bool bDuplicateSharedNode =
					Target.GraphNodeIds.Num() > 0 &&
					Target.GraphNodeIds.Last() == NodeId &&
					NodeId != INDEX_NONE;
				if (bDuplicateSharedNode)
				{
					continue;
				}
				Target.GraphNodeIds.Add(NodeId);
				Target.GraphNodeCapSpace.Add(
					Source.GraphNodeCapSpace.IsValidIndex(NodeIndex)
						? Source.GraphNodeCapSpace[NodeIndex]
						: FVector2D::ZeroVector);
				Target.GraphNodeUV.Add(
					Source.GraphNodeUV.IsValidIndex(NodeIndex)
						? Source.GraphNodeUV[NodeIndex]
						: FVector2D::ZeroVector);
			}
		};

		auto CanMerge = [&](const FWorldOrthogonalEdge& A, const FWorldOrthogonalEdge& B)
		{
			if (A.Type != B.Type || A.AxisType != B.AxisType)
			{
				return false;
			}
			if (A.Type == ECapBoundaryRunType::Red)
			{
				return true;
			}
			if (A.Type != ECapBoundaryRunType::Black ||
				A.GraphNodeIds.Num() < 2 ||
				B.GraphNodeIds.Num() < 2 ||
				A.GraphNodeIds.Last() == INDEX_NONE ||
				A.GraphNodeIds.Last() != B.GraphNodeIds[0])
			{
				return false;
			}

			TArray<FVector2D> CombinedSamples = A.SamplesUV;
			for (const FVector2D& Sample : B.SamplesUV)
			{
				if (CombinedSamples.Num() == 0 ||
					FVector2D::Distance(CombinedSamples.Last(), Sample) > 1e-7)
				{
					CombinedSamples.Add(Sample);
				}
			}
			const double CombinedSupport =
				WeightedMedianForEdgeOctilinear(CombinedSamples, A.AxisType);
			auto NodesFitSupport = [&](const FWorldOrthogonalEdge& Edge)
			{
				for (const FVector2D& NodeUV : Edge.GraphNodeUV)
				{
					const FVector2D Snapped = ProjectPointToOctilinearSupport(
						NodeUV,
						A.AxisType,
						CombinedSupport);
					if (DistancePixels(NodeUV, Snapped) >
						FMath::Max(0.0, double(Params.WorldOrthoBlackNodeSnapTolerancePixels)))
					{
						return false;
					}
				}
				return true;
			};
			return NodesFitSupport(A) && NodesFitSupport(B);
		};

		auto MergePair = [&](const FWorldOrthogonalEdge& A, const FWorldOrthogonalEdge& B, FWorldOrthogonalEdge& Out)
		{
			Out = A;
			for (const FVector2D& Sample : B.SamplesUV)
			{
				if (Out.SamplesUV.Num() == 0 ||
					FVector2D::Distance(Out.SamplesUV.Last(), Sample) > 1e-7)
				{
					Out.SamplesUV.Add(Sample);
				}
			}
			for (const FVector2D& Sample : B.SamplesCapSpace)
			{
				if (Out.SamplesCapSpace.Num() == 0 ||
					FVector2D::Distance(
						Out.SamplesCapSpace.Last(),
						Sample) > 1e-7)
				{
					Out.SamplesCapSpace.Add(Sample);
				}
			}
			Out.EndUV = B.EndUV;
			Out.EndCapSpace = B.EndCapSpace;
			Out.SourceRunIndices.Append(B.SourceRunIndices);
			Out.SourceStrokeIds.Append(B.SourceStrokeIds);
			Out.PrimitiveEdgeIndices.Append(B.PrimitiveEdgeIndices);
			AppendGraphNodes(B, Out);
			Out.SnappedGraphNodeUV.Reset();
			Out.GraphNodeSnapDistancesPixels.Reset();
			Out.bUseMajorChordDirection =
				A.bUseMajorChordDirection ||
				B.bUseMajorChordDirection;
			const EOctilinearAxis MergedAxisType = A.AxisType;
			if (!InitializeWorldOrthogonalEdge(Out, Params))
			{
				return false;
			}
			Out.AxisType = MergedAxisType;
			Out.AngleToAssignedAxis = DirectionAngleToOctilinearAxis(Out.DirectionUV, MergedAxisType);
			return true;
		};

		for (int32 Index = 0; Index + 1 < Edges.Num();)
		{
			if (!CanMerge(Edges[Index], Edges[Index + 1]))
			{
				++Index;
				continue;
			}

			FWorldOrthogonalEdge Merged;
			if (!MergePair(Edges[Index], Edges[Index + 1], Merged))
			{
				return false;
			}
			Edges[Index] = MoveTemp(Merged);
			Edges.RemoveAt(Index + 1);
			if (Index > 0)
			{
				--Index;
			}
		}

		if (Edges.Num() > 1 && CanMerge(Edges.Last(), Edges[0]))
		{
			FWorldOrthogonalEdge Merged;
			if (!MergePair(Edges.Last(), Edges[0], Merged))
			{
				return false;
			}
			Edges.RemoveAt(Edges.Num() - 1);
			Edges[0] = MoveTemp(Merged);
		}
		return true;
	}

	static double DirectionAngleToWorldUVAxis(const FVector2D& Direction, int32 Axis)
	{
		const FVector2D Unit = Direction.GetSafeNormal();
		const double Dot = Axis == 0 ? FMath::Abs(Unit.X) : FMath::Abs(Unit.Y);
		return FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(Dot, 0.0, 1.0)));
	}

	static double WeightedMedianForEdge(
		const TArray<FVector2D>& Points,
		int32 EdgeAxis)
	{
		struct FWeightedCoordinate
		{
			double Value = 0.0;
			double Weight = 0.0;
		};

		TArray<FWeightedCoordinate> Values;
		Values.Reserve(Points.Num());
		double TotalWeight = 0.0;
		for (int32 Index = 0; Index < Points.Num(); ++Index)
		{
			double Weight = 0.0;
			if (Index > 0)
			{
				Weight += 0.5 * FVector2D::Distance(Points[Index - 1], Points[Index]);
			}
			if (Index + 1 < Points.Num())
			{
				Weight += 0.5 * FVector2D::Distance(Points[Index], Points[Index + 1]);
			}
			Weight = FMath::Max(Weight, 1e-6);
			FWeightedCoordinate Item;
			Item.Value = EdgeAxis == 0 ? Points[Index].Y : Points[Index].X;
			Item.Weight = Weight;
			Values.Add(Item);
			TotalWeight += Weight;
		}
		Values.Sort([](const FWeightedCoordinate& A, const FWeightedCoordinate& B)
		{
			return A.Value < B.Value;
		});

		double Accumulated = 0.0;
		for (const FWeightedCoordinate& Item : Values)
		{
			Accumulated += Item.Weight;
			if (Accumulated * 2.0 >= TotalWeight)
			{
				return Item.Value;
			}
		}
		return Values.Num() > 0 ? Values.Last().Value : 0.0;
	}

	static double WeightedMedianForEdgeOctilinear(
		const TArray<FVector2D>& Points,
		EOctilinearAxis Axis)
	{
		struct FWeightedCoordinate
		{
			double Value = 0.0;
			double Weight = 0.0;
		};

		TArray<FWeightedCoordinate> Values;
		Values.Reserve(Points.Num());
		double TotalWeight = 0.0;
		for (int32 Index = 0; Index < Points.Num(); ++Index)
		{
			double Weight = 0.0;
			if (Index > 0)
			{
				Weight += 0.5 * FVector2D::Distance(Points[Index - 1], Points[Index]);
			}
			if (Index + 1 < Points.Num())
			{
				Weight += 0.5 * FVector2D::Distance(Points[Index], Points[Index + 1]);
			}
			Weight = FMath::Max(Weight, 1e-6);
			double Val = 0.0;
			switch (Axis)
			{
			case Octi_U:         Val = Points[Index].Y; break;
			case Octi_V:         Val = Points[Index].X; break;
			case Octi_DiagPlus:  Val = Points[Index].X - Points[Index].Y; break;
			case Octi_DiagMinus: Val = Points[Index].X + Points[Index].Y; break;
			default:             Val = Points[Index].Y; break;
			}
			FWeightedCoordinate Item;
			Item.Value = Val;
			Item.Weight = Weight;
			Values.Add(Item);
			TotalWeight += Weight;
		}
		Values.Sort([](const FWeightedCoordinate& A, const FWeightedCoordinate& B)
		{
			return A.Value < B.Value;
		});

		double Accumulated = 0.0;
		for (const FWeightedCoordinate& Item : Values)
		{
			Accumulated += Item.Weight;
			if (Accumulated * 2.0 >= TotalWeight)
			{
				return Item.Value;
			}
		}
		return Values.Num() > 0 ? Values.Last().Value : 0.0;
	}

	// Solve the intersection point of two octilinear support lines.
	// InAxis/OutAxis: the axis type of the previous (incoming) and current (outgoing) edge
	// InSupport/OutSupport: the support coordinate for each edge
	//
	// U-line:      V = InSupport  (horizontal, fixed V)
	// V-line:      U = InSupport  (vertical, fixed U)
	// Diag+ line:  U - V = InSupport  (45deg diagonal)
	// Diag- line:  U + V = InSupport  (135deg diagonal)
	//
	// Returns the (U, V) intersection point.
	static FVector2D SolveOctilinearVertexIntersection(
		EOctilinearAxis InAxis, double InSupport,
		EOctilinearAxis OutAxis, double OutSupport)
	{
		double U = 0.0, V = 0.0;

		// 12 valid intersection types (same-type pairs are rejected by adjacency_validation)
		if (InAxis == Octi_U && OutAxis == Octi_V)
		{
			U = OutSupport; V = InSupport;
		}
		else if (InAxis == Octi_V && OutAxis == Octi_U)
		{
			U = InSupport; V = OutSupport;
		}
		else if (InAxis == Octi_U && OutAxis == Octi_DiagPlus)
		{
			V = InSupport; U = V + OutSupport;
		}
		else if (InAxis == Octi_U && OutAxis == Octi_DiagMinus)
		{
			V = InSupport; U = OutSupport - V;
		}
		else if (InAxis == Octi_V && OutAxis == Octi_DiagPlus)
		{
			U = InSupport; V = U - OutSupport;
		}
		else if (InAxis == Octi_V && OutAxis == Octi_DiagMinus)
		{
			U = InSupport; V = OutSupport - U;
		}
		else if (InAxis == Octi_DiagPlus && OutAxis == Octi_U)
		{
			V = OutSupport; U = V + InSupport;
		}
		else if (InAxis == Octi_DiagMinus && OutAxis == Octi_U)
		{
			V = OutSupport; U = InSupport - V;
		}
		else if (InAxis == Octi_DiagPlus && OutAxis == Octi_V)
		{
			U = OutSupport; V = U - InSupport;
		}
		else if (InAxis == Octi_DiagMinus && OutAxis == Octi_V)
		{
			U = OutSupport; V = InSupport - U;
		}
		else if (InAxis == Octi_DiagPlus && OutAxis == Octi_DiagMinus)
		{
			U = (InSupport + OutSupport) / 2.0;
			V = (OutSupport - InSupport) / 2.0;
		}
		else if (InAxis == Octi_DiagMinus && OutAxis == Octi_DiagPlus)
		{
			U = (InSupport + OutSupport) / 2.0;
			V = (InSupport - OutSupport) / 2.0;
		}
		else
		{
			return FVector2D(0.0, 0.0);
		}

		return FVector2D(U, V);
	}

	static FVector2D PolygonAreaCentroid2D(const TArray<FVector2D>& Polygon)
	{
		double TwiceArea = 0.0;
		FVector2D Weighted = FVector2D::ZeroVector;
		for (int32 Index = 0; Index < Polygon.Num(); ++Index)
		{
			const FVector2D& A = Polygon[Index];
			const FVector2D& B = Polygon[(Index + 1) % Polygon.Num()];
			const double Cross = A.X * B.Y - B.X * A.Y;
			TwiceArea += Cross;
			Weighted += (A + B) * Cross;
		}
		if (FMath::Abs(TwiceArea) <= 1e-12)
		{
			FVector2D Average = FVector2D::ZeroVector;
			for (const FVector2D& Point : Polygon)
			{
				Average += Point;
			}
			return Polygon.Num() > 0 ? Average / double(Polygon.Num()) : FVector2D::ZeroVector;
		}
		return Weighted / (3.0 * TwiceArea);
	}

	static double Orientation2D(const FVector2D& A, const FVector2D& B, const FVector2D& C)
	{
		return FVector2D::CrossProduct(B - A, C - A);
	}

	static bool SegmentsIntersectStrict2D(
		const FVector2D& A,
		const FVector2D& B,
		const FVector2D& C,
		const FVector2D& D)
	{
		const double O1 = Orientation2D(A, B, C);
		const double O2 = Orientation2D(A, B, D);
		const double O3 = Orientation2D(C, D, A);
		const double O4 = Orientation2D(C, D, B);
		return ((O1 > 1e-8 && O2 < -1e-8) || (O1 < -1e-8 && O2 > 1e-8)) &&
			((O3 > 1e-8 && O4 < -1e-8) || (O3 < -1e-8 && O4 > 1e-8));
	}

	static bool PolygonHasSelfIntersection(const TArray<FVector2D>& Polygon)
	{
		const int32 Count = Polygon.Num();
		for (int32 AIndex = 0; AIndex < Count; ++AIndex)
		{
			const int32 ANext = (AIndex + 1) % Count;
			for (int32 BIndex = AIndex + 1; BIndex < Count; ++BIndex)
			{
				const int32 BNext = (BIndex + 1) % Count;
				if (AIndex == BIndex || ANext == BIndex || BNext == AIndex)
				{
					continue;
				}
				if (SegmentsIntersectStrict2D(
					Polygon[AIndex],
					Polygon[ANext],
					Polygon[BIndex],
					Polygon[BNext]))
				{
					return true;
				}
			}
		}
		return false;
	}

	static void FindPolygonSelfIntersections2D(
		const TArray<FVector2D>& Polygon,
		TArray<FIntPoint>& OutEdgePairs)
	{
		OutEdgePairs.Reset();
		const int32 Count = Polygon.Num();
		for (int32 AIndex = 0; AIndex < Count; ++AIndex)
		{
			const int32 ANext = (AIndex + 1) % Count;
			for (int32 BIndex = AIndex + 1; BIndex < Count; ++BIndex)
			{
				const int32 BNext = (BIndex + 1) % Count;
				if (AIndex == BIndex || ANext == BIndex || BNext == AIndex)
				{
					continue;
				}
				if (SegmentsIntersectStrict2D(
					Polygon[AIndex],
					Polygon[ANext],
					Polygon[BIndex],
					Polygon[BNext]))
				{
					OutEdgePairs.Add(FIntPoint(AIndex, BIndex));
				}
			}
		}
	}

	static double MinClosedLoopEdgeLength2D(const TArray<FVector2D>& Loop)
	{
		if (Loop.Num() < 2)
		{
			return 0.0;
		}
		double MinLen = TNumericLimits<double>::Max();
		for (int32 Index = 0; Index < Loop.Num(); ++Index)
		{
			MinLen = FMath::Min(
				MinLen,
				FVector2D::Distance(Loop[Index], Loop[(Index + 1) % Loop.Num()]));
		}
		return FMath::IsFinite(MinLen) ? MinLen : 0.0;
	}

	static bool CheckSupportPlaneProjectionTopology(
		const FString& Stage,
		const TArray<FVector2D>& SourceLoop2D,
		const TArray<FVector>& LoopWorld,
		const FVector& Origin,
		const FVector& AxisU,
		const FVector& AxisV,
		FSupportPlaneTopologyDebug& OutDebug)
	{
		constexpr double MinProjectedAreaCm2 = 1e-4;
		constexpr double MinProjectedEdgeCm = 1e-3;
		constexpr double MinSourceAreaPx2 = 1e-3;
		constexpr double MinSourceEdgePx = 1e-3;

		OutDebug = FSupportPlaneTopologyDebug();
		OutDebug.bChecked = true;
		OutDebug.Stage = Stage;
		OutDebug.SourceVertexCount = SourceLoop2D.Num();
		OutDebug.WorldVertexCount = LoopWorld.Num();
		OutDebug.SourceAreaPx2 = FMath::Abs(PolygonArea2D(SourceLoop2D));
		OutDebug.MinSourceEdgePx = MinClosedLoopEdgeLength2D(SourceLoop2D);

		if (SourceLoop2D.Num() < 3)
		{
			OutDebug.Reason = TEXT("source cap loop has fewer than three vertices");
			return false;
		}
		if (LoopWorld.Num() != SourceLoop2D.Num())
		{
			OutDebug.Reason = FString::Printf(
				TEXT("source/world vertex count mismatch: %d vs %d"),
				SourceLoop2D.Num(),
				LoopWorld.Num());
			return false;
		}
		if (OutDebug.SourceAreaPx2 <= MinSourceAreaPx2)
		{
			OutDebug.Reason = FString::Printf(
				TEXT("source cap loop area %.9f px^2 is degenerate"),
				OutDebug.SourceAreaPx2);
			return false;
		}
		if (OutDebug.MinSourceEdgePx <= MinSourceEdgePx)
		{
			OutDebug.Reason = FString::Printf(
				TEXT("source cap loop min edge %.9f px is degenerate"),
				OutDebug.MinSourceEdgePx);
			return false;
		}
		FindPolygonSelfIntersections2D(SourceLoop2D, OutDebug.SourceSelfIntersectionEdges);
		if (OutDebug.SourceSelfIntersectionEdges.Num() > 0)
		{
			OutDebug.Reason = TEXT("source cap loop self-intersects before support-plane projection");
			return false;
		}

		const FVector UnitU = AxisU.GetSafeNormal();
		const FVector UnitV = AxisV.GetSafeNormal();
		if (UnitU.IsNearlyZero() || UnitV.IsNearlyZero())
		{
			OutDebug.Reason = TEXT("virtual base plane topology basis is degenerate");
			return false;
		}

		OutDebug.ProjectedUVLoop.Reset();
		OutDebug.ProjectedUVLoop.Reserve(LoopWorld.Num());
		for (const FVector& WorldPoint : LoopWorld)
		{
			if (!IsFiniteVector(WorldPoint))
			{
				OutDebug.Reason = TEXT("projected support-plane world loop contains a non-finite point");
				return false;
			}
			const FVector Delta = WorldPoint - Origin;
			const FVector2D UV(
				FVector::DotProduct(Delta, UnitU),
				FVector::DotProduct(Delta, UnitV));
			if (!IsFiniteVector2D(UV))
			{
				OutDebug.Reason = TEXT("projected support-plane UV loop contains a non-finite point");
				return false;
			}
			OutDebug.ProjectedUVLoop.Add(UV);
		}

		OutDebug.ProjectedAreaCm2 = FMath::Abs(PolygonArea2D(OutDebug.ProjectedUVLoop));
		OutDebug.MinProjectedEdgeCm = MinClosedLoopEdgeLength2D(OutDebug.ProjectedUVLoop);
		if (OutDebug.ProjectedAreaCm2 <= MinProjectedAreaCm2)
		{
			OutDebug.Reason = FString::Printf(
				TEXT("support-plane projected cap area %.9f cm^2 is degenerate"),
				OutDebug.ProjectedAreaCm2);
			return false;
		}
		if (OutDebug.MinProjectedEdgeCm <= MinProjectedEdgeCm)
		{
			OutDebug.Reason = FString::Printf(
				TEXT("support-plane projected cap min edge %.9f cm is degenerate"),
				OutDebug.MinProjectedEdgeCm);
			return false;
		}
		FindPolygonSelfIntersections2D(OutDebug.ProjectedUVLoop, OutDebug.ProjectedSelfIntersectionEdges);
		if (OutDebug.ProjectedSelfIntersectionEdges.Num() > 0)
		{
			OutDebug.Reason = TEXT("support-plane projected cap self-intersects in virtual base UV");
			return false;
		}

		OutDebug.bPass = true;
		OutDebug.Reason = TEXT("passed");
		return true;
	}

	static void ComputeBoundaryDistanceStats(
		const TArray<FVector2D>& Samples,
		const TArray<FVector2D>& Polygon,
		double& OutMean,
		double& OutMax)
	{
		OutMean = 0.0;
		OutMax = 0.0;
		if (Samples.Num() == 0 || Polygon.Num() < 2)
		{
			return;
		}

		for (const FVector2D& Sample : Samples)
		{
			double BestSquared = TNumericLimits<double>::Max();
			for (int32 EdgeIndex = 0; EdgeIndex < Polygon.Num(); ++EdgeIndex)
			{
				BestSquared = FMath::Min(
					BestSquared,
					DistancePointToSegmentSquared2D(
						Sample,
						Polygon[EdgeIndex],
						Polygon[(EdgeIndex + 1) % Polygon.Num()]));
			}
			const double Distance = FMath::Sqrt(BestSquared);
			OutMean += Distance;
			OutMax = FMath::Max(OutMax, Distance);
		}
		OutMean /= double(Samples.Num());
	}

	struct FPureRedRootHypothesisSolve
	{
		bool bValid = false;
		FString RejectionReason;
		TArray<FWorldOrthogonalEdge> Edges;
		TArray<FVector2D> CorrectedUV;
		double AngleCost = 0.0;
		double AreaRatio = 0.0;
		double MeanBoundaryDistance = 0.0;
		double MaxBoundaryDistance = 0.0;
	};

	static FPureRedRootHypothesisSolve EvaluatePureRedRootHypothesis(
		const TArray<FWorldOrthogonalEdge>& PrimitiveEdges,
		const TArray<FVector2D>& OriginalCapUV,
		const TArray<FVector2D>& BoundarySamplesUV,
		const FVector2D& RotationCenterUV,
		double RotationDegrees,
		const FFromLZFaceReconstructionParams& Params)
	{
		FPureRedRootHypothesisSolve Result;
		Result.Edges = PrimitiveEdges;
		for (FWorldOrthogonalEdge& Edge : Result.Edges)
		{
			if (Edge.Type != ECapBoundaryRunType::Red ||
				!RotateAndReclassifyPureRedEdge(
					Edge,
					RotationCenterUV,
					RotationDegrees,
					Params))
			{
				Result.RejectionReason =
					TEXT("failed to rotate and classify a pure-red primitive edge");
				return Result;
			}
		}

		auto UnusedDistancePixels = [](
			const FVector2D& A,
			const FVector2D& B)
		{
			return FVector2D::Distance(A, B);
		};
		if (!MergeAdjacentWorldOrthogonalEdges(
			Result.Edges,
			UnusedDistancePixels,
			Params))
		{
			Result.RejectionReason =
				TEXT("failed to merge same-axis pure-red support edges");
			return Result;
		}
		if (Result.Edges.Num() < 3)
		{
			Result.RejectionReason =
				TEXT("pure-red hypothesis has fewer than three support edges");
			return Result;
		}
		for (int32 EdgeIndex = 0; EdgeIndex < Result.Edges.Num(); ++EdgeIndex)
		{
			if (Result.Edges[EdgeIndex].AxisType ==
				Result.Edges[(EdgeIndex + 1) % Result.Edges.Num()].AxisType)
			{
				Result.RejectionReason =
					TEXT("pure-red hypothesis retains adjacent same-axis support edges");
				return Result;
			}
		}

		for (FWorldOrthogonalEdge& Edge : Result.Edges)
		{
			Edge.SupportCoordinate = WeightedMedianForEdgeOctilinear(
				Edge.SamplesUV,
				Edge.AxisType);
			Result.AngleCost +=
				FMath::Max(Edge.PathLength, 1e-6) *
				Edge.AngleToAssignedAxis *
				Edge.AngleToAssignedAxis;
		}

		Result.CorrectedUV.SetNum(Result.Edges.Num());
		for (int32 EdgeIndex = 0; EdgeIndex < Result.Edges.Num(); ++EdgeIndex)
		{
			const int32 PreviousIndex =
				(EdgeIndex + Result.Edges.Num() - 1) % Result.Edges.Num();
			Result.CorrectedUV[EdgeIndex] =
				SolveOctilinearVertexIntersection(
					Result.Edges[PreviousIndex].AxisType,
					Result.Edges[PreviousIndex].SupportCoordinate,
					Result.Edges[EdgeIndex].AxisType,
					Result.Edges[EdgeIndex].SupportCoordinate);
			if (!IsFiniteVector2D(Result.CorrectedUV[EdgeIndex]))
			{
				Result.RejectionReason =
					TEXT("pure-red support-line intersection is not finite");
				return Result;
			}
		}

		const FVector2D CenterOffset =
			RotationCenterUV - PolygonAreaCentroid2D(Result.CorrectedUV);
		for (FVector2D& Point : Result.CorrectedUV)
		{
			Point += CenterOffset;
		}
		for (FWorldOrthogonalEdge& Edge : Result.Edges)
		{
			switch (Edge.AxisType)
			{
			case Octi_U:
				Edge.SupportCoordinate += CenterOffset.Y;
				break;
			case Octi_V:
				Edge.SupportCoordinate += CenterOffset.X;
				break;
			case Octi_DiagPlus:
				Edge.SupportCoordinate +=
					CenterOffset.X - CenterOffset.Y;
				break;
			case Octi_DiagMinus:
				Edge.SupportCoordinate +=
					CenterOffset.X + CenterOffset.Y;
				break;
			default:
				break;
			}
		}

		for (int32 VertexIndex = 0;
			VertexIndex < Result.CorrectedUV.Num();
			++VertexIndex)
		{
			if (FVector2D::Distance(
				Result.CorrectedUV[VertexIndex],
				Result.CorrectedUV[
					(VertexIndex + 1) % Result.CorrectedUV.Num()]) <= 1e-6)
			{
				Result.RejectionReason =
					TEXT("pure-red hypothesis creates a degenerate edge");
				return Result;
			}
		}
		if (PolygonHasSelfIntersection(Result.CorrectedUV))
		{
			Result.RejectionReason =
				TEXT("pure-red hypothesis self-intersects");
			return Result;
		}

		TArray<FVector2D> RotatedOriginalCapUV = OriginalCapUV;
		TArray<FVector2D> RotatedBoundarySamplesUV = BoundarySamplesUV;
		RotatePointsAround2D(
			RotatedOriginalCapUV,
			RotationCenterUV,
			RotationDegrees);
		RotatePointsAround2D(
			RotatedBoundarySamplesUV,
			RotationCenterUV,
			RotationDegrees);
		const double OriginalSignedArea =
			SignedPolygonAreaForRegularization(RotatedOriginalCapUV);
		const double CorrectedSignedArea =
			SignedPolygonAreaForRegularization(Result.CorrectedUV);
		if (FMath::Abs(OriginalSignedArea) <= 1e-8 ||
			FMath::Abs(CorrectedSignedArea) <= 1e-8 ||
			OriginalSignedArea * CorrectedSignedArea <= 0.0)
		{
			Result.RejectionReason =
				TEXT("pure-red hypothesis reverses or degenerates polygon area");
			return Result;
		}
		Result.AreaRatio =
			FMath::Abs(CorrectedSignedArea) /
			FMath::Abs(OriginalSignedArea);

		for (int32 VertexIndex = 0;
			VertexIndex < Result.Edges.Num();
			++VertexIndex)
		{
			const int32 PreviousIndex =
				(VertexIndex + Result.Edges.Num() - 1) %
				Result.Edges.Num();
			const double OriginalTurn = FVector2D::CrossProduct(
				Result.Edges[PreviousIndex].DirectionUV.GetSafeNormal(),
				Result.Edges[VertexIndex].DirectionUV.GetSafeNormal());
			const FVector2D CorrectedIncoming =
				Result.CorrectedUV[VertexIndex] -
				Result.CorrectedUV[PreviousIndex];
			const FVector2D CorrectedOutgoing =
				Result.CorrectedUV[
					(VertexIndex + 1) % Result.CorrectedUV.Num()] -
				Result.CorrectedUV[VertexIndex];
			const double CorrectedTurn = FVector2D::CrossProduct(
				CorrectedIncoming.GetSafeNormal(),
				CorrectedOutgoing.GetSafeNormal());
			if (FMath::Abs(OriginalTurn) >= 0.25 &&
				OriginalTurn * CorrectedTurn < 0.0)
			{
				Result.RejectionReason =
					TEXT("pure-red hypothesis changes a protected convex/concave turn");
				return Result;
			}
		}

		ComputeBoundaryDistanceStats(
			RotatedBoundarySamplesUV,
			Result.CorrectedUV,
			Result.MeanBoundaryDistance,
			Result.MaxBoundaryDistance);
		Result.bValid = true;
		return Result;
	}

	static bool PixelToFaceUV(
		const FVector2D& Pixel,
		const FFaceInfo& Face,
		const FCommonInputs& Inputs,
		const FVector& AxisU,
		const FVector& AxisV,
		FVector2D& OutUV)
	{
		FVector World;
		if (!IntersectPixelWithPlaneOrthographic(
			Inputs.Camera,
			Inputs.FacesWidth,
			Inputs.FacesHeight,
			Pixel,
			Face.PlanePoint,
			Face.Normal,
			World))
		{
			return false;
		}
		OutUV = WorldPointToFaceUV(World, Face.PlanePoint, AxisU, AxisV);
		return IsFiniteVector2D(OutUV);
	}

	static bool RegularizeCapToWorldOrthogonalPolygon(
		const FFaceInfo& Face,
		const FCommonInputs& Inputs,
		const FFromLZFaceReconstructionParams& Params,
		const FVector2D& OriginalDeltaPixels,
		const TArray<FVector>& OriginalCapWorld,
		const TArray<FCapBoundaryRunInput>& BoundaryRuns,
		const FVector2D& SourceTranslationCapSpace,
		double ScaleX,
		double ScaleY,
		TArray<FVector>& OutFinalCapWorld,
		FCapBBoxRegularizationResult& OutDebug,
		const FVector* OverrideAxisUWorld = nullptr,
		const FVector* OverrideAxisVWorld = nullptr,
		const TCHAR* OverrideFaceBoundarySource = nullptr)
	{
		OutDebug = FCapBBoxRegularizationResult();
		OutDebug.bAttempted = true;
		OutDebug.bWorldOrthogonal = true;
		ApplyWorldOrthogonalParamsToDebug(OutDebug, Params);
		InitializeWorldOrthogonalStages(OutDebug);
		BeginWorldOrthogonalStage(OutDebug, TEXT("input_validation"));
		OutDebug.FaceOriginWorld = Face.PlanePoint;
		OutDebug.FaceNormalWorld = Face.Normal.GetSafeNormal();
		OutDebug.OriginalSourceToCopiedDeltaPixels = OriginalDeltaPixels;
		OutDebug.OriginalCapWorld = OriginalCapWorld;
		OutDebug.BoundaryRunCount = BoundaryRuns.Num();
		OutFinalCapWorld = OriginalCapWorld;

		if (OriginalCapWorld.Num() < 3 || BoundaryRuns.Num() < 3)
		{
			SetRegularizationFallback(OutDebug, TEXT("ordered boundary runs are missing or insufficient"));
			return false;
		}
		PassWorldOrthogonalStage(
			OutDebug,
			TEXT("input_validation"),
			FString::Printf(
				TEXT("%d ordered runs and %d original cap vertices"),
				BoundaryRuns.Num(),
				OriginalCapWorld.Num()));

		BeginWorldOrthogonalStage(OutDebug, TEXT("world_aligned_face_uv"));
		int32 ExcludedWorldAxis = INDEX_NONE;
		if (OverrideAxisUWorld && OverrideAxisVWorld &&
			!OverrideAxisUWorld->IsNearlyZero() &&
			!OverrideAxisVWorld->IsNearlyZero())
		{
			OutDebug.FaceAxisUWorld = OverrideAxisUWorld->GetSafeNormal();
			OutDebug.FaceAxisVWorld = OverrideAxisVWorld->GetSafeNormal();
			if (FMath::Abs(FVector::DotProduct(OutDebug.FaceAxisUWorld, OutDebug.FaceAxisVWorld)) > 1e-3 ||
				FMath::Abs(FVector::DotProduct(OutDebug.FaceAxisUWorld, Face.Normal.GetSafeNormal())) > 1e-3 ||
				FMath::Abs(FVector::DotProduct(OutDebug.FaceAxisVWorld, Face.Normal.GetSafeNormal())) > 1e-3)
			{
				SetRegularizationFallback(OutDebug, TEXT("override face UV basis is not orthogonal to the virtual face normal"));
				return false;
			}
		}
		else if (!BuildWorldAlignedFaceBasis(
			Face.Normal,
			OutDebug.FaceAxisUWorld,
			OutDebug.FaceAxisVWorld,
			ExcludedWorldAxis))
		{
			SetRegularizationFallback(OutDebug, TEXT("failed to build world-aligned face UV basis"));
			return false;
		}

		for (const FVector& Point : Face.KeyPoints3D)
		{
			OutDebug.FaceBoundaryUV.Add(WorldPointToFaceUV(
				Point,
				Face.PlanePoint,
				OutDebug.FaceAxisUWorld,
				OutDebug.FaceAxisVWorld));
		}
		OutDebug.FaceBoundarySource = OverrideFaceBoundarySource ? OverrideFaceBoundarySource : TEXT("shared_camera_legacy");
		if (!OverrideFaceBoundarySource && Params.bWorldOrthoUsePerFaceCapture)
		{
			TArray<FVector2D> CleanBoundaryUV;
			if (BuildPerFaceCleanBoundaryUV(
				Face.KeyPoints2D,
				Face.KeyPoints3D,
				Face.PlanePoint,
				OutDebug.FaceAxisUWorld,
				OutDebug.FaceAxisVWorld,
				Inputs.FacesWidth,
				Inputs.FacesHeight,
				FMath::Max(0.0, double(Params.WorldOrthoPerFaceClipMarginPixels)),
				CleanBoundaryUV))
			{
				OutDebug.FaceBoundaryUV = MoveTemp(CleanBoundaryUV);
				OutDebug.FaceBoundarySource = TEXT("per_face_clip_clean");
			}
		}
		for (const FVector& Point : OriginalCapWorld)
		{
			OutDebug.OriginalCapUV.Add(WorldPointToFaceUV(
				Point,
				Face.PlanePoint,
				OutDebug.FaceAxisUWorld,
				OutDebug.FaceAxisVWorld));
		}

		double MinU, MaxU, MinV, MaxV;
		ComputeAxisAlignedBounds(OutDebug.OriginalCapUV, MinU, MaxU, MinV, MaxV);
		OutDebug.WorldOrthogonalCapDiagonal =
			FVector2D(MaxU - MinU, MaxV - MinV).Size();
		if (!FMath::IsFinite(OutDebug.WorldOrthogonalCapDiagonal) ||
			OutDebug.WorldOrthogonalCapDiagonal <= 1e-6)
		{
			SetRegularizationFallback(OutDebug, TEXT("original cap has a degenerate world-UV extent"));
			return false;
		}
		PassWorldOrthogonalStage(
			OutDebug,
			TEXT("world_aligned_face_uv"),
			FString::Printf(
				TEXT("excluded world axis %d; cap diagonal %.6f world units"),
				ExcludedWorldAxis,
				OutDebug.WorldOrthogonalCapDiagonal));

		auto FaceUVToCapSpace = [&](const FVector2D& UV, FVector2D& OutCapSpace)
		{
			if (FMath::Abs(ScaleX) <= 1e-12 || FMath::Abs(ScaleY) <= 1e-12)
			{
				return false;
			}
			FVector2D FacePixel;
			if (!ProjectWorldToImage(
				Inputs.Camera,
				Inputs.FacesWidth,
				Inputs.FacesHeight,
				FaceUVToWorld(
					UV,
					Face.PlanePoint,
					OutDebug.FaceAxisUWorld,
					OutDebug.FaceAxisVWorld),
				FacePixel))
			{
				return false;
			}
			OutCapSpace = FVector2D(
				FacePixel.X / ScaleX - SourceTranslationCapSpace.X,
				FacePixel.Y / ScaleY - SourceTranslationCapSpace.Y);
			return IsFiniteVector2D(OutCapSpace);
		};
		auto FaceUVDistancePixels = [&](const FVector2D& A, const FVector2D& B)
		{
			FVector2D CapA;
			FVector2D CapB;
			if (!FaceUVToCapSpace(A, CapA) || !FaceUVToCapSpace(B, CapB))
			{
				return TNumericLimits<double>::Max();
			}
			return FVector2D::Distance(CapA, CapB);
		};

		TArray<FWorldOrthogonalEdge> Edges;
		TArray<FVector2D> BoundarySamplesUV;
		BeginWorldOrthogonalStage(OutDebug, TEXT("boundary_run_unprojection"));
		for (int32 RunIndex = 0; RunIndex < BoundaryRuns.Num(); ++RunIndex)
		{
			const FCapBoundaryRunInput& Run = BoundaryRuns[RunIndex];
			FWorldOrthogonalRunDebug RunDebug;
			RunDebug.RunIndex = RunIndex;
			RunDebug.StrokeId = Run.StrokeId;
			RunDebug.Type =
				Run.Type == ECapBoundaryRunType::Connector
				? TEXT("connector")
				: (Run.Type == ECapBoundaryRunType::Black ? TEXT("black") : TEXT("red"));
			RunDebug.bIgnoredConnector = Run.Type == ECapBoundaryRunType::Connector;
			RunDebug.StartNodeId = Run.StartNodeId;
			RunDebug.EndNodeId = Run.EndNodeId;
			RunDebug.StartNodeCapSpace = Run.StartNodePosition;
			RunDebug.EndNodeCapSpace = Run.EndNodePosition;
			RunDebug.SourcePointsCapSpace = Run.Points;
			for (const FVector2D& Point : Run.Points)
			{
				RunDebug.FacePixels.Emplace(
					(Point.X + SourceTranslationCapSpace.X) * ScaleX,
					(Point.Y + SourceTranslationCapSpace.Y) * ScaleY);
			}
			OutDebug.WorldOrthogonalRuns.Add(MoveTemp(RunDebug));
			FWorldOrthogonalRunDebug& StoredRunDebug = OutDebug.WorldOrthogonalRuns.Last();

			if (Run.Type == ECapBoundaryRunType::Connector)
			{
				++OutDebug.IgnoredConnectorCount;
				continue;
			}

			TArray<FVector2D> RunSamplesUV;
			TArray<FVector2D> RunSamplesCapSpace;
			OutDebug.bContainsBlack |= Run.Type == ECapBoundaryRunType::Black;
			for (const FVector2D& Point : Run.Points)
			{
				const FVector2D FacePixel(
					(Point.X + SourceTranslationCapSpace.X) * ScaleX,
					(Point.Y + SourceTranslationCapSpace.Y) * ScaleY);
				FVector2D UV;
				if (!PixelToFaceUV(
					FacePixel,
					Face,
					Inputs,
					OutDebug.FaceAxisUWorld,
					OutDebug.FaceAxisVWorld,
					UV))
				{
					SetRegularizationFallback(OutDebug, TEXT("failed to unproject an ordered boundary run point"));
					return false;
				}
				if (RunSamplesUV.Num() == 0 ||
					FVector2D::Distance(RunSamplesUV.Last(), UV) > 1e-7)
				{
					RunSamplesUV.Add(UV);
					RunSamplesCapSpace.Add(Point);
					BoundarySamplesUV.Add(UV);
					StoredRunDebug.SamplesUV.Add(UV);
				}
			}
			if (RunSamplesUV.Num() < 2)
			{
				SetRegularizationFallback(OutDebug, TEXT("a non-connector boundary run has degenerate geometry"));
				return false;
			}

			const FVector2D StartPixel(
				(Run.StartNodePosition.X + SourceTranslationCapSpace.X) * ScaleX,
				(Run.StartNodePosition.Y + SourceTranslationCapSpace.Y) * ScaleY);
			const FVector2D EndPixel(
				(Run.EndNodePosition.X + SourceTranslationCapSpace.X) * ScaleX,
				(Run.EndNodePosition.Y + SourceTranslationCapSpace.Y) * ScaleY);
			FVector2D RunStartUV;
			FVector2D RunEndUV;
			if (!PixelToFaceUV(
					StartPixel,
					Face,
					Inputs,
					OutDebug.FaceAxisUWorld,
					OutDebug.FaceAxisVWorld,
					RunStartUV) ||
				!PixelToFaceUV(
					EndPixel,
					Face,
					Inputs,
					OutDebug.FaceAxisUWorld,
					OutDebug.FaceAxisVWorld,
					RunEndUV))
			{
				SetRegularizationFallback(OutDebug, TEXT("failed to unproject a boundary graph endpoint"));
				return false;
			}
			StoredRunDebug.StartNodeUV = RunStartUV;
			StoredRunDebug.EndNodeUV = RunEndUV;
			StoredRunDebug.bUnprojected = true;

			if (Run.Type == ECapBoundaryRunType::Black)
			{
				FWorldOrthogonalEdge Edge;
				Edge.Type = Run.Type;
				Edge.SourceRunIndices.Add(RunIndex);
				Edge.SourceStrokeIds.Add(Run.StrokeId);
				Edge.PrimitiveEdgeIndices.Add(Edges.Num());
				Edge.GraphNodeIds = { Run.StartNodeId, Run.EndNodeId };
				Edge.GraphNodeCapSpace = {
					Run.StartNodePosition,
					Run.EndNodePosition
				};
				Edge.GraphNodeUV = { RunStartUV, RunEndUV };
				Edge.SamplesUV = MoveTemp(RunSamplesUV);
				Edge.SamplesCapSpace = MoveTemp(RunSamplesCapSpace);
				Edge.StartUV = RunStartUV;
				Edge.EndUV = RunEndUV;
				Edge.StartCapSpace = Run.StartNodePosition;
				Edge.EndCapSpace = Run.EndNodePosition;
				if (!InitializeWorldOrthogonalEdge(Edge, Params))
				{
					SetRegularizationFallback(OutDebug, TEXT("a black boundary run has degenerate geometry"));
					return false;
				}
				Edges.Add(MoveTemp(Edge));
				continue;
			}

			BeginWorldOrthogonalStage(OutDebug, TEXT("red_corner_protection"));
			TArray<int32> ProtectedIndices;
			FindProtectedRedRunIndices(
				RunSamplesUV,
				RunSamplesCapSpace,
				ProtectedIndices,
				StoredRunDebug,
				Params);
			if (ProtectedIndices.Num() < 2)
			{
				SetRegularizationFallback(OutDebug, TEXT("a red boundary run has no usable protected path"));
				return false;
			}
			StoredRunDebug.ProtectedPointIndices = ProtectedIndices;
			for (int32 ProtectedIndex : ProtectedIndices)
			{
				if (RunSamplesUV.IsValidIndex(ProtectedIndex))
				{
					StoredRunDebug.ProtectedPointsUV.Add(RunSamplesUV[ProtectedIndex]);
				}
			}
			for (int32 SegmentIndex = 0; SegmentIndex + 1 < ProtectedIndices.Num(); ++SegmentIndex)
			{
				const int32 StartIndex = ProtectedIndices[SegmentIndex];
				const int32 EndIndex = ProtectedIndices[SegmentIndex + 1];
				if (EndIndex <= StartIndex)
				{
					continue;
				}

				FWorldOrthogonalEdge Edge;
				Edge.Type = Run.Type;
				Edge.SourceRunIndices.Add(RunIndex);
				Edge.SourceStrokeIds.Add(Run.StrokeId);
				Edge.PrimitiveEdgeIndices.Add(Edges.Num());
				for (int32 PointIndex = StartIndex; PointIndex <= EndIndex; ++PointIndex)
				{
					Edge.SamplesUV.Add(RunSamplesUV[PointIndex]);
					Edge.SamplesCapSpace.Add(
						RunSamplesCapSpace[PointIndex]);
				}
				Edge.StartUV = SegmentIndex == 0
					? RunStartUV
					: RunSamplesUV[StartIndex];
				Edge.EndUV = SegmentIndex + 2 == ProtectedIndices.Num()
					? RunEndUV
					: RunSamplesUV[EndIndex];
				Edge.StartCapSpace = SegmentIndex == 0
					? Run.StartNodePosition
					: RunSamplesCapSpace[StartIndex];
				Edge.EndCapSpace =
					SegmentIndex + 2 == ProtectedIndices.Num()
						? Run.EndNodePosition
						: RunSamplesCapSpace[EndIndex];
				if (!InitializeWorldOrthogonalEdge(Edge, Params))
				{
					SetRegularizationFallback(OutDebug, TEXT("a protected red boundary segment has degenerate geometry"));
					return false;
				}
				Edges.Add(MoveTemp(Edge));
			}
			BeginWorldOrthogonalStage(OutDebug, TEXT("boundary_run_unprojection"));
		}
		PassWorldOrthogonalStage(
			OutDebug,
			TEXT("boundary_run_unprojection"),
			FString::Printf(
				TEXT("%d runs unprojected; %d connector runs ignored for geometry"),
				BoundaryRuns.Num() - OutDebug.IgnoredConnectorCount,
				OutDebug.IgnoredConnectorCount));
		PassWorldOrthogonalStage(OutDebug, TEXT("red_corner_protection"));
		const int32 CollapsedShortEdgeCount =
			CollapseShortRedPrimitiveEdges(Edges, Params);
		OutDebug.PrimitiveGeometryEdgeCount = Edges.Num();
		BeginWorldOrthogonalStage(
			OutDebug,
			TEXT("pure_red_root_alignment"));
		if (!OutDebug.bContainsBlack)
		{
			OutDebug.bPureRedRootAlignmentAttempted = true;
			OutDebug.RootRotationCenterUV =
				PolygonAreaCentroid2D(OutDebug.OriginalCapUV);

			struct FRootCandidate
			{
				int32 PrimitiveEdgeIndex = INDEX_NONE;
				double LengthPixels = 0.0;
				double MaxChordDeviationPixels = 0.0;
				double Reliability = 0.0;
				EOctilinearAxis TargetAxis = Octi_U;
				double RotationDegrees = 0.0;
			};
			TArray<FRootCandidate> RootCandidates;
			for (int32 EdgeIndex = 0; EdgeIndex < Edges.Num(); ++EdgeIndex)
			{
				const FWorldOrthogonalEdge& Edge = Edges[EdgeIndex];
				TArray<FVector2D> SamplesCapSpace;
				SamplesCapSpace.Reserve(Edge.SamplesUV.Num());
				bool bMapped = true;
				for (const FVector2D& SampleUV : Edge.SamplesUV)
				{
					FVector2D SampleCapSpace;
					if (!FaceUVToCapSpace(SampleUV, SampleCapSpace))
					{
						bMapped = false;
						break;
					}
					SamplesCapSpace.Add(SampleCapSpace);
				}
				if (!bMapped || SamplesCapSpace.Num() < 2)
				{
					continue;
				}

				double LengthPixels = 0.0;
				for (int32 SampleIndex = 1;
					SampleIndex < SamplesCapSpace.Num();
					++SampleIndex)
				{
					LengthPixels += FVector2D::Distance(
						SamplesCapSpace[SampleIndex - 1],
						SamplesCapSpace[SampleIndex]);
				}
				const FVector2D ChordStart = SamplesCapSpace[0];
				const FVector2D ChordEnd = SamplesCapSpace.Last();
				const double ChordLengthPixels =
					FVector2D::Distance(ChordStart, ChordEnd);
				double MaxChordDeviationPixels = 0.0;
				for (const FVector2D& Sample : SamplesCapSpace)
				{
					MaxChordDeviationPixels = FMath::Max(
						MaxChordDeviationPixels,
						FMath::Sqrt(DistancePointToSegmentSquared2D(
							Sample,
							ChordStart,
							ChordEnd)));
				}
				if (LengthPixels <
						FMath::Max(0.0, double(Params.WorldOrthoRedMacroGroupMinLengthPixels)) ||
					MaxChordDeviationPixels >
						FMath::Max(0.0, double(Params.WorldOrthoRedMacroCorridorPixels)))
				{
					continue;
				}

				// Skip diagonal targets (45deg / 135deg) for pure-red roots unless
				// explicitly re-enabled by Step10 params. Without this gate a dominant
				// near-diagonal red stroke would snap the entire cap onto a 45deg /
				// 135deg frame, which is rarely the user's intent.
				const bool bDiagonalAllowedForRoot =
					Params.bWorldOrthoAllowDiagonalSupports &&
					Params.bWorldOrthoPureRedAllowDiagonalRoot;
				if (!bDiagonalAllowedForRoot &&
					(Edge.AxisType == Octi_DiagPlus ||
					 Edge.AxisType == Octi_DiagMinus))
				{
					continue;
				}

				FRootCandidate Candidate;
				Candidate.PrimitiveEdgeIndex = EdgeIndex;
				Candidate.LengthPixels = LengthPixels;
				Candidate.MaxChordDeviationPixels =
					MaxChordDeviationPixels;
				Candidate.Reliability = ChordLengthPixels;
				Candidate.TargetAxis = Edge.AxisType;
				Candidate.RotationDegrees =
					RotationToOctilinearAxisDegrees(
						Edge.DirectionUV,
						Candidate.TargetAxis);
				RootCandidates.Add(Candidate);
			}
			RootCandidates.Sort([](
				const FRootCandidate& A,
				const FRootCandidate& B)
			{
				return A.Reliability > B.Reliability;
			});
			if (RootCandidates.Num() >
				FMath::Max(0, Params.WorldOrthoPureRedMaxRootCandidates))
			{
				RootCandidates.SetNum(
					FMath::Max(0, Params.WorldOrthoPureRedMaxRootCandidates));
			}

			TArray<FRootCandidate> Hypotheses;
			FRootCandidate Baseline;
			Baseline.PrimitiveEdgeIndex = INDEX_NONE;
			Baseline.RotationDegrees = 0.0;
			Hypotheses.Add(Baseline);
			for (const FRootCandidate& Candidate : RootCandidates)
			{
				bool bDuplicateRotation = false;
				for (const FRootCandidate& Existing : Hypotheses)
				{
					if (FMath::Abs(
						Existing.RotationDegrees -
						Candidate.RotationDegrees) < 0.25)
					{
						bDuplicateRotation = true;
						break;
					}
				}
				if (!bDuplicateRotation)
				{
					Hypotheses.Add(Candidate);
				}
			}

			int32 BestHypothesisIndex = INDEX_NONE;
			FPureRedRootHypothesisSolve BestSolve;
			auto IsBetterHypothesis = [](
				const FPureRedRootHypothesisSolve& CandidateSolve,
				const FRootCandidate& Candidate,
				const FPureRedRootHypothesisSolve& Best,
				const FRootCandidate& BestCandidate)
			{
				const double Epsilon = 1e-6;
				if (CandidateSolve.MeanBoundaryDistance + Epsilon <
					Best.MeanBoundaryDistance)
				{
					return true;
				}
				if (FMath::Abs(
					CandidateSolve.MeanBoundaryDistance -
					Best.MeanBoundaryDistance) > Epsilon)
				{
					return false;
				}
				const double CandidateAreaError =
					FMath::Abs(CandidateSolve.AreaRatio - 1.0);
				const double BestAreaError =
					FMath::Abs(Best.AreaRatio - 1.0);
				if (CandidateAreaError + Epsilon < BestAreaError)
				{
					return true;
				}
				if (FMath::Abs(CandidateAreaError - BestAreaError) >
					Epsilon)
				{
					return false;
				}
				if (CandidateSolve.AngleCost + Epsilon <
					Best.AngleCost)
				{
					return true;
				}
				if (FMath::Abs(
					CandidateSolve.AngleCost -
					Best.AngleCost) > Epsilon)
				{
					return false;
				}
				if (FMath::Abs(Candidate.RotationDegrees) + Epsilon <
					FMath::Abs(BestCandidate.RotationDegrees))
				{
					return true;
				}
				return Candidate.Reliability >
					BestCandidate.Reliability;
			};

			OutDebug.WorldOrthogonalRootHypotheses.Reset();
			for (int32 HypothesisIndex = 0;
				HypothesisIndex < Hypotheses.Num();
				++HypothesisIndex)
			{
				const FRootCandidate& Hypothesis =
					Hypotheses[HypothesisIndex];
				const FPureRedRootHypothesisSolve Solve =
					EvaluatePureRedRootHypothesis(
						Edges,
						OutDebug.OriginalCapUV,
						BoundarySamplesUV,
						OutDebug.RootRotationCenterUV,
						Hypothesis.RotationDegrees,
						Params);

				FWorldOrthogonalRootHypothesisDebug HypothesisDebug;
				HypothesisDebug.HypothesisIndex = HypothesisIndex;
				HypothesisDebug.RootPrimitiveEdgeIndex =
					Hypothesis.PrimitiveEdgeIndex;
				HypothesisDebug.RootLengthPixels =
					Hypothesis.LengthPixels;
				HypothesisDebug.RootMaxChordDeviationPixels =
					Hypothesis.MaxChordDeviationPixels;
				HypothesisDebug.Reliability =
					Hypothesis.Reliability;
				HypothesisDebug.RotationDegrees =
					Hypothesis.RotationDegrees;
				HypothesisDebug.bValid = Solve.bValid;
				HypothesisDebug.RejectionReason =
					Solve.RejectionReason;
				HypothesisDebug.GeometryEdgeCount =
					Solve.Edges.Num();
				HypothesisDebug.AngleCost = Solve.AngleCost;
				HypothesisDebug.AreaRatio = Solve.AreaRatio;
				HypothesisDebug.MeanBoundaryDistance =
					Solve.MeanBoundaryDistance;
				HypothesisDebug.MaxBoundaryDistance =
					Solve.MaxBoundaryDistance;
				if (Edges.IsValidIndex(
					Hypothesis.PrimitiveEdgeIndex))
				{
					const FWorldOrthogonalEdge& RootEdge =
						Edges[Hypothesis.PrimitiveEdgeIndex];
					HypothesisDebug.RootStrokeIds =
						RootEdge.SourceStrokeIds;
					HypothesisDebug.TargetAxisTypeName =
						OctilinearAxisName(
							Hypothesis.TargetAxis);
					HypothesisDebug.RootStartUV =
						RootEdge.StartUV;
					HypothesisDebug.RootEndUV =
						RootEdge.EndUV;
					HypothesisDebug.AlignedRootStartUV =
						RotatePointAround2D(
							RootEdge.StartUV,
							OutDebug.RootRotationCenterUV,
							Hypothesis.RotationDegrees);
					HypothesisDebug.AlignedRootEndUV =
						RotatePointAround2D(
							RootEdge.EndUV,
							OutDebug.RootRotationCenterUV,
							Hypothesis.RotationDegrees);
				}
				else
				{
					HypothesisDebug.TargetAxisTypeName =
						TEXT("none");
				}
				OutDebug.WorldOrthogonalRootHypotheses.Add(
					MoveTemp(HypothesisDebug));

				if (Solve.bValid &&
					(BestHypothesisIndex == INDEX_NONE ||
						IsBetterHypothesis(
							Solve,
							Hypothesis,
							BestSolve,
							Hypotheses[BestHypothesisIndex])))
				{
					BestHypothesisIndex = HypothesisIndex;
					BestSolve = Solve;
				}
			}

			if (BestHypothesisIndex == INDEX_NONE)
			{
				SetRegularizationFallback(
					OutDebug,
					TEXT("all pure-red root alignment hypotheses failed"));
				return false;
			}

			const FRootCandidate& Selected =
				Hypotheses[BestHypothesisIndex];
			OutDebug.WorldOrthogonalRootHypotheses[
				BestHypothesisIndex].bSelected = true;
			OutDebug.SelectedRootHypothesisIndex =
				BestHypothesisIndex;
			OutDebug.SelectedRootPrimitiveEdgeIndex =
				Selected.PrimitiveEdgeIndex;
			OutDebug.SelectedRootRotationDegrees =
				Selected.RotationDegrees;
			if (Edges.IsValidIndex(Selected.PrimitiveEdgeIndex))
			{
				const FWorldOrthogonalEdge& RootEdge =
					Edges[Selected.PrimitiveEdgeIndex];
				OutDebug.SelectedRootStrokeIds =
					RootEdge.SourceStrokeIds;
				OutDebug.SelectedRootTargetAxisTypeName =
					OctilinearAxisName(Selected.TargetAxis);
				OutDebug.SelectedRootStartUV = RootEdge.StartUV;
				OutDebug.SelectedRootEndUV = RootEdge.EndUV;
				OutDebug.SelectedAlignedRootStartUV =
					RotatePointAround2D(
						RootEdge.StartUV,
						OutDebug.RootRotationCenterUV,
						Selected.RotationDegrees);
				OutDebug.SelectedAlignedRootEndUV =
					RotatePointAround2D(
						RootEdge.EndUV,
						OutDebug.RootRotationCenterUV,
						Selected.RotationDegrees);
			}
			else
			{
				OutDebug.SelectedRootTargetAxisTypeName =
					TEXT("none");
			}

			for (FWorldOrthogonalEdge& Edge : Edges)
			{
				if (!RotateAndReclassifyPureRedEdge(
					Edge,
					OutDebug.RootRotationCenterUV,
					Selected.RotationDegrees,
					Params))
				{
					SetRegularizationFallback(
						OutDebug,
						TEXT("selected pure-red root rotation failed to transform a primitive edge"));
					return false;
				}
			}
			RotatePointsAround2D(
				BoundarySamplesUV,
				OutDebug.RootRotationCenterUV,
				Selected.RotationDegrees);
			OutDebug.WorldOrthogonalPhaseAlignedCapUV =
				OutDebug.OriginalCapUV;
			RotatePointsAround2D(
				OutDebug.WorldOrthogonalPhaseAlignedCapUV,
				OutDebug.RootRotationCenterUV,
				Selected.RotationDegrees);
			PassWorldOrthogonalStage(
				OutDebug,
				TEXT("pure_red_root_alignment"),
				FString::Printf(
					TEXT("selected hypothesis %d root edge %d rotation %.6f degrees from %d hypotheses"),
					BestHypothesisIndex,
					Selected.PrimitiveEdgeIndex,
					Selected.RotationDegrees,
					Hypotheses.Num()));
		}
		else
		{
			OutDebug.WorldOrthogonalPhaseAlignedCapUV =
				OutDebug.OriginalCapUV;
			PassWorldOrthogonalStage(
				OutDebug,
				TEXT("pure_red_root_alignment"),
				TEXT("contains black; world phase locked and rotation fixed at 0 degrees"));
		}

		PassWorldOrthogonalStage(
			OutDebug,
			TEXT("primitive_edge_classification"),
			FString::Printf(
				TEXT("%d primitive geometric edges after collapsing %d short red edges"),
				Edges.Num(),
				CollapsedShortEdgeCount));
		CaptureWorldOrthogonalEdgeDebug(
			Edges,
			OutDebug.WorldOrthogonalPrimitiveEdges);

		// Black edges use the same U/V-vs-diagonal threshold classification as
		// red edges, then apply their stricter snap tolerance to that assigned axis.
		for (int32 EdgeIndex = 0; EdgeIndex < Edges.Num(); ++EdgeIndex)
		{
			const FWorldOrthogonalEdge& Edge = Edges[EdgeIndex];
			if (Edge.Type == ECapBoundaryRunType::Black)
			{
				if (Edge.AngleToAssignedAxis >
					FMath::Clamp(double(Params.WorldOrthoBlackAxisToleranceDegrees), 0.0, 180.0))
				{
					SetRegularizationFallback(
						OutDebug,
						FString::Printf(
							TEXT("black edge %d assigned to %s by the %.3f-degree diagonal threshold requires %.3f degrees of snap, exceeding %.3f"),
							EdgeIndex,
							*OctilinearAxisName(Edge.AxisType),
							FMath::Clamp(double(Params.WorldOrthoDiagThresholdDegrees), 0.0, 90.0),
							Edge.AngleToAssignedAxis,
							FMath::Clamp(double(Params.WorldOrthoBlackAxisToleranceDegrees), 0.0, 180.0)));
					return false;
				}
			}
		}

		const FString DiagNote = Params.bWorldOrthoAllowDiagonalSupports
			? TEXT("")
			: TEXT("; allow_diagonal_supports=false");

		BeginWorldOrthogonalStage(OutDebug, TEXT("same_axis_merge"));
		if (!MergeAdjacentWorldOrthogonalEdges(Edges, FaceUVDistancePixels, Params))
		{
			SetRegularizationFallback(
				OutDebug,
				FString::Printf(
					TEXT("failed to merge adjacent geometric support fragments with the same octilinear axis assignment%s"),
					*DiagNote));
			return false;
		}
		OutDebug.GeometryEdgeCount = Edges.Num();
		CaptureWorldOrthogonalEdgeDebug(
			Edges,
			OutDebug.WorldOrthogonalMergedEdges);
		PassWorldOrthogonalStage(
			OutDebug,
			TEXT("same_axis_merge"),
			FString::Printf(
				TEXT("%d graph primitives merged to %d geometric support edges; black graph nodes retained"),
				OutDebug.PrimitiveGeometryEdgeCount,
				OutDebug.GeometryEdgeCount));

		// Stage 7: topology_repair — insert synthetic edges to resolve same-axis
		// adjacency violations in the cyclic edge list (up to 2 violations allowed).
		BeginWorldOrthogonalStage(OutDebug, TEXT("topology_repair"));
		if (!TryTopologyRepair(Edges, FaceUVDistancePixels, Params, OutDebug, OutDebug.WorldOrthogonalCapDiagonal))
		{
			return false;
		}
		if (OutDebug.bTopologyRepairApplied)
		{
			OutDebug.GeometryEdgeCount = Edges.Num();
			CaptureWorldOrthogonalEdgeDebug(
				Edges,
				OutDebug.WorldOrthogonalMergedEdges);
			PassWorldOrthogonalStage(
				OutDebug,
				TEXT("topology_repair"),
				FString::Printf(
					TEXT("%d synthetic edge(s) inserted; max gap %.1fpx (%.1f%% of cap diagonal %.1fpx)"),
					OutDebug.TopologyRepairInsertedCount,
					OutDebug.TopologyRepairMaxGapPixels,
					(OutDebug.WorldOrthogonalCapDiagonal > 1e-6) ? (100.0 * OutDebug.TopologyRepairMaxGapPixels / OutDebug.WorldOrthogonalCapDiagonal) : 0.0,
					OutDebug.WorldOrthogonalCapDiagonal));
		}
		else
		{
			PassWorldOrthogonalStage(
				OutDebug,
				TEXT("topology_repair"),
				OutDebug.bTopologyRepairAttempted
					? TEXT("topology repair attempted but no insertions were necessary")
					: TEXT("no same-axis adjacency violations detected"));
		}

		// Stage 8: adjacency_validation — validate edge sequence can form an octilinear polygon
		BeginWorldOrthogonalStage(OutDebug, TEXT("adjacency_validation"));
		if (Edges.Num() < 3)
		{
			SetRegularizationFallback(
				OutDebug,
				FString::Printf(
					TEXT("merged edge count %d is below the minimum of 3 for an octilinear polygon"),
					Edges.Num()));
			return false;
		}
		// Check no adjacent edges share the same axis type
		for (int32 EdgeIndex = 0; EdgeIndex < Edges.Num(); ++EdgeIndex)
		{
			const int32 NextIndex = (EdgeIndex + 1) % Edges.Num();
			if (Edges[EdgeIndex].AxisType == Edges[NextIndex].AxisType)
			{
				SetRegularizationFallback(
					OutDebug,
					FString::Printf(
						TEXT("adjacent support edges %d and %d share axis type %s; they do not define an octilinear corner%s"),
						EdgeIndex, NextIndex, *OctilinearAxisName(Edges[EdgeIndex].AxisType),
						*DiagNote));
				return false;
			}
		}
		// Compute weighted angle cost for diagnostics
		double AngleCost = 0.0;
		for (const FWorldOrthogonalEdge& Edge : Edges)
		{
			AngleCost += FMath::Max(Edge.PathLength, 1e-6) * Edge.AngleToAssignedAxis * Edge.AngleToAssignedAxis;
		}
		OutDebug.WorldOrthogonalAngleCost = AngleCost;

		CaptureWorldOrthogonalEdgeDebug(
			Edges,
			OutDebug.WorldOrthogonalMergedEdges);
		PassWorldOrthogonalStage(
			OutDebug,
			TEXT("adjacency_validation"),
			FString::Printf(
				TEXT("%d support edges with distinct adjacent octilinear directions; weighted angle cost %.6f"),
				Edges.Num(),
				AngleCost));

		// Stage 7: support_line_solve — compute support coordinate for each edge
		BeginWorldOrthogonalStage(OutDebug, TEXT("support_line_solve"));

		for (int32 EdgeIndex = 0; EdgeIndex < Edges.Num(); ++EdgeIndex)
		{
			FWorldOrthogonalEdge& Edge = Edges[EdgeIndex];
			if (Edge.Type == ECapBoundaryRunType::Black)
			{
				Edge.SupportCoordinate = WeightedMedianForEdgeOctilinear(
					Edge.SamplesUV,
					Edge.AxisType);
				Edge.SnappedGraphNodeUV.Reset();
				Edge.GraphNodeSnapDistancesPixels.Reset();
				for (int32 NodeIndex = 0; NodeIndex < Edge.GraphNodeUV.Num(); ++NodeIndex)
				{
					const FVector2D& OriginalNodeUV = Edge.GraphNodeUV[NodeIndex];
					const FVector2D SnappedNodeUV = ProjectPointToOctilinearSupport(
						OriginalNodeUV,
						Edge.AxisType,
						Edge.SupportCoordinate);
					const double SnapDistancePixels =
						FaceUVDistancePixels(OriginalNodeUV, SnappedNodeUV);
					Edge.SnappedGraphNodeUV.Add(SnappedNodeUV);
					Edge.GraphNodeSnapDistancesPixels.Add(SnapDistancePixels);
					if (!FMath::IsFinite(SnapDistancePixels) ||
						SnapDistancePixels > FMath::Max(0.0, double(Params.WorldOrthoBlackNodeSnapTolerancePixels)))
					{
						const int32 NodeId = Edge.GraphNodeIds.IsValidIndex(NodeIndex)
							? Edge.GraphNodeIds[NodeIndex]
							: INDEX_NONE;
						SetRegularizationFallback(
							OutDebug,
							FString::Printf(
								TEXT("black support edge %d node %d requires %.3f px normal snap, exceeding %.3f px"),
								EdgeIndex,
								NodeId,
								SnapDistancePixels,
								FMath::Max(0.0, double(Params.WorldOrthoBlackNodeSnapTolerancePixels))));
						return false;
					}
				}
				continue;
			}

			const int32 PreviousIndex = (EdgeIndex + Edges.Num() - 1) % Edges.Num();
			const int32 NextIndex = (EdgeIndex + 1) % Edges.Num();
			const FWorldOrthogonalEdge& Previous = Edges[PreviousIndex];
			const FWorldOrthogonalEdge& Next = Edges[NextIndex];
			const bool bPreviousBlack = Previous.Type == ECapBoundaryRunType::Black;
			const bool bNextBlack = Next.Type == ECapBoundaryRunType::Black;
			const double PreviousFixed =
				OctilinearSupportValue(Previous.EndUV, Edge.AxisType);
			const double NextFixed =
				OctilinearSupportValue(Next.StartUV, Edge.AxisType);
			if (bPreviousBlack && bNextBlack)
			{
				Edge.SupportCoordinate = 0.5 * (PreviousFixed + NextFixed);
			}
			else if (bPreviousBlack)
			{
				Edge.SupportCoordinate = PreviousFixed;
			}
			else if (bNextBlack)
			{
				Edge.SupportCoordinate = NextFixed;
			}
			else
			{
				Edge.SupportCoordinate = WeightedMedianForEdgeOctilinear(
					Edge.SamplesUV, Edge.AxisType);
			}
		}
		CaptureWorldOrthogonalEdgeDebug(
			Edges,
			OutDebug.WorldOrthogonalMergedEdges);
		PassWorldOrthogonalStage(
			OutDebug,
			TEXT("support_line_solve"),
			OutDebug.bContainsBlack
				? FString::Printf(
					TEXT("black support lines fitted with graph-node snap tolerance %.3f px; remaining red supports solved"),
					FMath::Max(0.0, double(Params.WorldOrthoBlackNodeSnapTolerancePixels)))
				: TEXT("pure-red supports solved by weighted median"));

		// Stage 8: vertex_intersections — solve vertex positions from support intersections
		BeginWorldOrthogonalStage(OutDebug, TEXT("vertex_intersections"));
		TArray<FVector2D> CorrectedUV;
		CorrectedUV.SetNum(Edges.Num());
		for (int32 EdgeIndex = 0; EdgeIndex < Edges.Num(); ++EdgeIndex)
		{
			const int32 PreviousIndex = (EdgeIndex + Edges.Num() - 1) % Edges.Num();
			const FWorldOrthogonalEdge& Previous = Edges[PreviousIndex];
			const FWorldOrthogonalEdge& Current = Edges[EdgeIndex];
			FVector2D Vertex = SolveOctilinearVertexIntersection(
				Previous.AxisType, Previous.SupportCoordinate,
				Current.AxisType, Current.SupportCoordinate);
			if (!IsFiniteVector2D(Vertex))
			{
				SetRegularizationFallback(
					OutDebug,
					FString::Printf(
						TEXT("vertex intersection between edge %d (%s) and edge %d (%s) is not finite"),
						PreviousIndex, *OctilinearAxisName(Previous.AxisType),
						EdgeIndex, *OctilinearAxisName(Current.AxisType)));
				return false;
			}
			CorrectedUV[EdgeIndex] = Vertex;
		}
		OutDebug.WorldOrthogonalSolvedVerticesUV = CorrectedUV;

		if (!OutDebug.bContainsBlack)
		{
			const FVector2D CenterOffset =
				PolygonAreaCentroid2D(OutDebug.OriginalCapUV) -
				PolygonAreaCentroid2D(CorrectedUV);
			for (FVector2D& Point : CorrectedUV)
			{
				Point += CenterOffset;
			}
			for (FWorldOrthogonalEdge& Edge : Edges)
			{
				switch (Edge.AxisType)
				{
				case Octi_U:         Edge.SupportCoordinate += CenterOffset.Y; break;
				case Octi_V:         Edge.SupportCoordinate += CenterOffset.X; break;
				case Octi_DiagPlus:  Edge.SupportCoordinate += (CenterOffset.X - CenterOffset.Y); break;
				case Octi_DiagMinus: Edge.SupportCoordinate += (CenterOffset.X + CenterOffset.Y); break;
				default:             Edge.SupportCoordinate += CenterOffset.Y; break;
				}
			}
			OutDebug.WorldOrthogonalSolvedVerticesUV = CorrectedUV;
		}

		// Black graph nodes keep their identity and order while their geometric
		// positions may snap to the fitted support chain within the pixel budget.
		for (int32 EdgeIndex = 0; EdgeIndex < Edges.Num(); ++EdgeIndex)
		{
			FWorldOrthogonalEdge& Edge = Edges[EdgeIndex];
			const FVector2D& Start = CorrectedUV[EdgeIndex];
			const FVector2D& End = CorrectedUV[(EdgeIndex + 1) % Edges.Num()];
			if (Edge.Type == ECapBoundaryRunType::Black)
			{
				Edge.SnappedGraphNodeUV.Reset();
				Edge.GraphNodeSnapDistancesPixels.Reset();
				for (int32 NodeIndex = 0; NodeIndex < Edge.GraphNodeUV.Num(); ++NodeIndex)
				{
					FVector2D SnappedNodeUV = ProjectPointToOctilinearSupport(
						Edge.GraphNodeUV[NodeIndex],
						Edge.AxisType,
						Edge.SupportCoordinate);
					if (NodeIndex == 0)
					{
						SnappedNodeUV = Start;
					}
					else if (NodeIndex + 1 == Edge.GraphNodeUV.Num())
					{
						SnappedNodeUV = End;
					}
					const double SnapDistancePixels =
						FaceUVDistancePixels(Edge.GraphNodeUV[NodeIndex], SnappedNodeUV);
					Edge.SnappedGraphNodeUV.Add(SnappedNodeUV);
					Edge.GraphNodeSnapDistancesPixels.Add(SnapDistancePixels);
					if (!FMath::IsFinite(SnapDistancePixels) ||
						SnapDistancePixels > FMath::Max(0.0, double(Params.WorldOrthoBlackNodeSnapTolerancePixels)))
					{
						const int32 NodeId = Edge.GraphNodeIds.IsValidIndex(NodeIndex)
							? Edge.GraphNodeIds[NodeIndex]
							: INDEX_NONE;
						SetRegularizationFallback(
							OutDebug,
							FString::Printf(
								TEXT("octilinear solve moves black graph node %d by %.3f px, exceeding %.3f px"),
								NodeId,
								SnapDistancePixels,
								FMath::Max(0.0, double(Params.WorldOrthoBlackNodeSnapTolerancePixels))));
						return false;
					}
				}
			}
			if (OutDebug.WorldOrthogonalMergedEdges.IsValidIndex(EdgeIndex))
			{
				FWorldOrthogonalEdgeDebug& EdgeDebug =
					OutDebug.WorldOrthogonalMergedEdges[EdgeIndex];
				EdgeDebug.AxisType = static_cast<int32>(Edge.AxisType);
				EdgeDebug.SupportCoordinate = Edge.SupportCoordinate;
				EdgeDebug.SolvedStartUV = Start;
				EdgeDebug.SolvedEndUV = End;
				EdgeDebug.SnappedGraphNodeUV = Edge.SnappedGraphNodeUV;
				EdgeDebug.GraphNodeSnapDistancesPixels =
					Edge.GraphNodeSnapDistancesPixels;
			}
		}
		PassWorldOrthogonalStage(
			OutDebug,
			TEXT("vertex_intersections"),
			FString::Printf(TEXT("%d support-line intersections"), CorrectedUV.Num()));

		// Stage 9: topology_validation — self-intersection and protected turns only
		BeginWorldOrthogonalStage(OutDebug, TEXT("topology_validation"));
		const double OriginalSignedArea = SignedPolygonAreaForRegularization(OutDebug.OriginalCapUV);
		const double CorrectedSignedArea = SignedPolygonAreaForRegularization(CorrectedUV);
		OutDebug.WorldOrthogonalAreaRatio =
			FMath::Abs(CorrectedSignedArea) / FMath::Abs(OriginalSignedArea);
		if (!FMath::IsFinite(OutDebug.WorldOrthogonalAreaRatio) ||
			OutDebug.WorldOrthogonalAreaRatio < FMath::Max(0.0, double(Params.WorldOrthoMinAreaRatio)))
		{
			SetRegularizationFallback(
				OutDebug,
				FString::Printf(
					TEXT("orthogonal area ratio %.6f is below the minimum %.6f"),
					OutDebug.WorldOrthogonalAreaRatio,
					FMath::Max(0.0, double(Params.WorldOrthoMinAreaRatio))));
			return false;
		}

		if (PolygonHasSelfIntersection(CorrectedUV))
		{
			SetRegularizationFallback(OutDebug, TEXT("corrected cap self-intersects"));
			return false;
		}
		for (int32 VertexIndex = 0; VertexIndex < Edges.Num(); ++VertexIndex)
		{
			const int32 PreviousIndex = (VertexIndex + Edges.Num() - 1) % Edges.Num();
			const double OriginalTurn = FVector2D::CrossProduct(
				Edges[PreviousIndex].DirectionUV.GetSafeNormal(),
				Edges[VertexIndex].DirectionUV.GetSafeNormal());
			const FVector2D CorrectedIncoming =
				CorrectedUV[VertexIndex] - CorrectedUV[PreviousIndex];
			const FVector2D CorrectedOutgoing =
				CorrectedUV[(VertexIndex + 1) % CorrectedUV.Num()] - CorrectedUV[VertexIndex];
			const double CorrectedTurn = FVector2D::CrossProduct(
				CorrectedIncoming.GetSafeNormal(),
				CorrectedOutgoing.GetSafeNormal());
			if (FMath::Abs(OriginalTurn) >= 0.25 &&
				OriginalTurn * CorrectedTurn < 0.0)
			{
				SetRegularizationFallback(
					OutDebug,
					TEXT("corrected cap changes a protected convex/concave turn"));
				return false;
			}
		}
		PassWorldOrthogonalStage(
			OutDebug,
			TEXT("topology_validation"),
			TEXT("self-intersection and protected turns checks passed"));

		// Stage 10: metric_validation — diagnostic only, no hard rejection
		BeginWorldOrthogonalStage(OutDebug, TEXT("metric_validation"));
		ComputeBoundaryDistanceStats(
			BoundarySamplesUV,
			CorrectedUV,
			OutDebug.WorldOrthogonalMeanBoundaryDistance,
			OutDebug.WorldOrthogonalMaxBoundaryDistance);
		PassWorldOrthogonalStage(
			OutDebug,
			TEXT("metric_validation"),
			FString::Printf(
				TEXT("area ratio %.6f; boundary mean %.6f; max %.6f (diagnostic only)"),
				OutDebug.WorldOrthogonalAreaRatio,
				OutDebug.WorldOrthogonalMeanBoundaryDistance,
				OutDebug.WorldOrthogonalMaxBoundaryDistance));

		// Stage 11: world_conversion
		BeginWorldOrthogonalStage(OutDebug, TEXT("world_conversion"));
		if (!ConvertFaceUVLoopToWorld(
			CorrectedUV,
			Face.PlanePoint,
			OutDebug.FaceAxisUWorld,
			OutDebug.FaceAxisVWorld,
			OutFinalCapWorld))
		{
			SetRegularizationFallback(OutDebug, TEXT("corrected world-UV cap failed conversion back to world space"));
			return false;
		}
		PassWorldOrthogonalStage(
			OutDebug,
			TEXT("world_conversion"),
			FString::Printf(TEXT("%d corrected vertices converted to world space"), OutFinalCapWorld.Num()));

		OutDebug.CapArea = FMath::Abs(OriginalSignedArea);
		OutDebug.bApplied = true;
		OutDebug.bFallbackToOriginal = false;
		OutDebug.SelectedGeometry = OutDebug.bContainsBlack
			? TEXT("world_octilinear_black_anchored")
			: TEXT("world_octilinear_pure_red");
		OutDebug.RejectionReason.Reset();
		OutDebug.FinalCapUV = CorrectedUV;
		OutDebug.FinalCapWorld = OutFinalCapWorld;
		return true;
	}

	static bool RegularizeCapToFaceBBox(
		const FFaceInfo& Face,
		const FCommonInputs& Inputs,
		const FVector2D& OriginalDeltaPixels,
		const TArray<FVector>& OriginalCapWorld,
		TArray<FVector>& OutFinalCapWorld,
		FCapBBoxRegularizationResult& OutDebug)
	{
		OutDebug = FCapBBoxRegularizationResult();
		OutDebug.bAttempted = true;
		OutDebug.FaceOriginWorld = Face.PlanePoint;
		OutDebug.FaceNormalWorld = Face.Normal.GetSafeNormal();
		OutDebug.OriginalSourceToCopiedDeltaPixels = OriginalDeltaPixels;
		OutDebug.OriginalCapWorld = OriginalCapWorld;
		OutFinalCapWorld = OriginalCapWorld;

		if (Face.KeyPoints3D.Num() < 3 || OriginalCapWorld.Num() < 3)
		{
			SetRegularizationFallback(OutDebug, TEXT("face or cap has fewer than three points"));
			return false;
		}

		FVector FaceNormal;
		FVector TempX;
		FVector TempY;
		if (!BuildTemporaryFaceBasis(Face, Inputs.Camera, FaceNormal, TempX, TempY))
		{
			SetRegularizationFallback(OutDebug, TEXT("failed to build a temporary face-plane basis"));
			return false;
		}

		TArray<FVector2D> FaceTemp;
		for (const FVector& Point : Face.KeyPoints3D)
		{
			if (!IsFiniteVector(Point))
			{
				SetRegularizationFallback(OutDebug, TEXT("face boundary contains a non-finite world point"));
				return false;
			}
			FaceTemp.Add(WorldPointToFaceUV(Point, Face.PlanePoint, TempX, TempY));
		}

		TArray<FVector2D> TempHull;
		if (!BuildConvexHull2D(FaceTemp, TempHull))
		{
			SetRegularizationFallback(OutDebug, TEXT("face boundary convex hull is degenerate"));
			return false;
		}

		FMinimumAreaBBox2D TempBBox;
		if (!ComputeMinimumAreaBBox2D(TempHull, TempBBox))
		{
			SetRegularizationFallback(OutDebug, TEXT("face minimum-area oriented bbox is degenerate"));
			return false;
		}

		const double Width = TempBBox.MaxX - TempBBox.MinX;
		const double Height = TempBBox.MaxY - TempBBox.MinY;
		FVector2D AxisU2D = TempBBox.AxisX;
		FVector2D AxisV2D = TempBBox.AxisY;
		const double MaxExtent = FMath::Max(Width, Height);
		const bool bNearlySquare =
			MaxExtent > SMALL_NUMBER &&
			FMath::Abs(Width - Height) / MaxExtent <= FaceBBoxSquareRelativeTolerance;
		if (Height > Width && !bNearlySquare)
		{
			Swap(AxisU2D, AxisV2D);
		}
		else if (bNearlySquare)
		{
			const FVector WorldAxisX = (TempX * TempBBox.AxisX.X + TempY * TempBBox.AxisX.Y).GetSafeNormal();
			const FVector WorldAxisY = (TempX * TempBBox.AxisY.X + TempY * TempBBox.AxisY.Y).GetSafeNormal();
			FVector2D ProjectedX;
			FVector2D ProjectedY;
			const bool bProjectedX = ProjectSignedWorldDirectionToImage(
				Inputs.Camera, Inputs.FacesWidth, Inputs.FacesHeight, WorldAxisX, ProjectedX);
			const bool bProjectedY = ProjectSignedWorldDirectionToImage(
				Inputs.Camera, Inputs.FacesWidth, Inputs.FacesHeight, WorldAxisY, ProjectedY);
			const double HorizontalX = bProjectedX ? FMath::Abs(ProjectedX.GetSafeNormal().X) : -1.0;
			const double HorizontalY = bProjectedY ? FMath::Abs(ProjectedY.GetSafeNormal().X) : -1.0;
			if (HorizontalY > HorizontalX)
			{
				Swap(AxisU2D, AxisV2D);
			}
		}

		FVector AxisU = (TempX * AxisU2D.X + TempY * AxisU2D.Y).GetSafeNormal();
		if (AxisU.IsNearlyZero())
		{
			SetRegularizationFallback(OutDebug, TEXT("face bbox U axis is invalid"));
			return false;
		}
		FVector AxisV = FVector::CrossProduct(FaceNormal, AxisU).GetSafeNormal();
		if (AxisV.IsNearlyZero())
		{
			SetRegularizationFallback(OutDebug, TEXT("face bbox V axis is invalid"));
			return false;
		}

		FVector2D ProjectedU;
		if (ProjectSignedWorldDirectionToImage(
			Inputs.Camera, Inputs.FacesWidth, Inputs.FacesHeight, AxisU, ProjectedU) &&
			ProjectedU.X < 0.0)
		{
			AxisU *= -1.0;
			AxisV *= -1.0;
		}
		if (FVector::DotProduct(FVector::CrossProduct(AxisU, AxisV), FaceNormal) < 0.0)
		{
			AxisV *= -1.0;
		}

		OutDebug.FaceAxisUWorld = AxisU;
		OutDebug.FaceAxisVWorld = AxisV;
		OutDebug.FaceBoundarySource = TEXT("shared_camera_legacy");
		bool bUsedPerFaceClean = false;
		if (GFromLZUsePerFaceCapture != 0)
		{
			TArray<FVector2D> CleanBoundaryUV;
			if (BuildPerFaceCleanBoundaryUV(
				Face.KeyPoints2D,
				Face.KeyPoints3D,
				Face.PlanePoint,
				AxisU,
				AxisV,
				Inputs.FacesWidth,
				Inputs.FacesHeight,
				GFromLZPerFaceClipMarginPixels,
				CleanBoundaryUV))
			{
				OutDebug.FaceBoundaryUV = MoveTemp(CleanBoundaryUV);
				OutDebug.FaceBoundarySource = TEXT("per_face_clip_clean");
				bUsedPerFaceClean = true;
			}
		}
		if (!bUsedPerFaceClean)
		{
			for (const FVector& Point : Face.KeyPoints3D)
			{
				OutDebug.FaceBoundaryUV.Add(WorldPointToFaceUV(Point, Face.PlanePoint, AxisU, AxisV));
			}
		}
		if (!BuildConvexHull2D(OutDebug.FaceBoundaryUV, OutDebug.FaceHullUV))
		{
			SetRegularizationFallback(OutDebug, TEXT("face hull became invalid in stabilized UV coordinates"));
			return false;
		}

		double FaceMinU, FaceMaxU, FaceMinV, FaceMaxV;
		ComputeAxisAlignedBounds(OutDebug.FaceBoundaryUV, FaceMinU, FaceMaxU, FaceMinV, FaceMaxV);
		OutDebug.FaceBBoxUV = MakeBBoxCorners(FaceMinU, FaceMaxU, FaceMinV, FaceMaxV);

		for (const FVector& Point : OriginalCapWorld)
		{
			if (!IsFiniteVector(Point))
			{
				SetRegularizationFallback(OutDebug, TEXT("cap contains a non-finite world point"));
				return false;
			}
			OutDebug.OriginalCapUV.Add(WorldPointToFaceUV(Point, Face.PlanePoint, AxisU, AxisV));
		}

		OutDebug.CapArea = FMath::Abs(SignedPolygonAreaForRegularization(OutDebug.OriginalCapUV));
		double CapMinU, CapMaxU, CapMinV, CapMaxV;
		ComputeAxisAlignedBounds(OutDebug.OriginalCapUV, CapMinU, CapMaxU, CapMinV, CapMaxV);
		OutDebug.CapBaseBBoxArea = (CapMaxU - CapMinU) * (CapMaxV - CapMinV);
		if (!FMath::IsFinite(OutDebug.CapArea) ||
			!FMath::IsFinite(OutDebug.CapBaseBBoxArea) ||
			OutDebug.CapArea <= CapBBoxDegenerateAreaTolerance ||
			OutDebug.CapBaseBBoxArea <= CapBBoxDegenerateAreaTolerance)
		{
			SetRegularizationFallback(OutDebug, TEXT("cap polygon or cap base bbox area is degenerate"));
			return false;
		}

		OutDebug.CapBaseBBoxRatio = OutDebug.CapArea / OutDebug.CapBaseBBoxArea;
		OutDebug.CapBaseBBoxUV = MakeBBoxCorners(CapMinU, CapMaxU, CapMinV, CapMaxV);
		if (!ConvertFaceUVLoopToWorld(
			OutDebug.CapBaseBBoxUV,
			Face.PlanePoint,
			AxisU,
			AxisV,
			OutDebug.CapBaseBBoxWorld))
		{
			SetRegularizationFallback(OutDebug, TEXT("cap base bbox UV-to-world conversion produced a non-finite point"));
			return false;
		}

		OutDebug.bCapBaseBBoxPassed = OutDebug.CapBaseBBoxRatio >= OutDebug.FillRatioThreshold;
		if (OutDebug.bCapBaseBBoxPassed)
		{
			OutDebug.bApplied = true;
			OutDebug.bFallbackToOriginal = false;
			OutDebug.SelectedGeometry = TEXT("cap_base_bbox");
			OutDebug.FinalCapUV = OutDebug.CapBaseBBoxUV;
			OutDebug.FinalCapWorld = OutDebug.CapBaseBBoxWorld;
			OutFinalCapWorld = OutDebug.FinalCapWorld;
			return true;
		}

		OutDebug.bCapMinimumBBoxComputed = true;
		if (!BuildConvexHull2D(OutDebug.OriginalCapUV, OutDebug.OriginalCapHullUV))
		{
			SetRegularizationFallback(OutDebug, TEXT("cap base bbox failed and cap convex hull is degenerate"));
			return false;
		}

		FMinimumAreaBBox2D CapMinimumBBox;
		if (!ComputeMinimumAreaBBox2D(OutDebug.OriginalCapHullUV, CapMinimumBBox))
		{
			SetRegularizationFallback(OutDebug, TEXT("cap base bbox failed and cap minimum-area bbox is degenerate"));
			return false;
		}

		OutDebug.CapMinimumBBoxArea = CapMinimumBBox.Area;
		if (!FMath::IsFinite(OutDebug.CapMinimumBBoxArea) ||
			OutDebug.CapMinimumBBoxArea <= CapBBoxDegenerateAreaTolerance)
		{
			SetRegularizationFallback(OutDebug, TEXT("cap minimum-area bbox has invalid area"));
			return false;
		}

		OutDebug.CapMinimumBBoxRatio = OutDebug.CapArea / OutDebug.CapMinimumBBoxArea;
		OutDebug.CapMinimumBBoxUV = MakeOrientedBBoxCorners(CapMinimumBBox);
		if (!ConvertFaceUVLoopToWorld(
			OutDebug.CapMinimumBBoxUV,
			Face.PlanePoint,
			AxisU,
			AxisV,
			OutDebug.CapMinimumBBoxWorld))
		{
			SetRegularizationFallback(OutDebug, TEXT("cap minimum bbox UV-to-world conversion produced a non-finite point"));
			return false;
		}

		OutDebug.bCapMinimumBBoxPassed = OutDebug.CapMinimumBBoxRatio >= OutDebug.FillRatioThreshold;
		if (OutDebug.bCapMinimumBBoxPassed)
		{
			OutDebug.bApplied = true;
			OutDebug.bFallbackToOriginal = false;
			OutDebug.SelectedGeometry = TEXT("cap_minimum_bbox");
			OutDebug.FinalCapUV = OutDebug.CapMinimumBBoxUV;
			OutDebug.FinalCapWorld = OutDebug.CapMinimumBBoxWorld;
			OutFinalCapWorld = OutDebug.FinalCapWorld;
			return true;
		}

		OutDebug.bApplied = false;
		OutDebug.bFallbackToOriginal = false;
		OutDebug.SelectedGeometry = TEXT("original");
		OutDebug.RejectionReason = TEXT("cap base bbox and cap minimum bbox fill ratios are below the configured threshold");
		OutDebug.FinalCapUV = OutDebug.OriginalCapUV;
		OutDebug.FinalCapWorld = OutDebug.OriginalCapWorld;
		return false;
	}

	static TArray<FVector2D> MapUVPointsToDebugImage(
		const TArray<FVector2D>& Points,
		double MinU,
		double MaxU,
		double MinV,
		double MaxV,
		int32 ImageSize)
	{
		const double Margin = 64.0;
		const double SpanU = FMath::Max(MaxU - MinU, 1e-6);
		const double SpanV = FMath::Max(MaxV - MinV, 1e-6);
		const double Scale = FMath::Min(
			(double(ImageSize) - 2.0 * Margin) / SpanU,
			(double(ImageSize) - 2.0 * Margin) / SpanV);
		const double CenterU = 0.5 * (MinU + MaxU);
		const double CenterV = 0.5 * (MinV + MaxV);
		const FVector2D ImageCenter(0.5 * double(ImageSize), 0.5 * double(ImageSize));

		TArray<FVector2D> Mapped;
		Mapped.Reserve(Points.Num());
		for (const FVector2D& Point : Points)
		{
			Mapped.Emplace(
				ImageCenter.X + (Point.X - CenterU) * Scale,
				ImageCenter.Y - (Point.Y - CenterV) * Scale);
		}
		return Mapped;
	}

	static void AccumulateUVBounds(
		const TArray<FVector2D>& Points,
		double& InOutMinU,
		double& InOutMaxU,
		double& InOutMinV,
		double& InOutMaxV)
	{
		for (const FVector2D& Point : Points)
		{
			InOutMinU = FMath::Min(InOutMinU, Point.X);
			InOutMaxU = FMath::Max(InOutMaxU, Point.X);
			InOutMinV = FMath::Min(InOutMinV, Point.Y);
			InOutMaxV = FMath::Max(InOutMaxV, Point.Y);
		}
	}

	static bool SaveCapBBoxDebugPng(
		const FCapBBoxRegularizationResult& Debug,
		const FString& Path,
		bool bOverview)
	{
		const int32 Size = CapBBoxDebugImageSize;
		TArray<uint8> RGBA;
		RGBA.SetNumUninitialized(Size * Size * 4);
		for (int32 Pixel = 0; Pixel < Size * Size; ++Pixel)
		{
			const int32 Offset = Pixel * 4;
			RGBA[Offset + 0] = 24;
			RGBA[Offset + 1] = 24;
			RGBA[Offset + 2] = 28;
			RGBA[Offset + 3] = 255;
		}

		double MinU = TNumericLimits<double>::Max();
		double MaxU = -TNumericLimits<double>::Max();
		double MinV = TNumericLimits<double>::Max();
		double MaxV = -TNumericLimits<double>::Max();
		if (bOverview)
		{
			AccumulateUVBounds(Debug.FaceBoundaryUV, MinU, MaxU, MinV, MaxV);
			AccumulateUVBounds(Debug.FaceBBoxUV, MinU, MaxU, MinV, MaxV);
		}
		AccumulateUVBounds(Debug.OriginalCapUV, MinU, MaxU, MinV, MaxV);
		AccumulateUVBounds(Debug.CapBaseBBoxUV, MinU, MaxU, MinV, MaxV);
		AccumulateUVBounds(Debug.CapMinimumBBoxUV, MinU, MaxU, MinV, MaxV);
		AccumulateUVBounds(Debug.FinalCapUV, MinU, MaxU, MinV, MaxV);
		if (!FMath::IsFinite(MinU) || !FMath::IsFinite(MaxU) ||
			!FMath::IsFinite(MinV) || !FMath::IsFinite(MaxV) ||
			MinU > MaxU || MinV > MaxV)
		{
			DrawLineRGBA(
				RGBA,
				Size,
				Size,
				FVector2D(160.0, 160.0),
				FVector2D(double(Size - 160), double(Size - 160)),
				FColor(220, 70, 70, 255),
				6);
			DrawLineRGBA(
				RGBA,
				Size,
				Size,
				FVector2D(double(Size - 160), 160.0),
				FVector2D(160.0, double(Size - 160)),
				FColor(220, 70, 70, 255),
				6);
			return SaveRGBAToPng(RGBA, Size, Size, Path);
		}

		const double PadU = FMath::Max((MaxU - MinU) * 0.08, 1e-3);
		const double PadV = FMath::Max((MaxV - MinV) * 0.08, 1e-3);
		MinU -= PadU;
		MaxU += PadU;
		MinV -= PadV;
		MaxV += PadV;

		auto Map = [&](const TArray<FVector2D>& Points)
		{
			return MapUVPointsToDebugImage(Points, MinU, MaxU, MinV, MaxV, Size);
		};

		if (bOverview)
		{
			DrawClosedPolylineRGBA(RGBA, Size, Size, Map(Debug.FaceBoundaryUV), FColor(90, 90, 96, 255), 2);
			DrawClosedPolylineRGBA(RGBA, Size, Size, Map(Debug.FaceHullUV), FColor(210, 210, 210, 255), 1);
			DrawClosedPolylineRGBA(RGBA, Size, Size, Map(Debug.FaceBBoxUV), FColor(0, 210, 220, 255), 1);
		}
		DrawClosedPolylineRGBA(RGBA, Size, Size, Map(Debug.OriginalCapUV), FColor(255, 70, 70, 255), 2);
		DrawClosedPolylineRGBA(RGBA, Size, Size, Map(Debug.CapBaseBBoxUV), FColor(255, 210, 0, 255), 2);
		DrawClosedPolylineRGBA(RGBA, Size, Size, Map(Debug.CapMinimumBBoxUV), FColor(190, 80, 255, 255), 2);
		DrawClosedPolylineRGBA(RGBA, Size, Size, Map(Debug.FinalCapUV), FColor(40, 240, 100, 255), 2);

		const TArray<FVector2D> OriginUV = { FVector2D::ZeroVector };
		const FVector2D OriginPixel = Map(OriginUV)[0];
		const double AxisLengthUV = 0.15 * FMath::Max(MaxU - MinU, MaxV - MinV);
		DrawArrowRGBA(
			RGBA, Size, Size, OriginPixel,
			Map(TArray<FVector2D>{ FVector2D(AxisLengthUV, 0.0) })[0],
			FColor(255, 80, 80, 255), 2);
		DrawArrowRGBA(
			RGBA, Size, Size, OriginPixel,
			Map(TArray<FVector2D>{ FVector2D(0.0, AxisLengthUV) })[0],
			FColor(80, 160, 255, 255), 2);
		return SaveRGBAToPng(RGBA, Size, Size, Path);
	}

	static FColor WorldOrthogonalAxisColor(int32 Axis, const FString& Type)
	{
		if (Type == TEXT("black"))
		{
			switch (Axis)
			{
			case Octi_U:         return FColor(235, 235, 235, 255);
			case Octi_V:         return FColor(200, 200, 235, 255);
			case Octi_DiagPlus:  return FColor(235, 200, 200, 255);
			case Octi_DiagMinus: return FColor(200, 235, 200, 255);
			default:             return FColor(235, 235, 235, 255);
			}
		}
		switch (Axis)
		{
		case Octi_U:         return FColor(255, 90, 70, 255);   // red-ish
		case Octi_V:         return FColor(70, 150, 255, 255);  // blue-ish
		case Octi_DiagPlus:  return FColor(255, 180, 50, 255);  // orange
		case Octi_DiagMinus: return FColor(50, 220, 140, 255);  // teal/green
		default:             return FColor(255, 90, 70, 255);
		}
	}

	static bool SaveWorldOrthogonalStepPng(
		const FCapBBoxRegularizationResult& Debug,
		const FString& Path,
		int32 ViewIndex)
	{
		const int32 Size = CapBBoxDebugImageSize;
		TArray<uint8> RGBA;
		RGBA.SetNumUninitialized(Size * Size * 4);
		for (int32 Pixel = 0; Pixel < Size * Size; ++Pixel)
		{
			const int32 Offset = Pixel * 4;
			RGBA[Offset + 0] = 24;
			RGBA[Offset + 1] = 24;
			RGBA[Offset + 2] = 28;
			RGBA[Offset + 3] = 255;
		}

		double MinU = TNumericLimits<double>::Max();
		double MaxU = -TNumericLimits<double>::Max();
		double MinV = TNumericLimits<double>::Max();
		double MaxV = -TNumericLimits<double>::Max();
		AccumulateUVBounds(Debug.OriginalCapUV, MinU, MaxU, MinV, MaxV);
		AccumulateUVBounds(Debug.FinalCapUV, MinU, MaxU, MinV, MaxV);
		AccumulateUVBounds(
			Debug.WorldOrthogonalPhaseAlignedCapUV,
			MinU,
			MaxU,
			MinV,
			MaxV);
		for (const FWorldOrthogonalRunDebug& Run : Debug.WorldOrthogonalRuns)
		{
			AccumulateUVBounds(Run.SamplesUV, MinU, MaxU, MinV, MaxV);
		}
		for (const FWorldOrthogonalEdgeDebug& Edge : Debug.WorldOrthogonalPrimitiveEdges)
		{
			AccumulateUVBounds(Edge.SamplesUV, MinU, MaxU, MinV, MaxV);
		}
		for (const FWorldOrthogonalEdgeDebug& Edge : Debug.WorldOrthogonalMergedEdges)
		{
			AccumulateUVBounds(Edge.SamplesUV, MinU, MaxU, MinV, MaxV);
			AccumulateUVBounds(Edge.GraphNodeUV, MinU, MaxU, MinV, MaxV);
			AccumulateUVBounds(Edge.SnappedGraphNodeUV, MinU, MaxU, MinV, MaxV);
		}
		for (const FWorldOrthogonalRootHypothesisDebug& Hypothesis :
			Debug.WorldOrthogonalRootHypotheses)
		{
			AccumulateUVBounds(
				TArray<FVector2D>{
					Hypothesis.RootStartUV,
					Hypothesis.RootEndUV,
					Hypothesis.AlignedRootStartUV,
					Hypothesis.AlignedRootEndUV
				},
				MinU,
				MaxU,
				MinV,
				MaxV);
		}
		AccumulateUVBounds(Debug.WorldOrthogonalSolvedVerticesUV, MinU, MaxU, MinV, MaxV);
		if (!FMath::IsFinite(MinU) || !FMath::IsFinite(MaxU) ||
			!FMath::IsFinite(MinV) || !FMath::IsFinite(MaxV) ||
			MinU > MaxU || MinV > MaxV)
		{
			DrawLineRGBA(
				RGBA,
				Size,
				Size,
				FVector2D(160.0, 160.0),
				FVector2D(double(Size - 160), double(Size - 160)),
				FColor(220, 70, 70, 255),
				6);
			DrawLineRGBA(
				RGBA,
				Size,
				Size,
				FVector2D(double(Size - 160), 160.0),
				FVector2D(160.0, double(Size - 160)),
				FColor(220, 70, 70, 255),
				6);
			return SaveRGBAToPng(RGBA, Size, Size, Path);
		}

		const double PadU = FMath::Max((MaxU - MinU) * 0.08, 1e-3);
		const double PadV = FMath::Max((MaxV - MinV) * 0.08, 1e-3);
		MinU -= PadU;
		MaxU += PadU;
		MinV -= PadV;
		MaxV += PadV;
		auto Map = [&](const TArray<FVector2D>& Points)
		{
			return MapUVPointsToDebugImage(Points, MinU, MaxU, MinV, MaxV, Size);
		};

		DrawClosedPolylineRGBA(
			RGBA,
			Size,
			Size,
			Map(Debug.OriginalCapUV),
			FColor(85, 85, 92, 255),
			1);

		if (ViewIndex <= 1)
		{
			for (const FWorldOrthogonalRunDebug& Run : Debug.WorldOrthogonalRuns)
			{
				const FColor Color =
					Run.Type == TEXT("black")
						? FColor(240, 240, 240, 255)
						: (Run.Type == TEXT("connector")
							? FColor(255, 155, 45, 255)
							: FColor(255, 75, 75, 255));
				DrawOpenPolylineRGBA(RGBA, Size, Size, Map(Run.SamplesUV), Color, 2);
				if (ViewIndex == 1)
				{
					for (const FVector2D& Point : Map(Run.ProtectedPointsUV))
					{
						DrawPointRGBA(
							RGBA,
							Size,
							Size,
							FMath::RoundToInt(Point.X),
							FMath::RoundToInt(Point.Y),
							FColor(255, 225, 40, 255),
							5);
					}
				}
			}
		}
		else if (ViewIndex == 2)
		{
			for (const FWorldOrthogonalEdgeDebug& Edge : Debug.WorldOrthogonalPrimitiveEdges)
			{
				DrawOpenPolylineRGBA(
					RGBA,
					Size,
					Size,
					Map(Edge.SamplesUV),
					WorldOrthogonalAxisColor(Edge.AxisType, Edge.Type),
					2);
			}
		}
		else if (ViewIndex == 3 || ViewIndex == 4)
		{
			for (const FWorldOrthogonalEdgeDebug& Edge : Debug.WorldOrthogonalMergedEdges)
			{
				DrawOpenPolylineRGBA(
					RGBA,
					Size,
					Size,
					Map(Edge.SamplesUV),
					WorldOrthogonalAxisColor(Edge.AxisType, Edge.Type),
					3);
				if (Edge.Type == TEXT("black"))
				{
					for (const FVector2D& Point : Map(Edge.GraphNodeUV))
					{
						DrawPointRGBA(
							RGBA,
							Size,
							Size,
							FMath::RoundToInt(Point.X),
							FMath::RoundToInt(Point.Y),
							FColor(50, 210, 255, 255),
							5);
					}
				}
			}
		}
		else if (ViewIndex == 5)
		{
			for (const FWorldOrthogonalEdgeDebug& Edge : Debug.WorldOrthogonalMergedEdges)
			{
				TArray<FVector2D> Support;
				const EOctilinearAxis EdgeAxisType = static_cast<EOctilinearAxis>(Edge.AxisType);
				switch (EdgeAxisType)
				{
				case Octi_U:
					Support = {
						FVector2D(MinU, Edge.SupportCoordinate),
						FVector2D(MaxU, Edge.SupportCoordinate)
					};
					break;
				case Octi_V:
					Support = {
						FVector2D(Edge.SupportCoordinate, MinV),
						FVector2D(Edge.SupportCoordinate, MaxV)
					};
					break;
				case Octi_DiagPlus:
					{
						const double D = Edge.SupportCoordinate;
						const double V0 = MinV;
						const double V1 = MaxV;
						Support = {
							FVector2D(V0 + D, V0),
							FVector2D(V1 + D, V1)
						};
					}
					break;
				case Octi_DiagMinus:
					{
						const double D = Edge.SupportCoordinate;
						const double V0 = MinV;
						const double V1 = MaxV;
						Support = {
							FVector2D(D - V0, V0),
							FVector2D(D - V1, V1)
						};
					}
					break;
				default:
					continue;
				}
				DrawOpenPolylineRGBA(
					RGBA,
					Size,
					Size,
					Map(Support),
					WorldOrthogonalAxisColor(Edge.AxisType, Edge.Type),
					1);
				if (Edge.Type == TEXT("black"))
				{
					const int32 NodeCount = FMath::Min(
						Edge.GraphNodeUV.Num(),
						Edge.SnappedGraphNodeUV.Num());
					for (int32 NodeIndex = 0; NodeIndex < NodeCount; ++NodeIndex)
					{
						DrawOpenPolylineRGBA(
							RGBA,
							Size,
							Size,
							Map(TArray<FVector2D>{
								Edge.GraphNodeUV[NodeIndex],
								Edge.SnappedGraphNodeUV[NodeIndex]
							}),
							FColor(255, 80, 220, 255),
							2);
						const FVector2D SnappedPixel =
							Map(TArray<FVector2D>{ Edge.SnappedGraphNodeUV[NodeIndex] })[0];
						DrawPointRGBA(
							RGBA,
							Size,
							Size,
							FMath::RoundToInt(SnappedPixel.X),
							FMath::RoundToInt(SnappedPixel.Y),
							FColor(255, 225, 40, 255),
							5);
					}
				}
				if (!Edge.SolvedStartUV.IsNearlyZero() ||
					!Edge.SolvedEndUV.IsNearlyZero())
				{
					DrawOpenPolylineRGBA(
						RGBA,
						Size,
						Size,
						Map(TArray<FVector2D>{ Edge.SolvedStartUV, Edge.SolvedEndUV }),
						FColor(255, 225, 40, 255),
						3);
				}
			}
		}
		else if (ViewIndex == 8)
		{
			if (Debug.WorldOrthogonalPhaseAlignedCapUV.Num() >= 3)
			{
				DrawClosedPolylineRGBA(
					RGBA,
					Size,
					Size,
					Map(Debug.WorldOrthogonalPhaseAlignedCapUV),
					FColor(80, 160, 255, 255),
					2);
			}
			for (const FWorldOrthogonalRootHypothesisDebug& Hypothesis :
				Debug.WorldOrthogonalRootHypotheses)
			{
				if (Hypothesis.RootPrimitiveEdgeIndex == INDEX_NONE)
				{
					continue;
				}
				const FColor CandidateColor =
					Hypothesis.bSelected
						? FColor(255, 225, 40, 255)
						: (Hypothesis.bValid
							? FColor(110, 210, 130, 255)
							: FColor(150, 70, 70, 255));
				DrawOpenPolylineRGBA(
					RGBA,
					Size,
					Size,
					Map(TArray<FVector2D>{
						Hypothesis.RootStartUV,
						Hypothesis.RootEndUV
					}),
					CandidateColor,
					Hypothesis.bSelected ? 4 : 2);
			}
			if (Debug.SelectedRootPrimitiveEdgeIndex != INDEX_NONE)
			{
				DrawOpenPolylineRGBA(
					RGBA,
					Size,
					Size,
					Map(TArray<FVector2D>{
						Debug.SelectedAlignedRootStartUV,
						Debug.SelectedAlignedRootEndUV
					}),
					FColor(255, 80, 220, 255),
					4);
				const FVector2D CenterPixel =
					Map(TArray<FVector2D>{
						Debug.RootRotationCenterUV
					})[0];
				DrawPointRGBA(
					RGBA,
					Size,
					Size,
					FMath::RoundToInt(CenterPixel.X),
					FMath::RoundToInt(CenterPixel.Y),
					FColor(255, 255, 255, 255),
					5);
			}
		}
		else
		{
			if (Debug.WorldOrthogonalSolvedVerticesUV.Num() >= 3)
			{
				DrawClosedPolylineRGBA(
					RGBA,
					Size,
					Size,
					Map(Debug.WorldOrthogonalSolvedVerticesUV),
					FColor(255, 225, 40, 255),
					2);
			}
			if (Debug.bApplied)
			{
				DrawClosedPolylineRGBA(
					RGBA,
					Size,
					Size,
					Map(Debug.FinalCapUV),
					FColor(40, 240, 100, 255),
					3);
			}
			else if (Debug.bFallbackToOriginal)
			{
				DrawClosedPolylineRGBA(
					RGBA,
					Size,
					Size,
					Map(Debug.OriginalCapUV),
					FColor(255, 70, 70, 255),
					2);
			}
		}

		const double AxisLengthUV = 0.12 * FMath::Max(MaxU - MinU, MaxV - MinV);
		const FVector2D AxisOrigin(MinU + 0.08 * (MaxU - MinU), MinV + 0.08 * (MaxV - MinV));
		DrawArrowRGBA(
			RGBA,
			Size,
			Size,
			Map(TArray<FVector2D>{ AxisOrigin })[0],
			Map(TArray<FVector2D>{ AxisOrigin + FVector2D(AxisLengthUV, 0.0) })[0],
			FColor(255, 80, 80, 255),
			2);
		DrawArrowRGBA(
			RGBA,
			Size,
			Size,
			Map(TArray<FVector2D>{ AxisOrigin })[0],
			Map(TArray<FVector2D>{ AxisOrigin + FVector2D(0.0, AxisLengthUV) })[0],
			FColor(80, 160, 255, 255),
			2);
		return SaveRGBAToPng(RGBA, Size, Size, Path);
	}

	static void SetWorldOrthogonalTraceFields(
		TSharedRef<FJsonObject> Root,
		const FCapBBoxRegularizationResult& Debug)
	{
		TArray<TSharedPtr<FJsonValue>> StageValues;
		for (int32 Index = 0; Index < Debug.WorldOrthogonalStages.Num(); ++Index)
		{
			const FWorldOrthogonalStageDebug& Stage = Debug.WorldOrthogonalStages[Index];
			TSharedRef<FJsonObject> Object = MakeShared<FJsonObject>();
			Object->SetNumberField(TEXT("index"), Index);
			Object->SetStringField(TEXT("name"), Stage.Name);
			Object->SetStringField(TEXT("status"), Stage.Status);
			Object->SetStringField(TEXT("message"), Stage.Message);
			StageValues.Add(MakeShared<FJsonValueObject>(Object));
		}
		Root->SetArrayField(TEXT("stages"), StageValues);

		TArray<TSharedPtr<FJsonValue>> RunValues;
		for (const FWorldOrthogonalRunDebug& Run : Debug.WorldOrthogonalRuns)
		{
			TSharedRef<FJsonObject> Object = MakeShared<FJsonObject>();
			Object->SetNumberField(TEXT("run_index"), Run.RunIndex);
			Object->SetNumberField(TEXT("stroke_id"), Run.StrokeId);
			Object->SetStringField(TEXT("type"), Run.Type);
			Object->SetBoolField(TEXT("ignored_connector"), Run.bIgnoredConnector);
			Object->SetBoolField(TEXT("unprojected"), Run.bUnprojected);
			Object->SetNumberField(TEXT("start_node_id"), Run.StartNodeId);
			Object->SetNumberField(TEXT("end_node_id"), Run.EndNodeId);
			Object->SetArrayField(TEXT("start_node_cap_space"), JsonVector2D(Run.StartNodeCapSpace)->AsArray());
			Object->SetArrayField(TEXT("end_node_cap_space"), JsonVector2D(Run.EndNodeCapSpace)->AsArray());
			Object->SetArrayField(TEXT("start_node_uv"), JsonVector2D(Run.StartNodeUV)->AsArray());
			Object->SetArrayField(TEXT("end_node_uv"), JsonVector2D(Run.EndNodeUV)->AsArray());
			SetVector2DArrayField(Object, TEXT("source_points_cap_space"), Run.SourcePointsCapSpace);
			SetVector2DArrayField(Object, TEXT("face_pixels"), Run.FacePixels);
			SetVector2DArrayField(Object, TEXT("samples_uv"), Run.SamplesUV);
			SetIntArrayField(Object, TEXT("protected_point_indices"), Run.ProtectedPointIndices);
			SetVector2DArrayField(Object, TEXT("protected_points_uv"), Run.ProtectedPointsUV);
			Object->SetStringField(TEXT("red_protection_mode"), Run.RedProtectionMode);
			Object->SetNumberField(
				TEXT("red_max_chord_deviation_pixels"),
				Run.RedMaxChordDeviationPixels);
			{
				TArray<TSharedPtr<FJsonValue>> AxisValues;
				AxisValues.Reserve(Run.RedDirectionalGroupAxisTypes.Num());
				for (const FString& AxisType : Run.RedDirectionalGroupAxisTypes)
				{
					AxisValues.Add(MakeShared<FJsonValueString>(AxisType));
				}
				Object->SetArrayField(
					TEXT("red_directional_group_axis_types"),
					AxisValues);
			}
			SetDoubleArrayField(
				Object,
				TEXT("red_directional_group_lengths_pixels"),
				Run.RedDirectionalGroupLengthsPixels);
			SetIntArrayField(
				Object,
				TEXT("red_directional_group_start_indices"),
				Run.RedDirectionalGroupStartIndices);
			SetIntArrayField(
				Object,
				TEXT("red_directional_group_end_indices"),
				Run.RedDirectionalGroupEndIndices);
			RunValues.Add(MakeShared<FJsonValueObject>(Object));
		}
		Root->SetArrayField(TEXT("boundary_runs"), RunValues);

		auto SetEdges = [&](const TCHAR* Key, const TArray<FWorldOrthogonalEdgeDebug>& Edges)
		{
			TArray<TSharedPtr<FJsonValue>> EdgeValues;
			for (const FWorldOrthogonalEdgeDebug& Edge : Edges)
			{
				TSharedRef<FJsonObject> Object = MakeShared<FJsonObject>();
				Object->SetNumberField(TEXT("edge_index"), Edge.EdgeIndex);
				Object->SetStringField(TEXT("type"), Edge.Type);
				SetIntArrayField(Object, TEXT("source_run_indices"), Edge.SourceRunIndices);
				SetIntArrayField(Object, TEXT("source_stroke_ids"), Edge.SourceStrokeIds);
				SetIntArrayField(Object, TEXT("primitive_edge_indices"), Edge.PrimitiveEdgeIndices);
				SetIntArrayField(Object, TEXT("graph_node_ids"), Edge.GraphNodeIds);
				SetVector2DArrayField(
					Object,
					TEXT("graph_node_cap_space"),
					Edge.GraphNodeCapSpace);
				SetVector2DArrayField(Object, TEXT("graph_node_uv"), Edge.GraphNodeUV);
				SetVector2DArrayField(
					Object,
					TEXT("snapped_graph_node_uv"),
					Edge.SnappedGraphNodeUV);
				SetDoubleArrayField(
					Object,
					TEXT("graph_node_snap_distances_pixels"),
					Edge.GraphNodeSnapDistancesPixels);
				SetVector2DArrayField(Object, TEXT("samples_uv"), Edge.SamplesUV);
				Object->SetArrayField(TEXT("start_uv"), JsonVector2D(Edge.StartUV)->AsArray());
				Object->SetArrayField(TEXT("end_uv"), JsonVector2D(Edge.EndUV)->AsArray());
				Object->SetArrayField(TEXT("direction_uv"), JsonVector2D(Edge.DirectionUV)->AsArray());
				Object->SetArrayField(TEXT("solved_start_uv"), JsonVector2D(Edge.SolvedStartUV)->AsArray());
				Object->SetArrayField(TEXT("solved_end_uv"), JsonVector2D(Edge.SolvedEndUV)->AsArray());
				Object->SetStringField(TEXT("axis_type_name"), Edge.AxisTypeName);
				Object->SetNumberField(TEXT("axis_type"), Edge.AxisType);
				Object->SetNumberField(TEXT("angle_to_u_degrees"), Edge.AngleToU);
				Object->SetNumberField(TEXT("angle_to_v_degrees"), Edge.AngleToV);
				Object->SetNumberField(TEXT("angle_to_diag_plus_degrees"), Edge.AngleToDiagPlus);
				Object->SetNumberField(TEXT("angle_to_diag_minus_degrees"), Edge.AngleToDiagMinus);
				Object->SetNumberField(TEXT("angle_to_assigned_axis_degrees"), Edge.AngleToAssignedAxis);
				Object->SetNumberField(TEXT("support_coordinate"), Edge.SupportCoordinate);
				Object->SetNumberField(TEXT("path_length"), Edge.PathLength);
				EdgeValues.Add(MakeShared<FJsonValueObject>(Object));
			}
			Root->SetArrayField(Key, EdgeValues);
		};
		SetEdges(TEXT("primitive_edges"), Debug.WorldOrthogonalPrimitiveEdges);
		SetEdges(TEXT("merged_edges"), Debug.WorldOrthogonalMergedEdges);
		TArray<TSharedPtr<FJsonValue>> RootHypothesisValues;
		for (const FWorldOrthogonalRootHypothesisDebug& Hypothesis :
			Debug.WorldOrthogonalRootHypotheses)
		{
			TSharedRef<FJsonObject> Object = MakeShared<FJsonObject>();
			Object->SetNumberField(
				TEXT("hypothesis_index"),
				Hypothesis.HypothesisIndex);
			Object->SetNumberField(
				TEXT("root_primitive_edge_index"),
				Hypothesis.RootPrimitiveEdgeIndex);
			SetIntArrayField(
				Object,
				TEXT("root_stroke_ids"),
				Hypothesis.RootStrokeIds);
			Object->SetStringField(
				TEXT("target_axis_type_name"),
				Hypothesis.TargetAxisTypeName);
			Object->SetArrayField(
				TEXT("root_start_uv"),
				JsonVector2D(Hypothesis.RootStartUV)->AsArray());
			Object->SetArrayField(
				TEXT("root_end_uv"),
				JsonVector2D(Hypothesis.RootEndUV)->AsArray());
			Object->SetArrayField(
				TEXT("aligned_root_start_uv"),
				JsonVector2D(Hypothesis.AlignedRootStartUV)->AsArray());
			Object->SetArrayField(
				TEXT("aligned_root_end_uv"),
				JsonVector2D(Hypothesis.AlignedRootEndUV)->AsArray());
			Object->SetNumberField(
				TEXT("root_length_pixels"),
				Hypothesis.RootLengthPixels);
			Object->SetNumberField(
				TEXT("root_max_chord_deviation_pixels"),
				Hypothesis.RootMaxChordDeviationPixels);
			Object->SetNumberField(
				TEXT("reliability"),
				Hypothesis.Reliability);
			Object->SetNumberField(
				TEXT("rotation_degrees"),
				Hypothesis.RotationDegrees);
			Object->SetBoolField(TEXT("valid"), Hypothesis.bValid);
			Object->SetBoolField(TEXT("selected"), Hypothesis.bSelected);
			Object->SetStringField(
				TEXT("rejection_reason"),
				Hypothesis.RejectionReason);
			Object->SetNumberField(
				TEXT("geometry_edge_count"),
				Hypothesis.GeometryEdgeCount);
			Object->SetNumberField(
				TEXT("angle_cost"),
				Hypothesis.AngleCost);
			Object->SetNumberField(
				TEXT("area_ratio"),
				Hypothesis.AreaRatio);
			Object->SetNumberField(
				TEXT("mean_boundary_distance"),
				Hypothesis.MeanBoundaryDistance);
			Object->SetNumberField(
				TEXT("max_boundary_distance"),
				Hypothesis.MaxBoundaryDistance);
			RootHypothesisValues.Add(
				MakeShared<FJsonValueObject>(Object));
		}
		Root->SetArrayField(
			TEXT("pure_red_root_hypotheses"),
			RootHypothesisValues);
		SetVector2DArrayField(
			Root,
			TEXT("solved_vertices_uv"),
			Debug.WorldOrthogonalSolvedVerticesUV);
		SetVector2DArrayField(
			Root,
			TEXT("phase_aligned_cap_uv"),
			Debug.WorldOrthogonalPhaseAlignedCapUV);
	}

	static void SaveCapBBoxRegularizationJson(
		const FCapBBoxRegularizationResult& Debug,
		const FString& Path,
		bool bWorldOnly,
		const FString& FocusStage = FString())
	{
		TSharedRef<FJsonObject> Root = MakeShared<FJsonObject>();
		Root->SetStringField(TEXT("focus_stage"), FocusStage);
		if (!FocusStage.IsEmpty())
		{
			for (const FWorldOrthogonalStageDebug& Stage : Debug.WorldOrthogonalStages)
			{
				if (Stage.Name == FocusStage)
				{
					Root->SetStringField(TEXT("focus_stage_status"), Stage.Status);
					Root->SetStringField(TEXT("focus_stage_message"), Stage.Message);
					break;
				}
			}
		}
		Root->SetBoolField(TEXT("attempted"), Debug.bAttempted);
		Root->SetBoolField(TEXT("regularization_applied"), Debug.bApplied);
		Root->SetBoolField(TEXT("fallback_to_original"), Debug.bFallbackToOriginal);
		Root->SetNumberField(TEXT("selected_face_id"), Debug.SelectedFaceId);
		Root->SetStringField(TEXT("action"), Debug.Action);
		Root->SetStringField(TEXT("source_polygon_key"), Debug.SourcePolygonKey);
		Root->SetStringField(TEXT("copied_polygon_key"), Debug.CopiedPolygonKey);
		Root->SetStringField(TEXT("selected_geometry"), Debug.SelectedGeometry);
		Root->SetStringField(TEXT("rejection_reason"), Debug.RejectionReason);
		Root->SetNumberField(TEXT("fill_ratio_threshold"), Debug.FillRatioThreshold);
		Root->SetNumberField(TEXT("cap_area"), Debug.CapArea);
		Root->SetNumberField(TEXT("cap_bbox_area"), Debug.CapBaseBBoxArea);
		Root->SetNumberField(TEXT("fill_ratio"), Debug.CapBaseBBoxRatio);
		Root->SetNumberField(TEXT("cap_base_bbox_area"), Debug.CapBaseBBoxArea);
		Root->SetNumberField(TEXT("cap_base_bbox_ratio"), Debug.CapBaseBBoxRatio);
		Root->SetBoolField(TEXT("cap_base_bbox_passed"), Debug.bCapBaseBBoxPassed);
		Root->SetBoolField(TEXT("cap_minimum_bbox_computed"), Debug.bCapMinimumBBoxComputed);
		Root->SetNumberField(TEXT("cap_minimum_bbox_area"), Debug.CapMinimumBBoxArea);
		Root->SetNumberField(TEXT("cap_minimum_bbox_ratio"), Debug.CapMinimumBBoxRatio);
		Root->SetBoolField(TEXT("cap_minimum_bbox_passed"), Debug.bCapMinimumBBoxPassed);
		Root->SetBoolField(TEXT("world_orthogonal"), Debug.bWorldOrthogonal);
		Root->SetBoolField(TEXT("contains_black"), Debug.bContainsBlack);
		Root->SetNumberField(TEXT("boundary_run_count"), Debug.BoundaryRunCount);
		Root->SetNumberField(TEXT("primitive_geometry_edge_count"), Debug.PrimitiveGeometryEdgeCount);
		Root->SetNumberField(TEXT("geometry_edge_count"), Debug.GeometryEdgeCount);
		Root->SetNumberField(TEXT("ignored_connector_count"), Debug.IgnoredConnectorCount);
		Root->SetNumberField(TEXT("world_orthogonal_angle_cost"), Debug.WorldOrthogonalAngleCost);
		Root->SetNumberField(TEXT("world_orthogonal_area_ratio"), Debug.WorldOrthogonalAreaRatio);
		Root->SetNumberField(TEXT("world_orthogonal_mean_boundary_distance"), Debug.WorldOrthogonalMeanBoundaryDistance);
		Root->SetNumberField(TEXT("world_orthogonal_max_boundary_distance"), Debug.WorldOrthogonalMaxBoundaryDistance);
		Root->SetNumberField(TEXT("world_orthogonal_cap_diagonal"), Debug.WorldOrthogonalCapDiagonal);
		Root->SetBoolField(
			TEXT("pure_red_root_alignment_attempted"),
			Debug.bPureRedRootAlignmentAttempted);
		Root->SetNumberField(
			TEXT("selected_root_hypothesis_index"),
			Debug.SelectedRootHypothesisIndex);
		Root->SetNumberField(
			TEXT("selected_root_primitive_edge_index"),
			Debug.SelectedRootPrimitiveEdgeIndex);
		SetIntArrayField(
			Root,
			TEXT("selected_root_stroke_ids"),
			Debug.SelectedRootStrokeIds);
		Root->SetStringField(
			TEXT("selected_root_target_axis_type_name"),
			Debug.SelectedRootTargetAxisTypeName);
		Root->SetNumberField(
			TEXT("selected_root_rotation_degrees"),
			Debug.SelectedRootRotationDegrees);
		Root->SetArrayField(
			TEXT("root_rotation_center_uv"),
			JsonVector2D(Debug.RootRotationCenterUV)->AsArray());
		Root->SetArrayField(
			TEXT("selected_root_start_uv"),
			JsonVector2D(Debug.SelectedRootStartUV)->AsArray());
		Root->SetArrayField(
			TEXT("selected_root_end_uv"),
			JsonVector2D(Debug.SelectedRootEndUV)->AsArray());
		Root->SetArrayField(
			TEXT("selected_aligned_root_start_uv"),
			JsonVector2D(Debug.SelectedAlignedRootStartUV)->AsArray());
		Root->SetArrayField(
			TEXT("selected_aligned_root_end_uv"),
			JsonVector2D(Debug.SelectedAlignedRootEndUV)->AsArray());
		Root->SetBoolField(TEXT("use_per_face_capture"), Debug.bWorldOrthoUsePerFaceCapture);
		Root->SetNumberField(TEXT("per_face_clip_margin_pixels"), Debug.WorldOrthoPerFaceClipMarginPixels);
		Root->SetBoolField(TEXT("pure_red_allow_diagonal_root"), Debug.bWorldOrthoPureRedAllowDiagonalRoot);
		Root->SetBoolField(TEXT("allow_diagonal_supports"), Debug.bWorldOrthoAllowDiagonalSupports);
		Root->SetNumberField(TEXT("black_axis_tolerance_degrees"), Debug.WorldOrthoBlackAxisToleranceDegreesUsed);
		Root->SetNumberField(TEXT("diag_threshold_degrees"), Debug.WorldOrthoDiagThresholdDegreesUsed);
		Root->SetNumberField(
			TEXT("angle_comparison_epsilon_degrees"),
			Debug.WorldOrthoAngleComparisonEpsilonDegreesUsed);
		Root->SetNumberField(
			TEXT("short_red_edge_length_pixels"),
			Debug.WorldOrthoShortRedEdgeLengthPixelsUsed);
		Root->SetNumberField(
			TEXT("minimum_area_ratio"),
			Debug.WorldOrthoMinAreaRatioUsed);
		Root->SetNumberField(TEXT("max_wrap_gap_fraction"), Debug.WorldOrthoMaxWrapGapFractionUsed);
		Root->SetBoolField(TEXT("allow_topology_repair"), Debug.bWorldOrthoAllowTopologyRepair);
		Root->SetBoolField(TEXT("topology_repair_attempted"), Debug.bTopologyRepairAttempted);
		Root->SetBoolField(TEXT("topology_repair_applied"), Debug.bTopologyRepairApplied);
		Root->SetNumberField(TEXT("topology_repair_inserted_count"), Debug.TopologyRepairInsertedCount);
		Root->SetNumberField(TEXT("topology_repair_max_gap_pixels"), Debug.TopologyRepairMaxGapPixels);
		Root->SetStringField(TEXT("topology_repair_inserted_axis_types"), Debug.TopologyRepairInsertedAxisTypeNames);
		Root->SetNumberField(
			TEXT("black_graph_node_snap_tolerance_pixels"),
			Debug.WorldOrthoBlackNodeSnapTolerancePixelsUsed);
		Root->SetNumberField(
			TEXT("red_macro_corridor_pixels"),
			Debug.WorldOrthoRedMacroCorridorPixelsUsed);
		Root->SetNumberField(
			TEXT("red_macro_group_min_length_pixels"),
			Debug.WorldOrthoRedMacroGroupMinLengthPixelsUsed);
		Root->SetNumberField(
			TEXT("red_primitive_rdp_tolerance_pixels"),
			Debug.WorldOrthoRedPrimitiveRdpTolerancePixelsUsed);
		Root->SetNumberField(
			TEXT("pure_red_max_root_candidates"),
			Debug.WorldOrthoPureRedMaxRootCandidatesUsed);
		Root->SetArrayField(TEXT("original_source_to_copied_delta_pixels"), JsonVector2D(Debug.OriginalSourceToCopiedDeltaPixels)->AsArray());
		Root->SetArrayField(TEXT("face_origin_world"), JsonVector(Debug.FaceOriginWorld)->AsArray());
		Root->SetArrayField(TEXT("face_normal_world"), JsonVector(Debug.FaceNormalWorld)->AsArray());
		Root->SetArrayField(TEXT("face_axis_u_world"), JsonVector(Debug.FaceAxisUWorld)->AsArray());
		Root->SetArrayField(TEXT("face_axis_v_world"), JsonVector(Debug.FaceAxisVWorld)->AsArray());
		if (!bWorldOnly)
		{
			SetWorldOrthogonalTraceFields(Root, Debug);
			Root->SetStringField(TEXT("face_boundary_source"), Debug.FaceBoundarySource);
			SetVector2DArrayField(Root, TEXT("face_boundary_uv"), Debug.FaceBoundaryUV);
			SetVector2DArrayField(Root, TEXT("face_convex_hull_uv"), Debug.FaceHullUV);
			SetVector2DArrayField(Root, TEXT("face_oriented_bbox_uv"), Debug.FaceBBoxUV);
			SetVector2DArrayField(Root, TEXT("original_cap_uv"), Debug.OriginalCapUV);
			SetVector2DArrayField(Root, TEXT("original_cap_convex_hull_uv"), Debug.OriginalCapHullUV);
			SetVector2DArrayField(Root, TEXT("cap_bbox_uv"), Debug.CapBaseBBoxUV);
			SetVector2DArrayField(Root, TEXT("cap_base_bbox_uv"), Debug.CapBaseBBoxUV);
			SetVector2DArrayField(Root, TEXT("cap_minimum_bbox_uv"), Debug.CapMinimumBBoxUV);
			SetVector2DArrayField(Root, TEXT("final_cap_uv"), Debug.FinalCapUV);
		}
		SetVectorArrayField(Root, TEXT("original_cap_world"), Debug.OriginalCapWorld);
		SetVectorArrayField(Root, TEXT("cap_bbox_world"), Debug.CapBaseBBoxWorld);
		SetVectorArrayField(Root, TEXT("cap_base_bbox_world"), Debug.CapBaseBBoxWorld);
		SetVectorArrayField(Root, TEXT("cap_minimum_bbox_world"), Debug.CapMinimumBBoxWorld);
		SetVectorArrayField(Root, TEXT("final_cap_world"), Debug.FinalCapWorld);
		SaveJsonObject(Root, Path);
	}

	static void SaveCapBBoxRegularizationDebug(
		const FCapBBoxRegularizationResult& Debug,
		const FString& OutputDir)
	{
		SaveCapBBoxDebugPng(Debug, OutputDir / TEXT("10_cap_world_orthogonal_overview.png"), true);
		SaveCapBBoxDebugPng(Debug, OutputDir / TEXT("10_cap_world_orthogonal_detail.png"), false);
		SaveCapBBoxRegularizationJson(Debug, OutputDir / TEXT("10_cap_world_orthogonal_regularization.json"), false);
		SaveCapBBoxRegularizationJson(Debug, OutputDir / TEXT("10_cap_world_orthogonal_regularization_world.json"), true);

		const FString StepsDir = OutputDir / TEXT("10_cap_world_orthogonal_steps");
		IFileManager::Get().MakeDirectory(*StepsDir, true);
		SaveCapBBoxRegularizationJson(
			Debug,
			StepsDir / TEXT("trace.json"),
			false,
			TEXT("complete_trace"));

		struct FStepOutput
		{
			const TCHAR* Stem;
			const TCHAR* FocusStage;
			int32 ViewIndex;
		};
		const FStepOutput Outputs[] =
		{
			{ TEXT("00_world_uv_runs"), TEXT("boundary_run_unprojection"), 0 },
			{ TEXT("01_protected_corners"), TEXT("red_corner_protection"), 1 },
			{ TEXT("02_primitive_edges"), TEXT("primitive_edge_classification"), 2 },
			{ TEXT("02a_root_alignment"), TEXT("pure_red_root_alignment"), 8 },
			{ TEXT("03_same_axis_merged"), TEXT("same_axis_merge"), 3 },
			{ TEXT("04_axis_validation"), TEXT("adjacency_validation"), 4 },
			{ TEXT("05_support_lines"), TEXT("support_line_solve"), 5 },
			{ TEXT("06_corrected_polygon"), TEXT("vertex_intersections"), 6 },
			{ TEXT("07_final_validation"), TEXT("metric_validation"), 7 }
		};
		for (const FStepOutput& Output : Outputs)
		{
			SaveWorldOrthogonalStepPng(
				Debug,
				StepsDir / (FString(Output.Stem) + TEXT(".png")),
				Output.ViewIndex);
			SaveCapBBoxRegularizationJson(
				Debug,
				StepsDir / (FString(Output.Stem) + TEXT(".json")),
				false,
				Output.FocusStage);
		}
	}

	static void PrepareWorldOrthogonalNotAttemptedDebug(
		FCapBBoxRegularizationResult& Debug,
		const FString& Reason)
	{
		if (Debug.WorldOrthogonalStages.Num() == 0)
		{
			InitializeWorldOrthogonalStages(Debug);
		}
		Debug.bAttempted = false;
		Debug.bApplied = false;
		Debug.bFallbackToOriginal = true;
		Debug.bWorldOrthogonal = true;
		Debug.SelectedGeometry = TEXT("original");
		Debug.RejectionReason = Reason;
		SetWorldOrthogonalStage(
			Debug,
			TEXT("input_validation"),
			TEXT("failed"),
			Reason);
	}

	// Debug overlay: the cap-connected green lines (green) and every candidate face's
	// projected normal (drawn from the candidate's mask centroid). Normal arrow color:
	// bright-green = selected source face, cyan = passed the normal-parallel filter,
	// magenta = candidate that failed the filter. The faint gray loop is the cap mask.
	static bool SaveNormalGreenCheckPng(
		const TArray<uint8>& FacesRGBA, int32 Width, int32 Height,
		const TArray<FVector2D>& CapLoopFaceSpace,
		const TArray<FVector2D>& GreenStarts, const TArray<FVector2D>& GreenEnds,
		const TArray<FFaceCandidate>& Candidates, int32 SelectedFaceId,
		const FString& Path)
	{
		if (FacesRGBA.Num() < Width * Height * 4)
		{
			return false;
		}
		TArray<uint8> RGBA = FacesRGBA;

		DrawClosedPolylineRGBA(RGBA, Width, Height, CapLoopFaceSpace, FColor(140, 140, 140, 255), 1);

		for (int32 i = 0; i < GreenStarts.Num(); ++i)
		{
			DrawArrowRGBA(RGBA, Width, Height, GreenStarts[i], GreenEnds[i], FColor(0, 190, 0, 255), 2);
		}

		const double NormalArrowLen = 110.0;
		for (const FFaceCandidate& C : Candidates)
		{
			if (!C.bHasProjectedNormal)
			{
				continue;
			}
			FColor Col = FColor(230, 0, 180, 255);              // candidate, failed filter
			if (C.FaceId == SelectedFaceId) { Col = FColor(40, 230, 80, 255); }   // selected
			else if (C.bNormalParallelPass) { Col = FColor(0, 200, 255, 255); }   // passed filter
			const FVector2D Tip = C.MaskCentroid + C.ProjectedNormal2D.GetSafeNormal() * NormalArrowLen;
			DrawArrowRGBA(RGBA, Width, Height, C.MaskCentroid, Tip, Col, 2);
		}

		return SaveRGBAToPng(RGBA, Width, Height, Path);
	}

	static void SaveComponentResultJson(const FComponentResult& Result, const FString& Path)
	{
		TSharedRef<FJsonObject> Root = MakeShared<FJsonObject>();
		Root->SetStringField(TEXT("component"), Result.ComponentName);
		Root->SetStringField(TEXT("action"), Result.Action);
		Root->SetStringField(TEXT("polygon_key"), Result.PolygonKey);
		Root->SetBoolField(TEXT("success"), Result.bSuccess);
		Root->SetStringField(TEXT("error"), Result.Error);
		Root->SetStringField(TEXT("actor_name"), Result.ActorName);
		Root->SetStringField(TEXT("attach_path_mode"), Result.AttachPathMode);
		Root->SetStringField(TEXT("chosen_attach_path"), Result.ChosenAttachPath);
		Root->SetStringField(TEXT("attach_path_selection_reason"), Result.AttachPathSelectionReason);
		Root->SetNumberField(TEXT("cap_width"), Result.CapWidth);
		Root->SetNumberField(TEXT("cap_height"), Result.CapHeight);
		Root->SetNumberField(TEXT("faces_width"), Result.FacesWidth);
		Root->SetNumberField(TEXT("faces_height"), Result.FacesHeight);
		Root->SetNumberField(TEXT("cap_mask_pixels"), Result.CapMaskPixels);
		Root->SetNumberField(TEXT("min_overlap_pixels"), Result.MinOverlapPixels);
		Root->SetArrayField(TEXT("green_line_vector_2d"), JsonVector2D(Result.GreenLineVector2D)->AsArray());
		{
			TArray<TSharedPtr<FJsonValue>> GreenVectors;
			for (const FVector2D& V : Result.GreenLineVectors2D)
			{
				GreenVectors.Add(JsonVector2D(V));
			}
			Root->SetArrayField(TEXT("green_line_vectors_2d"), GreenVectors);
		}
		Root->SetNumberField(TEXT("overlap_ratio_threshold"), Result.CandidateFaceMinOverlapRatioUsed);
		Root->SetNumberField(TEXT("normal_parallel_threshold_degrees"), Result.NormalParallelThresholdDegreesUsed);
		Root->SetNumberField(TEXT("preferred_normal_angle_threshold_degrees"), Result.PreferredNormalAngleThresholdDegreesUsed);
		Root->SetNumberField(TEXT("selected_face_id"), Result.SelectedFaceId);
		Root->SetArrayField(TEXT("selected_plane_hit_3d"), JsonVector(Result.SelectedPlaneHit)->AsArray());

		TArray<TSharedPtr<FJsonValue>> CandidateValues;
		for (const FFaceCandidate& Candidate : Result.Candidates)
		{
			TSharedRef<FJsonObject> Obj = MakeShared<FJsonObject>();
			Obj->SetNumberField(TEXT("face_id"), Candidate.FaceId);
			Obj->SetNumberField(TEXT("overlap_pixels"), Candidate.OverlapPixels);
			Obj->SetNumberField(TEXT("overlap_ratio"), Candidate.OverlapRatio);
			Obj->SetArrayField(TEXT("mask_centroid_2d"), JsonVector2D(Candidate.MaskCentroid)->AsArray());
			Obj->SetBoolField(TEXT("has_plane_hit"), Candidate.bHasPlaneHit);
			Obj->SetArrayField(TEXT("plane_hit_3d"), JsonVector(Candidate.PlaneHit)->AsArray());
			Obj->SetNumberField(TEXT("distance_to_camera"), Candidate.DistanceToCamera);
			Obj->SetBoolField(TEXT("has_projected_normal"), Candidate.bHasProjectedNormal);
			Obj->SetArrayField(TEXT("projected_normal_2d"), JsonVector2D(Candidate.ProjectedNormal2D)->AsArray());
			Obj->SetNumberField(TEXT("normal_green_angle_degrees"), Candidate.NormalGreenAngleDegrees);
			Obj->SetBoolField(TEXT("normal_parallel_pass"), Candidate.bNormalParallelPass);
			CandidateValues.Add(MakeShared<FJsonValueObject>(Obj));
		}
		Root->SetArrayField(TEXT("candidates"), CandidateValues);

		TArray<TSharedPtr<FJsonValue>> VertexValues;
		for (const FVector& V : Result.MeshVerticesWorld)
		{
			VertexValues.Add(JsonVector(V));
		}
		Root->SetArrayField(TEXT("mesh_vertices_world"), VertexValues);

		TArray<TSharedPtr<FJsonValue>> TriangleValues;
		for (int32 i = 0; i + 2 < Result.MeshTriangles.Num(); i += 3)
		{
			TriangleValues.Add(JsonIntTriple(Result.MeshTriangles[i], Result.MeshTriangles[i + 1], Result.MeshTriangles[i + 2]));
		}
		Root->SetArrayField(TEXT("mesh_triangles"), TriangleValues);

		SaveJsonObject(Root, Path);
	}

	static int32 SupportForceableReasonPriority(const FString& Reason)
	{
		if (Reason.Contains(TEXT("no_penetration"), ESearchCase::IgnoreCase))
		{
			return 0;
		}
		if (Reason.Contains(TEXT("green_chord_below_preferred_length"), ESearchCase::IgnoreCase))
		{
			return 1;
		}
		return 10;
	}

	static int32 SelectBestForceableSupportAttemptIndex(const FAttachSupportPlaneFallbackDebug& Debug)
	{
		int32 BestIndex = INDEX_NONE;
		int32 BestPriority = TNumericLimits<int32>::Max();
		double BestViolation = TNumericLimits<double>::Max();
		double BestReprojection = TNumericLimits<double>::Max();
		double BestContact = TNumericLimits<double>::Max();
		double BestNegativePathLength = TNumericLimits<double>::Max();
		for (int32 AttemptIndex = 0; AttemptIndex < Debug.Attempts.Num(); ++AttemptIndex)
		{
			const FAttachSupportPlaneFallbackAttempt& Attempt = Debug.Attempts[AttemptIndex];
			if (!Attempt.bForceable ||
				Attempt.bSuccess ||
				!Attempt.bHasBaseProjection ||
				(!Attempt.RawProjectionTopology.bPass && !Attempt.FinalProjectionTopology.bPass) ||
				Attempt.BaseLoopWorld.Num() < 3 ||
				Attempt.BaseLoop2D.Num() != Attempt.BaseLoopWorld.Num() ||
				Attempt.ExtrusionVectorWorld.IsNearlyZero())
			{
				continue;
			}

			const int32 Priority = SupportForceableReasonPriority(Attempt.ForceableReason);
			const double Violation = FMath::IsFinite(Attempt.MaxNoPenetrationViolationCm)
				? Attempt.MaxNoPenetrationViolationCm
				: TNumericLimits<double>::Max();
			const double Reprojection = FMath::IsFinite(Attempt.MaxReprojectionErrorPixels)
				? Attempt.MaxReprojectionErrorPixels
				: TNumericLimits<double>::Max();
			const double Contact = FMath::IsFinite(Attempt.ContactDistancePixels)
				? Attempt.ContactDistancePixels
				: TNumericLimits<double>::Max();
			const double NegativePathLength = -Attempt.ChainPathLength;
			const bool bBetter =
				BestIndex == INDEX_NONE ||
				Priority < BestPriority ||
				(Priority == BestPriority && Violation < BestViolation - 1e-6) ||
				(Priority == BestPriority && FMath::IsNearlyEqual(Violation, BestViolation, 1e-6) &&
					Reprojection < BestReprojection - 1e-6) ||
				(Priority == BestPriority && FMath::IsNearlyEqual(Violation, BestViolation, 1e-6) &&
					FMath::IsNearlyEqual(Reprojection, BestReprojection, 1e-6) &&
					Contact < BestContact - 1e-6) ||
				(Priority == BestPriority && FMath::IsNearlyEqual(Violation, BestViolation, 1e-6) &&
					FMath::IsNearlyEqual(Reprojection, BestReprojection, 1e-6) &&
					FMath::IsNearlyEqual(Contact, BestContact, 1e-6) &&
					NegativePathLength < BestNegativePathLength);
			if (bBetter)
			{
				BestIndex = AttemptIndex;
				BestPriority = Priority;
				BestViolation = Violation;
				BestReprojection = Reprojection;
				BestContact = Contact;
				BestNegativePathLength = NegativePathLength;
			}
		}
		return BestIndex;
	}

	static void DrawTopologyIntersectionEdgesRGBA(
		TArray<uint8>& RGBA,
		int32 Width,
		int32 Height,
		const TArray<FVector2D>& Loop2D,
		const FSupportPlaneTopologyDebug& Topology,
		const FColor& Color,
		int32 Radius)
	{
		if (Loop2D.Num() < 2)
		{
			return;
		}
		auto DrawEdge = [&](int32 EdgeIndex)
		{
			if (!Loop2D.IsValidIndex(EdgeIndex))
			{
				return;
			}
			const int32 Next = (EdgeIndex + 1) % Loop2D.Num();
			if (Loop2D.IsValidIndex(Next))
			{
				DrawLineRGBA(RGBA, Width, Height, Loop2D[EdgeIndex], Loop2D[Next], Color, Radius);
			}
		};
		for (const FIntPoint& Pair : Topology.SourceSelfIntersectionEdges)
		{
			DrawEdge(Pair.X);
			DrawEdge(Pair.Y);
		}
		for (const FIntPoint& Pair : Topology.ProjectedSelfIntersectionEdges)
		{
			DrawEdge(Pair.X);
			DrawEdge(Pair.Y);
		}
	}

	static void SaveAttachSupportPlaneFallbackDebug(
		const FAttachSupportPlaneFallbackDebug& Debug,
		const FCommonInputs& Inputs,
		const TArray<FVector2D>& RawCapLoopFaceSpace,
		const FString& JsonPath,
		const FString& PngPath)
	{
		TSharedRef<FJsonObject> Root = MakeShared<FJsonObject>();
		Root->SetBoolField(TEXT("attempted"), Debug.bAttempted);
		Root->SetBoolField(TEXT("enabled"), Debug.bEnabled);
		Root->SetBoolField(TEXT("success"), Debug.bSuccess);
		Root->SetStringField(TEXT("error"), Debug.Error);
		Root->SetNumberField(TEXT("support_face_vote_min_coverage"), Debug.SupportFaceVoteMinCoverage);
		Root->SetNumberField(TEXT("support_face_vote_sample_step_px"), Debug.SupportFaceVoteSampleStepPx);
		Root->SetBoolField(TEXT("support_plane_polygon_check_enforced"), false);
		const int32 BestForceableAttemptIndex = Debug.BestForceableAttemptIndex != INDEX_NONE
			? Debug.BestForceableAttemptIndex
			: SelectBestForceableSupportAttemptIndex(Debug);
		int32 BestFailedAttemptIndex = INDEX_NONE;
		double BestFailedViolation = TNumericLimits<double>::Max();
		for (int32 AttemptIndex = 0; AttemptIndex < Debug.Attempts.Num(); ++AttemptIndex)
		{
			const FAttachSupportPlaneFallbackAttempt& Attempt = Debug.Attempts[AttemptIndex];
			if (Attempt.bSuccess ||
				!Attempt.bHasBaseProjection ||
				Attempt.NoPenetrationViolationCm.Num() == 0 ||
				!FMath::IsFinite(Attempt.MaxNoPenetrationViolationCm))
			{
				continue;
			}
			if (Attempt.MaxNoPenetrationViolationCm < BestFailedViolation)
			{
				BestFailedViolation = Attempt.MaxNoPenetrationViolationCm;
				BestFailedAttemptIndex = AttemptIndex;
			}
		}
		Root->SetNumberField(TEXT("best_failed_attempt_index"), BestFailedAttemptIndex);
		Root->SetNumberField(TEXT("best_failed_debug_attempt_index"), BestFailedAttemptIndex);
		Root->SetNumberField(
			TEXT("best_failed_max_no_penetration_violation_cm"),
			BestFailedAttemptIndex >= 0 ? BestFailedViolation : 0.0);
		Root->SetNumberField(TEXT("best_forceable_attempt_index"), BestForceableAttemptIndex);
		Root->SetStringField(
			TEXT("best_forceable_reason"),
			Debug.Attempts.IsValidIndex(BestForceableAttemptIndex)
				? Debug.Attempts[BestForceableAttemptIndex].ForceableReason
				: Debug.BestForceableReason);
		Root->SetBoolField(TEXT("forced_support_plane_output"), Debug.bForcedOutput);
		Root->SetNumberField(TEXT("forced_attempt_index"), Debug.ForcedAttemptIndex);
		Root->SetStringField(TEXT("forced_reason"), Debug.ForcedReason);

		TArray<TSharedPtr<FJsonValue>> AttemptValues;
		for (int32 AttemptIndex = 0; AttemptIndex < Debug.Attempts.Num(); ++AttemptIndex)
		{
			const FAttachSupportPlaneFallbackAttempt& Attempt = Debug.Attempts[AttemptIndex];
			TSharedRef<FJsonObject> Obj = MakeShared<FJsonObject>();
			Obj->SetNumberField(TEXT("chain_index"), Attempt.ChainIndex);
			Obj->SetNumberField(TEXT("seed_stroke_id"), Attempt.SeedStrokeId);
			Obj->SetNumberField(TEXT("support_face_id"), Attempt.SupportFaceId);
			Obj->SetBoolField(TEXT("success"), Attempt.bSuccess);
			Obj->SetBoolField(TEXT("best_failed_debug_attempt"), AttemptIndex == BestFailedAttemptIndex);
			Obj->SetBoolField(TEXT("best_forceable_attempt"), AttemptIndex == BestForceableAttemptIndex);
			Obj->SetBoolField(TEXT("forceable"), Attempt.bForceable);
			Obj->SetStringField(TEXT("forceable_reason"), Attempt.ForceableReason);
			Obj->SetStringField(TEXT("reject_reason"), Attempt.RejectReason);
			Obj->SetArrayField(TEXT("chain_start_face_space"), JsonVector2D(Attempt.ChainStartFaceSpace)->AsArray());
			Obj->SetArrayField(TEXT("chain_end_face_space"), JsonVector2D(Attempt.ChainEndFaceSpace)->AsArray());
			Obj->SetNumberField(TEXT("chain_path_length"), Attempt.ChainPathLength);
			Obj->SetNumberField(TEXT("chain_chord_length"), Attempt.ChainChordLength);
			SetSupportFaceVoteAttemptDebugFields(
				Obj,
				Attempt.SupportVotePixels,
				Attempt.SupportConsideredPixels,
				Attempt.SupportHitSampleCount,
				Attempt.SupportTotalSampleCount,
				Attempt.SupportVoteCoverage,
				Attempt.SupportVotePixelCoverage,
				Attempt.SupportFaceWorldZMax,
				Attempt.SupportFaceWorldZAverage,
				Attempt.SupportMinCameraDistance,
				Attempt.SupportWorstPolygonDistancePx,
				Attempt.SupportFaceCandidates);
			Obj->SetArrayField(TEXT("anchor_world"), JsonVector(Attempt.AnchorWorld)->AsArray());
			Obj->SetArrayField(TEXT("chain_end_world"), JsonVector(Attempt.ChainEndWorld)->AsArray());
			Obj->SetArrayField(TEXT("extrusion_vector_world"), JsonVector(Attempt.ExtrusionVectorWorld)->AsArray());
			Obj->SetArrayField(TEXT("base_plane_normal"), JsonVector(Attempt.BasePlaneNormal)->AsArray());
			Obj->SetArrayField(TEXT("support_plane_normal"), JsonVector(Attempt.SupportPlaneNormal)->AsArray());
			Obj->SetArrayField(TEXT("support_aware_axis_u"), JsonVector(Attempt.SupportAwareAxisU)->AsArray());
			Obj->SetArrayField(TEXT("support_aware_axis_v"), JsonVector(Attempt.SupportAwareAxisV)->AsArray());
			Obj->SetBoolField(TEXT("has_base_projection"), Attempt.bHasBaseProjection);
			SetVector2DArrayField(Obj, TEXT("base_loop_2d"), Attempt.BaseLoop2D);
			SetVector2DArrayField(Obj, TEXT("copied_target_loop_2d"), Attempt.CopiedTargetLoop2D);
			SetVector2DArrayField(Obj, TEXT("reprojected_base_loop_2d"), Attempt.ReprojectedBaseLoop2D);
			SetVector2DArrayField(Obj, TEXT("reprojected_copied_loop_2d"), Attempt.ReprojectedCopiedLoop2D);
			SetVector2DArrayField(Obj, TEXT("virtual_base_plane_quad_2d"), Attempt.VirtualBasePlaneQuad2D);
			SetVectorArrayField(Obj, TEXT("base_loop_world"), Attempt.BaseLoopWorld);
			SetVectorArrayField(Obj, TEXT("copied_loop_world"), Attempt.CopiedLoopWorld);
			SetVectorArrayField(Obj, TEXT("virtual_base_plane_quad_world"), Attempt.VirtualBasePlaneQuadWorld);
			Obj->SetObjectField(TEXT("raw_projection_topology"), MakeSupportTopologyDebugJson(Attempt.RawProjectionTopology));
			Obj->SetObjectField(TEXT("regularized_projection_topology"), MakeSupportTopologyDebugJson(Attempt.RegularizedProjectionTopology));
			Obj->SetObjectField(TEXT("final_projection_topology"), MakeSupportTopologyDebugJson(Attempt.FinalProjectionTopology));
			SetVector2DArrayField(Obj, TEXT("no_penetration_sample_pixels"), Attempt.NoPenetrationSamplePixels);
			SetVectorArrayField(Obj, TEXT("no_penetration_base_world"), Attempt.NoPenetrationBaseWorld);
			SetVectorArrayField(Obj, TEXT("no_penetration_support_world"), Attempt.NoPenetrationSupportWorld);
			SetDoubleArrayField(Obj, TEXT("no_penetration_violation_cm"), Attempt.NoPenetrationViolationCm);
			Obj->SetBoolField(TEXT("orthogonal_attempted"), Attempt.bOrthogonalAttempted);
			Obj->SetBoolField(TEXT("orthogonal_applied"), Attempt.bOrthogonalApplied);
			Obj->SetBoolField(TEXT("orthogonal_fallback_to_original"), Attempt.bOrthogonalFallbackToOriginal);
			Obj->SetNumberField(TEXT("contact_distance_pixels"), Attempt.ContactDistancePixels);
			Obj->SetBoolField(TEXT("contact_pass"), Attempt.bContactPass);
			Obj->SetNumberField(TEXT("max_no_penetration_violation_cm"), Attempt.MaxNoPenetrationViolationCm);
			Obj->SetBoolField(TEXT("no_penetration_pass"), Attempt.bNoPenetrationPass);
			Obj->SetNumberField(TEXT("max_reprojection_error_pixels"), Attempt.MaxReprojectionErrorPixels);
			Obj->SetBoolField(TEXT("reprojection_pass"), Attempt.bReprojectionPass);
			AttemptValues.Add(MakeShared<FJsonValueObject>(Obj));
		}
		Root->SetArrayField(TEXT("attempts"), AttemptValues);
		SaveJsonObject(Root, JsonPath);

		if (Inputs.FacesRGBA.Num() >= Inputs.FacesWidth * Inputs.FacesHeight * 4)
		{
			TArray<uint8> RGBA = Inputs.FacesRGBA;
			DrawClosedPolylineRGBA(RGBA, Inputs.FacesWidth, Inputs.FacesHeight, RawCapLoopFaceSpace, FColor(255, 255, 255, 255), 2);
			for (const FAttachSupportPlaneFallbackAttempt& Attempt : Debug.Attempts)
			{
				const FColor ChainColor = Attempt.bSuccess
					? FColor(40, 240, 80, 255)
					: (Attempt.SupportFaceId >= 0 ? FColor(255, 205, 30, 255) : FColor(230, 30, 50, 255));
				DrawArrowRGBA(
					RGBA,
					Inputs.FacesWidth,
					Inputs.FacesHeight,
					Attempt.ChainStartFaceSpace,
					Attempt.ChainEndFaceSpace,
					ChainColor,
					2);
				DrawPointRGBA(
					RGBA,
					Inputs.FacesWidth,
					Inputs.FacesHeight,
					FMath::RoundToInt(Attempt.ChainStartFaceSpace.X),
					FMath::RoundToInt(Attempt.ChainStartFaceSpace.Y),
					FColor(0, 255, 80, 255),
					3);
			}
			if (Debug.Attempts.IsValidIndex(BestFailedAttemptIndex))
			{
				const FAttachSupportPlaneFallbackAttempt& Best = Debug.Attempts[BestFailedAttemptIndex];
				DrawClosedPolylineRGBA(RGBA, Inputs.FacesWidth, Inputs.FacesHeight, Best.VirtualBasePlaneQuad2D, FColor(0, 230, 255, 255), 3);
				DrawClosedPolylineRGBA(RGBA, Inputs.FacesWidth, Inputs.FacesHeight, Best.BaseLoop2D, FColor(0, 180, 255, 255), 3);
				DrawClosedPolylineRGBA(RGBA, Inputs.FacesWidth, Inputs.FacesHeight, Best.CopiedTargetLoop2D, FColor(255, 140, 0, 255), 2);
				DrawTopologyIntersectionEdgesRGBA(RGBA, Inputs.FacesWidth, Inputs.FacesHeight, Best.BaseLoop2D, Best.RawProjectionTopology, FColor(255, 0, 0, 255), 5);
				DrawTopologyIntersectionEdgesRGBA(RGBA, Inputs.FacesWidth, Inputs.FacesHeight, Best.BaseLoop2D, Best.FinalProjectionTopology, FColor(255, 0, 0, 255), 5);
				for (int32 SampleIndex = 0; SampleIndex < Best.NoPenetrationSamplePixels.Num(); ++SampleIndex)
				{
					const double Violation = Best.NoPenetrationViolationCm.IsValidIndex(SampleIndex)
						? Best.NoPenetrationViolationCm[SampleIndex]
						: 0.0;
					const FColor SampleColor = Violation > 0.0
						? FColor(255, 40, 40, 255)
						: FColor(40, 240, 80, 255);
					DrawPointRGBA(
						RGBA,
						Inputs.FacesWidth,
						Inputs.FacesHeight,
						FMath::RoundToInt(Best.NoPenetrationSamplePixels[SampleIndex].X),
						FMath::RoundToInt(Best.NoPenetrationSamplePixels[SampleIndex].Y),
						SampleColor,
						4);
				}
			}
			SaveRGBAToPng(RGBA, Inputs.FacesWidth, Inputs.FacesHeight, PngPath);

			const FString AttemptDebugDir =
				FPaths::GetPath(PngPath) / TEXT("10_attach_support_plane_fallback_attempts");
			IFileManager::Get().MakeDirectory(*AttemptDebugDir, true);
			TArray<TSharedPtr<FJsonValue>> AttemptIndexValues;
			for (int32 AttemptIndex = 0; AttemptIndex < Debug.Attempts.Num(); ++AttemptIndex)
			{
				const FAttachSupportPlaneFallbackAttempt& Attempt = Debug.Attempts[AttemptIndex];
				const FString AttemptBaseName = FString::Printf(
					TEXT("attempt_%03d_chain_%03d"),
					AttemptIndex,
					Attempt.ChainIndex);
				const FString AttemptPngPath = AttemptDebugDir / (AttemptBaseName + TEXT(".png"));
				const FString AttemptJsonPath = AttemptDebugDir / (AttemptBaseName + TEXT(".json"));

				TArray<uint8> AttemptRGBA = Inputs.FacesRGBA;
				DrawClosedPolylineRGBA(
					AttemptRGBA,
					Inputs.FacesWidth,
					Inputs.FacesHeight,
					RawCapLoopFaceSpace,
					FColor(255, 255, 255, 255),
					2);
				if (const int32* SupportIndex = Inputs.FaceIndexById.Find(Attempt.SupportFaceId))
				{
					if (Inputs.Faces.IsValidIndex(*SupportIndex))
					{
						DrawClosedPolylineRGBA(
							AttemptRGBA,
							Inputs.FacesWidth,
							Inputs.FacesHeight,
							Inputs.Faces[*SupportIndex].KeyPoints2D,
							FColor(80, 80, 80, 255),
							2);
					}
				}

				if (Attempt.VirtualBasePlaneQuad2D.Num() >= 3)
				{
					DrawClosedPolylineRGBA(
						AttemptRGBA,
						Inputs.FacesWidth,
						Inputs.FacesHeight,
						Attempt.VirtualBasePlaneQuad2D,
						FColor(0, 230, 255, 255),
						4);
				}
				if (Attempt.BaseLoop2D.Num() >= 3)
				{
					DrawClosedPolylineRGBA(
						AttemptRGBA,
						Inputs.FacesWidth,
						Inputs.FacesHeight,
						Attempt.BaseLoop2D,
						FColor(0, 180, 255, 255),
						3);
				}
				if (Attempt.ReprojectedBaseLoop2D.Num() >= 3)
				{
					DrawClosedPolylineRGBA(
						AttemptRGBA,
						Inputs.FacesWidth,
						Inputs.FacesHeight,
						Attempt.ReprojectedBaseLoop2D,
						FColor(40, 240, 80, 255),
						2);
				}
				if (Attempt.CopiedTargetLoop2D.Num() >= 3)
				{
					DrawClosedPolylineRGBA(
						AttemptRGBA,
						Inputs.FacesWidth,
						Inputs.FacesHeight,
						Attempt.CopiedTargetLoop2D,
						FColor(255, 140, 0, 255),
						2);
				}
				if (Attempt.ReprojectedCopiedLoop2D.Num() >= 3)
				{
					DrawClosedPolylineRGBA(
						AttemptRGBA,
						Inputs.FacesWidth,
						Inputs.FacesHeight,
						Attempt.ReprojectedCopiedLoop2D,
						FColor(255, 0, 180, 255),
						2);
				}
				DrawTopologyIntersectionEdgesRGBA(
					AttemptRGBA,
					Inputs.FacesWidth,
					Inputs.FacesHeight,
					Attempt.BaseLoop2D,
					Attempt.RawProjectionTopology,
					FColor(255, 0, 0, 255),
					5);
				DrawTopologyIntersectionEdgesRGBA(
					AttemptRGBA,
					Inputs.FacesWidth,
					Inputs.FacesHeight,
					Attempt.BaseLoop2D,
					Attempt.FinalProjectionTopology,
					FColor(255, 0, 0, 255),
					5);

				const FColor ChainColor = Attempt.bSuccess
					? FColor(40, 240, 80, 255)
					: (Attempt.SupportFaceId >= 0 ? FColor(255, 225, 40, 255) : FColor(230, 30, 50, 255));
				DrawArrowRGBA(
					AttemptRGBA,
					Inputs.FacesWidth,
					Inputs.FacesHeight,
					Attempt.ChainStartFaceSpace,
					Attempt.ChainEndFaceSpace,
					ChainColor,
					3);
				DrawPointRGBA(
					AttemptRGBA,
					Inputs.FacesWidth,
					Inputs.FacesHeight,
					FMath::RoundToInt(Attempt.ChainStartFaceSpace.X),
					FMath::RoundToInt(Attempt.ChainStartFaceSpace.Y),
					FColor(255, 0, 0, 255),
					5);

				for (int32 SampleIndex = 0; SampleIndex < Attempt.NoPenetrationSamplePixels.Num(); ++SampleIndex)
				{
					const double Violation = Attempt.NoPenetrationViolationCm.IsValidIndex(SampleIndex)
						? Attempt.NoPenetrationViolationCm[SampleIndex]
						: 0.0;
					const FColor SampleColor = Violation > 0.0
						? FColor(255, 40, 40, 255)
						: FColor(40, 240, 80, 255);
					DrawPointRGBA(
						AttemptRGBA,
						Inputs.FacesWidth,
						Inputs.FacesHeight,
						FMath::RoundToInt(Attempt.NoPenetrationSamplePixels[SampleIndex].X),
						FMath::RoundToInt(Attempt.NoPenetrationSamplePixels[SampleIndex].Y),
						SampleColor,
						5);
				}
				SaveRGBAToPng(AttemptRGBA, Inputs.FacesWidth, Inputs.FacesHeight, AttemptPngPath);

				TSharedRef<FJsonObject> AttemptRoot = MakeShared<FJsonObject>();
				AttemptRoot->SetNumberField(TEXT("attempt_index"), AttemptIndex);
				AttemptRoot->SetNumberField(TEXT("chain_index"), Attempt.ChainIndex);
				AttemptRoot->SetNumberField(TEXT("seed_stroke_id"), Attempt.SeedStrokeId);
				AttemptRoot->SetNumberField(TEXT("support_face_id"), Attempt.SupportFaceId);
				AttemptRoot->SetBoolField(TEXT("success"), Attempt.bSuccess);
				AttemptRoot->SetBoolField(TEXT("best_failed_debug_attempt"), AttemptIndex == BestFailedAttemptIndex);
				AttemptRoot->SetBoolField(TEXT("best_forceable_attempt"), AttemptIndex == BestForceableAttemptIndex);
				AttemptRoot->SetBoolField(TEXT("forceable"), Attempt.bForceable);
				AttemptRoot->SetStringField(TEXT("forceable_reason"), Attempt.ForceableReason);
				AttemptRoot->SetStringField(TEXT("reject_reason"), Attempt.RejectReason);
				AttemptRoot->SetArrayField(TEXT("chain_start_face_space"), JsonVector2D(Attempt.ChainStartFaceSpace)->AsArray());
				AttemptRoot->SetArrayField(TEXT("chain_end_face_space"), JsonVector2D(Attempt.ChainEndFaceSpace)->AsArray());
				AttemptRoot->SetNumberField(TEXT("chain_path_length"), Attempt.ChainPathLength);
				AttemptRoot->SetNumberField(TEXT("chain_chord_length"), Attempt.ChainChordLength);
				SetSupportFaceVoteAttemptDebugFields(
					AttemptRoot,
					Attempt.SupportVotePixels,
					Attempt.SupportConsideredPixels,
					Attempt.SupportHitSampleCount,
					Attempt.SupportTotalSampleCount,
					Attempt.SupportVoteCoverage,
					Attempt.SupportVotePixelCoverage,
					Attempt.SupportFaceWorldZMax,
					Attempt.SupportFaceWorldZAverage,
					Attempt.SupportMinCameraDistance,
					Attempt.SupportWorstPolygonDistancePx,
					Attempt.SupportFaceCandidates);
				AttemptRoot->SetArrayField(TEXT("anchor_world"), JsonVector(Attempt.AnchorWorld)->AsArray());
				AttemptRoot->SetArrayField(TEXT("chain_end_world"), JsonVector(Attempt.ChainEndWorld)->AsArray());
				AttemptRoot->SetArrayField(TEXT("extrusion_vector_world"), JsonVector(Attempt.ExtrusionVectorWorld)->AsArray());
				AttemptRoot->SetArrayField(TEXT("base_plane_normal"), JsonVector(Attempt.BasePlaneNormal)->AsArray());
				AttemptRoot->SetArrayField(TEXT("support_plane_normal"), JsonVector(Attempt.SupportPlaneNormal)->AsArray());
				AttemptRoot->SetArrayField(TEXT("support_aware_axis_u"), JsonVector(Attempt.SupportAwareAxisU)->AsArray());
				AttemptRoot->SetArrayField(TEXT("support_aware_axis_v"), JsonVector(Attempt.SupportAwareAxisV)->AsArray());
				AttemptRoot->SetBoolField(TEXT("has_base_projection"), Attempt.bHasBaseProjection);
				SetVector2DArrayField(AttemptRoot, TEXT("base_loop_2d"), Attempt.BaseLoop2D);
				SetVector2DArrayField(AttemptRoot, TEXT("copied_target_loop_2d"), Attempt.CopiedTargetLoop2D);
				SetVector2DArrayField(AttemptRoot, TEXT("reprojected_base_loop_2d"), Attempt.ReprojectedBaseLoop2D);
				SetVector2DArrayField(AttemptRoot, TEXT("reprojected_copied_loop_2d"), Attempt.ReprojectedCopiedLoop2D);
				SetVector2DArrayField(AttemptRoot, TEXT("virtual_base_plane_quad_2d"), Attempt.VirtualBasePlaneQuad2D);
				SetVectorArrayField(AttemptRoot, TEXT("base_loop_world"), Attempt.BaseLoopWorld);
				SetVectorArrayField(AttemptRoot, TEXT("copied_loop_world"), Attempt.CopiedLoopWorld);
				SetVectorArrayField(AttemptRoot, TEXT("virtual_base_plane_quad_world"), Attempt.VirtualBasePlaneQuadWorld);
				AttemptRoot->SetObjectField(TEXT("raw_projection_topology"), MakeSupportTopologyDebugJson(Attempt.RawProjectionTopology));
				AttemptRoot->SetObjectField(TEXT("regularized_projection_topology"), MakeSupportTopologyDebugJson(Attempt.RegularizedProjectionTopology));
				AttemptRoot->SetObjectField(TEXT("final_projection_topology"), MakeSupportTopologyDebugJson(Attempt.FinalProjectionTopology));
				SetVector2DArrayField(AttemptRoot, TEXT("no_penetration_sample_pixels"), Attempt.NoPenetrationSamplePixels);
				SetVectorArrayField(AttemptRoot, TEXT("no_penetration_base_world"), Attempt.NoPenetrationBaseWorld);
				SetVectorArrayField(AttemptRoot, TEXT("no_penetration_support_world"), Attempt.NoPenetrationSupportWorld);
				SetDoubleArrayField(AttemptRoot, TEXT("no_penetration_violation_cm"), Attempt.NoPenetrationViolationCm);
				AttemptRoot->SetBoolField(TEXT("orthogonal_attempted"), Attempt.bOrthogonalAttempted);
				AttemptRoot->SetBoolField(TEXT("orthogonal_applied"), Attempt.bOrthogonalApplied);
				AttemptRoot->SetBoolField(TEXT("orthogonal_fallback_to_original"), Attempt.bOrthogonalFallbackToOriginal);
				AttemptRoot->SetNumberField(TEXT("contact_distance_pixels"), Attempt.ContactDistancePixels);
				AttemptRoot->SetBoolField(TEXT("contact_pass"), Attempt.bContactPass);
				AttemptRoot->SetNumberField(TEXT("max_no_penetration_violation_cm"), Attempt.MaxNoPenetrationViolationCm);
				AttemptRoot->SetBoolField(TEXT("no_penetration_pass"), Attempt.bNoPenetrationPass);
				AttemptRoot->SetNumberField(TEXT("max_reprojection_error_pixels"), Attempt.MaxReprojectionErrorPixels);
				AttemptRoot->SetBoolField(TEXT("reprojection_pass"), Attempt.bReprojectionPass);
				SaveJsonObject(AttemptRoot, AttemptJsonPath);

				TSharedRef<FJsonObject> IndexObj = MakeShared<FJsonObject>();
				IndexObj->SetNumberField(TEXT("attempt_index"), AttemptIndex);
				IndexObj->SetNumberField(TEXT("chain_index"), Attempt.ChainIndex);
				IndexObj->SetNumberField(TEXT("support_face_id"), Attempt.SupportFaceId);
				IndexObj->SetNumberField(TEXT("support_sample_coverage"), Attempt.SupportVoteCoverage);
				IndexObj->SetNumberField(TEXT("support_hit_sample_count"), Attempt.SupportHitSampleCount);
				IndexObj->SetNumberField(TEXT("support_total_sample_count"), Attempt.SupportTotalSampleCount);
				IndexObj->SetNumberField(TEXT("support_face_world_z_max"), Attempt.SupportFaceWorldZMax);
				IndexObj->SetNumberField(TEXT("support_min_camera_distance"), Attempt.SupportMinCameraDistance);
				IndexObj->SetBoolField(TEXT("success"), Attempt.bSuccess);
				IndexObj->SetBoolField(TEXT("best_failed_debug_attempt"), AttemptIndex == BestFailedAttemptIndex);
				IndexObj->SetBoolField(TEXT("best_forceable_attempt"), AttemptIndex == BestForceableAttemptIndex);
				IndexObj->SetBoolField(TEXT("forceable"), Attempt.bForceable);
				IndexObj->SetStringField(TEXT("forceable_reason"), Attempt.ForceableReason);
				IndexObj->SetBoolField(TEXT("raw_projection_topology_checked"), Attempt.RawProjectionTopology.bChecked);
				IndexObj->SetBoolField(TEXT("raw_projection_topology_pass"), Attempt.RawProjectionTopology.bPass);
				IndexObj->SetStringField(TEXT("raw_projection_topology_reason"), Attempt.RawProjectionTopology.Reason);
				IndexObj->SetBoolField(TEXT("regularized_projection_topology_checked"), Attempt.RegularizedProjectionTopology.bChecked);
				IndexObj->SetBoolField(TEXT("regularized_projection_topology_pass"), Attempt.RegularizedProjectionTopology.bPass);
				IndexObj->SetStringField(TEXT("regularized_projection_topology_reason"), Attempt.RegularizedProjectionTopology.Reason);
				IndexObj->SetBoolField(TEXT("final_projection_topology_checked"), Attempt.FinalProjectionTopology.bChecked);
				IndexObj->SetBoolField(TEXT("final_projection_topology_pass"), Attempt.FinalProjectionTopology.bPass);
				IndexObj->SetStringField(TEXT("final_projection_topology_reason"), Attempt.FinalProjectionTopology.Reason);
				IndexObj->SetStringField(TEXT("reject_reason"), Attempt.RejectReason);
				IndexObj->SetStringField(TEXT("png"), AttemptBaseName + TEXT(".png"));
				IndexObj->SetStringField(TEXT("json"), AttemptBaseName + TEXT(".json"));
				AttemptIndexValues.Add(MakeShared<FJsonValueObject>(IndexObj));
			}
			TSharedRef<FJsonObject> AttemptsIndexRoot = MakeShared<FJsonObject>();
			AttemptsIndexRoot->SetNumberField(TEXT("attempt_count"), Debug.Attempts.Num());
			AttemptsIndexRoot->SetBoolField(TEXT("success"), Debug.bSuccess);
			AttemptsIndexRoot->SetNumberField(TEXT("best_failed_attempt_index"), BestFailedAttemptIndex);
			AttemptsIndexRoot->SetNumberField(TEXT("best_failed_debug_attempt_index"), BestFailedAttemptIndex);
			AttemptsIndexRoot->SetNumberField(TEXT("best_forceable_attempt_index"), BestForceableAttemptIndex);
			AttemptsIndexRoot->SetStringField(
				TEXT("best_forceable_reason"),
				Debug.Attempts.IsValidIndex(BestForceableAttemptIndex)
					? Debug.Attempts[BestForceableAttemptIndex].ForceableReason
					: Debug.BestForceableReason);
			AttemptsIndexRoot->SetBoolField(TEXT("forced_support_plane_output"), Debug.bForcedOutput);
			AttemptsIndexRoot->SetNumberField(TEXT("forced_attempt_index"), Debug.ForcedAttemptIndex);
			AttemptsIndexRoot->SetStringField(TEXT("forced_reason"), Debug.ForcedReason);
			AttemptsIndexRoot->SetArrayField(TEXT("attempts"), AttemptIndexValues);
			SaveJsonObject(AttemptsIndexRoot, AttemptDebugDir / TEXT("index.json"));

			if (Debug.Attempts.IsValidIndex(BestFailedAttemptIndex))
			{
				const FAttachSupportPlaneFallbackAttempt& Best = Debug.Attempts[BestFailedAttemptIndex];
				TArray<uint8> BestRGBA = Inputs.FacesRGBA;
				DrawClosedPolylineRGBA(BestRGBA, Inputs.FacesWidth, Inputs.FacesHeight, RawCapLoopFaceSpace, FColor(255, 255, 255, 255), 2);
				if (const int32* SupportIndex = Inputs.FaceIndexById.Find(Best.SupportFaceId))
				{
					if (Inputs.Faces.IsValidIndex(*SupportIndex))
					{
						DrawClosedPolylineRGBA(BestRGBA, Inputs.FacesWidth, Inputs.FacesHeight, Inputs.Faces[*SupportIndex].KeyPoints2D, FColor(80, 80, 80, 255), 2);
					}
				}
				DrawClosedPolylineRGBA(BestRGBA, Inputs.FacesWidth, Inputs.FacesHeight, Best.VirtualBasePlaneQuad2D, FColor(0, 230, 255, 255), 4);
				DrawClosedPolylineRGBA(BestRGBA, Inputs.FacesWidth, Inputs.FacesHeight, Best.BaseLoop2D, FColor(0, 180, 255, 255), 3);
				DrawClosedPolylineRGBA(BestRGBA, Inputs.FacesWidth, Inputs.FacesHeight, Best.ReprojectedBaseLoop2D, FColor(40, 240, 80, 255), 2);
				DrawClosedPolylineRGBA(BestRGBA, Inputs.FacesWidth, Inputs.FacesHeight, Best.CopiedTargetLoop2D, FColor(255, 140, 0, 255), 2);
				DrawClosedPolylineRGBA(BestRGBA, Inputs.FacesWidth, Inputs.FacesHeight, Best.ReprojectedCopiedLoop2D, FColor(255, 0, 180, 255), 2);
				DrawTopologyIntersectionEdgesRGBA(BestRGBA, Inputs.FacesWidth, Inputs.FacesHeight, Best.BaseLoop2D, Best.RawProjectionTopology, FColor(255, 0, 0, 255), 5);
				DrawTopologyIntersectionEdgesRGBA(BestRGBA, Inputs.FacesWidth, Inputs.FacesHeight, Best.BaseLoop2D, Best.FinalProjectionTopology, FColor(255, 0, 0, 255), 5);
				DrawArrowRGBA(BestRGBA, Inputs.FacesWidth, Inputs.FacesHeight, Best.ChainStartFaceSpace, Best.ChainEndFaceSpace, FColor(255, 225, 40, 255), 3);
				for (int32 SampleIndex = 0; SampleIndex < Best.NoPenetrationSamplePixels.Num(); ++SampleIndex)
				{
					const double Violation = Best.NoPenetrationViolationCm.IsValidIndex(SampleIndex)
						? Best.NoPenetrationViolationCm[SampleIndex]
						: 0.0;
					const FColor SampleColor = Violation > 0.0
						? FColor(255, 40, 40, 255)
						: FColor(40, 240, 80, 255);
					DrawPointRGBA(
						BestRGBA,
						Inputs.FacesWidth,
						Inputs.FacesHeight,
						FMath::RoundToInt(Best.NoPenetrationSamplePixels[SampleIndex].X),
						FMath::RoundToInt(Best.NoPenetrationSamplePixels[SampleIndex].Y),
						SampleColor,
						5);
				}
				SaveRGBAToPng(
					BestRGBA,
					Inputs.FacesWidth,
					Inputs.FacesHeight,
					FPaths::GetPath(PngPath) / TEXT("10_attach_support_plane_fallback_best_failed.png"));

				TSharedRef<FJsonObject> BestRoot = MakeShared<FJsonObject>();
				BestRoot->SetNumberField(TEXT("best_failed_attempt_index"), BestFailedAttemptIndex);
				BestRoot->SetNumberField(TEXT("chain_index"), Best.ChainIndex);
				BestRoot->SetNumberField(TEXT("support_face_id"), Best.SupportFaceId);
				BestRoot->SetStringField(TEXT("reject_reason"), Best.RejectReason);
				BestRoot->SetNumberField(TEXT("max_no_penetration_violation_cm"), Best.MaxNoPenetrationViolationCm);
				BestRoot->SetArrayField(TEXT("anchor_world"), JsonVector(Best.AnchorWorld)->AsArray());
				BestRoot->SetArrayField(TEXT("chain_end_world"), JsonVector(Best.ChainEndWorld)->AsArray());
				BestRoot->SetArrayField(TEXT("base_plane_normal"), JsonVector(Best.BasePlaneNormal)->AsArray());
				BestRoot->SetArrayField(TEXT("support_plane_normal"), JsonVector(Best.SupportPlaneNormal)->AsArray());
				BestRoot->SetArrayField(TEXT("support_aware_axis_u"), JsonVector(Best.SupportAwareAxisU)->AsArray());
				BestRoot->SetArrayField(TEXT("support_aware_axis_v"), JsonVector(Best.SupportAwareAxisV)->AsArray());
				SetVector2DArrayField(BestRoot, TEXT("virtual_base_plane_quad_2d"), Best.VirtualBasePlaneQuad2D);
				SetVectorArrayField(BestRoot, TEXT("virtual_base_plane_quad_world"), Best.VirtualBasePlaneQuadWorld);
				SetVector2DArrayField(BestRoot, TEXT("base_loop_2d"), Best.BaseLoop2D);
				SetVectorArrayField(BestRoot, TEXT("base_loop_world"), Best.BaseLoopWorld);
				SetVector2DArrayField(BestRoot, TEXT("copied_target_loop_2d"), Best.CopiedTargetLoop2D);
				SetVectorArrayField(BestRoot, TEXT("copied_loop_world"), Best.CopiedLoopWorld);
				BestRoot->SetObjectField(TEXT("raw_projection_topology"), MakeSupportTopologyDebugJson(Best.RawProjectionTopology));
				BestRoot->SetObjectField(TEXT("regularized_projection_topology"), MakeSupportTopologyDebugJson(Best.RegularizedProjectionTopology));
				BestRoot->SetObjectField(TEXT("final_projection_topology"), MakeSupportTopologyDebugJson(Best.FinalProjectionTopology));
				SetVector2DArrayField(BestRoot, TEXT("no_penetration_sample_pixels"), Best.NoPenetrationSamplePixels);
				SetVectorArrayField(BestRoot, TEXT("no_penetration_base_world"), Best.NoPenetrationBaseWorld);
				SetVectorArrayField(BestRoot, TEXT("no_penetration_support_world"), Best.NoPenetrationSupportWorld);
				SetDoubleArrayField(BestRoot, TEXT("no_penetration_violation_cm"), Best.NoPenetrationViolationCm);
				SaveJsonObject(
					BestRoot,
					FPaths::GetPath(JsonPath) / TEXT("10_attach_support_plane_fallback_best_failed.json"));
			}

			if (Debug.Attempts.IsValidIndex(BestForceableAttemptIndex))
			{
				const FAttachSupportPlaneFallbackAttempt& Best = Debug.Attempts[BestForceableAttemptIndex];
				TArray<uint8> BestRGBA = Inputs.FacesRGBA;
				DrawClosedPolylineRGBA(BestRGBA, Inputs.FacesWidth, Inputs.FacesHeight, RawCapLoopFaceSpace, FColor(255, 255, 255, 255), 2);
				if (const int32* SupportIndex = Inputs.FaceIndexById.Find(Best.SupportFaceId))
				{
					if (Inputs.Faces.IsValidIndex(*SupportIndex))
					{
						DrawClosedPolylineRGBA(BestRGBA, Inputs.FacesWidth, Inputs.FacesHeight, Inputs.Faces[*SupportIndex].KeyPoints2D, FColor(80, 80, 80, 255), 2);
					}
				}
				DrawClosedPolylineRGBA(BestRGBA, Inputs.FacesWidth, Inputs.FacesHeight, Best.VirtualBasePlaneQuad2D, FColor(0, 230, 255, 255), 4);
				DrawClosedPolylineRGBA(BestRGBA, Inputs.FacesWidth, Inputs.FacesHeight, Best.BaseLoop2D, FColor(0, 180, 255, 255), 3);
				DrawClosedPolylineRGBA(BestRGBA, Inputs.FacesWidth, Inputs.FacesHeight, Best.ReprojectedBaseLoop2D, FColor(40, 240, 80, 255), 2);
				DrawClosedPolylineRGBA(BestRGBA, Inputs.FacesWidth, Inputs.FacesHeight, Best.CopiedTargetLoop2D, FColor(255, 140, 0, 255), 2);
				DrawClosedPolylineRGBA(BestRGBA, Inputs.FacesWidth, Inputs.FacesHeight, Best.ReprojectedCopiedLoop2D, FColor(255, 0, 180, 255), 2);
				DrawTopologyIntersectionEdgesRGBA(BestRGBA, Inputs.FacesWidth, Inputs.FacesHeight, Best.BaseLoop2D, Best.RawProjectionTopology, FColor(255, 0, 0, 255), 5);
				DrawTopologyIntersectionEdgesRGBA(BestRGBA, Inputs.FacesWidth, Inputs.FacesHeight, Best.BaseLoop2D, Best.FinalProjectionTopology, FColor(255, 0, 0, 255), 5);
				DrawArrowRGBA(BestRGBA, Inputs.FacesWidth, Inputs.FacesHeight, Best.ChainStartFaceSpace, Best.ChainEndFaceSpace, FColor(255, 80, 40, 255), 4);
				for (int32 SampleIndex = 0; SampleIndex < Best.NoPenetrationSamplePixels.Num(); ++SampleIndex)
				{
					const double Violation = Best.NoPenetrationViolationCm.IsValidIndex(SampleIndex)
						? Best.NoPenetrationViolationCm[SampleIndex]
						: 0.0;
					const FColor SampleColor = Violation > 0.0
						? FColor(255, 40, 40, 255)
						: FColor(40, 240, 80, 255);
					DrawPointRGBA(
						BestRGBA,
						Inputs.FacesWidth,
						Inputs.FacesHeight,
						FMath::RoundToInt(Best.NoPenetrationSamplePixels[SampleIndex].X),
						FMath::RoundToInt(Best.NoPenetrationSamplePixels[SampleIndex].Y),
						SampleColor,
						5);
				}
				SaveRGBAToPng(
					BestRGBA,
					Inputs.FacesWidth,
					Inputs.FacesHeight,
					FPaths::GetPath(PngPath) / TEXT("10_attach_support_plane_fallback_best_forceable.png"));

				TSharedRef<FJsonObject> BestRoot = MakeShared<FJsonObject>();
				BestRoot->SetNumberField(TEXT("best_forceable_attempt_index"), BestForceableAttemptIndex);
				BestRoot->SetNumberField(TEXT("chain_index"), Best.ChainIndex);
				BestRoot->SetNumberField(TEXT("support_face_id"), Best.SupportFaceId);
				BestRoot->SetStringField(TEXT("forceable_reason"), Best.ForceableReason);
				BestRoot->SetStringField(TEXT("reject_reason"), Best.RejectReason);
				BestRoot->SetNumberField(TEXT("max_no_penetration_violation_cm"), Best.MaxNoPenetrationViolationCm);
				BestRoot->SetNumberField(TEXT("max_reprojection_error_pixels"), Best.MaxReprojectionErrorPixels);
				BestRoot->SetNumberField(TEXT("contact_distance_pixels"), Best.ContactDistancePixels);
				BestRoot->SetArrayField(TEXT("anchor_world"), JsonVector(Best.AnchorWorld)->AsArray());
				BestRoot->SetArrayField(TEXT("chain_end_world"), JsonVector(Best.ChainEndWorld)->AsArray());
				BestRoot->SetArrayField(TEXT("base_plane_normal"), JsonVector(Best.BasePlaneNormal)->AsArray());
				BestRoot->SetArrayField(TEXT("support_plane_normal"), JsonVector(Best.SupportPlaneNormal)->AsArray());
				BestRoot->SetArrayField(TEXT("support_aware_axis_u"), JsonVector(Best.SupportAwareAxisU)->AsArray());
				BestRoot->SetArrayField(TEXT("support_aware_axis_v"), JsonVector(Best.SupportAwareAxisV)->AsArray());
				SetVector2DArrayField(BestRoot, TEXT("virtual_base_plane_quad_2d"), Best.VirtualBasePlaneQuad2D);
				SetVectorArrayField(BestRoot, TEXT("virtual_base_plane_quad_world"), Best.VirtualBasePlaneQuadWorld);
				SetVector2DArrayField(BestRoot, TEXT("base_loop_2d"), Best.BaseLoop2D);
				SetVectorArrayField(BestRoot, TEXT("base_loop_world"), Best.BaseLoopWorld);
				SetVector2DArrayField(BestRoot, TEXT("copied_target_loop_2d"), Best.CopiedTargetLoop2D);
				SetVectorArrayField(BestRoot, TEXT("copied_loop_world"), Best.CopiedLoopWorld);
				BestRoot->SetObjectField(TEXT("raw_projection_topology"), MakeSupportTopologyDebugJson(Best.RawProjectionTopology));
				BestRoot->SetObjectField(TEXT("regularized_projection_topology"), MakeSupportTopologyDebugJson(Best.RegularizedProjectionTopology));
				BestRoot->SetObjectField(TEXT("final_projection_topology"), MakeSupportTopologyDebugJson(Best.FinalProjectionTopology));
				SetVector2DArrayField(BestRoot, TEXT("no_penetration_sample_pixels"), Best.NoPenetrationSamplePixels);
				SetVectorArrayField(BestRoot, TEXT("no_penetration_base_world"), Best.NoPenetrationBaseWorld);
				SetVectorArrayField(BestRoot, TEXT("no_penetration_support_world"), Best.NoPenetrationSupportWorld);
				SetDoubleArrayField(BestRoot, TEXT("no_penetration_violation_cm"), Best.NoPenetrationViolationCm);
				SaveJsonObject(
					BestRoot,
					FPaths::GetPath(JsonPath) / TEXT("10_attach_support_plane_fallback_best_forceable.json"));
			}

			// 10_virtual_face_normal_vs_green_chord: parallelism overlay for n_virtual vs g_proj.
			int32 NormalAttemptIndex = INDEX_NONE;
			for (int32 AttemptIdx = 0; AttemptIdx < Debug.Attempts.Num(); ++AttemptIdx)
			{
				if (Debug.Attempts[AttemptIdx].bSuccess)
				{
					NormalAttemptIndex = AttemptIdx;
					break;
				}
			}
			if (NormalAttemptIndex == INDEX_NONE)
			{
				NormalAttemptIndex = BestFailedAttemptIndex;
			}
			if (Debug.Attempts.IsValidIndex(NormalAttemptIndex) &&
				Debug.Attempts[NormalAttemptIndex].VirtualBasePlaneQuadWorld.Num() == 4)
			{
				const FAttachSupportPlaneFallbackAttempt& Sel = Debug.Attempts[NormalAttemptIndex];
				TArray<uint8> NRGBA = Inputs.FacesRGBA;
				DrawClosedPolylineRGBA(NRGBA, Inputs.FacesWidth, Inputs.FacesHeight, RawCapLoopFaceSpace, FColor(255, 255, 255, 255), 2);
				if (const int32* SupportIndex = Inputs.FaceIndexById.Find(Sel.SupportFaceId))
				{
					if (Inputs.Faces.IsValidIndex(*SupportIndex))
					{
						DrawClosedPolylineRGBA(NRGBA, Inputs.FacesWidth, Inputs.FacesHeight, Inputs.Faces[*SupportIndex].KeyPoints2D, FColor(128, 128, 128, 255), 2);
					}
				}
				DrawClosedPolylineRGBA(NRGBA, Inputs.FacesWidth, Inputs.FacesHeight, Sel.VirtualBasePlaneQuad2D, FColor(0, 230, 255, 255), 4);
				DrawClosedPolylineRGBA(NRGBA, Inputs.FacesWidth, Inputs.FacesHeight, Sel.BaseLoop2D, FColor(0, 180, 255, 255), 2);

				FVector QuadCentroidWorld = FVector::ZeroVector;
				for (const FVector& P : Sel.VirtualBasePlaneQuadWorld)
				{
					QuadCentroidWorld += P;
				}
				QuadCentroidWorld /= double(Sel.VirtualBasePlaneQuadWorld.Num());

				double MinUC = TNumericLimits<double>::Max();
				double MaxUC = TNumericLimits<double>::Lowest();
				double MinVC = TNumericLimits<double>::Max();
				double MaxVC = TNumericLimits<double>::Lowest();
				for (const FVector& P : Sel.VirtualBasePlaneQuadWorld)
				{
					const FVector R = P - QuadCentroidWorld;
					const double UCoord = FVector::DotProduct(R, Sel.SupportAwareAxisU);
					const double VCoord = FVector::DotProduct(R, Sel.SupportAwareAxisV);
					MinUC = FMath::Min(MinUC, UCoord);
					MaxUC = FMath::Max(MaxUC, UCoord);
					MinVC = FMath::Min(MinVC, VCoord);
					MaxVC = FMath::Max(MaxVC, VCoord);
				}
				const double USpan = MaxUC - MinUC;
				const double VSpan = MaxVC - MinVC;
				const double ChordLen = Sel.ExtrusionVectorWorld.Size();
				const double L = FMath::Max(ChordLen, USpan * 0.4);
				const double LShort = FMath::Max(ChordLen * 0.5, VSpan * 0.4);

				const FVector NV = Sel.BasePlaneNormal.GetSafeNormal();
				const FVector NS = Sel.SupportPlaneNormal.GetSafeNormal();

				FVector2D AnchorPx, ChainEndPx, CentroidPx, NormalEndPx, SupportEndPx;
				const bool bAnchorOk = ProjectWorldToImageOrthographic(Inputs.Camera, Inputs.FacesWidth, Inputs.FacesHeight, Sel.AnchorWorld, AnchorPx);
				const bool bChainOk = ProjectWorldToImageOrthographic(Inputs.Camera, Inputs.FacesWidth, Inputs.FacesHeight, Sel.ChainEndWorld, ChainEndPx);
				const bool bCenOk = ProjectWorldToImageOrthographic(Inputs.Camera, Inputs.FacesWidth, Inputs.FacesHeight, QuadCentroidWorld, CentroidPx);
				const bool bNvOk = ProjectWorldToImageOrthographic(Inputs.Camera, Inputs.FacesWidth, Inputs.FacesHeight, QuadCentroidWorld + NV * L, NormalEndPx);
				const bool bNsOk = ProjectWorldToImageOrthographic(Inputs.Camera, Inputs.FacesWidth, Inputs.FacesHeight, QuadCentroidWorld + NS * LShort, SupportEndPx);

				if (bAnchorOk && bChainOk)
				{
					DrawArrowRGBA(NRGBA, Inputs.FacesWidth, Inputs.FacesHeight, AnchorPx, ChainEndPx, FColor(255, 225, 40, 255), 4);
				}
				if (bCenOk && bNvOk)
				{
					DrawArrowRGBA(NRGBA, Inputs.FacesWidth, Inputs.FacesHeight, CentroidPx, NormalEndPx, FColor(255, 40, 220, 255), 4);
				}
				if (bCenOk && bNsOk)
				{
					DrawArrowRGBA(NRGBA, Inputs.FacesWidth, Inputs.FacesHeight, CentroidPx, SupportEndPx, FColor(255, 255, 255, 255), 4);
				}
				if (bAnchorOk)
				{
					DrawPointRGBA(NRGBA, Inputs.FacesWidth, Inputs.FacesHeight, FMath::RoundToInt(AnchorPx.X), FMath::RoundToInt(AnchorPx.Y), FColor(255, 0, 0, 255), 6);
				}
				SaveRGBAToPng(
					NRGBA,
					Inputs.FacesWidth,
					Inputs.FacesHeight,
					FPaths::GetPath(PngPath) / TEXT("10_virtual_face_normal_vs_green_chord.png"));

				const FVector GP = Sel.ExtrusionVectorWorld.GetSafeNormal();
				const FVector AU = Sel.SupportAwareAxisU.GetSafeNormal();
				TSharedRef<FJsonObject> NRoot = MakeShared<FJsonObject>();
				NRoot->SetNumberField(TEXT("attempt_index"), NormalAttemptIndex);
				NRoot->SetBoolField(TEXT("attempt_success"), Sel.bSuccess);
				NRoot->SetNumberField(TEXT("chain_index"), Sel.ChainIndex);
				NRoot->SetNumberField(TEXT("support_face_id"), Sel.SupportFaceId);
				NRoot->SetArrayField(TEXT("g_proj_unit"), JsonVector(GP)->AsArray());
				NRoot->SetArrayField(TEXT("n_virtual"), JsonVector(NV)->AsArray());
				NRoot->SetArrayField(TEXT("n_support"), JsonVector(NS)->AsArray());
				NRoot->SetArrayField(TEXT("axis_u"), JsonVector(AU)->AsArray());
				NRoot->SetNumberField(TEXT("dot_n_virtual_g_proj"), FVector::DotProduct(NV, GP));
				NRoot->SetNumberField(TEXT("cross_n_virtual_g_proj_magnitude"), FVector::CrossProduct(NV, GP).Size());
				NRoot->SetNumberField(TEXT("dot_n_virtual_n_support"), FVector::DotProduct(NV, NS));
				NRoot->SetNumberField(TEXT("dot_axis_u_n_virtual"), FVector::DotProduct(AU, NV));
				NRoot->SetNumberField(TEXT("dot_axis_u_n_support"), FVector::DotProduct(AU, NS));
				NRoot->SetNumberField(TEXT("chord_length_cm"), ChordLen);
				NRoot->SetNumberField(TEXT("normal_arrow_length_cm"), L);
				NRoot->SetNumberField(TEXT("support_arrow_length_cm"), LShort);
				NRoot->SetArrayField(TEXT("anchor_world"), JsonVector(Sel.AnchorWorld)->AsArray());
				NRoot->SetArrayField(TEXT("chain_end_world"), JsonVector(Sel.ChainEndWorld)->AsArray());
				NRoot->SetArrayField(TEXT("quad_centroid_world"), JsonVector(QuadCentroidWorld)->AsArray());
				SaveJsonObject(
					NRoot,
					FPaths::GetPath(JsonPath) / TEXT("10_virtual_face_normal_vs_green_chord.json"));
			}
		}
	}

	static FComponentResult MakeFailureResult(const FString& ComponentName, const FString& Error, const FString& OutputDir)
	{
		FComponentResult Result;
		Result.ComponentName = ComponentName;
		Result.Error = Error;
		SaveComponentResultJson(Result, OutputDir / TEXT("10_face_reconstruction.json"));
		return Result;
	}

	static bool LoadAction(const FString& Path, FString& OutAction)
	{
		TSharedPtr<FJsonObject> Root;
		if (!LoadJsonObject(Path, Root))
		{
			return false;
		}
		return Root->TryGetStringField(TEXT("action"), OutAction) && !OutAction.IsEmpty();
	}

	static void InitializeSolidResult(
		FSolidReconstructionResult& Solid, const FString& ComponentName, const FString& PressDir,
		const FCommonInputs& Inputs)
	{
		Solid.ComponentName = ComponentName;
		Solid.FacesWidth = Inputs.FacesWidth;
		Solid.FacesHeight = Inputs.FacesHeight;
		Solid.ActorName = FString::Printf(TEXT("FromLZ_ReconstructedSolid_%s_%s"), *FPaths::GetCleanFilename(PressDir), *ComponentName);
		Solid.bHasProjectionMatrix = Inputs.Camera.bHasProjectionMatrix;
		Solid.ProjectionMatrixSource = Inputs.Camera.ProjectionMatrixSource;
		Solid.ProjectionValidation = Inputs.ProjectionValidation;
		Solid.ProjectionViewportWidth = Inputs.Camera.ViewportWidth;
		Solid.ProjectionViewportHeight = Inputs.Camera.ViewportHeight;
		Solid.ProjectionM00 = Inputs.Camera.ProjectionMatrix.M[0][0];
		Solid.ProjectionM11 = Inputs.Camera.ProjectionMatrix.M[1][1];
		Solid.ProjectionM30 = Inputs.Camera.ProjectionMatrix.M[3][0];
		Solid.ProjectionM31 = Inputs.Camera.ProjectionMatrix.M[3][1];
		if (Inputs.Camera.bHasProjectionMatrix)
		{
			Solid.ProjectionHorizontalSpan = 2.0 / FMath::Abs(Solid.ProjectionM00);
			Solid.ProjectionVerticalSpan = 2.0 / FMath::Abs(Solid.ProjectionM11);
		}
	}

	static void SaveSkippedSolidResult(
		FSolidReconstructionResult& Solid, const FString& Path, const FString& Error)
	{
		Solid.bSuccess = false;
		Solid.Error = Error;
		SaveSolidResultJson(Solid, Path);
	}

	static bool BuildSolidMeshTriangles(
		TArray<FVector2D>& SourceLoop2D,
		TArray<FVector2D>& CopiedTargetLoop2D,
		TArray<FVector>& SourceLoopWorld,
		TArray<FVector>& CopiedLoopWorld,
		const FVector& OrientedNormal,
		TArray<FVector>& OutVertices,
		TArray<int32>& OutTriangles,
		FVector& OutMeshNormal)
	{
		OutVertices.Reset();
		OutTriangles.Reset();
		OutMeshNormal = OrientedNormal.GetSafeNormal();

		TArray<int32> SourceTriangles;
		if (!TriangulatePolygon2D(SourceLoop2D, SourceTriangles))
		{
			return false;
		}

		FVector SourceTriNormal = ComputeTriangleNormal(SourceLoopWorld, SourceTriangles);
		if (!SourceTriNormal.IsNearlyZero() && FVector::DotProduct(SourceTriNormal, OrientedNormal) > 0.0)
		{
			Algo::Reverse(SourceLoop2D);
			Algo::Reverse(CopiedTargetLoop2D);
			Algo::Reverse(SourceLoopWorld);
			Algo::Reverse(CopiedLoopWorld);
			SourceTriangles.Reset();
			if (!TriangulatePolygon2D(SourceLoop2D, SourceTriangles))
			{
				return false;
			}
		}

		const int32 N = SourceLoopWorld.Num();
		if (N < 3 || CopiedLoopWorld.Num() != N)
		{
			return false;
		}

		OutVertices.Reserve(N * 2);
		for (const FVector& V : SourceLoopWorld)
		{
			OutVertices.Add(V);
		}
		for (const FVector& V : CopiedLoopWorld)
		{
			OutVertices.Add(V);
		}

		for (int32 i = 0; i + 2 < SourceTriangles.Num(); i += 3)
		{
			OutTriangles.Add(SourceTriangles[i]);
			OutTriangles.Add(SourceTriangles[i + 1]);
			OutTriangles.Add(SourceTriangles[i + 2]);
		}

		for (int32 i = 0; i + 2 < SourceTriangles.Num(); i += 3)
		{
			OutTriangles.Add(N + SourceTriangles[i]);
			OutTriangles.Add(N + SourceTriangles[i + 2]);
			OutTriangles.Add(N + SourceTriangles[i + 1]);
		}

		for (int32 i = 0; i < N; ++i)
		{
			const int32 J = (i + 1) % N;
			OutTriangles.Add(i);
			OutTriangles.Add(N + i);
			OutTriangles.Add(N + J);

			OutTriangles.Add(i);
			OutTriangles.Add(N + J);
			OutTriangles.Add(J);
		}

		return OutVertices.Num() >= 6 && OutTriangles.Num() >= 12;
	}

	static FSolidReconstructionResult BuildSolidReconstruction(
		const FString& ComponentName,
		const FString& Action,
		const FString& SourcePolygonKey,
		const FString& CopiedPolygonKey,
		const FString& PressDir,
		const FString& OutputDir,
		int32 CapWidth,
		int32 CapHeight,
		double ScaleX,
		double ScaleY,
		const TArray<FVector2D>& SourcePolygonCapSpace,
		const TArray<FVector2D>& CopiedPolygonCapSpace,
		const TArray<FCapBoundaryRunInput>& BoundaryRuns,
		const FVector2D& SourceTranslationCapSpace,
		const FFaceInfo& SelectedFace,
		const FCommonInputs& Inputs,
		const FFromLZFaceReconstructionParams& Params)
	{
		FSolidReconstructionResult Result;
		InitializeSolidResult(Result, ComponentName, PressDir, Inputs);
		Result.Action = Action;
		Result.SourcePolygonKey = SourcePolygonKey;
		Result.CopiedPolygonKey = CopiedPolygonKey;
		Result.CapWidth = CapWidth;
		Result.CapHeight = CapHeight;
		Result.SelectedFaceId = SelectedFace.Id;
		Result.SourcePlanePoint = SelectedFace.PlanePoint;
		Result.SourcePlaneNormal = SelectedFace.Normal.GetSafeNormal();
		Result.SourceFaceVerticesWorld = SelectedFace.KeyPoints3D;
		Result.OrientedNormal = Result.SourcePlaneNormal;
		Result.ReconstructionMethod = TEXT("direct_cap_face");
		Result.bAttachSupportPlaneFallback = false;

		if (SourcePolygonCapSpace.Num() != CopiedPolygonCapSpace.Num())
		{
			Result.Error = FString::Printf(
				TEXT("source/copy polygon point counts differ: %d vs %d"),
				SourcePolygonCapSpace.Num(), CopiedPolygonCapSpace.Num());
			return Result;
		}
		if (SourcePolygonCapSpace.Num() < 3)
		{
			Result.Error = TEXT("source/copy polygons need at least three points");
			return Result;
		}

		MapPolygonToFacesSpace(SourcePolygonCapSpace, ScaleX, ScaleY, Result.SourceLoop2D);
		MapPolygonToFacesSpace(CopiedPolygonCapSpace, ScaleX, ScaleY, Result.CopiedTargetLoop2D);
		SimplifyLoopPairs(Result.SourceLoop2D, Result.CopiedTargetLoop2D);
		if (Result.SourceLoop2D.Num() != Result.CopiedTargetLoop2D.Num() || Result.SourceLoop2D.Num() < 3)
		{
			Result.Error = TEXT("source/copy polygons became invalid after duplicate/collinear cleanup");
			return Result;
		}

		Result.SourceToCopiedVector2D = AverageVector2DDelta(Result.SourceLoop2D, Result.CopiedTargetLoop2D);
		if (Result.SourceToCopiedVector2D.SizeSquared() < 1e-8)
		{
			Result.Error = TEXT("source-to-copied 2D offset is too short");
			return Result;
		}

		if (!Inputs.Camera.bHasProjectionMatrix ||
			!IsUsableOrthographicProjectionMatrix(Inputs.Camera.ProjectionMatrix))
		{
			Result.Error = TEXT("Step 10 has no usable capture projection matrix");
			return Result;
		}

		Result.SourceLoopWorld.Reserve(Result.SourceLoop2D.Num());
		for (const FVector2D& P : Result.SourceLoop2D)
		{
			FVector Hit;
			if (!IntersectPixelWithPlaneOrthographic(
				Inputs.Camera, Inputs.FacesWidth, Inputs.FacesHeight,
				P, SelectedFace.PlanePoint, SelectedFace.Normal, Hit))
			{
				Result.Error = TEXT("failed to project a source cap point onto the selected source face plane");
				return Result;
			}
			Result.SourceLoopWorld.Add(Hit);
		}

		const TArray<FVector2D> OriginalSourceLoop2D = Result.SourceLoop2D;
		const TArray<FVector2D> OriginalCopiedLoop2D = Result.CopiedTargetLoop2D;
		const TArray<FVector> OriginalSourceLoopWorld = Result.SourceLoopWorld;
		TArray<FVector> RegularizedSourceLoopWorld;
		const bool bRegularizationApplied = RegularizeCapToWorldOrthogonalPolygon(
			SelectedFace,
			Inputs,
			Params,
			Result.SourceToCopiedVector2D,
			OriginalSourceLoopWorld,
			BoundaryRuns,
			SourceTranslationCapSpace,
			ScaleX,
			ScaleY,
			RegularizedSourceLoopWorld,
			Result.CapBBoxRegularization);
		Result.CapBBoxRegularization.SelectedFaceId = SelectedFace.Id;
		Result.CapBBoxRegularization.Action = Action;
		Result.CapBBoxRegularization.SourcePolygonKey = SourcePolygonKey;
		Result.CapBBoxRegularization.CopiedPolygonKey = CopiedPolygonKey;
		if (bRegularizationApplied)
		{
			BeginWorldOrthogonalStage(
				Result.CapBBoxRegularization,
				TEXT("orthographic_reprojection"));
			Result.SourceLoopWorld = MoveTemp(RegularizedSourceLoopWorld);
			Result.SourceLoop2D.Reset();
			Result.CopiedTargetLoop2D.Reset();
			bool bProjectionValid = true;
			for (const FVector& WorldPoint : Result.SourceLoopWorld)
			{
				FVector2D SourcePixel;
				if (!ProjectWorldToImageOrthographic(
					Inputs.Camera, Inputs.FacesWidth, Inputs.FacesHeight,
					WorldPoint, SourcePixel) ||
					!IsFiniteVector2D(SourcePixel))
				{
					bProjectionValid = false;
					break;
				}
				Result.SourceLoop2D.Add(SourcePixel);
				Result.CopiedTargetLoop2D.Add(SourcePixel + Result.SourceToCopiedVector2D);
			}

			if (!bProjectionValid ||
				Result.SourceLoop2D.Num() < 3 ||
				Result.CopiedTargetLoop2D.Num() != Result.SourceLoop2D.Num())
			{
				Result.SourceLoop2D = OriginalSourceLoop2D;
				Result.CopiedTargetLoop2D = OriginalCopiedLoop2D;
				Result.SourceLoopWorld = OriginalSourceLoopWorld;
				SetRegularizationFallback(
					Result.CapBBoxRegularization,
					TEXT("regularized orthogonal polygon failed orthographic reprojection; restored original cap"));
			}
			else
			{
				PassWorldOrthogonalStage(
					Result.CapBBoxRegularization,
					TEXT("orthographic_reprojection"),
					FString::Printf(
						TEXT("%d corrected source vertices reprojected"),
						Result.SourceLoop2D.Num()));
			}
		}
		SaveCapBBoxRegularizationDebug(Result.CapBBoxRegularization, OutputDir);

		Result.SourceMaterialProbePointsWorld = Result.SourceLoopWorld;
		Result.SourceMaterialProbePointsWorld.Add(AverageVector(Result.SourceLoopWorld));

		if (!ProjectSignedWorldDirectionToImage(
			Inputs.Camera, Inputs.FacesWidth, Inputs.FacesHeight,
			Result.OrientedNormal, Result.ProjectedNormal2D))
		{
			Result.Error = TEXT("source face normal projects to a near-zero image direction; cannot orient extrusion");
			return Result;
		}

		if (FVector2D::DotProduct(Result.ProjectedNormal2D, Result.SourceToCopiedVector2D.GetSafeNormal()) < 0.0)
		{
			Result.OrientedNormal *= -1.0;
			Result.ProjectedNormal2D *= -1.0;
		}

		Result.MaxDepthSampleReprojectionErrorPixels = FMath::Max(25.0, Result.SourceToCopiedVector2D.Size() * 0.75);
		if (!SolveExtrusionDepthOrthographic(
			Inputs.Camera, Inputs.FacesWidth, Inputs.FacesHeight,
			Result.OrientedNormal, Result.SourceToCopiedVector2D,
			Result.ExtrusionDepth, Result.Error))
		{
			return Result;
		}

		int32 ValidVertexCount = 0;
		Result.DepthSamples.Reserve(Result.SourceLoopWorld.Num());
		for (int32 i = 0; i < Result.SourceLoopWorld.Num(); ++i)
		{
			FSolidDepthSample Sample;
			Sample.Index = i;
			Sample.SourcePixel = Result.SourceLoop2D[i];
			Sample.CopiedPixel = Result.CopiedTargetLoop2D[i];
			Sample.SourceWorld = Result.SourceLoopWorld[i];
			Sample.Depth = Result.ExtrusionDepth;
			Sample.PointOnExtrusion = Result.SourceLoopWorld[i] + Result.OrientedNormal * Result.ExtrusionDepth;
			Sample.PointOnRay = Sample.PointOnExtrusion;
			Sample.ClosestWorldDistance = 0.0;

			FVector2D Reprojected;
			if (!ProjectWorldToImageOrthographic(
				Inputs.Camera, Inputs.FacesWidth, Inputs.FacesHeight,
				Sample.PointOnExtrusion, Reprojected))
			{
				Sample.Error = TEXT("extruded vertex failed to reproject");
				Result.DepthSamples.Add(Sample);
				continue;
			}

			Sample.ReprojectionErrorPixels = FVector2D::Distance(Reprojected, Sample.CopiedPixel);
			if (Sample.ReprojectionErrorPixels > Result.MaxDepthSampleReprojectionErrorPixels)
			{
				Sample.Error = TEXT("vertex reprojection error is too large");
			}
			else
			{
				Sample.bValid = true;
				++ValidVertexCount;
			}
			Result.DepthSamples.Add(Sample);
		}

		if (ValidVertexCount < MinSolidDepthSamples)
		{
			Result.Error = FString::Printf(
				TEXT("not enough vertices match copied cap after orthographic extrusion solve: %d valid, need %d"),
				ValidVertexCount, MinSolidDepthSamples);
			return Result;
		}
		Result.CopiedLoopWorld.Reserve(Result.SourceLoopWorld.Num());
		for (const FVector& P : Result.SourceLoopWorld)
		{

			Result.CopiedLoopWorld.Add(P + Result.OrientedNormal * Result.ExtrusionDepth);
		}
		Result.ExtrusionVectorWorld = Result.OrientedNormal * Result.ExtrusionDepth;

		Result.ReprojectedSourceLoop2D.Reserve(Result.SourceLoopWorld.Num());
		Result.ReprojectedCopiedLoop2D.Reserve(Result.CopiedLoopWorld.Num());
		double SourceErrorSum = 0.0;
		double CopiedErrorSum = 0.0;
		for (int32 i = 0; i < Result.SourceLoopWorld.Num(); ++i)
		{
			FVector2D SourceProj;
			FVector2D CopiedProj;
			if (!ProjectWorldToImageOrthographic(Inputs.Camera, Inputs.FacesWidth, Inputs.FacesHeight, Result.SourceLoopWorld[i], SourceProj) ||
				!ProjectWorldToImageOrthographic(Inputs.Camera, Inputs.FacesWidth, Inputs.FacesHeight, Result.CopiedLoopWorld[i], CopiedProj))
			{
				Result.Error = TEXT("failed to reproject generated source/copied solid loop");
				return Result;
			}
			Result.ReprojectedSourceLoop2D.Add(SourceProj);
			Result.ReprojectedCopiedLoop2D.Add(CopiedProj);

			const double SourceError = FVector2D::Distance(SourceProj, Result.SourceLoop2D[i]);
			SourceErrorSum += SourceError;
			Result.MaxSourceReprojectionErrorPixels = FMath::Max(Result.MaxSourceReprojectionErrorPixels, SourceError);
			const double CopiedError = FVector2D::Distance(CopiedProj, Result.CopiedTargetLoop2D[i]);
			CopiedErrorSum += CopiedError;
			Result.MaxCopiedReprojectionErrorPixels = FMath::Max(Result.MaxCopiedReprojectionErrorPixels, CopiedError);
		}
		Result.MeanSourceReprojectionErrorPixels = SourceErrorSum / double(FMath::Max(1, Result.ReprojectedSourceLoop2D.Num()));
		Result.MeanCopiedReprojectionErrorPixels = CopiedErrorSum / double(FMath::Max(1, Result.ReprojectedCopiedLoop2D.Num()));
		if (Result.MaxCopiedReprojectionErrorPixels > Result.MaxDepthSampleReprojectionErrorPixels)
		{
			Result.Warning = FString::Printf(
				TEXT("copied loop reprojection max error %.3f px exceeds depth-sample threshold %.3f px"),
				Result.MaxCopiedReprojectionErrorPixels, Result.MaxDepthSampleReprojectionErrorPixels);
		}

		if (!BuildSolidMeshTriangles(
			Result.SourceLoop2D, Result.CopiedTargetLoop2D,
			Result.SourceLoopWorld, Result.CopiedLoopWorld,
			Result.OrientedNormal, Result.MeshVerticesWorld, Result.MeshTriangles, Result.MeshNormal))
		{
			Result.Error = TEXT("failed to triangulate source cap or build solid side faces");
			return Result;
		}

		Result.ReprojectedSourceLoop2D.Reset();
		Result.ReprojectedCopiedLoop2D.Reset();
		Result.ReprojectedSourceLoop2D.Reserve(Result.SourceLoopWorld.Num());
		Result.ReprojectedCopiedLoop2D.Reserve(Result.CopiedLoopWorld.Num());
		Result.MeanSourceReprojectionErrorPixels = 0.0;
		Result.MaxSourceReprojectionErrorPixels = 0.0;
		Result.MeanCopiedReprojectionErrorPixels = 0.0;
		Result.MaxCopiedReprojectionErrorPixels = 0.0;
		double FinalSourceErrorSum = 0.0;
		double FinalCopiedErrorSum = 0.0;
		for (int32 i = 0; i < Result.SourceLoopWorld.Num(); ++i)
		{
			FVector2D SourceProj;
			FVector2D CopiedProj;
			if (!ProjectWorldToImageOrthographic(Inputs.Camera, Inputs.FacesWidth, Inputs.FacesHeight, Result.SourceLoopWorld[i], SourceProj) ||
				!ProjectWorldToImageOrthographic(Inputs.Camera, Inputs.FacesWidth, Inputs.FacesHeight, Result.CopiedLoopWorld[i], CopiedProj))
			{
				Result.Error = TEXT("failed to reproject final solid loop");
				Result.bSuccess = false;
				return Result;
			}
			Result.ReprojectedSourceLoop2D.Add(SourceProj);
			Result.ReprojectedCopiedLoop2D.Add(CopiedProj);
			const double SourceError = FVector2D::Distance(SourceProj, Result.SourceLoop2D[i]);
			FinalSourceErrorSum += SourceError;
			Result.MaxSourceReprojectionErrorPixels = FMath::Max(Result.MaxSourceReprojectionErrorPixels, SourceError);
			const double CopiedError = FVector2D::Distance(CopiedProj, Result.CopiedTargetLoop2D[i]);
			FinalCopiedErrorSum += CopiedError;
			Result.MaxCopiedReprojectionErrorPixels = FMath::Max(Result.MaxCopiedReprojectionErrorPixels, CopiedError);
		}
		Result.MeanSourceReprojectionErrorPixels = FinalSourceErrorSum / double(FMath::Max(1, Result.ReprojectedSourceLoop2D.Num()));
		Result.MeanCopiedReprojectionErrorPixels = FinalCopiedErrorSum / double(FMath::Max(1, Result.ReprojectedCopiedLoop2D.Num()));
		Result.Warning.Reset();
		if (Result.MaxCopiedReprojectionErrorPixels > Result.MaxDepthSampleReprojectionErrorPixels)
		{
			Result.Warning = FString::Printf(
				TEXT("copied loop reprojection max error %.3f px exceeds depth-sample threshold %.3f px"),
				Result.MaxCopiedReprojectionErrorPixels, Result.MaxDepthSampleReprojectionErrorPixels);
		}

		Result.bSuccess = true;
		return Result;
	}

	static bool BuildSupportAwareFaceBasis(
		const FVector& BaseNormalIn,
		const FVector& SupportNormalIn,
		FVector& OutAxisU,
		FVector& OutAxisV)
	{
		const FVector BaseNormal = BaseNormalIn.GetSafeNormal();
		const FVector SupportNormal = SupportNormalIn.GetSafeNormal();
		if (BaseNormal.IsNearlyZero() || SupportNormal.IsNearlyZero())
		{
			return false;
		}

		FVector AxisV = SupportNormal - BaseNormal * FVector::DotProduct(SupportNormal, BaseNormal);
		if (AxisV.SizeSquared() < 1e-8)
		{
			AxisV = FVector::UpVector - BaseNormal * FVector::DotProduct(FVector::UpVector, BaseNormal);
		}
		if (AxisV.SizeSquared() < 1e-8)
		{
			AxisV = FVector::RightVector - BaseNormal * FVector::DotProduct(FVector::RightVector, BaseNormal);
		}
		AxisV = AxisV.GetSafeNormal();
		if (AxisV.IsNearlyZero())
		{
			return false;
		}

		FVector AxisU = FVector::CrossProduct(AxisV, BaseNormal).GetSafeNormal();
		if (AxisU.IsNearlyZero())
		{
			return false;
		}
		if (FVector::DotProduct(FVector::CrossProduct(AxisU, AxisV), BaseNormal) < 0.0)
		{
			AxisU *= -1.0;
		}

		OutAxisU = AxisU;
		OutAxisV = AxisV;
		return true;
	}

	static double DistancePointToClosedLoop2D(const FVector2D& P, const TArray<FVector2D>& Loop)
	{
		return FMath::Sqrt(DistancePointToPolylineSquared2D(P, Loop, true));
	}

	static bool CheckAttachSupportNoPenetration(
		const FCommonInputs& Inputs,
		const FFaceInfo& SupportFace,
		const TArray<FVector2D>& SourceLoop2D,
		const TArray<FVector>& SourceLoopWorld,
		double TolCm,
		double& OutMaxViolationCm,
		FString& OutError,
		TArray<FVector2D>* OutSamplePixels = nullptr,
		TArray<FVector>* OutBaseWorld = nullptr,
		TArray<FVector>* OutSupportWorld = nullptr,
		TArray<double>* OutViolationCm = nullptr,
		bool bStopOnFirstFailure = true)
	{
		OutMaxViolationCm = 0.0;
		OutError.Reset();
		if (OutSamplePixels) { OutSamplePixels->Reset(); }
		if (OutBaseWorld) { OutBaseWorld->Reset(); }
		if (OutSupportWorld) { OutSupportWorld->Reset(); }
		if (OutViolationCm) { OutViolationCm->Reset(); }
		if (SourceLoop2D.Num() < 3 || SourceLoop2D.Num() != SourceLoopWorld.Num())
		{
			OutError = TEXT("source loop is invalid for no-penetration test");
			return false;
		}

		auto TestSample = [&](const FVector2D& Pixel, const FVector& World, int32 SampleIndex) -> bool
		{
			if (OutSamplePixels) { OutSamplePixels->Add(Pixel); }
			if (OutBaseWorld) { OutBaseWorld->Add(World); }
			FVector SupportHit;
			if (!IntersectPixelWithPlaneOrthographic(
				Inputs.Camera,
				Inputs.FacesWidth,
				Inputs.FacesHeight,
				Pixel,
				SupportFace.PlanePoint,
				SupportFace.Normal,
				SupportHit))
			{
				OutError = FString::Printf(TEXT("sample %d failed support-plane raycast"), SampleIndex);
				if (OutSupportWorld) { OutSupportWorld->Add(FVector::ZeroVector); }
				if (OutViolationCm) { OutViolationCm->Add(TNumericLimits<double>::Max()); }
				return false;
			}
			if (OutSupportWorld) { OutSupportWorld->Add(SupportHit); }
			const double Violation = FVector::DotProduct(World - SupportHit, Inputs.Camera.Forward.GetSafeNormal());
			OutMaxViolationCm = FMath::Max(OutMaxViolationCm, Violation);
			if (OutViolationCm) { OutViolationCm->Add(Violation); }
			if (Violation > TolCm)
			{
				if (OutError.IsEmpty())
				{
					OutError = FString::Printf(
						TEXT("sample %d is %.6f cm behind support point (tol %.6f)"),
						SampleIndex,
						Violation,
						TolCm);
				}
				return false;
			}
			return true;
		};

		bool bAllPass = true;
		int32 SampleIndex = 0;
		for (int32 i = 0; i < SourceLoop2D.Num(); ++i)
		{
			if (!TestSample(SourceLoop2D[i], SourceLoopWorld[i], SampleIndex++))
			{
				bAllPass = false;
				if (bStopOnFirstFailure)
				{
					return false;
				}
			}
			const int32 Next = (i + 1) % SourceLoop2D.Num();
			if (!TestSample(
				(SourceLoop2D[i] + SourceLoop2D[Next]) * 0.5,
				(SourceLoopWorld[i] + SourceLoopWorld[Next]) * 0.5,
				SampleIndex++))
			{
				bAllPass = false;
				if (bStopOnFirstFailure)
				{
					return false;
				}
			}
		}
		if (!bAllPass && OutError.IsEmpty())
		{
			OutError = FString::Printf(
				TEXT("max no-penetration violation %.6f cm exceeds tolerance %.6f"),
				OutMaxViolationCm,
				TolCm);
		}
		return bAllPass;
	}

	static bool ReprojectSolidLoops(
		const FCommonInputs& Inputs,
		FSolidReconstructionResult& Result,
		FString& OutError)
	{
		OutError.Reset();
		Result.DepthSamples.Reset();
		Result.ReprojectedSourceLoop2D.Reset();
		Result.ReprojectedCopiedLoop2D.Reset();
		Result.MeanSourceReprojectionErrorPixels = 0.0;
		Result.MaxSourceReprojectionErrorPixels = 0.0;
		Result.MeanCopiedReprojectionErrorPixels = 0.0;
		Result.MaxCopiedReprojectionErrorPixels = 0.0;

		if (Result.SourceLoop2D.Num() < 3 ||
			Result.CopiedTargetLoop2D.Num() != Result.SourceLoop2D.Num() ||
			Result.SourceLoopWorld.Num() != Result.SourceLoop2D.Num() ||
			Result.CopiedLoopWorld.Num() != Result.SourceLoopWorld.Num())
		{
			OutError = TEXT("solid loops have inconsistent counts before reprojection");
			return false;
		}

		int32 ValidVertexCount = 0;
		double SourceErrorSum = 0.0;
		double CopiedErrorSum = 0.0;
		Result.DepthSamples.Reserve(Result.SourceLoopWorld.Num());
		Result.ReprojectedSourceLoop2D.Reserve(Result.SourceLoopWorld.Num());
		Result.ReprojectedCopiedLoop2D.Reserve(Result.CopiedLoopWorld.Num());
		for (int32 i = 0; i < Result.SourceLoopWorld.Num(); ++i)
		{
			FVector2D SourceProj;
			FVector2D CopiedProj;
			if (!ProjectWorldToImageOrthographic(
				Inputs.Camera,
				Inputs.FacesWidth,
				Inputs.FacesHeight,
				Result.SourceLoopWorld[i],
				SourceProj) ||
				!ProjectWorldToImageOrthographic(
					Inputs.Camera,
					Inputs.FacesWidth,
					Inputs.FacesHeight,
					Result.CopiedLoopWorld[i],
					CopiedProj))
			{
				OutError = TEXT("failed to reproject fallback solid loop");
				return false;
			}

			Result.ReprojectedSourceLoop2D.Add(SourceProj);
			Result.ReprojectedCopiedLoop2D.Add(CopiedProj);
			const double SourceError = FVector2D::Distance(SourceProj, Result.SourceLoop2D[i]);
			const double CopiedError = FVector2D::Distance(CopiedProj, Result.CopiedTargetLoop2D[i]);
			SourceErrorSum += SourceError;
			CopiedErrorSum += CopiedError;
			Result.MaxSourceReprojectionErrorPixels = FMath::Max(Result.MaxSourceReprojectionErrorPixels, SourceError);
			Result.MaxCopiedReprojectionErrorPixels = FMath::Max(Result.MaxCopiedReprojectionErrorPixels, CopiedError);

			FSolidDepthSample Sample;
			Sample.Index = i;
			Sample.bValid = CopiedError <= Result.MaxDepthSampleReprojectionErrorPixels;
			Sample.Error = Sample.bValid ? FString() : TEXT("fallback copied vertex reprojection error is too large");
			Sample.SourcePixel = Result.SourceLoop2D[i];
			Sample.CopiedPixel = Result.CopiedTargetLoop2D[i];
			Sample.SourceWorld = Result.SourceLoopWorld[i];
			Sample.PointOnExtrusion = Result.CopiedLoopWorld[i];
			Sample.PointOnRay = Result.CopiedLoopWorld[i];
			Sample.Depth = Result.ExtrusionDepth;
			Sample.ClosestWorldDistance = 0.0;
			Sample.ReprojectionErrorPixels = CopiedError;
			if (Sample.bValid)
			{
				++ValidVertexCount;
			}
			Result.DepthSamples.Add(Sample);
		}

		Result.MeanSourceReprojectionErrorPixels = SourceErrorSum / double(FMath::Max(1, Result.ReprojectedSourceLoop2D.Num()));
		Result.MeanCopiedReprojectionErrorPixels = CopiedErrorSum / double(FMath::Max(1, Result.ReprojectedCopiedLoop2D.Num()));
		if (ValidVertexCount < MinSolidDepthSamples)
		{
			OutError = FString::Printf(
				TEXT("fallback reprojection has only %d valid vertices, need %d"),
				ValidVertexCount,
				MinSolidDepthSamples);
			return false;
		}
		if (Result.MaxCopiedReprojectionErrorPixels > Result.MaxDepthSampleReprojectionErrorPixels)
		{
			OutError = FString::Printf(
				TEXT("fallback max copied reprojection error %.3f exceeds threshold %.3f"),
				Result.MaxCopiedReprojectionErrorPixels,
				Result.MaxDepthSampleReprojectionErrorPixels);
			return false;
		}
		return true;
	}

	static void BuildVirtualBasePlaneDebugQuad(
		const FCommonInputs& Inputs,
		const FVector& AnchorWorld,
		const FVector& AxisUWorld,
		const FVector& AxisVWorld,
		const TArray<FVector>& BaseLoopWorld,
		double FallbackExtent,
		TArray<FVector>& OutQuadWorld,
		TArray<FVector2D>& OutQuad2D)
	{
		OutQuadWorld.Reset();
		OutQuad2D.Reset();
		const FVector AxisU = AxisUWorld.GetSafeNormal();
		const FVector AxisV = AxisVWorld.GetSafeNormal();
		if (AxisU.IsNearlyZero() || AxisV.IsNearlyZero())
		{
			return;
		}

		double MinU = TNumericLimits<double>::Max();
		double MaxU = -TNumericLimits<double>::Max();
		double MinV = TNumericLimits<double>::Max();
		double MaxV = -TNumericLimits<double>::Max();
		for (const FVector& P : BaseLoopWorld)
		{
			const FVector Rel = P - AnchorWorld;
			const double U = FVector::DotProduct(Rel, AxisU);
			const double V = FVector::DotProduct(Rel, AxisV);
			MinU = FMath::Min(MinU, U);
			MaxU = FMath::Max(MaxU, U);
			MinV = FMath::Min(MinV, V);
			MaxV = FMath::Max(MaxV, V);
		}

		const double Extent = FMath::Max(50.0, FallbackExtent);
		if (!FMath::IsFinite(MinU) || !FMath::IsFinite(MaxU) || MinU > MaxU)
		{
			MinU = -Extent * 0.5;
			MaxU = Extent * 0.5;
		}
		if (!FMath::IsFinite(MinV) || !FMath::IsFinite(MaxV) || MinV > MaxV)
		{
			MinV = -Extent * 0.5;
			MaxV = Extent * 0.5;
		}
		if (MaxU - MinU < 1.0)
		{
			const double Center = 0.5 * (MinU + MaxU);
			MinU = Center - Extent * 0.5;
			MaxU = Center + Extent * 0.5;
		}
		if (MaxV - MinV < 1.0)
		{
			const double Center = 0.5 * (MinV + MaxV);
			MinV = Center - Extent * 0.5;
			MaxV = Center + Extent * 0.5;
		}

		const double PadU = FMath::Max((MaxU - MinU) * 0.15, 5.0);
		const double PadV = FMath::Max((MaxV - MinV) * 0.15, 5.0);
		MinU -= PadU;
		MaxU += PadU;
		MinV -= PadV;
		MaxV += PadV;

		OutQuadWorld.Add(AnchorWorld + AxisU * MinU + AxisV * MinV);
		OutQuadWorld.Add(AnchorWorld + AxisU * MaxU + AxisV * MinV);
		OutQuadWorld.Add(AnchorWorld + AxisU * MaxU + AxisV * MaxV);
		OutQuadWorld.Add(AnchorWorld + AxisU * MinU + AxisV * MaxV);
		for (const FVector& P : OutQuadWorld)
		{
			FVector2D Pixel;
			if (ProjectWorldToImageOrthographic(
				Inputs.Camera,
				Inputs.FacesWidth,
				Inputs.FacesHeight,
				P,
				Pixel))
			{
				OutQuad2D.Add(Pixel);
			}
		}
		if (OutQuad2D.Num() != OutQuadWorld.Num())
		{
			OutQuad2D.Reset();
		}
	}

	static void PopulateAttachFallbackAttemptProjectionDebug(
		const FCommonInputs& Inputs,
		const FSolidReconstructionResult& CandidateSolid,
		FAttachSupportPlaneFallbackAttempt& Attempt)
	{
		Attempt.bHasBaseProjection = CandidateSolid.SourceLoopWorld.Num() >= 3 &&
			CandidateSolid.SourceLoopWorld.Num() == CandidateSolid.SourceLoop2D.Num();
		Attempt.BaseLoop2D = CandidateSolid.SourceLoop2D;
		Attempt.CopiedTargetLoop2D = CandidateSolid.CopiedTargetLoop2D;
		Attempt.BaseLoopWorld = CandidateSolid.SourceLoopWorld;
		Attempt.CopiedLoopWorld.Reset();
		Attempt.ReprojectedBaseLoop2D.Reset();
		Attempt.ReprojectedCopiedLoop2D.Reset();
		if (!Attempt.bHasBaseProjection)
		{
			return;
		}

		Attempt.CopiedLoopWorld.Reserve(Attempt.BaseLoopWorld.Num());
		for (const FVector& P : Attempt.BaseLoopWorld)
		{
			Attempt.CopiedLoopWorld.Add(P + Attempt.ExtrusionVectorWorld);
		}
		for (int32 i = 0; i < Attempt.BaseLoopWorld.Num(); ++i)
		{
			FVector2D SourcePixel;
			if (ProjectWorldToImageOrthographic(
				Inputs.Camera,
				Inputs.FacesWidth,
				Inputs.FacesHeight,
				Attempt.BaseLoopWorld[i],
				SourcePixel))
			{
				Attempt.ReprojectedBaseLoop2D.Add(SourcePixel);
			}
			FVector2D CopiedPixel;
			if (Attempt.CopiedLoopWorld.IsValidIndex(i) &&
				ProjectWorldToImageOrthographic(
					Inputs.Camera,
					Inputs.FacesWidth,
					Inputs.FacesHeight,
					Attempt.CopiedLoopWorld[i],
					CopiedPixel))
			{
				Attempt.ReprojectedCopiedLoop2D.Add(CopiedPixel);
			}
		}
		BuildVirtualBasePlaneDebugQuad(
			Inputs,
			Attempt.AnchorWorld,
			Attempt.SupportAwareAxisU,
			Attempt.SupportAwareAxisV,
			Attempt.BaseLoopWorld,
			Attempt.ExtrusionVectorWorld.Size(),
			Attempt.VirtualBasePlaneQuadWorld,
			Attempt.VirtualBasePlaneQuad2D);
	}

	static bool TryBuildAttachSupportPlaneFallback(
		const FString& ComponentName,
		const FString& PressDir,
		const FString& ComponentDir,
		int32 CapWidth,
		int32 CapHeight,
		double ScaleX,
		double ScaleY,
		const TArray<FVector2D>& RawCapPolygon,
		const TArray<FCapBoundaryRunInput>& BoundaryRuns,
		const TArray<FFromLZGreenChainCandidate2D>& GreenChains,
		const FCommonInputs& Inputs,
		const FFromLZFaceReconstructionParams& Params,
		FSolidReconstructionResult& OutSolid,
		FAttachSupportPlaneFallbackDebug& OutDebug)
	{
		OutDebug = FAttachSupportPlaneFallbackDebug();
		OutDebug.bAttempted = true;
		OutDebug.bEnabled = Params.bEnableAttachSupportPlaneFallback;
		OutDebug.SupportFaceVoteMinCoverage = FMath::Clamp(double(Params.SupportFaceVoteMinCoverage), 0.0, 1.0);
		OutDebug.SupportFaceVoteSampleStepPx = FMath::Max(1.0, double(Params.SupportFaceVoteSampleStepPx));
		InitializeSolidResult(OutSolid, ComponentName, PressDir, Inputs);
		OutSolid.Action = TEXT("attach");
		OutSolid.ReconstructionMethod = TEXT("attach_support_plane_fallback");
		OutSolid.bAttachSupportPlaneFallback = true;
		OutSolid.SourcePolygonKey = TEXT("cap_polygon_support_plane_base");
		OutSolid.CopiedPolygonKey = TEXT("cap_polygon_support_plane_extruded");
		OutSolid.CapWidth = CapWidth;
		OutSolid.CapHeight = CapHeight;

		TArray<FVector2D> RawCapLoopFaceSpace;
		MapPolygonToFacesSpace(RawCapPolygon, ScaleX, ScaleY, RawCapLoopFaceSpace);

		if (!Params.bEnableAttachSupportPlaneFallback)
		{
			OutDebug.Error = TEXT("attach support-plane fallback is disabled");
			SaveAttachSupportPlaneFallbackDebug(
				OutDebug, Inputs, RawCapLoopFaceSpace,
				ComponentDir / TEXT("10_attach_support_plane_fallback.json"),
				ComponentDir / TEXT("10_attach_support_plane_fallback.png"));
			return false;
		}
		if (RawCapPolygon.Num() < 3)
		{
			OutDebug.Error = TEXT("raw cap polygon has fewer than three points");
			SaveAttachSupportPlaneFallbackDebug(
				OutDebug, Inputs, RawCapLoopFaceSpace,
				ComponentDir / TEXT("10_attach_support_plane_fallback.json"),
				ComponentDir / TEXT("10_attach_support_plane_fallback.png"));
			return false;
		}
		if (GreenChains.Num() == 0)
		{
			OutDebug.Error = TEXT("no green chain candidates are available");
			SaveAttachSupportPlaneFallbackDebug(
				OutDebug, Inputs, RawCapLoopFaceSpace,
				ComponentDir / TEXT("10_attach_support_plane_fallback.json"),
				ComponentDir / TEXT("10_attach_support_plane_fallback.png"));
			return false;
		}
		if (!Inputs.Camera.bHasProjectionMatrix ||
			!IsUsableOrthographicProjectionMatrix(Inputs.Camera.ProjectionMatrix))
		{
			OutDebug.Error = TEXT("fallback requires a usable orthographic projection matrix");
			SaveAttachSupportPlaneFallbackDebug(
				OutDebug, Inputs, RawCapLoopFaceSpace,
				ComponentDir / TEXT("10_attach_support_plane_fallback.json"),
				ComponentDir / TEXT("10_attach_support_plane_fallback.png"));
			return false;
		}

		bool bHasBestSuccessfulAttempt = false;
		double BestSuccessfulSupportMinCameraDistance = TNumericLimits<double>::Max();
		double BestSuccessfulNoPenetrationViolationCm = TNumericLimits<double>::Max();
		double BestSuccessfulReprojectionErrorPixels = TNumericLimits<double>::Max();
		int32 BestSuccessfulAttemptIndex = INDEX_NONE;
		FSolidReconstructionResult BestSuccessfulSolid;

		for (int32 ChainIndex = 0; ChainIndex < GreenChains.Num(); ++ChainIndex)
		{
			const FFromLZGreenChainCandidate2D& Chain = GreenChains[ChainIndex];
			FAttachSupportPlaneFallbackAttempt Attempt;
			Attempt.ChainIndex = ChainIndex;
			Attempt.SeedStrokeId = Chain.SeedStrokeId;
			Attempt.ChainPathLength = Chain.PathLength;
			Attempt.ChainChordLength = Chain.ChordLength;

			FVector2D StartCap = Chain.Start;
			FVector2D EndCap = Chain.End;
			if (DistancePointToClosedLoop2D(FVector2D(StartCap.X * ScaleX, StartCap.Y * ScaleY), RawCapLoopFaceSpace) >
				DistancePointToClosedLoop2D(FVector2D(EndCap.X * ScaleX, EndCap.Y * ScaleY), RawCapLoopFaceSpace))
			{
				Swap(StartCap, EndCap);
			}
			Attempt.ChainStartFaceSpace = FVector2D(StartCap.X * ScaleX, StartCap.Y * ScaleY);
			Attempt.ChainEndFaceSpace = FVector2D(EndCap.X * ScaleX, EndCap.Y * ScaleY);
			if ((Attempt.ChainEndFaceSpace - Attempt.ChainStartFaceSpace).SizeSquared() < 1e-8)
			{
				Attempt.RejectReason = TEXT("chain chord is too short in faces space");
				OutDebug.Attempts.Add(Attempt);
				continue;
			}

			FSupportFaceVoteResult Vote;
			if (!VoteSupportFaceForSegment(
				Inputs,
				Attempt.ChainStartFaceSpace,
				Attempt.ChainEndFaceSpace,
				Params.SupportFaceVoteRadiusPx,
				OutDebug.SupportFaceVoteSampleStepPx,
				OutDebug.SupportFaceVoteMinCoverage,
				Vote))
			{
				Attempt.SupportFaceCandidates = Vote.FaceCandidates;
				Attempt.SupportVoteCoverage = Vote.Coverage;
				Attempt.SupportVotePixelCoverage = Vote.VotePixelCoverage;
				Attempt.SupportVotePixels = Vote.VotePixels;
				Attempt.SupportConsideredPixels = Vote.ConsideredPixels;
				Attempt.SupportHitSampleCount = Vote.HitSampleCount;
				Attempt.SupportTotalSampleCount = Vote.TotalSampleCount;
				Attempt.SupportFaceWorldZMax = Vote.WorldZMax;
				Attempt.SupportFaceWorldZAverage = Vote.WorldZAverage;
				Attempt.SupportMinCameraDistance = Vote.MinCameraDistance;
				Attempt.SupportWorstPolygonDistancePx = Vote.WorstPolygonDistancePx;
				Attempt.RejectReason = Vote.Error;
				OutDebug.Attempts.Add(Attempt);
				continue;
			}
			Attempt.SupportFaceId = Vote.FaceId;
			Attempt.SupportFaceCandidates = Vote.FaceCandidates;
			Attempt.SupportVoteCoverage = Vote.Coverage;
			Attempt.SupportVotePixelCoverage = Vote.VotePixelCoverage;
			Attempt.SupportVotePixels = Vote.VotePixels;
			Attempt.SupportConsideredPixels = Vote.ConsideredPixels;
			Attempt.SupportHitSampleCount = Vote.HitSampleCount;
			Attempt.SupportTotalSampleCount = Vote.TotalSampleCount;
			Attempt.SupportFaceWorldZMax = Vote.WorldZMax;
			Attempt.SupportFaceWorldZAverage = Vote.WorldZAverage;
			Attempt.SupportMinCameraDistance = Vote.MinCameraDistance;
			Attempt.SupportWorstPolygonDistancePx = Vote.WorstPolygonDistancePx;

			const int32* SupportIndex = Inputs.FaceIndexById.Find(Vote.FaceId);
			if (!SupportIndex)
			{
				Attempt.RejectReason = TEXT("support face id was not found in face table");
				OutDebug.Attempts.Add(Attempt);
				continue;
			}
			const FFaceInfo& SupportFace = Inputs.Faces[*SupportIndex];
			Attempt.SupportPlaneNormal = SupportFace.Normal.GetSafeNormal();

			if (!IntersectPixelWithPlaneOrthographic(
				Inputs.Camera,
				Inputs.FacesWidth,
				Inputs.FacesHeight,
				Attempt.ChainStartFaceSpace,
				SupportFace.PlanePoint,
				SupportFace.Normal,
				Attempt.AnchorWorld) ||
				!IntersectPixelWithPlaneOrthographic(
					Inputs.Camera,
					Inputs.FacesWidth,
					Inputs.FacesHeight,
					Attempt.ChainEndFaceSpace,
					SupportFace.PlanePoint,
					SupportFace.Normal,
					Attempt.ChainEndWorld))
			{
				Attempt.RejectReason = TEXT("failed to raycast green chain endpoints onto support face");
				OutDebug.Attempts.Add(Attempt);
				continue;
			}

			Attempt.ExtrusionVectorWorld = Attempt.ChainEndWorld - Attempt.AnchorWorld;
			const double GreenChordWorldCm = Attempt.ExtrusionVectorWorld.Size();
			const bool bBelowPreferredGreenChord =
				GreenChordWorldCm < FMath::Max(
					double(Params.SupportForceHardMinGreenChordCm),
					double(Params.SupportForcePreferredMinGreenChordCm));
			if (GreenChordWorldCm < FMath::Max(0.0, double(Params.SupportForceHardMinGreenChordCm)))
			{
				Attempt.RejectReason = FString::Printf(
					TEXT("3D green chord %.6f cm is below force hard minimum %.6f cm"),
					GreenChordWorldCm,
					double(Params.SupportForceHardMinGreenChordCm));
				OutDebug.Attempts.Add(Attempt);
				continue;
			}
			Attempt.BasePlaneNormal = Attempt.ExtrusionVectorWorld.GetSafeNormal();
			if (Attempt.BasePlaneNormal.IsNearlyZero())
			{
				Attempt.RejectReason = TEXT("3D green chord is too short");
				OutDebug.Attempts.Add(Attempt);
				continue;
			}
			if (!BuildSupportAwareFaceBasis(
				Attempt.BasePlaneNormal,
				SupportFace.Normal,
				Attempt.SupportAwareAxisU,
				Attempt.SupportAwareAxisV))
			{
				Attempt.RejectReason = TEXT("failed to build support-aware base-face basis");
				OutDebug.Attempts.Add(Attempt);
				continue;
			}

			FSolidReconstructionResult CandidateSolid;
			InitializeSolidResult(CandidateSolid, ComponentName, PressDir, Inputs);
			CandidateSolid.Action = TEXT("attach");
			CandidateSolid.ReconstructionMethod = TEXT("attach_support_plane_fallback");
			CandidateSolid.bAttachSupportPlaneFallback = true;
			CandidateSolid.SourcePolygonKey = TEXT("cap_polygon_support_plane_base");
			CandidateSolid.CopiedPolygonKey = TEXT("cap_polygon_support_plane_extruded");
			CandidateSolid.CapWidth = CapWidth;
			CandidateSolid.CapHeight = CapHeight;
			CandidateSolid.SelectedFaceId = SupportFace.Id;
			CandidateSolid.SupportFaceId = SupportFace.Id;
			CandidateSolid.SupportGreenChainIndex = ChainIndex;
			CandidateSolid.SourcePlanePoint = Attempt.AnchorWorld;
			CandidateSolid.SourcePlaneNormal = Attempt.BasePlaneNormal;
			CandidateSolid.SupportPlanePoint = SupportFace.PlanePoint;
			CandidateSolid.SupportPlaneNormal = SupportFace.Normal.GetSafeNormal();
			CandidateSolid.SourceFaceVerticesWorld = SupportFace.KeyPoints3D;
			CandidateSolid.OrientedNormal = Attempt.BasePlaneNormal;
			CandidateSolid.ExtrusionVectorWorld = Attempt.ExtrusionVectorWorld;
			CandidateSolid.ExtrusionDepth = Attempt.ExtrusionVectorWorld.Size();
			CandidateSolid.SourceLoop2D = RawCapLoopFaceSpace;
			CandidateSolid.SourceToCopiedVector2D = Attempt.ChainEndFaceSpace - Attempt.ChainStartFaceSpace;
			CandidateSolid.CopiedTargetLoop2D.Reset();
			CandidateSolid.CopiedTargetLoop2D.Reserve(CandidateSolid.SourceLoop2D.Num());
			for (const FVector2D& P : CandidateSolid.SourceLoop2D)
			{
				CandidateSolid.CopiedTargetLoop2D.Add(P + CandidateSolid.SourceToCopiedVector2D);
			}
			SimplifyLoopPairs(CandidateSolid.SourceLoop2D, CandidateSolid.CopiedTargetLoop2D);
			if (CandidateSolid.SourceLoop2D.Num() < 3 ||
				CandidateSolid.SourceLoop2D.Num() != CandidateSolid.CopiedTargetLoop2D.Num())
			{
				Attempt.RejectReason = TEXT("fallback cap loop became invalid after cleanup");
				OutDebug.Attempts.Add(Attempt);
				continue;
			}

			CandidateSolid.SourceLoopWorld.Reserve(CandidateSolid.SourceLoop2D.Num());
			bool bBaseProjectionOk = true;
			for (const FVector2D& P : CandidateSolid.SourceLoop2D)
			{
				FVector Hit;
				if (!IntersectPixelWithPlaneOrthographic(
					Inputs.Camera,
					Inputs.FacesWidth,
					Inputs.FacesHeight,
					P,
					Attempt.AnchorWorld,
					Attempt.BasePlaneNormal,
					Hit))
				{
					bBaseProjectionOk = false;
					break;
				}
				CandidateSolid.SourceLoopWorld.Add(Hit);
			}
			if (!bBaseProjectionOk)
			{
				Attempt.RejectReason = TEXT("failed to project raw cap onto fallback base plane");
				OutDebug.Attempts.Add(Attempt);
				continue;
			}
			PopulateAttachFallbackAttemptProjectionDebug(Inputs, CandidateSolid, Attempt);
			if (!CheckSupportPlaneProjectionTopology(
				TEXT("raw_projection"),
				CandidateSolid.SourceLoop2D,
				CandidateSolid.SourceLoopWorld,
				Attempt.AnchorWorld,
				Attempt.SupportAwareAxisU,
				Attempt.SupportAwareAxisV,
				Attempt.RawProjectionTopology))
			{
				Attempt.RejectReason = FString::Printf(
					TEXT("support_projection_topology_invalid(raw): %s"),
					*Attempt.RawProjectionTopology.Reason);
				OutDebug.Attempts.Add(Attempt);
				continue;
			}
			const TArray<FVector2D> RawProjectedSourceLoop2D = CandidateSolid.SourceLoop2D;
			const TArray<FVector2D> RawProjectedCopiedLoop2D = CandidateSolid.CopiedTargetLoop2D;
			const TArray<FVector> RawProjectedSourceLoopWorld = CandidateSolid.SourceLoopWorld;

			FString NoPenetrationError;
			if (!CheckAttachSupportNoPenetration(
				Inputs,
				SupportFace,
				CandidateSolid.SourceLoop2D,
				CandidateSolid.SourceLoopWorld,
				Params.NoPenetrationTolCm,
				Attempt.MaxNoPenetrationViolationCm,
				NoPenetrationError,
				&Attempt.NoPenetrationSamplePixels,
				&Attempt.NoPenetrationBaseWorld,
				&Attempt.NoPenetrationSupportWorld,
				&Attempt.NoPenetrationViolationCm,
				/*bStopOnFirstFailure*/ false))
			{
				Attempt.bForceable = true;
				Attempt.ForceableReason = TEXT("no_penetration_exceeded_threshold_raw");
				Attempt.RejectReason = FString::Printf(TEXT("raw no-penetration failed: %s"), *NoPenetrationError);
				OutDebug.Attempts.Add(Attempt);
				continue;
			}

			TArray<FVector> RegularizedSourceLoopWorld;
			FFaceInfo VirtualBaseFace;
			VirtualBaseFace.Id = SupportFace.Id;
			VirtualBaseFace.Color = SupportFace.Color;
			VirtualBaseFace.PlanePoint = Attempt.AnchorWorld;
			VirtualBaseFace.Normal = Attempt.BasePlaneNormal;
			VirtualBaseFace.KeyPoints2D = CandidateSolid.SourceLoop2D;
			VirtualBaseFace.KeyPoints3D = CandidateSolid.SourceLoopWorld;
			Attempt.bOrthogonalAttempted = true;
			const bool bRegularizationApplied = RegularizeCapToWorldOrthogonalPolygon(
				VirtualBaseFace,
				Inputs,
				Params,
				CandidateSolid.SourceToCopiedVector2D,
				CandidateSolid.SourceLoopWorld,
				BoundaryRuns,
				FVector2D::ZeroVector,
				ScaleX,
				ScaleY,
				RegularizedSourceLoopWorld,
				CandidateSolid.CapBBoxRegularization,
				&Attempt.SupportAwareAxisU,
				&Attempt.SupportAwareAxisV,
				TEXT("attach_support_plane_virtual_base"));
			CandidateSolid.CapBBoxRegularization.SelectedFaceId = SupportFace.Id;
			CandidateSolid.CapBBoxRegularization.Action = TEXT("attach");
			CandidateSolid.CapBBoxRegularization.SourcePolygonKey = CandidateSolid.SourcePolygonKey;
			CandidateSolid.CapBBoxRegularization.CopiedPolygonKey = CandidateSolid.CopiedPolygonKey;
			Attempt.bOrthogonalApplied = bRegularizationApplied;
			Attempt.bOrthogonalFallbackToOriginal = !bRegularizationApplied || CandidateSolid.CapBBoxRegularization.bFallbackToOriginal;
			if (bRegularizationApplied)
			{
				CandidateSolid.SourceLoopWorld = MoveTemp(RegularizedSourceLoopWorld);
				CandidateSolid.SourceLoop2D.Reset();
				CandidateSolid.CopiedTargetLoop2D.Reset();
				bool bProjectionValid = true;
				for (const FVector& WorldPoint : CandidateSolid.SourceLoopWorld)
				{
					FVector2D SourcePixel;
					if (!ProjectWorldToImageOrthographic(
						Inputs.Camera,
						Inputs.FacesWidth,
						Inputs.FacesHeight,
						WorldPoint,
						SourcePixel) ||
						!IsFiniteVector2D(SourcePixel))
					{
						bProjectionValid = false;
						break;
					}
					CandidateSolid.SourceLoop2D.Add(SourcePixel);
					CandidateSolid.CopiedTargetLoop2D.Add(SourcePixel + CandidateSolid.SourceToCopiedVector2D);
				}
				if (!bProjectionValid ||
					CandidateSolid.SourceLoop2D.Num() < 3 ||
					CandidateSolid.CopiedTargetLoop2D.Num() != CandidateSolid.SourceLoop2D.Num())
				{
					Attempt.RegularizedProjectionTopology.bChecked = true;
					Attempt.RegularizedProjectionTopology.Stage = TEXT("regularized_projection");
					Attempt.RegularizedProjectionTopology.Reason =
						TEXT("fallback regularized polygon failed orthographic reprojection; using raw projected cap");
					SetRegularizationFallback(
						CandidateSolid.CapBBoxRegularization,
						TEXT("fallback regularized polygon failed orthographic reprojection; using raw projected cap"));
					CandidateSolid.SourceLoop2D = RawProjectedSourceLoop2D;
					CandidateSolid.CopiedTargetLoop2D = RawProjectedCopiedLoop2D;
					CandidateSolid.SourceLoopWorld = RawProjectedSourceLoopWorld;
					Attempt.bOrthogonalApplied = false;
					Attempt.bOrthogonalFallbackToOriginal = true;
				}
				else if (!CheckSupportPlaneProjectionTopology(
					TEXT("regularized_projection"),
					CandidateSolid.SourceLoop2D,
					CandidateSolid.SourceLoopWorld,
					Attempt.AnchorWorld,
					Attempt.SupportAwareAxisU,
					Attempt.SupportAwareAxisV,
					Attempt.RegularizedProjectionTopology))
				{
					SetRegularizationFallback(
						CandidateSolid.CapBBoxRegularization,
						FString::Printf(
							TEXT("fallback regularized topology invalid: %s; using raw projected cap"),
							*Attempt.RegularizedProjectionTopology.Reason));
					CandidateSolid.SourceLoop2D = RawProjectedSourceLoop2D;
					CandidateSolid.CopiedTargetLoop2D = RawProjectedCopiedLoop2D;
					CandidateSolid.SourceLoopWorld = RawProjectedSourceLoopWorld;
					Attempt.bOrthogonalApplied = false;
					Attempt.bOrthogonalFallbackToOriginal = true;
				}
			}

			if (CandidateSolid.SourceLoopWorld.Num() != CandidateSolid.SourceLoop2D.Num() ||
				CandidateSolid.SourceLoop2D.Num() < 3)
			{
				Attempt.RejectReason = TEXT("fallback source loop invalid after orthogonal fallback");
				OutDebug.Attempts.Add(Attempt);
				continue;
			}
			PopulateAttachFallbackAttemptProjectionDebug(Inputs, CandidateSolid, Attempt);
			if (!CheckSupportPlaneProjectionTopology(
				TEXT("final_projection"),
				CandidateSolid.SourceLoop2D,
				CandidateSolid.SourceLoopWorld,
				Attempt.AnchorWorld,
				Attempt.SupportAwareAxisU,
				Attempt.SupportAwareAxisV,
				Attempt.FinalProjectionTopology))
			{
				Attempt.RejectReason = FString::Printf(
					TEXT("support_projection_topology_invalid(final): %s"),
					*Attempt.FinalProjectionTopology.Reason);
				OutDebug.Attempts.Add(Attempt);
				continue;
			}

			Attempt.ContactDistancePixels = DistancePointToClosedLoop2D(
				Attempt.ChainStartFaceSpace,
				CandidateSolid.SourceLoop2D);
			Attempt.bContactPass = Attempt.ContactDistancePixels <= Params.ContactAnchorTolPx;
			if (!Attempt.bContactPass)
			{
				Attempt.RejectReason = FString::Printf(
					TEXT("green anchor distance %.3f px exceeds contact tolerance %.3f"),
					Attempt.ContactDistancePixels,
					Params.ContactAnchorTolPx);
				OutDebug.Attempts.Add(Attempt);
				continue;
			}

			if (!CheckAttachSupportNoPenetration(
				Inputs,
				SupportFace,
				CandidateSolid.SourceLoop2D,
				CandidateSolid.SourceLoopWorld,
				Params.NoPenetrationTolCm,
				Attempt.MaxNoPenetrationViolationCm,
				NoPenetrationError,
				&Attempt.NoPenetrationSamplePixels,
				&Attempt.NoPenetrationBaseWorld,
				&Attempt.NoPenetrationSupportWorld,
				&Attempt.NoPenetrationViolationCm,
				/*bStopOnFirstFailure*/ false))
			{
				Attempt.bForceable = true;
				Attempt.ForceableReason = TEXT("no_penetration_exceeded_threshold_final");
				Attempt.RejectReason = FString::Printf(TEXT("final no-penetration failed: %s"), *NoPenetrationError);
				OutDebug.Attempts.Add(Attempt);
				continue;
			}
			Attempt.bNoPenetrationPass = true;

			CandidateSolid.CopiedLoopWorld.Reset();
			CandidateSolid.CopiedLoopWorld.Reserve(CandidateSolid.SourceLoopWorld.Num());
			for (const FVector& P : CandidateSolid.SourceLoopWorld)
			{
				CandidateSolid.CopiedLoopWorld.Add(P + Attempt.ExtrusionVectorWorld);
			}
			CandidateSolid.MaxDepthSampleReprojectionErrorPixels =
				FMath::Max(25.0, CandidateSolid.SourceToCopiedVector2D.Size() * 0.75);
			if (!ProjectSignedWorldDirectionToImage(
				Inputs.Camera,
				Inputs.FacesWidth,
				Inputs.FacesHeight,
				CandidateSolid.OrientedNormal,
				CandidateSolid.ProjectedNormal2D))
			{
				Attempt.RejectReason = TEXT("fallback base-plane normal projects to a near-zero image direction");
				OutDebug.Attempts.Add(Attempt);
				continue;
			}
			if (FVector2D::DotProduct(
				CandidateSolid.ProjectedNormal2D,
				CandidateSolid.SourceToCopiedVector2D.GetSafeNormal()) < 0.0)
			{
				CandidateSolid.ProjectedNormal2D *= -1.0;
			}

			FString ReprojectionError;
			if (!ReprojectSolidLoops(Inputs, CandidateSolid, ReprojectionError))
			{
				Attempt.MaxReprojectionErrorPixels = CandidateSolid.MaxCopiedReprojectionErrorPixels;
				Attempt.RejectReason = ReprojectionError;
				OutDebug.Attempts.Add(Attempt);
				continue;
			}
			Attempt.MaxReprojectionErrorPixels = CandidateSolid.MaxCopiedReprojectionErrorPixels;
			Attempt.bReprojectionPass = true;

			if (!BuildSolidMeshTriangles(
				CandidateSolid.SourceLoop2D,
				CandidateSolid.CopiedTargetLoop2D,
				CandidateSolid.SourceLoopWorld,
				CandidateSolid.CopiedLoopWorld,
				CandidateSolid.OrientedNormal,
				CandidateSolid.MeshVerticesWorld,
				CandidateSolid.MeshTriangles,
				CandidateSolid.MeshNormal))
			{
				Attempt.RejectReason = TEXT("failed to triangulate fallback solid");
				OutDebug.Attempts.Add(Attempt);
				continue;
			}
			if (bBelowPreferredGreenChord)
			{
				Attempt.bForceable = true;
				Attempt.ForceableReason = TEXT("green_chord_below_preferred_length");
				Attempt.RejectReason = FString::Printf(
					TEXT("3D green chord %.6f cm is below preferred %.6f cm but above hard minimum %.6f cm"),
					GreenChordWorldCm,
					double(Params.SupportForcePreferredMinGreenChordCm),
					double(Params.SupportForceHardMinGreenChordCm));
				OutDebug.Attempts.Add(Attempt);
				continue;
			}

			CandidateSolid.SourceMaterialProbePointsWorld = SupportFace.KeyPoints3D;
			CandidateSolid.SourceMaterialProbePointsWorld.Add(AverageVector(SupportFace.KeyPoints3D));
			CandidateSolid.bSuccess = true;
			Attempt.bSuccess = true;
			Attempt.RejectReason = TEXT("passed; candidate for nearest-camera support selection");
			const int32 AttemptIndex = OutDebug.Attempts.Add(Attempt);
			const bool bCloserSupport =
				!bHasBestSuccessfulAttempt ||
				Attempt.SupportMinCameraDistance < BestSuccessfulSupportMinCameraDistance - 1e-6;
			const bool bTieBetterNoPenetration =
				bHasBestSuccessfulAttempt &&
				FMath::IsNearlyEqual(
					Attempt.SupportMinCameraDistance,
					BestSuccessfulSupportMinCameraDistance,
					1e-6) &&
				Attempt.MaxNoPenetrationViolationCm < BestSuccessfulNoPenetrationViolationCm - 1e-6;
			const bool bTieBetterReprojection =
				bHasBestSuccessfulAttempt &&
				FMath::IsNearlyEqual(
					Attempt.SupportMinCameraDistance,
					BestSuccessfulSupportMinCameraDistance,
					1e-6) &&
				FMath::IsNearlyEqual(
					Attempt.MaxNoPenetrationViolationCm,
					BestSuccessfulNoPenetrationViolationCm,
					1e-6) &&
				Attempt.MaxReprojectionErrorPixels < BestSuccessfulReprojectionErrorPixels;
			if (bCloserSupport || bTieBetterNoPenetration || bTieBetterReprojection)
			{
				bHasBestSuccessfulAttempt = true;
				BestSuccessfulSupportMinCameraDistance = Attempt.SupportMinCameraDistance;
				BestSuccessfulNoPenetrationViolationCm = Attempt.MaxNoPenetrationViolationCm;
				BestSuccessfulReprojectionErrorPixels = Attempt.MaxReprojectionErrorPixels;
				BestSuccessfulAttemptIndex = AttemptIndex;
				BestSuccessfulSolid = MoveTemp(CandidateSolid);
			}
		}

		if (bHasBestSuccessfulAttempt)
		{
			if (OutDebug.Attempts.IsValidIndex(BestSuccessfulAttemptIndex))
			{
				OutDebug.Attempts[BestSuccessfulAttemptIndex].RejectReason =
					TEXT("selected_nearest_camera_support_face");
			}
			OutSolid = MoveTemp(BestSuccessfulSolid);
			OutDebug.bSuccess = true;
			OutDebug.Error = FString::Printf(
				TEXT("selected nearest-camera support face %.6f cm; max no-penetration violation %.6f cm"),
				BestSuccessfulSupportMinCameraDistance,
				BestSuccessfulNoPenetrationViolationCm);
			OutDebug.BestForceableAttemptIndex = SelectBestForceableSupportAttemptIndex(OutDebug);
			OutDebug.BestForceableReason = OutDebug.Attempts.IsValidIndex(OutDebug.BestForceableAttemptIndex)
				? OutDebug.Attempts[OutDebug.BestForceableAttemptIndex].ForceableReason
				: FString();
			SaveAttachSupportPlaneFallbackDebug(
				OutDebug, Inputs, RawCapLoopFaceSpace,
				ComponentDir / TEXT("10_attach_support_plane_fallback.json"),
				ComponentDir / TEXT("10_attach_support_plane_fallback.png"));
			return true;
		}

		OutDebug.Error = OutDebug.Attempts.Num() > 0
			? OutDebug.Attempts.Last().RejectReason
			: TEXT("no fallback attempt was made");
		OutDebug.BestForceableAttemptIndex = SelectBestForceableSupportAttemptIndex(OutDebug);
		OutDebug.BestForceableReason = OutDebug.Attempts.IsValidIndex(OutDebug.BestForceableAttemptIndex)
			? OutDebug.Attempts[OutDebug.BestForceableAttemptIndex].ForceableReason
			: FString();
		SaveAttachSupportPlaneFallbackDebug(
			OutDebug, Inputs, RawCapLoopFaceSpace,
			ComponentDir / TEXT("10_attach_support_plane_fallback.json"),
			ComponentDir / TEXT("10_attach_support_plane_fallback.png"));
		return false;
	}

	static FVector ComputeWorldLoopAreaCentroid(const TArray<FVector>& Loop)
	{
		if (Loop.Num() == 0)
		{
			return FVector::ZeroVector;
		}
		if (Loop.Num() < 3)
		{
			return AverageVector(Loop);
		}

		const FVector Origin = Loop[0];
		FVector Weighted = FVector::ZeroVector;
		double AreaSum = 0.0;
		for (int32 i = 1; i + 1 < Loop.Num(); ++i)
		{
			const FVector A = Loop[i] - Origin;
			const FVector B = Loop[i + 1] - Origin;
			const double Area = FVector::CrossProduct(A, B).Size() * 0.5;
			if (Area <= KINDA_SMALL_NUMBER || !FMath::IsFinite(Area))
			{
				continue;
			}
			Weighted += (Origin + Loop[i] + Loop[i + 1]) * (Area / 3.0);
			AreaSum += Area;
		}
		if (AreaSum <= KINDA_SMALL_NUMBER)
		{
			return AverageVector(Loop);
		}
		return Weighted / AreaSum;
	}

	static bool ComputeSolidFrontCapDistanceCm(
		const FSolidReconstructionResult& Solid,
		const FCommonInputs& Inputs,
		double& OutFrontDistanceCm,
		double& OutSourceDistanceCm,
		double& OutCopiedDistanceCm,
		FString& OutFrontLoopKey)
	{
		OutFrontDistanceCm = 0.0;
		OutSourceDistanceCm = 0.0;
		OutCopiedDistanceCm = 0.0;
		OutFrontLoopKey.Reset();
		if (!Solid.bSuccess ||
			Solid.SourceLoopWorld.Num() < 3 ||
			Solid.CopiedLoopWorld.Num() < 3)
		{
			return false;
		}
		const FVector SourceCentroid = ComputeWorldLoopAreaCentroid(Solid.SourceLoopWorld);
		const FVector CopiedCentroid = ComputeWorldLoopAreaCentroid(Solid.CopiedLoopWorld);
		OutSourceDistanceCm = FVector::Distance(Inputs.Camera.Location, SourceCentroid);
		OutCopiedDistanceCm = FVector::Distance(Inputs.Camera.Location, CopiedCentroid);
		if (OutSourceDistanceCm <= OutCopiedDistanceCm)
		{
			OutFrontDistanceCm = OutSourceDistanceCm;
			OutFrontLoopKey = Solid.SourcePolygonKey;
		}
		else
		{
			OutFrontDistanceCm = OutCopiedDistanceCm;
			OutFrontLoopKey = Solid.CopiedPolygonKey;
		}
		return FMath::IsFinite(OutFrontDistanceCm);
	}

	struct FAttachPathPlaneRelationDebug
	{
		bool bEvaluated = false;
		bool bValid = false;
		FString Reason;
		FString DecisionRule;
		double AngleTolDeg = 0.0;
		double DistanceTolCm = 0.0;
		double VirtualBaseVsDirectBaseAngleDeg = -1.0;
		double SupportFaceVsDirectBaseAngleDeg = -1.0;
		double VirtualBaseDirectPlaneDistanceCm = -1.0;
		double SupportFaceDirectPlaneDistanceCm = -1.0;
		bool bVirtualBasePerpendicularToDirectBase = false;
		bool bSupportFacePerpendicularToDirectBase = false;
		bool bVirtualBaseParallelToDirectBase = false;
		bool bSupportFaceParallelToDirectBase = false;
		bool bVirtualBaseMatchesDirectBase = false;
		bool bSupportFaceMatchesDirectBase = false;
		bool bSupportFaceIdMatchesDirectFaceId = false;
		FVector DirectBasePoint = FVector::ZeroVector;
		FVector DirectBaseNormal = FVector::UpVector;
		FVector SupportVirtualBasePoint = FVector::ZeroVector;
		FVector SupportVirtualBaseNormal = FVector::UpVector;
		FVector SupportFacePoint = FVector::ZeroVector;
		FVector SupportFaceNormal = FVector::UpVector;
	};

	static double ComputeUnorientedNormalAngleDeg(const FVector& A, const FVector& B)
	{
		const FVector NA = A.GetSafeNormal();
		const FVector NB = B.GetSafeNormal();
		if (NA.IsNearlyZero() || NB.IsNearlyZero() || !IsFiniteVector(NA) || !IsFiniteVector(NB))
		{
			return -1.0;
		}

		const double Dot = FMath::Clamp(FMath::Abs(FVector::DotProduct(NA, NB)), 0.0, 1.0);
		return FMath::RadiansToDegrees(FMath::Acos(Dot));
	}

	static double ComputeAbsPointToPlaneDistanceCm(
		const FVector& Point,
		const FVector& PlanePoint,
		const FVector& PlaneNormal)
	{
		const FVector N = PlaneNormal.GetSafeNormal();
		if (N.IsNearlyZero() ||
			!IsFiniteVector(Point) ||
			!IsFiniteVector(PlanePoint) ||
			!IsFiniteVector(N))
		{
			return -1.0;
		}
		return FMath::Abs(FVector::DotProduct(Point - PlanePoint, N));
	}

	static FAttachPathPlaneRelationDebug AnalyzeAttachPathPlaneRelation(
		const FSolidReconstructionResult& DirectSolid,
		const FSolidReconstructionResult& SupportSolid,
		const FFromLZFaceReconstructionParams& Params)
	{
		FAttachPathPlaneRelationDebug Out;
		Out.bEvaluated = DirectSolid.bSuccess && SupportSolid.bSuccess;
		Out.AngleTolDeg = FMath::Max(0.0, double(Params.AttachPathPlaneRelationAngleTolDeg));
		Out.DistanceTolCm = FMath::Max(0.0, double(Params.AttachPathPlaneRelationDistanceTolCm));
		Out.DirectBasePoint = DirectSolid.SourcePlanePoint;
		Out.DirectBaseNormal = DirectSolid.SourcePlaneNormal.GetSafeNormal();
		Out.SupportVirtualBasePoint = SupportSolid.SourcePlanePoint;
		Out.SupportVirtualBaseNormal = SupportSolid.SourcePlaneNormal.GetSafeNormal();
		Out.SupportFacePoint = SupportSolid.SupportPlanePoint;
		Out.SupportFaceNormal = SupportSolid.SupportPlaneNormal.GetSafeNormal();
		Out.bSupportFaceIdMatchesDirectFaceId =
			DirectSolid.SelectedFaceId >= 0 &&
			SupportSolid.SupportFaceId >= 0 &&
			DirectSolid.SelectedFaceId == SupportSolid.SupportFaceId;

		if (!Out.bEvaluated)
		{
			Out.Reason = TEXT("direct and support solids were not both successful");
			return Out;
		}
		if (Out.DirectBaseNormal.IsNearlyZero() ||
			Out.SupportVirtualBaseNormal.IsNearlyZero() ||
			Out.SupportFaceNormal.IsNearlyZero())
		{
			Out.Reason = TEXT("one or more plane normals are invalid");
			return Out;
		}

		Out.VirtualBaseVsDirectBaseAngleDeg = ComputeUnorientedNormalAngleDeg(
			Out.SupportVirtualBaseNormal,
			Out.DirectBaseNormal);
		Out.SupportFaceVsDirectBaseAngleDeg = ComputeUnorientedNormalAngleDeg(
			Out.SupportFaceNormal,
			Out.DirectBaseNormal);
		Out.VirtualBaseDirectPlaneDistanceCm = ComputeAbsPointToPlaneDistanceCm(
			Out.SupportVirtualBasePoint,
			Out.DirectBasePoint,
			Out.DirectBaseNormal);
		Out.SupportFaceDirectPlaneDistanceCm = ComputeAbsPointToPlaneDistanceCm(
			Out.SupportFacePoint,
			Out.DirectBasePoint,
			Out.DirectBaseNormal);

		const bool bVirtualAngleValid = Out.VirtualBaseVsDirectBaseAngleDeg >= 0.0;
		const bool bSupportAngleValid = Out.SupportFaceVsDirectBaseAngleDeg >= 0.0;
		const bool bVirtualDistanceValid = Out.VirtualBaseDirectPlaneDistanceCm >= 0.0;
		const bool bSupportDistanceValid = Out.SupportFaceDirectPlaneDistanceCm >= 0.0;
		if (!bVirtualAngleValid || !bSupportAngleValid || !bVirtualDistanceValid || !bSupportDistanceValid)
		{
			Out.Reason = TEXT("failed to compute one or more plane relation metrics");
			return Out;
		}

		Out.bVirtualBaseParallelToDirectBase = Out.VirtualBaseVsDirectBaseAngleDeg <= Out.AngleTolDeg;
		Out.bSupportFaceParallelToDirectBase = Out.SupportFaceVsDirectBaseAngleDeg <= Out.AngleTolDeg;
		Out.bVirtualBasePerpendicularToDirectBase =
			FMath::Abs(90.0 - Out.VirtualBaseVsDirectBaseAngleDeg) <= Out.AngleTolDeg;
		Out.bSupportFacePerpendicularToDirectBase =
			FMath::Abs(90.0 - Out.SupportFaceVsDirectBaseAngleDeg) <= Out.AngleTolDeg;
		Out.bVirtualBaseMatchesDirectBase =
			Out.bVirtualBaseParallelToDirectBase &&
			Out.VirtualBaseDirectPlaneDistanceCm <= Out.DistanceTolCm;
		Out.bSupportFaceMatchesDirectBase =
			Out.bSupportFaceIdMatchesDirectFaceId ||
			(Out.bSupportFaceParallelToDirectBase &&
				Out.SupportFaceDirectPlaneDistanceCm <= Out.DistanceTolCm);

		Out.bValid = true;
		Out.Reason = TEXT("plane relation metrics computed");
		return Out;
	}

	static bool TryBuildForcedSupportSolidFromAttempt(
		const FString& ComponentName,
		const FString& PressDir,
		int32 CapWidth,
		int32 CapHeight,
		int32 AttemptIndex,
		const FAttachSupportPlaneFallbackAttempt& Attempt,
		const FCommonInputs& Inputs,
		const FFromLZFaceReconstructionParams& Params,
		FSolidReconstructionResult& OutSolid,
		FString& OutError)
	{
		OutError.Reset();
		InitializeSolidResult(OutSolid, ComponentName, PressDir, Inputs);
		OutSolid.Action = TEXT("attach");
		OutSolid.ReconstructionMethod = TEXT("attach_support_plane_fallback_forced");
		OutSolid.bAttachSupportPlaneFallback = true;
		OutSolid.bForcedSupportPlaneOutput = true;
		OutSolid.ForcedSupportPlaneReason = Attempt.ForceableReason;
		OutSolid.ForcedSupportPlaneAttemptIndex = AttemptIndex;
		OutSolid.SourcePolygonKey = TEXT("cap_polygon_support_plane_base");
		OutSolid.CopiedPolygonKey = TEXT("cap_polygon_support_plane_extruded");
		OutSolid.CapWidth = CapWidth;
		OutSolid.CapHeight = CapHeight;

		if (!Attempt.bForceable)
		{
			OutError = TEXT("attempt is not marked forceable");
			return false;
		}
		if (!Attempt.RawProjectionTopology.bPass && !Attempt.FinalProjectionTopology.bPass)
		{
			OutError = TEXT("forceable support attempt did not pass raw or final projection topology checks");
			return false;
		}
		if (!Attempt.bHasBaseProjection ||
			Attempt.BaseLoop2D.Num() < 3 ||
			Attempt.BaseLoopWorld.Num() != Attempt.BaseLoop2D.Num())
		{
			OutError = TEXT("forceable support attempt has no valid projected base loop");
			return false;
		}
		if (Attempt.ExtrusionVectorWorld.Size() < FMath::Max(0.0f, Params.SupportForceHardMinGreenChordCm))
		{
			OutError = FString::Printf(
				TEXT("forceable support attempt green chord %.6f cm is below hard minimum %.6f cm"),
				Attempt.ExtrusionVectorWorld.Size(),
				double(Params.SupportForceHardMinGreenChordCm));
			return false;
		}

		const int32* SupportIndex = Inputs.FaceIndexById.Find(Attempt.SupportFaceId);
		if (!SupportIndex || !Inputs.Faces.IsValidIndex(*SupportIndex))
		{
			OutError = TEXT("forceable support attempt references an unknown support face");
			return false;
		}
		const FFaceInfo& SupportFace = Inputs.Faces[*SupportIndex];

		OutSolid.SelectedFaceId = SupportFace.Id;
		OutSolid.SupportFaceId = SupportFace.Id;
		OutSolid.SupportGreenChainIndex = Attempt.ChainIndex;
		OutSolid.SourcePlanePoint = Attempt.AnchorWorld;
		OutSolid.SourcePlaneNormal = Attempt.BasePlaneNormal.GetSafeNormal();
		OutSolid.SupportPlanePoint = SupportFace.PlanePoint;
		OutSolid.SupportPlaneNormal = SupportFace.Normal.GetSafeNormal();
		OutSolid.SourceFaceVerticesWorld = SupportFace.KeyPoints3D;
		OutSolid.SourceMaterialProbePointsWorld = SupportFace.KeyPoints3D;
		OutSolid.SourceMaterialProbePointsWorld.Add(AverageVector(SupportFace.KeyPoints3D));
		OutSolid.OrientedNormal = OutSolid.SourcePlaneNormal;
		OutSolid.ExtrusionVectorWorld = Attempt.ExtrusionVectorWorld;
		OutSolid.ExtrusionDepth = Attempt.ExtrusionVectorWorld.Size();
		OutSolid.SourceLoop2D = Attempt.BaseLoop2D;
		OutSolid.SourceLoopWorld = Attempt.BaseLoopWorld;
		OutSolid.SourceToCopiedVector2D = Attempt.ChainEndFaceSpace - Attempt.ChainStartFaceSpace;
		OutSolid.CopiedTargetLoop2D = Attempt.CopiedTargetLoop2D;
		if (OutSolid.CopiedTargetLoop2D.Num() != OutSolid.SourceLoop2D.Num())
		{
			OutSolid.CopiedTargetLoop2D.Reset();
			OutSolid.CopiedTargetLoop2D.Reserve(OutSolid.SourceLoop2D.Num());
			for (const FVector2D& P : OutSolid.SourceLoop2D)
			{
				OutSolid.CopiedTargetLoop2D.Add(P + OutSolid.SourceToCopiedVector2D);
			}
		}
		OutSolid.CopiedLoopWorld = Attempt.CopiedLoopWorld;
		if (OutSolid.CopiedLoopWorld.Num() != OutSolid.SourceLoopWorld.Num())
		{
			OutSolid.CopiedLoopWorld.Reset();
			OutSolid.CopiedLoopWorld.Reserve(OutSolid.SourceLoopWorld.Num());
			for (const FVector& P : OutSolid.SourceLoopWorld)
			{
				OutSolid.CopiedLoopWorld.Add(P + OutSolid.ExtrusionVectorWorld);
			}
		}

		OutSolid.CapBBoxRegularization.bAttempted = Attempt.bOrthogonalAttempted;
		OutSolid.CapBBoxRegularization.bApplied = Attempt.bOrthogonalApplied;
		OutSolid.CapBBoxRegularization.bFallbackToOriginal = Attempt.bOrthogonalFallbackToOriginal;
		OutSolid.CapBBoxRegularization.SelectedFaceId = SupportFace.Id;
		OutSolid.CapBBoxRegularization.Action = TEXT("attach");
		OutSolid.CapBBoxRegularization.SourcePolygonKey = OutSolid.SourcePolygonKey;
		OutSolid.CapBBoxRegularization.CopiedPolygonKey = OutSolid.CopiedPolygonKey;
		OutSolid.CapBBoxRegularization.RejectionReason =
			FString::Printf(TEXT("forced support-plane output from attempt %d: %s"), AttemptIndex, *Attempt.ForceableReason);

		OutSolid.MaxDepthSampleReprojectionErrorPixels =
			FMath::Max(25.0, OutSolid.SourceToCopiedVector2D.Size() * 0.75);
		if (!ProjectSignedWorldDirectionToImage(
			Inputs.Camera,
			Inputs.FacesWidth,
			Inputs.FacesHeight,
			OutSolid.OrientedNormal,
			OutSolid.ProjectedNormal2D))
		{
			OutError = TEXT("forced support base-plane normal projects to a near-zero image direction");
			return false;
		}
		if (FVector2D::DotProduct(
			OutSolid.ProjectedNormal2D,
			OutSolid.SourceToCopiedVector2D.GetSafeNormal()) < 0.0)
		{
			OutSolid.ProjectedNormal2D *= -1.0;
		}

		FString ReprojectionError;
		if (!ReprojectSolidLoops(Inputs, OutSolid, ReprojectionError))
		{
			OutError = ReprojectionError;
			return false;
		}
		if (!BuildSolidMeshTriangles(
			OutSolid.SourceLoop2D,
			OutSolid.CopiedTargetLoop2D,
			OutSolid.SourceLoopWorld,
			OutSolid.CopiedLoopWorld,
			OutSolid.OrientedNormal,
			OutSolid.MeshVerticesWorld,
			OutSolid.MeshTriangles,
			OutSolid.MeshNormal))
		{
			OutError = TEXT("failed to triangulate forced support fallback solid");
			return false;
		}

		OutSolid.Warning = FString::Printf(
			TEXT("forced support-plane fallback output from attempt %d: %s"),
			AttemptIndex,
			*Attempt.ForceableReason);
		OutSolid.bSuccess = true;
		return true;
	}

	static bool TryBuildBestForceableSupportSolid(
		const FString& ComponentName,
		const FString& PressDir,
		int32 CapWidth,
		int32 CapHeight,
		const FCommonInputs& Inputs,
		const FFromLZFaceReconstructionParams& Params,
		const FAttachSupportPlaneFallbackDebug& Debug,
		FSolidReconstructionResult& OutSolid,
		int32& OutAttemptIndex,
		FString& OutReason)
	{
		OutAttemptIndex = SelectBestForceableSupportAttemptIndex(Debug);
		OutReason.Reset();
		if (!Debug.Attempts.IsValidIndex(OutAttemptIndex))
		{
			OutAttemptIndex = INDEX_NONE;
			return false;
		}
		const FAttachSupportPlaneFallbackAttempt& Attempt = Debug.Attempts[OutAttemptIndex];
		OutReason = Attempt.ForceableReason;
		FString BuildError;
		if (!TryBuildForcedSupportSolidFromAttempt(
			ComponentName,
			PressDir,
			CapWidth,
			CapHeight,
			OutAttemptIndex,
			Attempt,
			Inputs,
			Params,
			OutSolid,
			BuildError))
		{
			OutReason = FString::Printf(TEXT("%s; forced rebuild failed: %s"), *Attempt.ForceableReason, *BuildError);
			OutAttemptIndex = INDEX_NONE;
			return false;
		}
		return true;
	}

	static const TArray<FVector2D>& SolidDebugSourceLoop2D(const FSolidReconstructionResult& Solid)
	{
		return Solid.ReprojectedSourceLoop2D.Num() >= 3 ? Solid.ReprojectedSourceLoop2D : Solid.SourceLoop2D;
	}

	static const TArray<FVector2D>& SolidDebugCopiedLoop2D(const FSolidReconstructionResult& Solid)
	{
		return Solid.ReprojectedCopiedLoop2D.Num() >= 3 ? Solid.ReprojectedCopiedLoop2D : Solid.CopiedTargetLoop2D;
	}

	static void SaveAttachPathSelectionDebug(
		const FCommonInputs& Inputs,
		const FSolidReconstructionResult* DirectSolid,
		bool bDirectSuccess,
		const FString& DirectError,
		double DirectFrontDistanceCm,
		double DirectSourceDistanceCm,
		double DirectCopiedDistanceCm,
		const FString& DirectFrontLoopKey,
		const FSolidReconstructionResult* SupportSolid,
		bool bSupportSuccess,
		const FString& SupportError,
		double SupportFrontDistanceCm,
		double SupportSourceDistanceCm,
		double SupportCopiedDistanceCm,
		const FString& SupportFrontLoopKey,
		const FSolidReconstructionResult* ForcedSolid,
		bool bForcedSupportAvailable,
		int32 ForcedAttemptIndex,
		const FString& ForcedReason,
		const FString& ChosenPath,
		const FString& SelectionReason,
		double AttachPathFrontDistanceTieTolCm,
		const FAttachPathPlaneRelationDebug& PlaneRelation,
		const FString& JsonPath,
		const FString& PngPath)
	{
		TSharedRef<FJsonObject> Root = MakeShared<FJsonObject>();
		Root->SetStringField(TEXT("chosen_attach_path"), ChosenPath);
		Root->SetStringField(TEXT("selection_reason"), SelectionReason);
		Root->SetNumberField(TEXT("attach_path_front_distance_tie_tol_cm"), AttachPathFrontDistanceTieTolCm);
		Root->SetBoolField(TEXT("plane_relation_evaluated"), PlaneRelation.bEvaluated);
		Root->SetBoolField(TEXT("plane_relation_valid"), PlaneRelation.bValid);
		Root->SetStringField(TEXT("plane_relation_reason"), PlaneRelation.Reason);
		Root->SetStringField(TEXT("plane_relation_decision_rule"), PlaneRelation.DecisionRule);
		Root->SetNumberField(TEXT("attach_path_plane_relation_angle_tol_deg"), PlaneRelation.AngleTolDeg);
		Root->SetNumberField(TEXT("attach_path_plane_relation_distance_tol_cm"), PlaneRelation.DistanceTolCm);
		Root->SetNumberField(TEXT("support_virtual_base_vs_direct_base_angle_degrees"), PlaneRelation.VirtualBaseVsDirectBaseAngleDeg);
		Root->SetNumberField(TEXT("support_face_vs_direct_base_angle_degrees"), PlaneRelation.SupportFaceVsDirectBaseAngleDeg);
		Root->SetNumberField(TEXT("support_virtual_base_direct_plane_distance_cm"), PlaneRelation.VirtualBaseDirectPlaneDistanceCm);
		Root->SetNumberField(TEXT("support_face_direct_plane_distance_cm"), PlaneRelation.SupportFaceDirectPlaneDistanceCm);
		Root->SetBoolField(TEXT("support_virtual_base_perpendicular_to_direct_base"), PlaneRelation.bVirtualBasePerpendicularToDirectBase);
		Root->SetBoolField(TEXT("support_face_perpendicular_to_direct_base"), PlaneRelation.bSupportFacePerpendicularToDirectBase);
		Root->SetBoolField(TEXT("support_virtual_base_parallel_to_direct_base"), PlaneRelation.bVirtualBaseParallelToDirectBase);
		Root->SetBoolField(TEXT("support_face_parallel_to_direct_base"), PlaneRelation.bSupportFaceParallelToDirectBase);
		Root->SetBoolField(TEXT("support_virtual_base_matches_direct_base"), PlaneRelation.bVirtualBaseMatchesDirectBase);
		Root->SetBoolField(TEXT("support_face_matches_direct_base"), PlaneRelation.bSupportFaceMatchesDirectBase);
		Root->SetBoolField(TEXT("support_face_id_matches_direct_face_id"), PlaneRelation.bSupportFaceIdMatchesDirectFaceId);
		Root->SetArrayField(TEXT("direct_base_point"), JsonVector(PlaneRelation.DirectBasePoint)->AsArray());
		Root->SetArrayField(TEXT("direct_base_normal"), JsonVector(PlaneRelation.DirectBaseNormal)->AsArray());
		Root->SetArrayField(TEXT("support_virtual_base_point"), JsonVector(PlaneRelation.SupportVirtualBasePoint)->AsArray());
		Root->SetArrayField(TEXT("support_virtual_base_normal"), JsonVector(PlaneRelation.SupportVirtualBaseNormal)->AsArray());
		Root->SetArrayField(TEXT("support_face_point"), JsonVector(PlaneRelation.SupportFacePoint)->AsArray());
		Root->SetArrayField(TEXT("support_face_normal"), JsonVector(PlaneRelation.SupportFaceNormal)->AsArray());
		Root->SetBoolField(TEXT("direct_success"), bDirectSuccess);
		Root->SetStringField(TEXT("direct_error"), DirectError);
		Root->SetNumberField(TEXT("direct_front_distance_cm"), DirectFrontDistanceCm);
		Root->SetNumberField(TEXT("direct_source_distance_cm"), DirectSourceDistanceCm);
		Root->SetNumberField(TEXT("direct_copied_distance_cm"), DirectCopiedDistanceCm);
		Root->SetStringField(TEXT("direct_front_loop_key"), DirectFrontLoopKey);
		if (DirectSolid)
		{
			Root->SetStringField(TEXT("direct_reconstruction_method"), DirectSolid->ReconstructionMethod);
			Root->SetNumberField(TEXT("direct_selected_face_id"), DirectSolid->SelectedFaceId);
		}
		Root->SetBoolField(TEXT("support_success"), bSupportSuccess);
		Root->SetStringField(TEXT("support_error"), SupportError);
		Root->SetNumberField(TEXT("support_front_distance_cm"), SupportFrontDistanceCm);
		Root->SetNumberField(TEXT("support_source_distance_cm"), SupportSourceDistanceCm);
		Root->SetNumberField(TEXT("support_copied_distance_cm"), SupportCopiedDistanceCm);
		Root->SetStringField(TEXT("support_front_loop_key"), SupportFrontLoopKey);
		if (SupportSolid)
		{
			Root->SetStringField(TEXT("support_reconstruction_method"), SupportSolid->ReconstructionMethod);
			Root->SetNumberField(TEXT("support_selected_face_id"), SupportSolid->SelectedFaceId);
			Root->SetNumberField(TEXT("support_face_id"), SupportSolid->SupportFaceId);
			Root->SetNumberField(TEXT("support_green_chain_index"), SupportSolid->SupportGreenChainIndex);
		}
		Root->SetBoolField(TEXT("forced_support_available"), bForcedSupportAvailable);
		Root->SetNumberField(TEXT("forced_support_attempt_index"), ForcedAttemptIndex);
		Root->SetStringField(TEXT("forced_support_reason"), ForcedReason);
		if (ForcedSolid)
		{
			Root->SetStringField(TEXT("forced_support_reconstruction_method"), ForcedSolid->ReconstructionMethod);
			Root->SetNumberField(TEXT("forced_support_face_id"), ForcedSolid->SupportFaceId);
			Root->SetNumberField(TEXT("forced_support_green_chain_index"), ForcedSolid->SupportGreenChainIndex);
		}
		SaveJsonObject(Root, JsonPath);

		if (Inputs.FacesRGBA.Num() < Inputs.FacesWidth * Inputs.FacesHeight * 4)
		{
			return;
		}
		TArray<uint8> RGBA = Inputs.FacesRGBA;
		if (DirectSolid)
		{
			DrawClosedPolylineRGBA(RGBA, Inputs.FacesWidth, Inputs.FacesHeight, SolidDebugSourceLoop2D(*DirectSolid), FColor(40, 120, 255, 255), 3);
			DrawClosedPolylineRGBA(RGBA, Inputs.FacesWidth, Inputs.FacesHeight, SolidDebugCopiedLoop2D(*DirectSolid), FColor(160, 80, 255, 255), 2);
		}
		if (SupportSolid)
		{
			DrawClosedPolylineRGBA(RGBA, Inputs.FacesWidth, Inputs.FacesHeight, SolidDebugSourceLoop2D(*SupportSolid), FColor(0, 230, 255, 255), 3);
			DrawClosedPolylineRGBA(RGBA, Inputs.FacesWidth, Inputs.FacesHeight, SolidDebugCopiedLoop2D(*SupportSolid), FColor(255, 150, 0, 255), 2);
		}
		if (ForcedSolid && !bSupportSuccess)
		{
			DrawClosedPolylineRGBA(RGBA, Inputs.FacesWidth, Inputs.FacesHeight, SolidDebugSourceLoop2D(*ForcedSolid), FColor(255, 80, 40, 255), 4);
			DrawClosedPolylineRGBA(RGBA, Inputs.FacesWidth, Inputs.FacesHeight, SolidDebugCopiedLoop2D(*ForcedSolid), FColor(255, 225, 40, 255), 2);
		}

		auto DrawWorldNormalArrow = [&](const FVector& OriginWorld, const FVector& NormalWorld, const FColor& Color, int32 Radius)
		{
			const FVector N = NormalWorld.GetSafeNormal();
			if (N.IsNearlyZero())
			{
				return;
			}
			FVector2D A;
			FVector2D B;
			const double ArrowLengthCm = 60.0;
			if (ProjectWorldToImageOrthographic(Inputs.Camera, Inputs.FacesWidth, Inputs.FacesHeight, OriginWorld, A) &&
				ProjectWorldToImageOrthographic(Inputs.Camera, Inputs.FacesWidth, Inputs.FacesHeight, OriginWorld + N * ArrowLengthCm, B))
			{
				DrawArrowRGBA(RGBA, Inputs.FacesWidth, Inputs.FacesHeight, A, B, Color, Radius);
			}
		};

		auto SourceLoopCentroidOrPlanePoint = [](const FSolidReconstructionResult* Solid) -> FVector
		{
			if (!Solid)
			{
				return FVector::ZeroVector;
			}
			return Solid->SourceLoopWorld.Num() >= 3
				? ComputeWorldLoopAreaCentroid(Solid->SourceLoopWorld)
				: Solid->SourcePlanePoint;
		};

		auto SupportFaceCentroidOrPlanePoint = [](const FSolidReconstructionResult* Solid) -> FVector
		{
			if (!Solid)
			{
				return FVector::ZeroVector;
			}
			return Solid->SourceFaceVerticesWorld.Num() > 0
				? AverageVector(Solid->SourceFaceVerticesWorld)
				: Solid->SupportPlanePoint;
		};

		if (DirectSolid)
		{
			DrawWorldNormalArrow(
				SourceLoopCentroidOrPlanePoint(DirectSolid),
				DirectSolid->SourcePlaneNormal,
				FColor(40, 120, 255, 255),
				3);
		}
		if (SupportSolid)
		{
			DrawWorldNormalArrow(
				SourceLoopCentroidOrPlanePoint(SupportSolid),
				SupportSolid->SourcePlaneNormal,
				FColor(255, 40, 220, 255),
				3);
			DrawWorldNormalArrow(
				SupportFaceCentroidOrPlanePoint(SupportSolid),
				SupportSolid->SupportPlaneNormal,
				FColor(255, 255, 255, 255),
				3);
		}

		const FSolidReconstructionResult* ChosenSolid = nullptr;
		if (ChosenPath == TEXT("direct"))
		{
			ChosenSolid = DirectSolid;
		}
		else if (ChosenPath == TEXT("support_plane"))
		{
			ChosenSolid = SupportSolid;
		}
		else if (ChosenPath == TEXT("forced_support_plane"))
		{
			ChosenSolid = ForcedSolid;
		}
		if (ChosenSolid)
		{
			DrawClosedPolylineRGBA(RGBA, Inputs.FacesWidth, Inputs.FacesHeight, SolidDebugSourceLoop2D(*ChosenSolid), FColor(40, 255, 40, 255), 5);
		}
		SaveRGBAToPng(RGBA, Inputs.FacesWidth, Inputs.FacesHeight, PngPath);
	}

	static FComponentResult ProcessComponent(
		const FString& ComponentName,
		const FString& PressDir,
		const FString& ActionPressDir,
		const FCommonInputs& Inputs,
		const FFromLZFaceReconstructionParams& Params)
	{
		const FString ComponentDir = PressDir / ComponentName;
		FComponentResult Result;
		Result.ComponentName = ComponentName;
		Result.FacesWidth = Inputs.FacesWidth;
		Result.FacesHeight = Inputs.FacesHeight;
		Result.ActorName = FString::Printf(TEXT("FromLZ_ReconstructedFace_%s_%s"), *FPaths::GetCleanFilename(PressDir), *ComponentName);
		InitializeSolidResult(Result.Solid, ComponentName, PressDir, Inputs);

		const FString OutputJson = ComponentDir / TEXT("10_face_reconstruction.json");
		const FString SolidJson = ComponentDir / TEXT("10_solid_reconstruction.json");
		auto SaveFaceAndSkippedSolid = [&](const FString& SolidError)
		{
			Result.Solid.Action = Result.Action;

			Result.Solid.SelectedFaceId = Result.SelectedFaceId;
			Result.Solid.CapWidth = Result.CapWidth;
			Result.Solid.CapHeight = Result.CapHeight;
			Result.Solid.SourcePolygonKey = Result.PolygonKey;
			SaveComponentResultJson(Result, OutputJson);
			PrepareWorldOrthogonalNotAttemptedDebug(
				Result.Solid.CapBBoxRegularization,
				SolidError);
			Result.Solid.CapBBoxRegularization.SelectedFaceId = Result.SelectedFaceId;
			Result.Solid.CapBBoxRegularization.Action = Result.Action;
			Result.Solid.CapBBoxRegularization.SourcePolygonKey = Result.PolygonKey;
			SaveCapBBoxRegularizationDebug(
				Result.Solid.CapBBoxRegularization,
				ComponentDir);
			SaveSkippedSolidResult(Result.Solid, SolidJson, SolidError);
		};

		const FString ActionPath = ActionPressDir / ComponentName / TEXT("Action.json");
		if (!LoadAction(ActionPath, Result.Action))
		{
			Result.Error = FString::Printf(TEXT("Failed to read action from %s"), *ActionPath);
			SaveFaceAndSkippedSolid(FString::Printf(TEXT("Solid skipped because face reconstruction failed: %s"), *Result.Error));
			return Result;
		}
		Result.Solid.Action = Result.Action;
		if (Result.Action.Equals(TEXT("skip"), ESearchCase::IgnoreCase) || Result.Action.Equals(TEXT("undetermined"), ESearchCase::IgnoreCase))
		{
			Result.Error = FString::Printf(TEXT("Component skipped by Step 9 action decision: %s"), *Result.Action);
			SaveFaceAndSkippedSolid(FString::Printf(TEXT("Solid skipped because Step 9 action decision is %s"), *Result.Action));
			return Result;
		}


		if (Result.Action == TEXT("excavate"))
		{
			Result.PolygonKey = TEXT("cap_polygon");
			Result.Solid.SourcePolygonKey = TEXT("cap_polygon");
			Result.Solid.CopiedPolygonKey = TEXT("cap_polygon_translated");
		}
		else if (Result.Action == TEXT("attach"))
		{
			Result.PolygonKey = TEXT("cap_polygon_translated");
			Result.Solid.SourcePolygonKey = TEXT("cap_polygon_translated");
			Result.Solid.CopiedPolygonKey = TEXT("cap_polygon");
		}
		else
		{
			Result.Error = FString::Printf(TEXT("Unsupported action '%s'"), *Result.Action);
			SaveFaceAndSkippedSolid(FString::Printf(TEXT("Solid skipped because face reconstruction failed: %s"), *Result.Error));
			return Result;
		}

		TSharedPtr<FJsonObject> CapJson;
		const FString CapJsonPath = ComponentDir / TEXT("09_cap_extrusion.json");
		if (!LoadJsonObject(CapJsonPath, CapJson))
		{
			Result.Error = FString::Printf(TEXT("Failed to read %s"), *CapJsonPath);
			SaveFaceAndSkippedSolid(FString::Printf(TEXT("Solid skipped because face reconstruction failed: %s"), *Result.Error));
			return Result;
		}

		TArray<FVector2D> RawCapPolygon;
		TArray<FVector2D> RawTranslatedPolygon;
		TArray<FCapBoundaryRunInput> BoundaryRuns;
		const bool bHasCapPolygon = ParseVector2DArray(CapJson, TEXT("cap_polygon"), RawCapPolygon);
		const bool bHasTranslatedPolygon = ParseVector2DArray(CapJson, TEXT("cap_polygon_translated"), RawTranslatedPolygon);
		ParseOrderedBoundaryRuns(CapJson, BoundaryRuns);
		double PreselectedFaceIdNumber = -1.0;
		CapJson->TryGetNumberField(TEXT("preselected_face_id"), PreselectedFaceIdNumber);
		const int32 PreselectedFaceId = FMath::RoundToInt(PreselectedFaceIdNumber);
		const TArray<FVector2D>* SourcePolyForFace = Result.PolygonKey == TEXT("cap_polygon") ? &RawCapPolygon : &RawTranslatedPolygon;
		if (!SourcePolyForFace || SourcePolyForFace->Num() < 3 ||
			(Result.PolygonKey == TEXT("cap_polygon") && !bHasCapPolygon) ||
			(Result.PolygonKey == TEXT("cap_polygon_translated") && !bHasTranslatedPolygon))
		{
			Result.Error = FString::Printf(TEXT("Missing or invalid polygon '%s' in %s"), *Result.PolygonKey, *CapJsonPath);
			SaveFaceAndSkippedSolid(FString::Printf(TEXT("Solid skipped because face reconstruction failed: %s"), *Result.Error));
			return Result;
		}
		const TArray<FVector2D>& CapPoly = *SourcePolyForFace;

		FVector2D SideVector = FVector2D::ZeroVector;
		if (!ParseVector2DField(CapJson, TEXT("side_vector"), SideVector))
		{
			Result.Error = FString::Printf(TEXT("Missing or invalid side_vector in %s"), *CapJsonPath);
			SaveFaceAndSkippedSolid(FString::Printf(TEXT("Solid skipped because face reconstruction failed: %s"), *Result.Error));
			return Result;
		}

		TArray<uint8> CapRGBA;
		const FString CapPngPath = ComponentDir / TEXT("09_cap_extrusion.png");
		if (!DecodePngToRGBA(CapPngPath, CapRGBA, Result.CapWidth, Result.CapHeight))
		{
			Result.Error = FString::Printf(TEXT("Failed to read cap image size from %s"), *CapPngPath);
			SaveFaceAndSkippedSolid(FString::Printf(TEXT("Solid skipped because face reconstruction failed: %s"), *Result.Error));
			return Result;
		}
		Result.Solid.CapWidth = Result.CapWidth;
		Result.Solid.CapHeight = Result.CapHeight;

		const double ScaleX = double(Inputs.FacesWidth) / double(Result.CapWidth);
		const double ScaleY = double(Inputs.FacesHeight) / double(Result.CapHeight);
		const double DirectMinOverlapRatio = FMath::Max(0.0, double(Params.CandidateFaceMinOverlapRatio));
		const double DirectMaxNormalSideAngleDegrees = FMath::Max(0.0, double(Params.CandidateFaceMaxNormalSideAngleDegrees));
		const double DirectPreferredNormalAngleDegrees = FMath::Max(0.0, double(Params.CandidateFacePreferredNormalSideAngleDegrees));
		Result.CandidateFaceMinOverlapRatioUsed = DirectMinOverlapRatio;
		Result.NormalParallelThresholdDegreesUsed = DirectMaxNormalSideAngleDegrees;
		Result.PreferredNormalAngleThresholdDegreesUsed = DirectPreferredNormalAngleDegrees;
		const FVector2D ScaledSideVector(SideVector.X * ScaleX, SideVector.Y * ScaleY);
		if (ScaledSideVector.SizeSquared() < 1e-8)
		{
			Result.Error = TEXT("side_vector is too short after mapping to faces image space");
			SaveFaceAndSkippedSolid(FString::Printf(TEXT("Solid skipped because face reconstruction failed: %s"), *Result.Error));
			return Result;
		}
		Result.GreenLineVector2D = ScaledSideVector.GetSafeNormal();

		// Selected green side vector(s) mapped to faces image space. The parallel
		// filter below passes a face if its normal aligns with any provided vector.
		// Scaling is per-axis (ScaleX != ScaleY rotates the vector), so map then normalize.
		TArray<FVector2D> RawSideVectors;
		ParseVector2DArray(CapJson, TEXT("side_vectors"), RawSideVectors);
		for (const FVector2D& V : RawSideVectors)
		{
			const FVector2D Scaled(V.X * ScaleX, V.Y * ScaleY);
			if (Scaled.SizeSquared() >= 1e-8)
			{
				Result.GreenLineVectors2D.Add(Scaled.GetSafeNormal());
			}
		}
		// Fall back to the single side_vector when the array is absent (older cap json).
		if (Result.GreenLineVectors2D.Num() == 0)
		{
			Result.GreenLineVectors2D.Add(Result.GreenLineVector2D);
		}

		// Endpoint segments (faces space) for the debug overlay.
		TArray<FVector2D> RawSegStarts, RawSegEnds;
		if (ParseSegment2DArray(CapJson, TEXT("side_segments"), RawSegStarts, RawSegEnds))
		{
			for (int32 i = 0; i < RawSegStarts.Num(); ++i)
			{
				Result.GreenSegStarts.Emplace(RawSegStarts[i].X * ScaleX, RawSegStarts[i].Y * ScaleY);
				Result.GreenSegEnds.Emplace(RawSegEnds[i].X * ScaleX, RawSegEnds[i].Y * ScaleY);
			}
		}
		TArray<FFromLZGreenChainCandidate2D> GreenChains;
		ParseGreenChainCandidates(CapJson, GreenChains);
		if (GreenChains.Num() == 0)
		{
			const int32 LegacyCount = FMath::Min(RawSegStarts.Num(), RawSegEnds.Num());
			for (int32 i = 0; i < LegacyCount; ++i)
			{
				FFromLZGreenChainCandidate2D Chain;
				Chain.SeedStrokeId = i;
				Chain.Start = RawSegStarts[i];
				Chain.End = RawSegEnds[i];
				Chain.Vector = Chain.End - Chain.Start;
				Chain.ChordLength = Chain.Vector.Size();
				Chain.PathLength = Chain.ChordLength;
				Chain.StopReason = TEXT("legacy_side_segments_fallback");
				GreenChains.Add(MoveTemp(Chain));
			}
		}

		TArray<FVector2D> FaceSpacePoly;
		FaceSpacePoly.Reserve(CapPoly.Num());
		for (const FVector2D& P : CapPoly)
		{
			FaceSpacePoly.Emplace(P.X * ScaleX, P.Y * ScaleY);
		}

		TArray<uint8> Mask;
		RasterizePolygonMask(FaceSpacePoly, Inputs.FacesWidth, Inputs.FacesHeight, Mask, Result.CapMaskPixels);
		Result.MinOverlapPixels = FMath::Max(
			1,
			FMath::CeilToInt(
				double(Result.CapMaskPixels) *
				DirectMinOverlapRatio));
		SaveMaskPng(Mask, Inputs.FacesWidth, Inputs.FacesHeight, ComponentDir / TEXT("10_cap_mask.png"));
		if (Result.CapMaskPixels <= 0)
		{
			Result.Error = TEXT("Cap mask is empty after mapping to faces image space");
			SaveFaceAndSkippedSolid(FString::Printf(TEXT("Solid skipped because face reconstruction failed: %s"), *Result.Error));
			return Result;
		}

		TMap<int32, FOverlapAccum> AccumByFace;
		for (int32 y = 0; y < Inputs.FacesHeight; ++y)
		{
			for (int32 x = 0; x < Inputs.FacesWidth; ++x)
			{
				const int32 PixIdx = y * Inputs.FacesWidth + x;
				if (Mask[PixIdx] == 0)
				{
					continue;
				}
				const int32 Off = PixIdx * 4;
				const uint32 Key = ColorKey(Inputs.FacesRGBA[Off + 0], Inputs.FacesRGBA[Off + 1], Inputs.FacesRGBA[Off + 2]);
				if (const int32* FaceId = Inputs.FaceIdByColorKey.Find(Key))
				{
					FOverlapAccum& Acc = AccumByFace.FindOrAdd(*FaceId);
					Acc.Pixels += 1;
					Acc.SumX += double(x) + 0.5;
					Acc.SumY += double(y) + 0.5;
				}
			}
		}

		for (const TPair<int32, FOverlapAccum>& Pair : AccumByFace)
		{
			if (Pair.Value.Pixels < Result.MinOverlapPixels)
			{
				continue;
			}
			const int32* FaceIndex = Inputs.FaceIndexById.Find(Pair.Key);
			if (!FaceIndex)
			{
				continue;
			}

			const FFaceInfo& Face = Inputs.Faces[*FaceIndex];
			FFaceCandidate Candidate;
			Candidate.FaceId = Pair.Key;
			Candidate.OverlapPixels = Pair.Value.Pixels;
			Candidate.OverlapRatio = double(Pair.Value.Pixels) / double(Result.CapMaskPixels);
			Candidate.MaskCentroid = FVector2D(Pair.Value.SumX / double(Pair.Value.Pixels), Pair.Value.SumY / double(Pair.Value.Pixels));
			Candidate.bHasPlaneHit = IntersectMaskCentroidWithFacePlane(
				Inputs.Camera, Inputs.FacesWidth, Inputs.FacesHeight,
				Candidate.MaskCentroid, Face, Candidate.PlaneHit, Candidate.DistanceToCamera);
			if (Candidate.bHasPlaneHit)
			{
				Candidate.bHasProjectedNormal = ProjectFaceNormalToImage(
					Inputs.Camera, Inputs.FacesWidth, Inputs.FacesHeight,
					Face, Candidate.ProjectedNormal2D);
				if (Candidate.bHasProjectedNormal)
				{
					// Test the projected normal against every cap-connected green line.
					// Direct attach reconstructs from the translated/source cap back to
					// the raw/copied cap, so its expected normal is opposite the green
					// chain direction.
					const FVector2D NormalDir = Candidate.ProjectedNormal2D.GetSafeNormal();
					double BestAngle = 180.0;
					FVector2D BestOrientedNormalDir = NormalDir;
					const bool bAttachNormalCheck = Result.Action.Equals(TEXT("attach"), ESearchCase::IgnoreCase);
					for (const FVector2D& Green : Result.GreenLineVectors2D)
					{
						const FVector2D GreenDir = Green.GetSafeNormal();
						if (GreenDir.IsNearlyZero())
						{
							continue;
						}
						const FVector2D ExpectedNormalDir = bAttachNormalCheck ? -GreenDir : GreenDir;
						const double SignedDot = FVector2D::DotProduct(NormalDir, ExpectedNormalDir);
						const double Dot = FMath::Clamp(FMath::Abs(SignedDot), 0.0, 1.0);
						const double Angle = FMath::RadiansToDegrees(FMath::Acos(Dot));
						if (Angle < BestAngle)
						{
							BestAngle = Angle;
							BestOrientedNormalDir = SignedDot >= 0.0 ? NormalDir : -NormalDir;
						}
					}
					Candidate.ProjectedNormal2D = BestOrientedNormalDir;
					Candidate.RawProjectedNormal2D = NormalDir;
					Candidate.OrientedProjectedNormal2D = BestOrientedNormalDir;
					Candidate.NormalGreenAngleDegrees = BestAngle;
					Candidate.bNormalParallelPass = BestAngle <= DirectMaxNormalSideAngleDegrees;
				}
			}
			Result.Candidates.Add(Candidate);
		}

		Result.Candidates.Sort([](const FFaceCandidate& A, const FFaceCandidate& B)
		{
			return A.FaceId < B.FaceId;
		});

		int32 PreferredFaceId = -1;
		FVector PreferredPlaneHit = FVector::ZeroVector;
		double BestPreferredDistance = TNumericLimits<double>::Max();
		int32 FallbackFaceId = -1;
		FVector FallbackPlaneHit = FVector::ZeroVector;
		double BestFallbackDistance = TNumericLimits<double>::Max();
		for (const FFaceCandidate& Candidate : Result.Candidates)
		{
			if (!Candidate.bHasPlaneHit || !Candidate.bNormalParallelPass)
			{
				continue;
			}

			if (Candidate.DistanceToCamera < BestFallbackDistance)
			{
				BestFallbackDistance = Candidate.DistanceToCamera;
				FallbackFaceId = Candidate.FaceId;
				FallbackPlaneHit = Candidate.PlaneHit;
			}

			if (Candidate.NormalGreenAngleDegrees < DirectPreferredNormalAngleDegrees &&
				Candidate.DistanceToCamera < BestPreferredDistance)
			{
				BestPreferredDistance = Candidate.DistanceToCamera;
				PreferredFaceId = Candidate.FaceId;
				PreferredPlaneHit = Candidate.PlaneHit;
			}
		}
		if (PreferredFaceId >= 0)
		{
			Result.SelectedFaceId = PreferredFaceId;
			Result.SelectedPlaneHit = PreferredPlaneHit;
		}
		else if (FallbackFaceId >= 0)
		{
			Result.SelectedFaceId = FallbackFaceId;
			Result.SelectedPlaneHit = FallbackPlaneHit;
		}

		TSet<int32> CandidateIds;
		for (const FFaceCandidate& Candidate : Result.Candidates)
		{
			CandidateIds.Add(Candidate.FaceId);
		}
		TSet<int32> ParallelFaceIds;
		for (const FFaceCandidate& Candidate : Result.Candidates)
		{
			if (Candidate.bNormalParallelPass)
			{
				ParallelFaceIds.Add(Candidate.FaceId);
			}
		}
		SaveOverlapPng(
			Inputs.FacesRGBA, Mask, Inputs.FaceIdByColorKey, CandidateIds, ParallelFaceIds, Result.SelectedFaceId,
			Inputs.FacesWidth, Inputs.FacesHeight, ComponentDir / TEXT("10_face_overlap.png"));

		// Visualize the cap-connected green lines vs each candidate face's projected normal.
		SaveNormalGreenCheckPng(
			Inputs.FacesRGBA, Inputs.FacesWidth, Inputs.FacesHeight,
			FaceSpacePoly, Result.GreenSegStarts, Result.GreenSegEnds,
			Result.Candidates, Result.SelectedFaceId,
			ComponentDir / TEXT("10_normal_green_check.png"));

		const bool bIsAttachAction = Result.Action.Equals(TEXT("attach"), ESearchCase::IgnoreCase);
		FAttachSupportPlaneFallbackDebug SupportFallbackDebug;
		FSolidReconstructionResult SupportFallbackSolid;
		FSolidReconstructionResult ForcedSupportFallbackSolid;
		bool bSupportFallbackSuccess = false;
		bool bForcedSupportFallbackAvailable = false;
		int32 ForcedSupportAttemptIndex = INDEX_NONE;
		FString ForcedSupportReason;
		if (bIsAttachAction)
		{
			bSupportFallbackSuccess = TryBuildAttachSupportPlaneFallback(
				ComponentName,
				PressDir,
				ComponentDir,
				Result.CapWidth,
				Result.CapHeight,
				ScaleX,
				ScaleY,
				RawCapPolygon,
				BoundaryRuns,
				GreenChains,
				Inputs,
				Params,
				SupportFallbackSolid,
				SupportFallbackDebug);
			bForcedSupportFallbackAvailable = TryBuildBestForceableSupportSolid(
				ComponentName,
				PressDir,
				Result.CapWidth,
				Result.CapHeight,
				Inputs,
				Params,
				SupportFallbackDebug,
				ForcedSupportFallbackSolid,
				ForcedSupportAttemptIndex,
				ForcedSupportReason);
		}

		auto SaveSupportFallbackDebugState = [&]()
		{
			if (!bIsAttachAction)
			{
				return;
			}
			TArray<FVector2D> RawCapLoopFaceSpace;
			MapPolygonToFacesSpace(RawCapPolygon, ScaleX, ScaleY, RawCapLoopFaceSpace);
			SaveAttachSupportPlaneFallbackDebug(
				SupportFallbackDebug,
				Inputs,
				RawCapLoopFaceSpace,
				ComponentDir / TEXT("10_attach_support_plane_fallback.json"),
				ComponentDir / TEXT("10_attach_support_plane_fallback.png"));
		};

		auto ApplySupportSolidToResult = [&](const FSolidReconstructionResult& ChosenSolid, const FString& Warning)
		{
			Result.SelectedFaceId = ChosenSolid.SelectedFaceId;
			Result.SelectedPlaneHit = ChosenSolid.SourcePlanePoint;
			Result.Solid = ChosenSolid;
			Result.Solid.Warning = Warning;
			Result.PolygonKey = Result.Solid.SourcePolygonKey;

			if (const int32* SupportIndex = Inputs.FaceIndexById.Find(Result.SelectedFaceId))
			{
				const FFaceInfo& SupportFace = Inputs.Faces[*SupportIndex];
				Result.MeshVerticesWorld = SupportFace.KeyPoints3D;
				Result.MeshTriangles.Reset();
				TriangulatePolygon2D(SupportFace.KeyPoints2D, Result.MeshTriangles);
				Result.MeshNormal = ComputeTriangleNormal(Result.MeshVerticesWorld, Result.MeshTriangles);
				if (Result.MeshNormal.IsNearlyZero())
				{
					Result.MeshNormal = SupportFace.Normal;
				}

				if (Result.Solid.bSuccess && HasActorMaterialIdBuffer(Inputs))
				{
					FSolidReconstructionResult MaterialProbe = Result.Solid;
					MaterialProbe.SourceLoop2D = SupportFace.KeyPoints2D;
					MaterialProbe.SelectedFaceId = SupportFace.Id;
					SelectAttachMaterialFromIdBuffer(Inputs, MaterialProbe, Result.Solid.AttachMaterialId);
				}
			}
		};

		auto SaveFinalSolidOutputs = [&]()
		{
			if (Result.Solid.CapBBoxRegularization.WorldOrthogonalStages.Num() == 0 &&
				!Result.Solid.CapBBoxRegularization.bAttempted)
			{
				PrepareWorldOrthogonalNotAttemptedDebug(
					Result.Solid.CapBBoxRegularization,
					Result.Solid.Error.IsEmpty()
						? TEXT("orthogonal regularization was not reached")
						: Result.Solid.Error);
			}
			SaveComponentResultJson(Result, OutputJson);
			SaveCapBBoxRegularizationDebug(
				Result.Solid.CapBBoxRegularization,
				ComponentDir);
			SaveSolidResultJson(Result.Solid, SolidJson);
			SaveSolidProjectionCheckPng(
				Result.Solid, Inputs.FacesRGBA, Inputs.FacesWidth, Inputs.FacesHeight,
				ComponentDir / TEXT("10_solid_projection_check.png"));
		};

		auto FinalizeAttachPathSelection = [&](bool bDirectSuccess, const FSolidReconstructionResult* DirectSolid, const FString& DirectFailureReason) -> bool
		{
			if (!bIsAttachAction)
			{
				return false;
			}

			double DirectFrontDistanceCm = 0.0;
			double DirectSourceDistanceCm = 0.0;
			double DirectCopiedDistanceCm = 0.0;
			FString DirectFrontLoopKey;
			const bool bDirectDistanceValid = DirectSolid &&
				ComputeSolidFrontCapDistanceCm(
					*DirectSolid,
					Inputs,
					DirectFrontDistanceCm,
					DirectSourceDistanceCm,
					DirectCopiedDistanceCm,
					DirectFrontLoopKey);

			double SupportFrontDistanceCm = 0.0;
			double SupportSourceDistanceCm = 0.0;
			double SupportCopiedDistanceCm = 0.0;
			FString SupportFrontLoopKey;
			const bool bSupportDistanceValid = bSupportFallbackSuccess &&
				ComputeSolidFrontCapDistanceCm(
					SupportFallbackSolid,
					Inputs,
					SupportFrontDistanceCm,
					SupportSourceDistanceCm,
					SupportCopiedDistanceCm,
					SupportFrontLoopKey);

			FAttachPathPlaneRelationDebug PlaneRelation;
			if (bDirectSuccess && bSupportFallbackSuccess && DirectSolid)
			{
				PlaneRelation = AnalyzeAttachPathPlaneRelation(*DirectSolid, SupportFallbackSolid, Params);
			}

			FString ChosenPath = TEXT("none");
			FString SelectionReason;
			if (bDirectSuccess && bSupportFallbackSuccess)
			{
				if (!DirectSolid)
				{
					ChosenPath = TEXT("support_plane");
					PlaneRelation.DecisionRule = TEXT("support_selected_direct_solid_missing");
					SelectionReason = TEXT("support-plane fallback selected because direct attach reported success without a direct solid result");
				}
				else if (!PlaneRelation.bValid)
				{
					ChosenPath = TEXT("direct");
					PlaneRelation.DecisionRule = TEXT("direct_selected_plane_relation_invalid");
					SelectionReason = FString::Printf(
						TEXT("direct selected because plane relation was invalid: %s"),
						*PlaneRelation.Reason);
				}
				else if (PlaneRelation.bSupportFaceMatchesDirectBase)
				{
					ChosenPath = TEXT("direct");
					PlaneRelation.DecisionRule = TEXT("direct_selected_support_face_matches_direct_base");
					SelectionReason = FString::Printf(
						TEXT("direct selected because support face matches direct base face (face id match=%s, support/direct angle %.6f deg, support/direct plane distance %.6f cm)"),
						PlaneRelation.bSupportFaceIdMatchesDirectFaceId ? TEXT("true") : TEXT("false"),
						PlaneRelation.SupportFaceVsDirectBaseAngleDeg,
						PlaneRelation.SupportFaceDirectPlaneDistanceCm);
				}
				else if (PlaneRelation.bVirtualBasePerpendicularToDirectBase)
				{
					ChosenPath = TEXT("direct");
					PlaneRelation.DecisionRule = TEXT("direct_selected_virtual_base_perpendicular_to_direct_base");
					SelectionReason = FString::Printf(
						TEXT("direct selected because support virtual base is perpendicular to direct base (virtual/direct angle %.6f deg, tol %.6f deg)"),
						PlaneRelation.VirtualBaseVsDirectBaseAngleDeg,
						PlaneRelation.AngleTolDeg);
				}
				else if (PlaneRelation.bSupportFacePerpendicularToDirectBase)
				{
					if (bDirectDistanceValid &&
						bSupportDistanceValid &&
						SupportFrontDistanceCm + double(Params.AttachPathFrontDistanceTieTolCm) < DirectFrontDistanceCm)
					{
						ChosenPath = TEXT("support_plane");
						PlaneRelation.DecisionRule = TEXT("support_selected_support_face_perpendicular_and_front_closer");
						SelectionReason = FString::Printf(
							TEXT("support selected because support face is perpendicular to direct base and support front cap is closer to camera: %.6f cm vs direct %.6f cm, tie tol %.6f cm"),
							SupportFrontDistanceCm,
							DirectFrontDistanceCm,
							double(Params.AttachPathFrontDistanceTieTolCm));
					}
					else
					{
						ChosenPath = TEXT("direct");
						PlaneRelation.DecisionRule = TEXT("direct_selected_support_face_perpendicular_distance_tie_or_invalid");
						SelectionReason = FString::Printf(
							TEXT("direct selected because support face is perpendicular to direct base but support is not closer beyond tie tolerance: direct %.6f cm (valid=%s), support %.6f cm (valid=%s), tie tol %.6f cm"),
							DirectFrontDistanceCm,
							bDirectDistanceValid ? TEXT("true") : TEXT("false"),
							SupportFrontDistanceCm,
							bSupportDistanceValid ? TEXT("true") : TEXT("false"),
							double(Params.AttachPathFrontDistanceTieTolCm));
					}
				}
				else
				{
					ChosenPath = TEXT("direct");
					PlaneRelation.DecisionRule = TEXT("direct_selected_support_face_not_perpendicular_to_direct_base");
					SelectionReason = FString::Printf(
						TEXT("direct selected because support face is not perpendicular to direct base (support/direct angle %.6f deg, tol %.6f deg; virtual/direct angle %.6f deg)"),
						PlaneRelation.SupportFaceVsDirectBaseAngleDeg,
						PlaneRelation.AngleTolDeg,
						PlaneRelation.VirtualBaseVsDirectBaseAngleDeg);
				}
			}
			else if (bSupportFallbackSuccess)
			{
				ChosenPath = TEXT("support_plane");
				PlaneRelation.DecisionRule = TEXT("support_selected_direct_failed_or_unavailable");
				SelectionReason = DirectFailureReason.IsEmpty()
					? TEXT("support-plane fallback succeeded while direct attach was unavailable")
					: FString::Printf(TEXT("direct attach failed: %s; support-plane fallback succeeded"), *DirectFailureReason);
			}
			else if (bDirectSuccess)
			{
				ChosenPath = TEXT("direct");
				PlaneRelation.DecisionRule = TEXT("direct_selected_support_failed");
				SelectionReason = SupportFallbackDebug.Error.IsEmpty()
					? TEXT("direct attach succeeded and support-plane fallback did not produce a successful candidate")
					: FString::Printf(TEXT("direct attach succeeded; support-plane fallback failed: %s"), *SupportFallbackDebug.Error);
			}
			else if (bForcedSupportFallbackAvailable)
			{
				ChosenPath = TEXT("forced_support_plane");
				PlaneRelation.DecisionRule = TEXT("forced_support_selected_both_regular_paths_failed");
				SelectionReason = DirectFailureReason.IsEmpty()
					? FString::Printf(TEXT("forced support-plane output from attempt %d: %s"), ForcedSupportAttemptIndex, *ForcedSupportReason)
					: FString::Printf(TEXT("direct attach failed: %s; forced support-plane output from attempt %d: %s"),
						*DirectFailureReason,
						ForcedSupportAttemptIndex,
						*ForcedSupportReason);
			}
			else
			{
				PlaneRelation.DecisionRule = TEXT("none_selected_both_paths_failed");
				SelectionReason = DirectFailureReason.IsEmpty()
					? FString::Printf(TEXT("direct and support-plane attach both failed; support error: %s"), *SupportFallbackDebug.Error)
					: FString::Printf(TEXT("direct attach failed: %s; support-plane fallback failed: %s"),
						*DirectFailureReason,
						*SupportFallbackDebug.Error);
			}

			SaveAttachPathSelectionDebug(
				Inputs,
				DirectSolid,
				bDirectSuccess,
				DirectFailureReason,
				DirectFrontDistanceCm,
				DirectSourceDistanceCm,
				DirectCopiedDistanceCm,
				DirectFrontLoopKey,
				bSupportFallbackSuccess ? &SupportFallbackSolid : nullptr,
				bSupportFallbackSuccess,
				SupportFallbackDebug.Error,
				SupportFrontDistanceCm,
				SupportSourceDistanceCm,
				SupportCopiedDistanceCm,
				SupportFrontLoopKey,
				bForcedSupportFallbackAvailable ? &ForcedSupportFallbackSolid : nullptr,
				bForcedSupportFallbackAvailable,
				ForcedSupportAttemptIndex,
				ForcedSupportReason,
				ChosenPath,
				SelectionReason,
				double(Params.AttachPathFrontDistanceTieTolCm),
				PlaneRelation,
				ComponentDir / TEXT("10_attach_path_selection.json"),
				ComponentDir / TEXT("10_attach_path_selection.png"));

			Result.AttachPathMode = TEXT("direct_and_support_evaluated");
			Result.ChosenAttachPath = ChosenPath;
			Result.AttachPathSelectionReason = SelectionReason;
			if (ChosenPath == TEXT("support_plane"))
			{
				ApplySupportSolidToResult(
					SupportFallbackSolid,
					DirectFailureReason.IsEmpty()
						? TEXT("support-plane fallback selected over direct attach")
						: FString::Printf(TEXT("direct attach failed: %s; used support-plane fallback"), *DirectFailureReason));
			}
			else if (ChosenPath == TEXT("forced_support_plane"))
			{
				SupportFallbackDebug.bForcedOutput = true;
				SupportFallbackDebug.ForcedAttemptIndex = ForcedSupportAttemptIndex;
				SupportFallbackDebug.ForcedReason = ForcedSupportReason;
				SaveSupportFallbackDebugState();
				ApplySupportSolidToResult(
					ForcedSupportFallbackSolid,
					FString::Printf(TEXT("forced support-plane fallback output: %s"), *ForcedSupportReason));
			}
			else if (ChosenPath == TEXT("direct") && DirectSolid)
			{
				Result.Solid = *DirectSolid;
			}
			else
			{
				return false;
			}

			Result.Error.Reset();
			Result.bSuccess = true;
			SaveFinalSolidOutputs();
			return true;
		};

		auto TryAttachSupportPlaneFallbackAndSave = [&](const FString& DirectFailureReason) -> bool
		{
			return FinalizeAttachPathSelection(false, nullptr, DirectFailureReason);
		};

		if (Result.SelectedFaceId < 0)
		{
			if (Result.Candidates.Num() == 0)
			{
				Result.Error = FString::Printf(
					TEXT("No face survived the %.1f%% candidate mask-overlap threshold"),
					DirectMinOverlapRatio * 100.0);
			}
			else if (ParallelFaceIds.Num() == 0)
			{
				Result.Error = FString::Printf(
					TEXT("No %.1f%% mask-overlap candidate passed the %.1f degree normal-to-green-line parallel filter"),
					DirectMinOverlapRatio * 100.0,
					DirectMaxNormalSideAngleDegrees);
			}
			else
			{
				Result.Error = TEXT("No parallel face candidate had a valid camera-to-plane intersection");
			}
			if (TryAttachSupportPlaneFallbackAndSave(Result.Error))
			{
				return Result;
			}
			SaveFaceAndSkippedSolid(FString::Printf(TEXT("Solid skipped because Step 10 did not select a source face: %s"), *Result.Error));
			return Result;
		}
		if (PreselectedFaceId < 0)
		{
			Result.Error = TEXT("Step 9 did not provide a valid preselected_face_id");
			if (TryAttachSupportPlaneFallbackAndSave(Result.Error))
			{
				return Result;
			}
			SaveFaceAndSkippedSolid(FString::Printf(TEXT("Solid skipped because Step 9/10 face selection validation failed: %s"), *Result.Error));
			return Result;
		}
		if (Result.SelectedFaceId != PreselectedFaceId)
		{
			Result.Error = FString::Printf(
				TEXT("Step 10 selected face %d but Step 9 preselected face %d"),
				Result.SelectedFaceId,
				PreselectedFaceId);
			if (TryAttachSupportPlaneFallbackAndSave(Result.Error))
			{
				return Result;
			}
			SaveFaceAndSkippedSolid(FString::Printf(TEXT("Solid skipped because Step 9/10 face selection validation failed: %s"), *Result.Error));
			return Result;
		}

		const int32* SelectedIndex = Inputs.FaceIndexById.Find(Result.SelectedFaceId);
		if (!SelectedIndex)
		{
			Result.Error = TEXT("Selected face id was not found in face table");
			if (TryAttachSupportPlaneFallbackAndSave(Result.Error))
			{
				return Result;
			}
			SaveFaceAndSkippedSolid(FString::Printf(TEXT("Solid skipped because face reconstruction failed: %s"), *Result.Error));
			return Result;
		}

		const FFaceInfo& SelectedFace = Inputs.Faces[*SelectedIndex];
		Result.MeshVerticesWorld = SelectedFace.KeyPoints3D;
		if (!TriangulatePolygon2D(SelectedFace.KeyPoints2D, Result.MeshTriangles))
		{
			Result.Error = TEXT("Failed to triangulate selected face");
			if (TryAttachSupportPlaneFallbackAndSave(Result.Error))
			{
				return Result;
			}
			SaveFaceAndSkippedSolid(FString::Printf(TEXT("Solid skipped because face reconstruction failed: %s"), *Result.Error));
			return Result;
		}

		FVector Center = FVector::ZeroVector;
		for (const FVector& V : Result.MeshVerticesWorld)
		{
			Center += V;
		}
		Center /= double(FMath::Max(1, Result.MeshVerticesWorld.Num()));

		Result.MeshNormal = ComputeTriangleNormal(Result.MeshVerticesWorld, Result.MeshTriangles);
		if (Result.MeshNormal.IsNearlyZero())
		{
			Result.MeshNormal = SelectedFace.Normal;
		}
		if (FVector::DotProduct(Result.MeshNormal, Inputs.Camera.Location - Center) < 0.0)
		{
			for (int32 i = 0; i + 2 < Result.MeshTriangles.Num(); i += 3)
			{
				Swap(Result.MeshTriangles[i + 1], Result.MeshTriangles[i + 2]);
			}
			Result.MeshNormal *= -1.0;
		}

		Result.bSuccess = true;
		SaveComponentResultJson(Result, OutputJson);
		if (!bHasCapPolygon || !bHasTranslatedPolygon)
		{
			const FString SolidError =
				TEXT("Solid skipped because cap_polygon or cap_polygon_translated is missing from 09_cap_extrusion.json");
			if (bHasCapPolygon && TryAttachSupportPlaneFallbackAndSave(SolidError))
			{
				return Result;
			}
			PrepareWorldOrthogonalNotAttemptedDebug(
				Result.Solid.CapBBoxRegularization,
				SolidError);
			Result.Solid.CapBBoxRegularization.SelectedFaceId = Result.SelectedFaceId;
			Result.Solid.CapBBoxRegularization.Action = Result.Action;
			Result.Solid.CapBBoxRegularization.SourcePolygonKey = Result.Solid.SourcePolygonKey;
			Result.Solid.CapBBoxRegularization.CopiedPolygonKey = Result.Solid.CopiedPolygonKey;
			SaveCapBBoxRegularizationDebug(
				Result.Solid.CapBBoxRegularization,
				ComponentDir);
			SaveSkippedSolidResult(
				Result.Solid,
				SolidJson,
				SolidError);
			return Result;
		}

		const TArray<FVector2D>& SolidSourcePoly = Result.Action == TEXT("excavate") ? RawCapPolygon : RawTranslatedPolygon;
		const TArray<FVector2D>& SolidCopiedPoly = Result.Action == TEXT("excavate") ? RawTranslatedPolygon : RawCapPolygon;
		const FVector2D SourceRunTranslation =
			Result.Action == TEXT("attach") ? SideVector : FVector2D::ZeroVector;

		Result.Solid = BuildSolidReconstruction(
			ComponentName,
			Result.Action,
			Result.Solid.SourcePolygonKey,
			Result.Solid.CopiedPolygonKey,
			PressDir,
			ComponentDir,
			Result.CapWidth,
			Result.CapHeight,
			ScaleX,
			ScaleY,
			SolidSourcePoly,
			SolidCopiedPoly,
			BoundaryRuns,
			SourceRunTranslation,
			SelectedFace,
			Inputs,
			Params);
		if (!Result.Solid.bSuccess && TryAttachSupportPlaneFallbackAndSave(Result.Solid.Error))
		{
			return Result;
		}
		if (Result.Solid.bSuccess && Result.Action.Equals(TEXT("attach"), ESearchCase::IgnoreCase) && HasActorMaterialIdBuffer(Inputs))
		{
			SelectAttachMaterialFromIdBuffer(Inputs, Result.Solid, Result.Solid.AttachMaterialId);
		}
		if (bIsAttachAction)
		{
			if (FinalizeAttachPathSelection(Result.Solid.bSuccess, &Result.Solid, Result.Solid.Error))
			{
				return Result;
			}
		}
		if (Result.Solid.CapBBoxRegularization.WorldOrthogonalStages.Num() == 0 &&
			!Result.Solid.CapBBoxRegularization.bAttempted)
		{
			PrepareWorldOrthogonalNotAttemptedDebug(
				Result.Solid.CapBBoxRegularization,
				Result.Solid.Error.IsEmpty()
					? TEXT("orthogonal regularization was not reached")
					: Result.Solid.Error);
		}
		SaveCapBBoxRegularizationDebug(
			Result.Solid.CapBBoxRegularization,
			ComponentDir);
		SaveSolidResultJson(Result.Solid, SolidJson);
		SaveSolidProjectionCheckPng(
			Result.Solid, Inputs.FacesRGBA, Inputs.FacesWidth, Inputs.FacesHeight,
			ComponentDir / TEXT("10_solid_projection_check.png"));
		return Result;
	}

	static FString ObjSafeName(FString Name)
	{
		Name.TrimStartAndEndInline();
		for (TCHAR& Ch : Name)
		{
			if (!FChar::IsAlnum(Ch) && Ch != TCHAR('_') && Ch != TCHAR('-'))
			{
				Ch = TCHAR('_');
			}
		}
		return Name.IsEmpty() ? TEXT("Mesh") : Name;
	}

	static FName Step11GeneratedByPressTag(const FString& PressId)
	{
		return FName(*FString::Printf(TEXT("FromLZ_GeneratedBy_%s"), *PressId));
	}

	static FName Step11HiddenByPressTag(const FString& PressId)
	{
		return FName(*FString::Printf(TEXT("FromLZ_HiddenBy_%s"), *PressId));
	}

	static bool ComponentHasAnyStep11HiddenByTag(const UActorComponent* Component)
	{
		if (!Component)
		{
			return false;
		}

		static const FString HiddenByPrefix(TEXT("FromLZ_HiddenBy_Press_"));
		for (const FName& Tag : Component->ComponentTags)
		{
			if (Tag.ToString().StartsWith(HiddenByPrefix, ESearchCase::CaseSensitive))
			{
				return true;
			}
		}
		return false;
	}

	static bool ComponentHasActiveStep11GeneratedByTag(const UActorComponent* Component)
	{
		if (!Component)
		{
			return false;
		}

		for (const FString& PressId : GActiveStep11PressStack)
		{
			if (Component->ComponentTags.Contains(Step11GeneratedByPressTag(PressId)))
			{
				return true;
			}
		}
		return false;
	}

	static void RefreshActiveUndoPressId()
	{
		GActiveUndoPressId = GActiveStep11PressStack.Num() > 0 ? GActiveStep11PressStack.Last() : FString();
	}

	static void RegisterActiveStep11Press(const FString& PressId)
	{
		if (PressId.IsEmpty())
		{
			return;
		}

		GActiveStep11PressStack.Remove(PressId);
		GActiveStep11PressStack.Add(PressId);
		RefreshActiveUndoPressId();
	}

	static FString PopActiveStep11Press()
	{
		if (GActiveStep11PressStack.Num() == 0)
		{
			RefreshActiveUndoPressId();
			return FString();
		}

		const FString PressId = GActiveStep11PressStack.Last();
		GActiveStep11PressStack.RemoveAt(GActiveStep11PressStack.Num() - 1);
		RefreshActiveUndoPressId();
		return PressId;
	}

	static bool ActorHasActiveStep11GeneratedByTag(const AActor* Actor)
	{
		if (!Actor)
		{
			return false;
		}

		for (const FString& PressId : GActiveStep11PressStack)
		{
			if (Actor->ActorHasTag(Step11GeneratedByPressTag(PressId)))
			{
				return true;
			}
		}
		return false;
	}
	static bool ActorHasAnyStep11RuntimeTag(const AActor* Actor)
	{
		return Actor && (
			Actor->ActorHasTag(Step11BooleanResultTag) ||
			Actor->ActorHasTag(Step11ActionAttachTag) ||
			Actor->ActorHasTag(Step11ActionExcavateCutterTag));
	}

	static bool ActorIsStep11Cutter(const AActor* Actor)
	{
		return Actor && Actor->ActorHasTag(Step11ActionExcavateCutterTag);
	}

	static bool ActorIsTaggedBaseCaptureSubject(const AActor* Actor)
	{
		return Actor &&
			Actor->ActorHasTag(FromLZCaptureSubjectTag) &&
			!Actor->ActorHasTag(FromLZCapturePlaneTag);
	}

	static void MergeStep11StringSet(TSet<FString>& Dest, const TSet<FString>& Source)
	{
		for (const FString& Value : Source)
		{
			Dest.Add(Value);
		}
	}

	static void AppendDoubleSidedTriangles(const TArray<int32>& Triangles, TArray<int32>& OutTriangles)
	{
		OutTriangles.Reset();
		OutTriangles.Reserve(Triangles.Num() * 2);
		for (int32 i = 0; i + 2 < Triangles.Num(); i += 3)
		{
			const int32 A = Triangles[i];
			const int32 B = Triangles[i + 1];
			const int32 C = Triangles[i + 2];
			OutTriangles.Add(A);
			OutTriangles.Add(B);
			OutTriangles.Add(C);
			OutTriangles.Add(A);
			OutTriangles.Add(C);
			OutTriangles.Add(B);
		}
	}

	static UMaterialInterface* GetReconstructionVertexColorMaterial()
	{
		static TWeakObjectPtr<UMaterialInterface> CachedMaterial;
		if (UMaterialInterface* Cached = CachedMaterial.Get())
		{
			return Cached;
		}

		const TCHAR* MaterialPaths[] =
		{
			TEXT("/Engine/EngineDebugMaterials/VertexColorViewMode_ColorOnly.VertexColorViewMode_ColorOnly"),
			TEXT("/Engine/EngineMaterials/VertexColorMaterial.VertexColorMaterial")
		};
		for (const TCHAR* Path : MaterialPaths)
		{
			if (UMaterialInterface* Material = LoadObject<UMaterialInterface>(nullptr, Path))
			{
				CachedMaterial = Material;
				return Material;
			}
		}

		if (GEngine && GEngine->VertexColorMaterial)
		{
			CachedMaterial = GEngine->VertexColorMaterial;
			return GEngine->VertexColorMaterial;
		}

		return UMaterial::GetDefaultMaterial(MD_Surface);
	}

	static UMaterialInterface* GetReconstructionCutterBaseMaterial()
	{
		static TWeakObjectPtr<UMaterialInterface> CachedMaterial;
		if (UMaterialInterface* Cached = CachedMaterial.Get())
		{
			return Cached;
		}

		const TCHAR* MaterialPaths[] =
		{
			TEXT("/Engine/EngineDebugMaterials/M_SimpleUnlitTranslucent.M_SimpleUnlitTranslucent"),
			TEXT("/Engine/EngineDebugMaterials/M_SimpleTranslucent.M_SimpleTranslucent"),
			TEXT("/Engine/EngineMaterials/Widget3DPassThrough_Translucent.Widget3DPassThrough_Translucent")
		};
		for (const TCHAR* Path : MaterialPaths)
		{
			if (UMaterialInterface* Material = LoadObject<UMaterialInterface>(nullptr, Path))
			{
				CachedMaterial = Material;
				return Material;
			}
		}

		return GetReconstructionVertexColorMaterial();
	}

	static UMaterialInterface* CreateCutterMaterial(UObject* Outer)
	{
		UMaterialInterface* BaseMaterial = GetReconstructionCutterBaseMaterial();
		UMaterialInstanceDynamic* DynamicMaterial = UMaterialInstanceDynamic::Create(BaseMaterial, Outer);
		if (!DynamicMaterial)
		{
			return BaseMaterial;
		}

		const FLinearColor CutterColor(0.0f, 0.47f, 1.0f, 0.25f);
		const FName ColorParameterNames[] =
		{
			TEXT("Color"),
			TEXT("BaseColor"),
			TEXT("Tint"),
			TEXT("TintColor"),
			TEXT("EmissiveColor")
		};
		for (const FName& ParameterName : ColorParameterNames)
		{
			DynamicMaterial->SetVectorParameterValue(ParameterName, CutterColor);
		}

		const FName OpacityParameterNames[] =
		{
			TEXT("Opacity"),
			TEXT("Alpha"),
			TEXT("Transparency")
		};
		for (const FName& ParameterName : OpacityParameterNames)
		{
			DynamicMaterial->SetScalarParameterValue(ParameterName, CutterColor.A);
		}
		return DynamicMaterial;
	}

	static FBox BuildWorldBounds(const TArray<FVector>& Vertices)
	{
		FBox Box(ForceInit);
		for (const FVector& V : Vertices)
		{
			Box += V;
		}
		return Box;
	}

	static FBox BuildDynamicMeshBounds(const UE::Geometry::FDynamicMesh3& Mesh)
	{
		FBox Box(ForceInit);
		for (int32 VertexId : Mesh.VertexIndicesItr())
		{
			const FVector3d V = Mesh.GetVertex(VertexId);
			Box += FVector(V.X, V.Y, V.Z);
		}
		return Box;
	}

	static FString Step11EdgeKey(int32 A, int32 B)
	{
		if (A > B)
		{
			Swap(A, B);
		}
		return FString::Printf(TEXT("%d_%d"), A, B);
	}

	static FString Step11DirectedEdgeKey(int32 A, int32 B)
	{
		return FString::Printf(TEXT("%d_%d"), A, B);
	}

	static FString Step11TriangleKey(int32 A, int32 B, int32 C)
	{
		TArray<int32> Indices;
		Indices.Add(A);
		Indices.Add(B);
		Indices.Add(C);
		Indices.Sort();
		return FString::Printf(TEXT("%d_%d_%d"), Indices[0], Indices[1], Indices[2]);
	}

	static void AddStep11EdgeDiagnostics(
		int32 A,
		int32 B,
		double EdgeLength,
		TMap<FString, int32>& UndirectedEdgeCounts,
		TMap<FString, int32>& DirectedEdgeCounts,
		TMap<FString, double>& EdgeLengthByKey)
	{
		const FString UndirectedKey = Step11EdgeKey(A, B);
		UndirectedEdgeCounts.FindOrAdd(UndirectedKey) += 1;
		DirectedEdgeCounts.FindOrAdd(Step11DirectedEdgeKey(A, B)) += 1;
		if (!EdgeLengthByKey.Contains(UndirectedKey))
		{
			EdgeLengthByKey.Add(UndirectedKey, EdgeLength);
		}
	}

	static FStep11MeshDiagnostics AnalyzeStep11DynamicMesh(
		const UE::Geometry::FDynamicMesh3& Mesh,
		const FString& Label,
		const FString& SourceType)
	{
		constexpr double DegenerateAreaTolerance = 1e-8;
		constexpr double TinyAreaTolerance = 1e-4;

		FStep11MeshDiagnostics Diagnostics;
		Diagnostics.Label = Label;
		Diagnostics.SourceType = SourceType;
		Diagnostics.VertexCount = Mesh.VertexCount();
		Diagnostics.TriangleCount = Mesh.TriangleCount();
		Diagnostics.Bounds = BuildDynamicMeshBounds(Mesh);

		TMap<FString, int32> UndirectedEdgeCounts;
		TMap<FString, int32> DirectedEdgeCounts;
		TMap<FString, double> EdgeLengthByKey;
		TSet<FString> TriangleKeys;

		double MinTriangleArea = TNumericLimits<double>::Max();
		double MaxTriangleArea = 0.0;
		for (int32 TriangleId : Mesh.TriangleIndicesItr())
		{
			const UE::Geometry::FIndex3i Tri = Mesh.GetTriangle(TriangleId);
			if (Tri.A == Tri.B || Tri.B == Tri.C || Tri.C == Tri.A)
			{
				++Diagnostics.InvalidTriangleCount;
				continue;
			}

			const FString TriKey = Step11TriangleKey(Tri.A, Tri.B, Tri.C);
			if (TriangleKeys.Contains(TriKey))
			{
				++Diagnostics.DuplicateTriangleCount;
			}
			else
			{
				TriangleKeys.Add(TriKey);
			}

			const FVector3d A3 = Mesh.GetVertex(Tri.A);
			const FVector3d B3 = Mesh.GetVertex(Tri.B);
			const FVector3d C3 = Mesh.GetVertex(Tri.C);
			const FVector A(A3.X, A3.Y, A3.Z);
			const FVector B(B3.X, B3.Y, B3.Z);
			const FVector C(C3.X, C3.Y, C3.Z);
			const double Area = 0.5 * FVector::CrossProduct(B - A, C - A).Size();
			Diagnostics.SurfaceArea += Area;
			MinTriangleArea = FMath::Min(MinTriangleArea, Area);
			MaxTriangleArea = FMath::Max(MaxTriangleArea, Area);
			if (Area <= DegenerateAreaTolerance)
			{
				++Diagnostics.DegenerateTriangleCount;
			}
			else if (Area <= TinyAreaTolerance)
			{
				++Diagnostics.TinyTriangleCount;
			}

			Diagnostics.SignedVolume += FVector::DotProduct(A, FVector::CrossProduct(B, C)) / 6.0;

			AddStep11EdgeDiagnostics(Tri.A, Tri.B, FVector::Distance(A, B), UndirectedEdgeCounts, DirectedEdgeCounts, EdgeLengthByKey);
			AddStep11EdgeDiagnostics(Tri.B, Tri.C, FVector::Distance(B, C), UndirectedEdgeCounts, DirectedEdgeCounts, EdgeLengthByKey);
			AddStep11EdgeDiagnostics(Tri.C, Tri.A, FVector::Distance(C, A), UndirectedEdgeCounts, DirectedEdgeCounts, EdgeLengthByKey);
		}

		Diagnostics.EdgeCount = UndirectedEdgeCounts.Num();
		for (const TPair<FString, int32>& Pair : UndirectedEdgeCounts)
		{
			if (Pair.Value == 1)
			{
				++Diagnostics.BoundaryEdgeCount;
			}
			else if (Pair.Value > 2)
			{
				++Diagnostics.NonManifoldEdgeCount;
			}

			TArray<FString> Parts;
			Pair.Key.ParseIntoArray(Parts, TEXT("_"), true);
			if (Parts.Num() == 2)
			{
				const int32 A = FCString::Atoi(*Parts[0]);
				const int32 B = FCString::Atoi(*Parts[1]);
				const int32 AB = DirectedEdgeCounts.FindRef(Step11DirectedEdgeKey(A, B));
				const int32 BA = DirectedEdgeCounts.FindRef(Step11DirectedEdgeKey(B, A));
				if (Pair.Value == 2 && !(AB == 1 && BA == 1))
				{
					++Diagnostics.InconsistentOrientationEdgeCount;
				}
			}
		}

		double EdgeLengthSum = 0.0;
		double MinEdgeLength = TNumericLimits<double>::Max();
		double MaxEdgeLength = 0.0;
		for (const TPair<FString, double>& Pair : EdgeLengthByKey)
		{
			EdgeLengthSum += Pair.Value;
			MinEdgeLength = FMath::Min(MinEdgeLength, Pair.Value);
			MaxEdgeLength = FMath::Max(MaxEdgeLength, Pair.Value);
		}

		Diagnostics.MinTriangleArea = MinTriangleArea == TNumericLimits<double>::Max() ? 0.0 : MinTriangleArea;
		Diagnostics.MaxTriangleArea = MaxTriangleArea;
		Diagnostics.AbsVolume = FMath::Abs(Diagnostics.SignedVolume);
		Diagnostics.SignedVolumeBeforeOrientationFix = Diagnostics.SignedVolume;
		Diagnostics.SignedVolumeAfterOrientationFix = Diagnostics.SignedVolume;
		Diagnostics.MinEdgeLength = MinEdgeLength == TNumericLimits<double>::Max() ? 0.0 : MinEdgeLength;
		Diagnostics.MaxEdgeLength = MaxEdgeLength;
		Diagnostics.MeanEdgeLength = EdgeLengthByKey.Num() > 0 ? EdgeLengthSum / double(EdgeLengthByKey.Num()) : 0.0;
		return Diagnostics;
	}

	static bool NormalizeStep11MeshOrientationForBoolean(
		UE::Geometry::FDynamicMesh3& Mesh,
		FStep11MeshDiagnostics& Diagnostics)
	{
		constexpr double SignedVolumeTolerance = 1e-6;
		const double SignedVolumeBefore = Diagnostics.SignedVolume;
		if (SignedVolumeBefore >= -SignedVolumeTolerance)
		{
			Diagnostics.SignedVolumeBeforeOrientationFix = SignedVolumeBefore;
			Diagnostics.SignedVolumeAfterOrientationFix = SignedVolumeBefore;
			Diagnostics.bOrientationReversedForBoolean = false;
			return false;
		}

		const FString Label = Diagnostics.Label;
		const FString SourceType = Diagnostics.SourceType;
		Mesh.ReverseOrientation(false);
		Diagnostics = AnalyzeStep11DynamicMesh(Mesh, Label, SourceType);
		Diagnostics.SignedVolumeBeforeOrientationFix = SignedVolumeBefore;
		Diagnostics.SignedVolumeAfterOrientationFix = Diagnostics.SignedVolume;
		Diagnostics.bOrientationReversedForBoolean = true;

		UE_LOG(
			LogTemp,
			Log,
			TEXT("Step11Diag: reversed mesh orientation for boolean label=%s source=%s signed_volume_before=%.6f signed_volume_after=%.6f."),
			*Diagnostics.Label,
			*Diagnostics.SourceType,
			SignedVolumeBefore,
			Diagnostics.SignedVolume);
		return true;
	}

	static int32 Step11SignedVolumeSign(double SignedVolume)
	{
		constexpr double SignedVolumeTolerance = 1e-6;
		if (SignedVolume > SignedVolumeTolerance)
		{
			return 1;
		}
		if (SignedVolume < -SignedVolumeTolerance)
		{
			return -1;
		}
		return 0;
	}

	static bool ReverseStep11MeshIfSignMismatch(UE::Geometry::FDynamicMesh3& Mesh, int32 DesiredSign)
	{
		if (DesiredSign == 0)
		{
			return false;
		}

		const FStep11MeshDiagnostics Diagnostics = AnalyzeStep11DynamicMesh(Mesh, TEXT("orientation_check"), TEXT("orientation_check"));
		const int32 CurrentSign = Step11SignedVolumeSign(Diagnostics.SignedVolume);
		if (CurrentSign != 0 && CurrentSign != DesiredSign)
		{
			Mesh.ReverseOrientation(false);
			return true;
		}
		return false;
	}

	struct FStep11BooleanAttemptResult
	{
		bool bComputeSuccess = false;
		bool bAccepted = false;
		bool bEmptyResult = false;
		bool bTargetReversedForPass = false;
		bool bCutterReversedForPass = false;
		bool bResultReversedToTargetSign = false;
		FString PassName;
		FString Status;
		FString RejectReason;
		double VolumeDelta = 0.0;
		double MinRequiredVolumeDelta = 0.0;
		FStep11MeshDiagnostics ResultDiagnostics;
		FFromLZManifoldBooleanDiagnostics ManifoldDiagnostics;
		UE::Geometry::FDynamicMesh3 ResultMesh;
		TArray<int8> TriangleSourceMeshById;
	};

	static TSharedRef<FJsonObject> Step11BoxJson(const FBox& Box)
	{
		TSharedRef<FJsonObject> Object = MakeShared<FJsonObject>();
		Object->SetBoolField(TEXT("valid"), Box.IsValid != 0);
		if (Box.IsValid)
		{
			Object->SetArrayField(TEXT("min"), JsonVector(Box.Min)->AsArray());
			Object->SetArrayField(TEXT("max"), JsonVector(Box.Max)->AsArray());
			Object->SetArrayField(TEXT("center"), JsonVector(Box.GetCenter())->AsArray());
			Object->SetArrayField(TEXT("extent"), JsonVector(Box.GetExtent())->AsArray());
			Object->SetArrayField(TEXT("size"), JsonVector(Box.GetSize())->AsArray());
			Object->SetNumberField(TEXT("diagonal"), Box.GetSize().Size());
		}
		return Object;
	}

	static TSharedRef<FJsonObject> Step11MeshDiagnosticsJson(const FStep11MeshDiagnostics& Diagnostics)
	{
		TSharedRef<FJsonObject> Object = MakeShared<FJsonObject>();
		Object->SetStringField(TEXT("label"), Diagnostics.Label);
		Object->SetStringField(TEXT("source_type"), Diagnostics.SourceType);
		Object->SetNumberField(TEXT("vertex_count"), Diagnostics.VertexCount);
		Object->SetNumberField(TEXT("triangle_count"), Diagnostics.TriangleCount);
		Object->SetNumberField(TEXT("edge_count"), Diagnostics.EdgeCount);
		Object->SetNumberField(TEXT("boundary_edge_count"), Diagnostics.BoundaryEdgeCount);
		Object->SetNumberField(TEXT("non_manifold_edge_count"), Diagnostics.NonManifoldEdgeCount);
		Object->SetNumberField(TEXT("inconsistent_orientation_edge_count"), Diagnostics.InconsistentOrientationEdgeCount);
		Object->SetNumberField(TEXT("invalid_triangle_count"), Diagnostics.InvalidTriangleCount);
		Object->SetNumberField(TEXT("degenerate_triangle_count"), Diagnostics.DegenerateTriangleCount);
		Object->SetNumberField(TEXT("tiny_triangle_count"), Diagnostics.TinyTriangleCount);
		Object->SetNumberField(TEXT("duplicate_triangle_count"), Diagnostics.DuplicateTriangleCount);
		Object->SetNumberField(TEXT("surface_area"), Diagnostics.SurfaceArea);
		Object->SetNumberField(TEXT("min_triangle_area"), Diagnostics.MinTriangleArea);
		Object->SetNumberField(TEXT("max_triangle_area"), Diagnostics.MaxTriangleArea);
		Object->SetNumberField(TEXT("signed_volume"), Diagnostics.SignedVolume);
		Object->SetNumberField(TEXT("abs_volume"), Diagnostics.AbsVolume);
		Object->SetBoolField(TEXT("orientation_reversed_for_boolean"), Diagnostics.bOrientationReversedForBoolean);
		Object->SetNumberField(TEXT("signed_volume_before_orientation_fix"), Diagnostics.SignedVolumeBeforeOrientationFix);
		Object->SetNumberField(TEXT("signed_volume_after_orientation_fix"), Diagnostics.SignedVolumeAfterOrientationFix);
		Object->SetNumberField(TEXT("min_edge_length"), Diagnostics.MinEdgeLength);
		Object->SetNumberField(TEXT("max_edge_length"), Diagnostics.MaxEdgeLength);
		Object->SetNumberField(TEXT("mean_edge_length"), Diagnostics.MeanEdgeLength);
		Object->SetNumberField(TEXT("euler_characteristic"), Diagnostics.VertexCount - Diagnostics.EdgeCount + Diagnostics.TriangleCount);
		Object->SetBoolField(TEXT("closed_two_manifold"), Diagnostics.BoundaryEdgeCount == 0 && Diagnostics.NonManifoldEdgeCount == 0);
		Object->SetObjectField(TEXT("bounds"), Step11BoxJson(Diagnostics.Bounds));
		return Object;
	}

	static TSharedRef<FJsonObject> Step11BooleanAttemptJson(const FStep11BooleanAttemptResult& Attempt)
	{
		TSharedRef<FJsonObject> Object = MakeShared<FJsonObject>();
		Object->SetStringField(TEXT("pass_name"), Attempt.PassName);
		Object->SetStringField(TEXT("boolean_backend"), Attempt.ManifoldDiagnostics.BooleanBackend);
		Object->SetStringField(TEXT("manifold_library_version"), Attempt.ManifoldDiagnostics.LibraryVersion);
		Object->SetStringField(TEXT("status"), Attempt.Status);
		Object->SetStringField(TEXT("reject_reason"), Attempt.RejectReason);
		Object->SetBoolField(TEXT("compute_success"), Attempt.bComputeSuccess);
		Object->SetBoolField(TEXT("accepted"), Attempt.bAccepted);
		Object->SetBoolField(TEXT("empty_result"), Attempt.bEmptyResult);
		Object->SetBoolField(TEXT("target_reversed_for_pass"), Attempt.bTargetReversedForPass);
		Object->SetBoolField(TEXT("cutter_reversed_for_pass"), Attempt.bCutterReversedForPass);
		Object->SetBoolField(TEXT("result_reversed_to_target_sign"), Attempt.bResultReversedToTargetSign);
		Object->SetBoolField(TEXT("target_manifold_input_valid"), Attempt.ManifoldDiagnostics.bTargetManifoldInputValid);
		Object->SetBoolField(TEXT("cutter_manifold_input_valid"), Attempt.ManifoldDiagnostics.bCutterManifoldInputValid);
		Object->SetBoolField(TEXT("manifold_difference_success"), Attempt.ManifoldDiagnostics.bManifoldDifferenceSuccess);
		Object->SetStringField(TEXT("manifold_error_message"), Attempt.ManifoldDiagnostics.ManifoldErrorMessage);
		Object->SetNumberField(TEXT("target_input_triangles"), Attempt.ManifoldDiagnostics.TargetInputTriangles);
		Object->SetNumberField(TEXT("cutter_input_triangles"), Attempt.ManifoldDiagnostics.CutterInputTriangles);
		Object->SetNumberField(TEXT("result_output_triangles"), Attempt.ManifoldDiagnostics.ResultOutputTriangles);
		Object->SetBoolField(TEXT("target_orientation_reversed_for_manifold"), Attempt.ManifoldDiagnostics.bTargetOrientationReversedForManifold);
		Object->SetBoolField(TEXT("cutter_orientation_reversed_for_manifold"), Attempt.ManifoldDiagnostics.bCutterOrientationReversedForManifold);
		Object->SetNumberField(TEXT("target_signed_volume_for_manifold"), Attempt.ManifoldDiagnostics.TargetSignedVolumeForManifold);
		Object->SetNumberField(TEXT("cutter_signed_volume_for_manifold"), Attempt.ManifoldDiagnostics.CutterSignedVolumeForManifold);
		Object->SetNumberField(TEXT("result_signed_volume_before_render_fix"), Attempt.ManifoldDiagnostics.ResultSignedVolumeBeforeRenderFix);
		Object->SetBoolField(TEXT("cap_section_unavailable"), Attempt.ManifoldDiagnostics.bCapSectionUnavailable);
		Object->SetNumberField(TEXT("volume_delta"), Attempt.VolumeDelta);
		Object->SetNumberField(TEXT("min_required_volume_delta"), Attempt.MinRequiredVolumeDelta);
		if (Attempt.bComputeSuccess && !Attempt.bEmptyResult)
		{
			Object->SetObjectField(TEXT("result"), Step11MeshDiagnosticsJson(Attempt.ResultDiagnostics));
		}
		return Object;
	}

	static void LogStep11MeshDiagnostics(const FStep11MeshDiagnostics& Diagnostics)
	{
		const FVector BoundsMin = Diagnostics.Bounds.IsValid ? Diagnostics.Bounds.Min : FVector::ZeroVector;
		const FVector BoundsMax = Diagnostics.Bounds.IsValid ? Diagnostics.Bounds.Max : FVector::ZeroVector;
		UE_LOG(
			LogTemp,
			Log,
			TEXT("Step11Diag: mesh label=%s source=%s vertices=%d triangles=%d edges=%d boundary=%d nonmanifold=%d inconsistent_orientation=%d invalid_triangles=%d degenerate=%d tiny=%d duplicate=%d area=%.6f signed_volume=%.6f abs_volume=%.6f orientation_reversed=%d signed_before=%.6f signed_after=%.6f min_edge=%.6f max_edge=%.6f mean_edge=%.6f bounds_min=(%.3f, %.3f, %.3f) bounds_max=(%.3f, %.3f, %.3f) closed_two_manifold=%d"),
			*Diagnostics.Label,
			*Diagnostics.SourceType,
			Diagnostics.VertexCount,
			Diagnostics.TriangleCount,
			Diagnostics.EdgeCount,
			Diagnostics.BoundaryEdgeCount,
			Diagnostics.NonManifoldEdgeCount,
			Diagnostics.InconsistentOrientationEdgeCount,
			Diagnostics.InvalidTriangleCount,
			Diagnostics.DegenerateTriangleCount,
			Diagnostics.TinyTriangleCount,
			Diagnostics.DuplicateTriangleCount,
			Diagnostics.SurfaceArea,
			Diagnostics.SignedVolume,
			Diagnostics.AbsVolume,
			Diagnostics.bOrientationReversedForBoolean ? 1 : 0,
			Diagnostics.SignedVolumeBeforeOrientationFix,
			Diagnostics.SignedVolumeAfterOrientationFix,
			Diagnostics.MinEdgeLength,
			Diagnostics.MaxEdgeLength,
			Diagnostics.MeanEdgeLength,
			BoundsMin.X,
			BoundsMin.Y,
			BoundsMin.Z,
			BoundsMax.X,
			BoundsMax.Y,
			BoundsMax.Z,
			Diagnostics.BoundaryEdgeCount == 0 && Diagnostics.NonManifoldEdgeCount == 0 ? 1 : 0);
	}

	static FString QuantizedVertexKey(const FVector& Position)
	{
		constexpr double WeldTolerance = 0.01;
		const int64 X = FMath::RoundToInt64(Position.X / WeldTolerance);
		const int64 Y = FMath::RoundToInt64(Position.Y / WeldTolerance);
		const int64 Z = FMath::RoundToInt64(Position.Z / WeldTolerance);
		return FString::Printf(TEXT("%lld_%lld_%lld"), X, Y, Z);
	}

	static bool BuildDynamicMeshFromStaticMeshComponent(UStaticMeshComponent* Component, UE::Geometry::FDynamicMesh3& OutMesh, bool bRequireVisible = true)
	{
		OutMesh = UE::Geometry::FDynamicMesh3();
		if (!Component || !Component->IsRegistered() || (bRequireVisible && !Component->IsVisible()))
		{
			return false;
		}

		UStaticMesh* StaticMesh = Component->GetStaticMesh();
		if (!StaticMesh || !StaticMesh->GetRenderData() || StaticMesh->GetRenderData()->LODResources.Num() == 0)
		{
			return false;
		}

		const FStaticMeshLODResources& LOD = StaticMesh->GetRenderData()->LODResources[0];
		const FPositionVertexBuffer& PositionBuffer = LOD.VertexBuffers.PositionVertexBuffer;
		const int32 NumRenderVertices = PositionBuffer.GetNumVertices();
		const FIndexArrayView Indices = LOD.IndexBuffer.GetArrayView();
		if (NumRenderVertices <= 0 || Indices.Num() < 3)
		{
			return false;
		}

		const FTransform& ComponentTransform = Component->GetComponentTransform();
		TMap<FString, int32> VertexIdByPosition;
		TArray<int32> RenderToDynamic;
		RenderToDynamic.Init(UE::Geometry::FDynamicMesh3::InvalidID, NumRenderVertices);
		for (int32 RenderVertexIndex = 0; RenderVertexIndex < NumRenderVertices; ++RenderVertexIndex)
		{
			const FVector3f LocalPosition = PositionBuffer.VertexPosition(RenderVertexIndex);
			const FVector WorldPosition = ComponentTransform.TransformPosition(
				FVector(double(LocalPosition.X), double(LocalPosition.Y), double(LocalPosition.Z)));
			const FString Key = QuantizedVertexKey(WorldPosition);
			if (const int32* ExistingId = VertexIdByPosition.Find(Key))
			{
				RenderToDynamic[RenderVertexIndex] = *ExistingId;
			}
			else
			{
				const int32 DynamicId = OutMesh.AppendVertex(FVector3d(WorldPosition.X, WorldPosition.Y, WorldPosition.Z));
				VertexIdByPosition.Add(Key, DynamicId);
				RenderToDynamic[RenderVertexIndex] = DynamicId;
			}
		}

		int32 AddedTriangles = 0;
		for (const FStaticMeshSection& Section : LOD.Sections)
		{
			for (uint32 TriIndex = 0; TriIndex < Section.NumTriangles; ++TriIndex)
			{
				const uint32 IndexBase = Section.FirstIndex + TriIndex * 3;
				if (IndexBase + 2 >= uint32(Indices.Num()))
				{
					continue;
				}

				const int32 A = int32(Indices[IndexBase]);
				const int32 B = int32(Indices[IndexBase + 1]);
				const int32 C = int32(Indices[IndexBase + 2]);
				if (!RenderToDynamic.IsValidIndex(A) || !RenderToDynamic.IsValidIndex(B) || !RenderToDynamic.IsValidIndex(C))
				{
					continue;
				}

				const int32 VA = RenderToDynamic[A];
				const int32 VB = RenderToDynamic[B];
				const int32 VC = RenderToDynamic[C];
				if (VA == VB || VB == VC || VC == VA)
				{
					continue;
				}

				if (OutMesh.AppendTriangle(VA, VB, VC) >= 0)
				{
					++AddedTriangles;
				}
			}
		}

		return AddedTriangles > 0;
	}

	static bool BuildDynamicMeshFromWorldMesh(
		const TArray<FVector>& Vertices,
		const TArray<int32>& Triangles,
		UE::Geometry::FDynamicMesh3& OutMesh)
	{
		OutMesh = UE::Geometry::FDynamicMesh3();
		if (Vertices.Num() < 3 || Triangles.Num() < 3)
		{
			return false;
		}

		TArray<int32> VertexIds;
		VertexIds.Reserve(Vertices.Num());
		for (const FVector& V : Vertices)
		{
			VertexIds.Add(OutMesh.AppendVertex(FVector3d(V.X, V.Y, V.Z)));
		}

		int32 AddedTriangles = 0;
		for (int32 i = 0; i + 2 < Triangles.Num(); i += 3)
		{
			const int32 A = Triangles[i];
			const int32 B = Triangles[i + 1];
			const int32 C = Triangles[i + 2];
			if (!VertexIds.IsValidIndex(A) || !VertexIds.IsValidIndex(B) || !VertexIds.IsValidIndex(C))
			{
				continue;
			}
			if (OutMesh.AppendTriangle(VertexIds[A], VertexIds[B], VertexIds[C]) >= 0)
			{
				++AddedTriangles;
			}
		}
		return AddedTriangles > 0;
	}

	static bool BuildDynamicMeshFromProceduralMeshComponent(UProceduralMeshComponent* Component, UE::Geometry::FDynamicMesh3& OutMesh, bool bRequireVisible = true)
	{
		OutMesh = UE::Geometry::FDynamicMesh3();
		if (!Component || !Component->IsRegistered() || (bRequireVisible && !Component->IsVisible()))
		{
			return false;
		}

		const FTransform& ComponentTransform = Component->GetComponentTransform();
		TMap<FString, int32> VertexIdByPosition;
		int32 AddedTriangles = 0;
		for (int32 SectionIndex = 0; SectionIndex < Component->GetNumSections(); ++SectionIndex)
		{
			if (bRequireVisible && !Component->IsMeshSectionVisible(SectionIndex))
			{
				continue;
			}

			const FProcMeshSection* Section = Component->GetProcMeshSection(SectionIndex);
			if (!Section || Section->ProcVertexBuffer.Num() < 3 || Section->ProcIndexBuffer.Num() < 3)
			{
				continue;
			}

			TArray<int32> SectionVertexIds;
			SectionVertexIds.Reserve(Section->ProcVertexBuffer.Num());
			for (const FProcMeshVertex& Vertex : Section->ProcVertexBuffer)
			{
				const FVector WorldPosition = ComponentTransform.TransformPosition(Vertex.Position);
				const FString Key = QuantizedVertexKey(WorldPosition);
				if (const int32* ExistingId = VertexIdByPosition.Find(Key))
				{
					SectionVertexIds.Add(*ExistingId);
				}
				else
				{
					const int32 DynamicId = OutMesh.AppendVertex(FVector3d(WorldPosition.X, WorldPosition.Y, WorldPosition.Z));
					VertexIdByPosition.Add(Key, DynamicId);
					SectionVertexIds.Add(DynamicId);
				}
			}

			for (int32 Index = 0; Index + 2 < Section->ProcIndexBuffer.Num(); Index += 3)
			{
				const int32 A = int32(Section->ProcIndexBuffer[Index]);
				const int32 B = int32(Section->ProcIndexBuffer[Index + 1]);
				const int32 C = int32(Section->ProcIndexBuffer[Index + 2]);
				if (!SectionVertexIds.IsValidIndex(A) || !SectionVertexIds.IsValidIndex(B) || !SectionVertexIds.IsValidIndex(C))
				{
					continue;
				}

				const int32 VA = SectionVertexIds[A];
				const int32 VB = SectionVertexIds[B];
				const int32 VC = SectionVertexIds[C];
				if (VA == VB || VB == VC || VC == VA)
				{
					continue;
				}

				if (OutMesh.AppendTriangle(VA, VB, VC) >= 0)
				{
					++AddedTriangles;
				}
			}
		}

		return AddedTriangles > 0;
	}

	static double PointTriangleDistanceSquared(const FVector& P, const FVector& A, const FVector& B, const FVector& C)
	{
		const FVector AB = B - A;
		const FVector AC = C - A;
		const FVector AP = P - A;
		const double D1 = FVector::DotProduct(AB, AP);
		const double D2 = FVector::DotProduct(AC, AP);
		if (D1 <= 0.0 && D2 <= 0.0)
		{
			return FVector::DistSquared(P, A);
		}

		const FVector BP = P - B;
		const double D3 = FVector::DotProduct(AB, BP);
		const double D4 = FVector::DotProduct(AC, BP);
		if (D3 >= 0.0 && D4 <= D3)
		{
			return FVector::DistSquared(P, B);
		}

		const double VC = D1 * D4 - D3 * D2;
		if (VC <= 0.0 && D1 >= 0.0 && D3 <= 0.0)
		{
			const double V = D1 / (D1 - D3);
			return FVector::DistSquared(P, A + AB * V);
		}

		const FVector CP = P - C;
		const double D5 = FVector::DotProduct(AB, CP);
		const double D6 = FVector::DotProduct(AC, CP);
		if (D6 >= 0.0 && D5 <= D6)
		{
			return FVector::DistSquared(P, C);
		}

		const double VB = D5 * D2 - D1 * D6;
		if (VB <= 0.0 && D2 >= 0.0 && D6 <= 0.0)
		{
			const double W = D2 / (D2 - D6);
			return FVector::DistSquared(P, A + AC * W);
		}

		const double VA = D3 * D6 - D5 * D4;
		if (VA <= 0.0 && (D4 - D3) >= 0.0 && (D5 - D6) >= 0.0)
		{
			const double W = (D4 - D3) / ((D4 - D3) + (D5 - D6));
			return FVector::DistSquared(P, B + (C - B) * W);
		}

		const FVector Normal = FVector::CrossProduct(AB, AC).GetSafeNormal();
		if (Normal.IsNearlyZero())
		{
			return FMath::Min3(FVector::DistSquared(P, A), FVector::DistSquared(P, B), FVector::DistSquared(P, C));
		}
		const double SignedDistance = FVector::DotProduct(P - A, Normal);
		return SignedDistance * SignedDistance;
	}

	static bool ComputeAverageProbeDistanceToMesh(
		const UE::Geometry::FDynamicMesh3& Mesh,
		const TArray<FVector>& ProbePoints,
		double& OutAverageDistance)
	{
		OutAverageDistance = TNumericLimits<double>::Max();
		if (Mesh.TriangleCount() <= 0 || ProbePoints.Num() == 0)
		{
			return false;
		}

		double DistanceSum = 0.0;
		int32 ValidProbeCount = 0;

		for (const FVector& Probe : ProbePoints)
		{
			if (!FMath::IsFinite(Probe.X) || !FMath::IsFinite(Probe.Y) || !FMath::IsFinite(Probe.Z))
			{
				continue;
			}

			double BestDistanceSq = TNumericLimits<double>::Max();
			for (int32 TriangleId : Mesh.TriangleIndicesItr())
			{
				const UE::Geometry::FIndex3i Tri = Mesh.GetTriangle(TriangleId);
				if (!Mesh.IsVertex(Tri.A) || !Mesh.IsVertex(Tri.B) || !Mesh.IsVertex(Tri.C))
				{
					continue;
				}

				const FVector3d A3 = Mesh.GetVertex(Tri.A);
				const FVector3d B3 = Mesh.GetVertex(Tri.B);
				const FVector3d C3 = Mesh.GetVertex(Tri.C);
				const FVector A(A3.X, A3.Y, A3.Z);
				const FVector B(B3.X, B3.Y, B3.Z);
				const FVector C(C3.X, C3.Y, C3.Z);
				const double DistanceSq = PointTriangleDistanceSquared(Probe, A, B, C);
				if (DistanceSq < BestDistanceSq)
				{
					BestDistanceSq = DistanceSq;
				}
			}

			if (BestDistanceSq < TNumericLimits<double>::Max())
			{
				++ValidProbeCount;
				DistanceSum += FMath::Sqrt(BestDistanceSq);
			}
		}

		if (ValidProbeCount == 0)
		{
			return false;
		}

		OutAverageDistance = DistanceSum / double(ValidProbeCount);
		return FMath::IsFinite(OutAverageDistance);
	}

	static bool StringEqualsNonEmpty(const FString& A, const FString& B)
	{
		return !A.IsEmpty() && !B.IsEmpty() && A.Equals(B, ESearchCase::CaseSensitive);
	}

	static bool PathEqualsOrSuffixMatches(const FString& Expected, const FString& Actual)
	{
		if (Expected.IsEmpty() || Actual.IsEmpty())
		{
			return false;
		}
		return Expected.Equals(Actual, ESearchCase::CaseSensitive) ||
			Expected.EndsWith(Actual, ESearchCase::CaseSensitive) ||
			Actual.EndsWith(Expected, ESearchCase::CaseSensitive);
	}

	static bool IsActorAllowedAsAttachMaterialSource(const AActor* Actor)
	{
		if (!Actor || Actor->IsHidden() || Actor->ActorHasTag(ReconstructedFaceTag) || ActorIsStep11Cutter(Actor))
		{
			return false;
		}

		if (!ActorHasAnyStep11RuntimeTag(Actor))
		{
			return true;
		}

		const bool bAllowedRuntimeSource =
			Actor->ActorHasTag(Step11ActionAttachTag) ||
			Actor->ActorHasTag(Step11BooleanResultTag);
		return bAllowedRuntimeSource && ActorHasActiveStep11GeneratedByTag(Actor);
	}

	static int32 ScoreActorMaterialIdComponent(UPrimitiveComponent* Component, const FActorMaterialIdEntry& Entry, const UPrimitiveComponent* SelfComponent)
	{
		if (!Component || Component == SelfComponent || !Component->IsRegistered() || !Component->IsVisible())
		{
			return -1;
		}

		AActor* Owner = Component->GetOwner();
		if (!IsActorAllowedAsAttachMaterialSource(Owner))
		{
			return -1;
		}

		const FString ComponentPath = Component->GetPathName();
		if (StringEqualsNonEmpty(Entry.ComponentPath, ComponentPath))
		{
			return 1000;
		}

		const FString ActorPath = Owner->GetPathName();
		bool bActorMatched = false;
		bool bComponentMatched = false;
		int32 Score = 0;

		if (PathEqualsOrSuffixMatches(Entry.ActorPath, ActorPath))
		{
			bActorMatched = true;
			Score += 100;
		}
		if (StringEqualsNonEmpty(Entry.ActorName, Owner->GetName()))
		{
			bActorMatched = true;
			Score += 40;
		}

		if (PathEqualsOrSuffixMatches(Entry.ComponentPath, ComponentPath))
		{
			bComponentMatched = true;
			Score += 200;
		}
		if (StringEqualsNonEmpty(Entry.ComponentName, Component->GetName()))
		{
			bComponentMatched = true;
			Score += 40;
		}
		if (StringEqualsNonEmpty(Entry.ComponentType, Component->GetClass() ? Component->GetClass()->GetName() : FString()))
		{
			Score += 10;
		}

		const bool bHasActorHint = !Entry.ActorPath.IsEmpty() || !Entry.ActorName.IsEmpty();
		if (!bComponentMatched || (bHasActorHint && !bActorMatched))
		{
			return -1;
		}
		return Score;
	}
	static UPrimitiveComponent* FindActorMaterialIdComponent(UWorld* World, const FActorMaterialIdEntry& Entry, const UPrimitiveComponent* SelfComponent)
	{
		if (!World)
		{
			return nullptr;
		}

		UPrimitiveComponent* BestComponent = nullptr;
		int32 BestScore = -1;
		for (TActorIterator<AActor> It(World); It; ++It)
		{
			AActor* Actor = *It;
			if (!Actor)
			{
				continue;
			}

			TArray<UPrimitiveComponent*> Components;
			Actor->GetComponents<UPrimitiveComponent>(Components);
			for (UPrimitiveComponent* Component : Components)
			{
				const int32 Score = ScoreActorMaterialIdComponent(Component, Entry, SelfComponent);
				if (Score > BestScore)
				{
					BestComponent = Component;
					BestScore = Score;
					if (Score >= 1000)
					{
						return BestComponent;
					}
				}
			}
		}

		return BestComponent;
	}

	static UMaterialInterface* ResolveAttachMaterialFromIdSelection(
		UWorld* World,
		const FReconstructedMesh& MeshData,
		const UPrimitiveComponent* SelfComponent)
	{
		const FAttachMaterialIdSelection& Selection = MeshData.AttachMaterialId;
		if (!Selection.bLookupAttempted)
		{
			return nullptr;
		}
		if (!Selection.bFound)
		{
			UE_LOG(
				LogTemp,
				Warning,
				TEXT("Step10Diag: attach material id lookup failed; falling back to nearest visible geometry source_face_id=%d attach_actor=%s error=%s."),
				MeshData.SourceFaceId,
				*MeshData.ActorName,
				*Selection.Error);
			return nullptr;
		}

		UPrimitiveComponent* SourceComponent = FindActorMaterialIdComponent(World, Selection.Entry, SelfComponent);
		if (!SourceComponent)
		{
			if (!Selection.Entry.MaterialPath.IsEmpty())
			{
				if (UMaterialInterface* MaterialFromPath = LoadObject<UMaterialInterface>(nullptr, *Selection.Entry.MaterialPath))
				{
					UE_LOG(
						LogTemp,
						Log,
						TEXT("Step10Diag: attach material inherited from id buffer material_path=%s after source component was unavailable; source_actor=%s source_component=%s attach_actor=%s."),
						*Selection.Entry.MaterialPath,
						*Selection.Entry.ActorName,
						*Selection.Entry.ComponentName,
						*MeshData.ActorName);
					return MaterialFromPath;
				}
			}

			UE_LOG(
				LogTemp,
				Warning,
				TEXT("Step10Diag: attach material id matched actor=%s component=%s slot=%d, but source component was not found and material_path could not be loaded; falling back to nearest visible geometry attach_actor=%s."),
				*Selection.Entry.ActorName,
				*Selection.Entry.ComponentName,
				Selection.Entry.MaterialSlot,
				*MeshData.ActorName);
			return nullptr;
		}

		const int32 MaterialSlot = FMath::Max(0, Selection.Entry.MaterialSlot);
		UMaterialInterface* SourceMaterial = SourceComponent->GetMaterial(MaterialSlot);
		if (!SourceMaterial && !Selection.Entry.MaterialPath.IsEmpty())
		{
			SourceMaterial = LoadObject<UMaterialInterface>(nullptr, *Selection.Entry.MaterialPath);
		}
		if (!SourceMaterial)
		{
			UE_LOG(
				LogTemp,
				Warning,
				TEXT("Step10Diag: attach material id source component found but material is null actor=%s component=%s slot=%d material_path=%s; falling back to nearest visible geometry attach_actor=%s."),
				*GetNameSafe(SourceComponent->GetOwner()),
				*GetNameSafe(SourceComponent),
				MaterialSlot,
				*Selection.Entry.MaterialPath,
				*MeshData.ActorName);
			return nullptr;
		}

		UE_LOG(
			LogTemp,
			Log,
			TEXT("Step10Diag: attach material inherited from id buffer source_actor=%s source_component=%s material_slot=%d material=%s vote_pixels=%d considered_pixels=%d coverage=%.6f attach_actor=%s."),
			*GetNameSafe(SourceComponent->GetOwner()),
			*GetNameSafe(SourceComponent),
			MaterialSlot,
			*GetNameSafe(SourceMaterial),
			Selection.PixelCount,
			Selection.ConsideredPixelCount,
			Selection.Coverage,
			*MeshData.ActorName);
		return SourceMaterial;
	}

	static UMaterialInterface* ResolveAttachSourceMaterial(
		UWorld* World,
		const FReconstructedMesh& MeshData,
		UPrimitiveComponent* SelfComponent)
	{
		if (!World || MeshData.bIsExcavateCutter)
		{
			return nullptr;
		}

		if (MeshData.AttachMaterialId.bLookupAttempted)
		{
			if (UMaterialInterface* IdMaterial = ResolveAttachMaterialFromIdSelection(World, MeshData, SelfComponent))
			{
				return IdMaterial;
			}
		}

		TArray<FVector> ProbePoints = MeshData.SourceMaterialProbePointsWorld;
		if (ProbePoints.Num() == 0)
		{
			ProbePoints = MeshData.SourceFaceVerticesWorld;
		}
		if (ProbePoints.Num() == 0)
		{
			ProbePoints.Add(MeshData.SourcePlanePoint);
		}

		UPrimitiveComponent* BestComponent = nullptr;
		UMaterialInterface* BestMaterial = nullptr;
		double BestAverageDistance = TNumericLimits<double>::Max();

		auto ConsiderComponent = [&](UPrimitiveComponent* Component, UE::Geometry::FDynamicMesh3& CandidateMesh)
		{
			if (!Component ||
				Component == SelfComponent ||
				!Component->IsRegistered() ||
				!Component->IsVisible())
			{
				return;
			}

			UMaterialInterface* CandidateMaterial = Component->GetMaterial(0);
			if (!CandidateMaterial)
			{
				return;
			}

			double AverageDistance = TNumericLimits<double>::Max();
			if (!ComputeAverageProbeDistanceToMesh(CandidateMesh, ProbePoints, AverageDistance))
			{
				return;
			}
			if (AverageDistance < BestAverageDistance)
			{
				BestComponent = Component;
				BestMaterial = CandidateMaterial;
				BestAverageDistance = AverageDistance;
			}
		};

		for (TActorIterator<AActor> It(World); It; ++It)
		{
			AActor* Actor = *It;
			if (!IsActorAllowedAsAttachMaterialSource(Actor))
			{
				continue;
			}

			TArray<UStaticMeshComponent*> StaticComponents;
			Actor->GetComponents<UStaticMeshComponent>(StaticComponents);
			for (UStaticMeshComponent* Component : StaticComponents)
			{
				UE::Geometry::FDynamicMesh3 CandidateMesh;
				if (BuildDynamicMeshFromStaticMeshComponent(Component, CandidateMesh))
				{
					ConsiderComponent(Component, CandidateMesh);
				}
			}

			TArray<UProceduralMeshComponent*> ProceduralComponents;
			Actor->GetComponents<UProceduralMeshComponent>(ProceduralComponents);
			for (UProceduralMeshComponent* Component : ProceduralComponents)
			{
				UE::Geometry::FDynamicMesh3 CandidateMesh;
				if (BuildDynamicMeshFromProceduralMeshComponent(Component, CandidateMesh))
				{
					ConsiderComponent(Component, CandidateMesh);
				}
			}
		}

		if (BestComponent)
		{
			UE_LOG(
				LogTemp,
				Log,
				TEXT("Step10Diag: attach material inherited from nearest visible geometry source_actor=%s source_component=%s source_face_id=%d material_slot=0 material=%s average_probe_distance_cm=%.6f probe_count=%d attach_actor=%s."),
				*GetNameSafe(BestComponent->GetOwner()),
				*GetNameSafe(BestComponent),
				MeshData.SourceFaceId,
				*GetNameSafe(BestMaterial),
				BestAverageDistance,
				ProbePoints.Num(),
				*MeshData.ActorName);
			return BestMaterial;
		}

		UE_LOG(
			LogTemp,
			Warning,
			TEXT("Step10Diag: attach material fallback to vertex color; no visible material-bearing source component could be scored source_face_id=%d attach_actor=%s probe_count=%d."),
			MeshData.SourceFaceId,
			*MeshData.ActorName,
			ProbePoints.Num());
		return nullptr;
	}

	static bool ConvertDynamicMeshToProceduralArrays(
		const UE::Geometry::FDynamicMesh3& Mesh,
		TArray<FVector>& OutVertices,
		TArray<int32>& OutTriangles,
		TArray<FVector>& OutNormals)
	{
		OutVertices.Reset();
		OutTriangles.Reset();
		OutNormals.Reset();
		if (Mesh.VertexCount() <= 0 || Mesh.TriangleCount() <= 0)
		{
			return false;
		}

		TMap<int32, int32> CompactVertexIndex;
		CompactVertexIndex.Reserve(Mesh.VertexCount());
		OutVertices.Reserve(Mesh.VertexCount());
		for (int32 VertexId : Mesh.VertexIndicesItr())
		{
			const FVector3d V = Mesh.GetVertex(VertexId);
			CompactVertexIndex.Add(VertexId, OutVertices.Num());
			OutVertices.Emplace(V.X, V.Y, V.Z);
		}

		OutTriangles.Reserve(Mesh.TriangleCount() * 3);
		OutNormals.Init(FVector::ZeroVector, OutVertices.Num());
		for (int32 TriangleId : Mesh.TriangleIndicesItr())
		{
			const UE::Geometry::FIndex3i Tri = Mesh.GetTriangle(TriangleId);
			const int32* A = CompactVertexIndex.Find(Tri.A);
			const int32* B = CompactVertexIndex.Find(Tri.B);
			const int32* C = CompactVertexIndex.Find(Tri.C);
			if (!A || !B || !C)
			{
				continue;
			}

			OutTriangles.Add(*A);
			OutTriangles.Add(*B);
			OutTriangles.Add(*C);
			const FVector& VA = OutVertices[*A];
			const FVector& VB = OutVertices[*B];
			const FVector& VC = OutVertices[*C];
			const FVector TriNormal = FVector::CrossProduct(VB - VA, VC - VA).GetSafeNormal();
			OutNormals[*A] += TriNormal;
			OutNormals[*B] += TriNormal;
			OutNormals[*C] += TriNormal;
		}

		const bool bFlipNormalsForLighting = Step11SignedVolumeSign(AnalyzeStep11DynamicMesh(Mesh, TEXT("procedural_normal_check"), TEXT("procedural_normal_check")).SignedVolume) < 0;
		for (FVector& Normal : OutNormals)
		{
			Normal = Normal.GetSafeNormal();
			if (Normal.IsNearlyZero())
			{
				Normal = FVector::UpVector;
			}
			if (bFlipNormalsForLighting)
			{
				Normal *= -1.0;
			}
		}
		return OutVertices.Num() >= 3 && OutTriangles.Num() >= 3;
	}

	struct FStep11ProcMeshSectionData
	{
		TArray<FVector> Vertices;
		TArray<int32> Triangles;
		TArray<FVector> Normals;
		TArray<FVector2D> UV0;
		TArray<FColor> Colors;
		TArray<FProcMeshTangent> Tangents;

		bool HasGeometry() const
		{
			return Vertices.Num() >= 3 && Triangles.Num() >= 3;
		}
	};

	static FVector Step11StableTangent(const FVector& Normal)
	{
		FVector Tangent = FVector::CrossProduct(FVector::UpVector, Normal);
		if (Tangent.IsNearlyZero())
		{
			Tangent = FVector::CrossProduct(FVector::RightVector, Normal);
		}
		Tangent.Normalize();
		return Tangent.IsNearlyZero() ? FVector::ForwardVector : Tangent;
	}

	static FVector2D Step11PlanarUV(const FVector& WorldPosition, const FVector& Normal)
	{
		constexpr double UvScaleCm = 100.0;
		const FVector AbsNormal(FMath::Abs(Normal.X), FMath::Abs(Normal.Y), FMath::Abs(Normal.Z));
		if (AbsNormal.Z >= AbsNormal.X && AbsNormal.Z >= AbsNormal.Y)
		{
			return FVector2D(WorldPosition.X / UvScaleCm, WorldPosition.Y / UvScaleCm);
		}
		if (AbsNormal.Y >= AbsNormal.X)
		{
			return FVector2D(WorldPosition.X / UvScaleCm, WorldPosition.Z / UvScaleCm);
		}
		return FVector2D(WorldPosition.Y / UvScaleCm, WorldPosition.Z / UvScaleCm);
	}

	static void AppendStep11FlatTriangle(
		FStep11ProcMeshSectionData& Section,
		const FVector& AWorld,
		const FVector& BWorld,
		const FVector& CWorld,
		const FVector& Origin,
		const FColor& Color,
		bool bFlipNormalForLighting)
	{
		FVector Normal = FVector::CrossProduct(BWorld - AWorld, CWorld - AWorld).GetSafeNormal();
		if (Normal.IsNearlyZero())
		{
			Normal = FVector::UpVector;
		}
		if (bFlipNormalForLighting)
		{
			Normal *= -1.0;
		}
		const FVector Tangent = Step11StableTangent(Normal);
		const int32 BaseIndex = Section.Vertices.Num();
		Section.Vertices.Add(AWorld - Origin);
		Section.Vertices.Add(BWorld - Origin);
		Section.Vertices.Add(CWorld - Origin);
		Section.Triangles.Add(BaseIndex);
		Section.Triangles.Add(BaseIndex + 1);
		Section.Triangles.Add(BaseIndex + 2);
		Section.Normals.Add(Normal);
		Section.Normals.Add(Normal);
		Section.Normals.Add(Normal);
		Section.UV0.Add(Step11PlanarUV(AWorld, Normal));
		Section.UV0.Add(Step11PlanarUV(BWorld, Normal));
		Section.UV0.Add(Step11PlanarUV(CWorld, Normal));
		Section.Colors.Add(Color);
		Section.Colors.Add(Color);
		Section.Colors.Add(Color);
		Section.Tangents.Add(FProcMeshTangent(Tangent, false));
		Section.Tangents.Add(FProcMeshTangent(Tangent, false));
		Section.Tangents.Add(FProcMeshTangent(Tangent, false));
	}

	static bool ConvertDynamicMeshToFlatProceduralSections(
		const UE::Geometry::FDynamicMesh3& Mesh,
		const FVector& Origin,
		const TArray<int8>* TriangleSourceMeshById,
		FStep11ProcMeshSectionData& OutSourceSection,
		FStep11ProcMeshSectionData& OutCapSection)
	{
		OutSourceSection = FStep11ProcMeshSectionData();
		OutCapSection = FStep11ProcMeshSectionData();
		if (Mesh.TriangleCount() <= 0)
		{
			return false;
		}

		const bool bFlipNormalsForLighting = Step11SignedVolumeSign(AnalyzeStep11DynamicMesh(Mesh, TEXT("flat_procedural_normal_check"), TEXT("flat_procedural_normal_check")).SignedVolume) < 0;
		for (int32 TriangleId : Mesh.TriangleIndicesItr())
		{
			const UE::Geometry::FIndex3i Tri = Mesh.GetTriangle(TriangleId);
			if (!Mesh.IsVertex(Tri.A) || !Mesh.IsVertex(Tri.B) || !Mesh.IsVertex(Tri.C))
			{
				continue;
			}

			const FVector3d A3 = Mesh.GetVertex(Tri.A);
			const FVector3d B3 = Mesh.GetVertex(Tri.B);
			const FVector3d C3 = Mesh.GetVertex(Tri.C);
			const FVector A(A3.X, A3.Y, A3.Z);
			const FVector B(B3.X, B3.Y, B3.Z);
			const FVector C(C3.X, C3.Y, C3.Z);
			if (FVector::CrossProduct(B - A, C - A).IsNearlyZero())
			{
				continue;
			}

			const bool bFromCutter = TriangleSourceMeshById &&
				TriangleSourceMeshById->IsValidIndex(TriangleId) &&
				(*TriangleSourceMeshById)[TriangleId] == 1;
			AppendStep11FlatTriangle(
				bFromCutter ? OutCapSection : OutSourceSection,
				A,
				B,
				C,
				Origin,
				bFromCutter ? FColor(180, 180, 180, 255) : FColor::White,
				bFlipNormalsForLighting);
		}

		return OutSourceSection.HasGeometry() || OutCapSection.HasGeometry();
	}

	static FStep11BooleanAttemptResult RunStep11BooleanDifferencePass(
		const UE::Geometry::FDynamicMesh3& CurrentMesh,
		const UE::Geometry::FDynamicMesh3& CutterMesh,
		const FStep11MeshDiagnostics& TargetBeforeDiagnostics,
		int32 TargetRenderSign,
		bool bReverseTargetForPass,
		const FString& PassName,
		const FString& ResultLabel)
	{
		FStep11BooleanAttemptResult Attempt;
		Attempt.PassName = PassName;
		Attempt.MinRequiredVolumeDelta = FMath::Max(1.0, TargetBeforeDiagnostics.AbsVolume * 1e-4);
		Attempt.ManifoldDiagnostics.TargetInputTriangles = CurrentMesh.TriangleCount();
		Attempt.ManifoldDiagnostics.CutterInputTriangles = CutterMesh.TriangleCount();
		if (TargetBeforeDiagnostics.AbsVolume <= 1e-6 || TargetRenderSign == 0)
		{
			Attempt.Status = TEXT("rejected");
			Attempt.RejectReason = TEXT("target_zero_volume");
			return Attempt;
		}
		if (TargetBeforeDiagnostics.TriangleCount <= 0 ||
			TargetBeforeDiagnostics.BoundaryEdgeCount > 0 ||
			TargetBeforeDiagnostics.NonManifoldEdgeCount > 0)
		{
			Attempt.Status = TEXT("rejected");
			Attempt.RejectReason = TEXT("target_not_closed_manifold");
			return Attempt;
		}
		const FStep11MeshDiagnostics CutterBeforeDiagnostics = AnalyzeStep11DynamicMesh(CutterMesh, TEXT("manifold_cutter_precheck"), TEXT("excavate_cutter"));
		if (CutterBeforeDiagnostics.TriangleCount <= 0 ||
			CutterBeforeDiagnostics.BoundaryEdgeCount > 0 ||
			CutterBeforeDiagnostics.NonManifoldEdgeCount > 0)
		{
			Attempt.Status = TEXT("rejected");
			Attempt.RejectReason = TEXT("cutter_not_closed_manifold");
			return Attempt;
		}

		UE::Geometry::FDynamicMesh3 TargetWork = CurrentMesh;
		if (bReverseTargetForPass)
		{
			TargetWork.ReverseOrientation(false);
			Attempt.bTargetReversedForPass = true;
		}
		const int32 RenderSignForAttempt = bReverseTargetForPass ? -TargetRenderSign : TargetRenderSign;
		Attempt.bComputeSuccess = FFromLZManifoldBoolean::Difference(
			TargetWork,
			CutterMesh,
			RenderSignForAttempt,
			Attempt.ResultMesh,
			Attempt.ManifoldDiagnostics);
		Attempt.bCutterReversedForPass = Attempt.ManifoldDiagnostics.bCutterOrientationReversedForManifold;
		Attempt.bResultReversedToTargetSign = Attempt.ManifoldDiagnostics.bResultOrientationReversedToTargetSign;
		if (!Attempt.bComputeSuccess)
		{
			Attempt.Status = TEXT("rejected");
			Attempt.RejectReason = Attempt.ManifoldDiagnostics.ManifoldErrorMessage.IsEmpty()
				? TEXT("manifold_difference_failed")
				: Attempt.ManifoldDiagnostics.ManifoldErrorMessage;
			return Attempt;
		}

		if (Attempt.ResultMesh.TriangleCount() <= 0)
		{
			Attempt.bAccepted = true;
			Attempt.bEmptyResult = true;
			Attempt.Status = TEXT("success_empty_result");
			Attempt.VolumeDelta = TargetBeforeDiagnostics.AbsVolume;
			return Attempt;
		}

		Attempt.TriangleSourceMeshById.Init(0, Attempt.ResultMesh.TriangleCount());

		Attempt.ResultDiagnostics = AnalyzeStep11DynamicMesh(Attempt.ResultMesh, ResultLabel, TEXT("boolean_result"));
		const int32 ResultSign = Step11SignedVolumeSign(Attempt.ResultDiagnostics.SignedVolume);
		if (ResultSign != 0 && ResultSign != TargetRenderSign)
		{
			Attempt.ResultMesh.ReverseOrientation(false);
			Attempt.bResultReversedToTargetSign = true;
			Attempt.ManifoldDiagnostics.bResultOrientationReversedToTargetSign = true;
			Attempt.ResultDiagnostics = AnalyzeStep11DynamicMesh(Attempt.ResultMesh, ResultLabel, TEXT("boolean_result"));
		}

		Attempt.VolumeDelta = TargetBeforeDiagnostics.AbsVolume - Attempt.ResultDiagnostics.AbsVolume;
		if (Attempt.ResultDiagnostics.BoundaryEdgeCount > 0)
		{
			Attempt.Status = TEXT("rejected");
			Attempt.RejectReason = TEXT("open_boundary");
			return Attempt;
		}
		if (Attempt.ResultDiagnostics.NonManifoldEdgeCount > 0)
		{
			Attempt.Status = TEXT("rejected");
			Attempt.RejectReason = TEXT("nonmanifold_result");
			return Attempt;
		}
		if (Attempt.VolumeDelta < Attempt.MinRequiredVolumeDelta)
		{
			Attempt.Status = TEXT("rejected");
			Attempt.RejectReason = TEXT("no_volume_removed");
			return Attempt;
		}
		if (Attempt.ResultDiagnostics.MinEdgeLength > 0.0 &&
			Attempt.ResultDiagnostics.MinEdgeLength < Step11BooleanMinRenderableEdgeCm)
		{
			Attempt.Status = TEXT("rejected");
			Attempt.RejectReason = TEXT("tiny_edges_after_boolean");
			return Attempt;
		}

		Attempt.bAccepted = true;
		Attempt.Status = TEXT("accepted");
		return Attempt;
	}

	static bool ApplyCuttersToDynamicMesh(
		UE::Geometry::FDynamicMesh3& CurrentMesh,
		const FString& TargetActorName,
		const FString& TargetComponentName,
		const FString& TargetSourceType,
		const TArray<const FReconstructedMesh*>& Cutters,
		const TArray<UE::Geometry::FDynamicMesh3>& CutterMeshes,
		const TArray<FBox>& CutterBounds,
		const TArray<FStep11MeshDiagnostics>& CutterDiagnostics,
		TArray<TSharedPtr<FJsonValue>>* OutOperationDiagnostics,
		TArray<int8>& OutFinalTriangleSourceMeshById,
		TSet<FString>& OutAcceptedCutterActorNames,
		bool& bOutEmptyResult,
		int32& FailedBooleanCount)
	{
		bOutEmptyResult = false;
		bool bModified = false;
		for (int32 CutterIndex = 0; CutterIndex < CutterMeshes.Num(); ++CutterIndex)
		{
			const FString TargetLabel = FString::Printf(TEXT("%s/%s"), *TargetActorName, *TargetComponentName);
			FStep11MeshDiagnostics TargetBeforeDiagnostics = AnalyzeStep11DynamicMesh(CurrentMesh, TargetLabel, TargetSourceType);
			const FBox TargetBounds = TargetBeforeDiagnostics.Bounds;
			const FBox ExpandedTargetBounds = TargetBounds.ExpandBy(Step11BooleanBoundsExpandCm);
			const FBox ExpandedCutterBounds = CutterBounds[CutterIndex].ExpandBy(Step11BooleanBoundsExpandCm);
			const bool bBoundsIntersect = TargetBounds.Intersect(CutterBounds[CutterIndex]);
			const bool bExpandedBoundsIntersect = ExpandedTargetBounds.Intersect(ExpandedCutterBounds);
			const double CenterDistance = (TargetBounds.IsValid && CutterBounds[CutterIndex].IsValid)
				? FVector::Distance(TargetBounds.GetCenter(), CutterBounds[CutterIndex].GetCenter())
				: 0.0;
			const int32 TargetRenderSign = Step11SignedVolumeSign(TargetBeforeDiagnostics.SignedVolume);

			TSharedRef<FJsonObject> OperationJson = MakeShared<FJsonObject>();
			OperationJson->SetStringField(TEXT("target_actor"), TargetActorName);
			OperationJson->SetStringField(TEXT("target_component"), TargetComponentName);
			OperationJson->SetStringField(TEXT("target_source_type"), TargetSourceType);
			OperationJson->SetStringField(TEXT("cutter_actor"), Cutters[CutterIndex]->ActorName);
			OperationJson->SetNumberField(TEXT("cutter_index"), CutterIndex);
			OperationJson->SetStringField(TEXT("boolean_backend"), FFromLZManifoldBoolean::BackendName());
			OperationJson->SetStringField(TEXT("manifold_library_version"), FFromLZManifoldBoolean::LibraryVersion());
			OperationJson->SetBoolField(TEXT("bounds_intersect"), bBoundsIntersect);
			OperationJson->SetBoolField(TEXT("expanded_bounds_intersect"), bExpandedBoundsIntersect);
			OperationJson->SetNumberField(TEXT("center_distance"), CenterDistance);
			OperationJson->SetObjectField(TEXT("target_bounds"), Step11BoxJson(TargetBounds));
			OperationJson->SetObjectField(TEXT("cutter_bounds"), Step11BoxJson(CutterBounds[CutterIndex]));
			OperationJson->SetObjectField(TEXT("target_before"), Step11MeshDiagnosticsJson(TargetBeforeDiagnostics));
			OperationJson->SetNumberField(TEXT("target_render_sign"), TargetRenderSign);
			OperationJson->SetNumberField(TEXT("min_renderable_edge_cm"), Step11BooleanMinRenderableEdgeCm);
			if (CutterDiagnostics.IsValidIndex(CutterIndex))
			{
				OperationJson->SetObjectField(TEXT("cutter_diagnostics"), Step11MeshDiagnosticsJson(CutterDiagnostics[CutterIndex]));
			}

			UE_LOG(
				LogTemp,
				Log,
				TEXT("Step11Diag: pair target=%s/%s source=%s cutter=%s bounds_intersect=%d expanded_bounds_intersect=%d center_distance=%.6f target_triangles=%d cutter_triangles=%d"),
				*TargetActorName,
				*TargetComponentName,
				*TargetSourceType,
				*Cutters[CutterIndex]->ActorName,
				bBoundsIntersect ? 1 : 0,
				bExpandedBoundsIntersect ? 1 : 0,
				CenterDistance,
				TargetBeforeDiagnostics.TriangleCount,
				CutterDiagnostics.IsValidIndex(CutterIndex) ? CutterDiagnostics[CutterIndex].TriangleCount : CutterMeshes[CutterIndex].TriangleCount());

			if (!bExpandedBoundsIntersect)
			{
				OperationJson->SetStringField(TEXT("status"), TEXT("skipped_bounds_no_intersection"));
				if (OutOperationDiagnostics)
				{
					OutOperationDiagnostics->Add(MakeShared<FJsonValueObject>(OperationJson));
				}
				continue;
			}

			TArray<TSharedPtr<FJsonValue>> AttemptDiagnostics;
			FStep11BooleanAttemptResult PrimaryAttempt = RunStep11BooleanDifferencePass(
				CurrentMesh,
				CutterMeshes[CutterIndex],
				TargetBeforeDiagnostics,
				TargetRenderSign,
				false,
				TEXT("primary_target_render_orientation"),
				FString::Printf(TEXT("%s/%s result_after_%s_primary"), *TargetActorName, *TargetComponentName, *Cutters[CutterIndex]->ActorName));
			AttemptDiagnostics.Add(MakeShared<FJsonValueObject>(Step11BooleanAttemptJson(PrimaryAttempt)));
			FStep11BooleanAttemptResult AcceptedAttempt = MoveTemp(PrimaryAttempt);
			if (!AcceptedAttempt.bAccepted)
			{
				FStep11BooleanAttemptResult FallbackAttempt = RunStep11BooleanDifferencePass(
					CurrentMesh,
					CutterMeshes[CutterIndex],
					TargetBeforeDiagnostics,
					TargetRenderSign,
					true,
					TEXT("fallback_reversed_pair_orientation"),
					FString::Printf(TEXT("%s/%s result_after_%s_fallback"), *TargetActorName, *TargetComponentName, *Cutters[CutterIndex]->ActorName));
				AttemptDiagnostics.Add(MakeShared<FJsonValueObject>(Step11BooleanAttemptJson(FallbackAttempt)));
				if (FallbackAttempt.bAccepted)
				{
					AcceptedAttempt = MoveTemp(FallbackAttempt);
				}
			}

			OperationJson->SetArrayField(TEXT("attempts"), AttemptDiagnostics);
			if (AcceptedAttempt.bAccepted && !AcceptedAttempt.bEmptyResult)
			{
				OperationJson->SetStringField(TEXT("status"), TEXT("accepted_non_empty"));
				OperationJson->SetObjectField(TEXT("accepted_result"), Step11MeshDiagnosticsJson(AcceptedAttempt.ResultDiagnostics));
				OperationJson->SetStringField(TEXT("accepted_pass"), AcceptedAttempt.PassName);
				OperationJson->SetNumberField(TEXT("volume_delta"), AcceptedAttempt.VolumeDelta);
				OperationJson->SetNumberField(TEXT("min_required_volume_delta"), AcceptedAttempt.MinRequiredVolumeDelta);
				LogStep11MeshDiagnostics(AcceptedAttempt.ResultDiagnostics);
				UE_LOG(
					LogTemp,
					Log,
					TEXT("Step11Diag: boolean accepted target=%s/%s cutter=%s pass=%s result_triangles=%d volume_delta=%.6f result_boundary=%d result_nonmanifold=%d"),
					*TargetActorName,
					*TargetComponentName,
					*Cutters[CutterIndex]->ActorName,
					*AcceptedAttempt.PassName,
					AcceptedAttempt.ResultDiagnostics.TriangleCount,
					AcceptedAttempt.VolumeDelta,
					AcceptedAttempt.ResultDiagnostics.BoundaryEdgeCount,
					AcceptedAttempt.ResultDiagnostics.NonManifoldEdgeCount);
				CurrentMesh = MoveTemp(AcceptedAttempt.ResultMesh);
				OutFinalTriangleSourceMeshById = MoveTemp(AcceptedAttempt.TriangleSourceMeshById);
				OutAcceptedCutterActorNames.Add(Cutters[CutterIndex]->ActorName);
				bModified = true;
			}
			else if (AcceptedAttempt.bAccepted)
			{
				OperationJson->SetStringField(TEXT("status"), TEXT("accepted_empty_result"));
				OperationJson->SetStringField(TEXT("accepted_pass"), AcceptedAttempt.PassName);
				UE_LOG(
					LogTemp,
					Log,
					TEXT("Step11Diag: boolean accepted empty result target=%s/%s cutter=%s pass=%s."),
					*TargetActorName,
					*TargetComponentName,
					*Cutters[CutterIndex]->ActorName,
					*AcceptedAttempt.PassName);
				OutAcceptedCutterActorNames.Add(Cutters[CutterIndex]->ActorName);
				bModified = true;
				bOutEmptyResult = true;
				if (OutOperationDiagnostics)
				{
					OutOperationDiagnostics->Add(MakeShared<FJsonValueObject>(OperationJson));
				}
				break;
			}
			else
			{
				++FailedBooleanCount;
				OperationJson->SetStringField(TEXT("status"), TEXT("rejected"));
				OperationJson->SetStringField(TEXT("reject_reason"), AcceptedAttempt.RejectReason.IsEmpty() ? TEXT("compute_failed") : AcceptedAttempt.RejectReason);
				UE_LOG(
					LogTemp,
					Warning,
					TEXT("Step11: boolean subtract rejected for actor=%s component=%s cutter=%s reason=%s."),
					*TargetActorName,
					*TargetComponentName,
					*Cutters[CutterIndex]->ActorName,
					*OperationJson->GetStringField(TEXT("reject_reason")));
			}

			if (OutOperationDiagnostics)
			{
				OutOperationDiagnostics->Add(MakeShared<FJsonValueObject>(OperationJson));
			}
		}

		return bModified;
	}

	static bool UpdateBooleanResultComponent(UProceduralMeshComponent* Component, const UE::Geometry::FDynamicMesh3& Mesh)
	{
		if (!Component)
		{
			return false;
		}

		TArray<FVector> Vertices;
		TArray<int32> Triangles;
		TArray<FVector> Normals;
		if (!ConvertDynamicMeshToProceduralArrays(Mesh, Vertices, Triangles, Normals))
		{
			return false;
		}

		UMaterialInterface* Material = Component->GetMaterial(0);
		const ECollisionEnabled::Type CollisionEnabled = Component->GetCollisionEnabled();
		Component->ClearAllMeshSections();
		Component->SetWorldTransform(FTransform::Identity, false, nullptr, ETeleportType::TeleportPhysics);

		TArray<FVector2D> UV0;
		UV0.Init(FVector2D::ZeroVector, Vertices.Num());
		TArray<FColor> Colors;
		Colors.Init(FColor::White, Vertices.Num());
		TArray<FProcMeshTangent> Tangents;
		Component->CreateMeshSection(0, Vertices, Triangles, Normals, UV0, Colors, Tangents, CollisionEnabled != ECollisionEnabled::NoCollision);
		Component->SetMaterial(0, Material);
		return true;
	}

	static UProceduralMeshComponent* CreateBooleanResultComponent(
		AActor* Owner,
		UStaticMeshComponent* SourceComponent,
		const UE::Geometry::FDynamicMesh3& Mesh,
		const FString& PressId = FString())
	{
		if (!Owner || !SourceComponent)
		{
			return nullptr;
		}

		TArray<FVector> Vertices;
		TArray<int32> Triangles;
		TArray<FVector> Normals;
		if (!ConvertDynamicMeshToProceduralArrays(Mesh, Vertices, Triangles, Normals))
		{
			return nullptr;
		}

		UMaterialInterface* SourceMaterial = SourceComponent->GetMaterial(0);
		UE_LOG(
			LogTemp,
			Log,
			TEXT("Step11Diag: boolean result component material source_actor=%s source_component=%s material_slot=0 material=%s."),
			*GetNameSafe(SourceComponent->GetOwner()),
			*GetNameSafe(SourceComponent),
			*GetNameSafe(SourceMaterial));

		const FName ComponentName = MakeUniqueObjectName(Owner, UProceduralMeshComponent::StaticClass(), TEXT("FromLZ_Step11BooleanMesh"));
		UProceduralMeshComponent* ResultComponent = NewObject<UProceduralMeshComponent>(Owner, ComponentName);
		if (!ResultComponent)
		{
			return nullptr;
		}

		ResultComponent->ComponentTags.AddUnique(Step11BooleanResultTag);
		if (!PressId.IsEmpty())
		{
			ResultComponent->ComponentTags.AddUnique(Step11GeneratedByPressTag(PressId));
		}
		ResultComponent->SetMobility(EComponentMobility::Movable);
		ResultComponent->SetCollisionEnabled(SourceComponent->GetCollisionEnabled());
		ResultComponent->bUseAsyncCooking = false;
		ResultComponent->SetCastShadow(SourceComponent->CastShadow);
		Owner->AddInstanceComponent(ResultComponent);

		TArray<FVector2D> UV0;
		UV0.Init(FVector2D::ZeroVector, Vertices.Num());
		TArray<FColor> Colors;
		Colors.Init(FColor::White, Vertices.Num());
		TArray<FProcMeshTangent> Tangents;
		ResultComponent->CreateMeshSection(0, Vertices, Triangles, Normals, UV0, Colors, Tangents, SourceComponent->GetCollisionEnabled() != ECollisionEnabled::NoCollision);
		ResultComponent->SetMaterial(0, SourceMaterial);
		ResultComponent->RegisterComponent();
		ResultComponent->SetWorldTransform(FTransform::Identity);
		return ResultComponent;
	}

	static AActor* CreateBooleanResultActor(
		UWorld* World,
		UPrimitiveComponent* SourceComponent,
		const FString& SourceActorName,
		const FString& SourceComponentName,
		const FString& PressId,
		const UE::Geometry::FDynamicMesh3& Mesh,
		const TArray<int8>& TriangleSourceMeshById)
	{
		if (!World || !SourceComponent || Mesh.TriangleCount() <= 0)
		{
			return nullptr;
		}

		const FBox Bounds = BuildDynamicMeshBounds(Mesh);
		const FVector Origin = Bounds.IsValid ? Bounds.GetCenter() : FVector::ZeroVector;
		FStep11ProcMeshSectionData SourceSection;
		FStep11ProcMeshSectionData CapSection;
		const TArray<int8>* SourceMapPtr = TriangleSourceMeshById.Num() > 0 ? &TriangleSourceMeshById : nullptr;
		if (!ConvertDynamicMeshToFlatProceduralSections(Mesh, Origin, SourceMapPtr, SourceSection, CapSection))
		{
			return nullptr;
		}

		UMaterialInterface* SourceMaterial = SourceComponent->GetMaterial(0);
		UE_LOG(
			LogTemp,
			Log,
			TEXT("Step11Diag: boolean result actor material source_actor=%s source_component=%s material_slot=0 material=%s source_section=%d cap_section=%d."),
			*GetNameSafe(SourceComponent->GetOwner()),
			*GetNameSafe(SourceComponent),
			*GetNameSafe(SourceMaterial),
			SourceSection.HasGeometry() ? 1 : 0,
			CapSection.HasGeometry() ? 1 : 0);

		const FString BaseName = ObjSafeName(FString::Printf(TEXT("FromLZ_BooleanResult_%s_%s"), *SourceActorName, *SourceComponentName));
		FActorSpawnParameters Params;
		Params.Name = MakeUniqueObjectName(World->GetCurrentLevel(), AActor::StaticClass(), FName(*BaseName));
		AActor* ResultActor = World->SpawnActor<AActor>(AActor::StaticClass(), FTransform(Origin), Params);
		if (!ResultActor)
		{
			return nullptr;
		}

		ResultActor->Tags.AddUnique(Step11BooleanResultTag);
		ResultActor->Tags.AddUnique(Step11GeneratedByPressTag(PressId));
#if WITH_EDITOR
		ResultActor->SetActorLabel(BaseName);
#endif

		UProceduralMeshComponent* MeshComponent = NewObject<UProceduralMeshComponent>(ResultActor, TEXT("FromLZ_Step11BooleanMesh"));
		if (!MeshComponent)
		{
			ResultActor->Destroy();
			return nullptr;
		}

		MeshComponent->ComponentTags.AddUnique(Step11BooleanResultTag);
		MeshComponent->ComponentTags.AddUnique(Step11GeneratedByPressTag(PressId));
		MeshComponent->SetMobility(EComponentMobility::Movable);
		MeshComponent->SetCollisionEnabled(SourceComponent->GetCollisionEnabled());
		MeshComponent->bUseAsyncCooking = false;
		MeshComponent->SetCastShadow(SourceComponent->CastShadow);
		ResultActor->SetRootComponent(MeshComponent);
		ResultActor->AddInstanceComponent(MeshComponent);
		MeshComponent->RegisterComponent();

		if (SourceSection.HasGeometry())
		{
			MeshComponent->CreateMeshSection(
				0,
				SourceSection.Vertices,
				SourceSection.Triangles,
				SourceSection.Normals,
				SourceSection.UV0,
				SourceSection.Colors,
				SourceSection.Tangents,
				SourceComponent->GetCollisionEnabled() != ECollisionEnabled::NoCollision);
			MeshComponent->SetMaterial(0, SourceMaterial);
		}
		if (CapSection.HasGeometry())
		{
			const int32 SectionIndex = SourceSection.HasGeometry() ? 1 : 0;
			MeshComponent->CreateMeshSection(
				SectionIndex,
				CapSection.Vertices,
				CapSection.Triangles,
				CapSection.Normals,
				CapSection.UV0,
				CapSection.Colors,
				CapSection.Tangents,
				SourceComponent->GetCollisionEnabled() != ECollisionEnabled::NoCollision);
			MeshComponent->SetMaterial(SectionIndex, SourceMaterial);
		}

		ResultActor->SetActorLocation(Origin, false, nullptr, ETeleportType::TeleportPhysics);
		return ResultActor;
	}

	static void HideStep11SourceComponentForPress(UPrimitiveComponent* Component, const FString& PressId)
	{
		if (!Component)
		{
			return;
		}
		Component->ComponentTags.AddUnique(Step11HiddenSourceTag);
		Component->ComponentTags.AddUnique(Step11HiddenByPressTag(PressId));
		Component->SetVisibility(false, true);
		Component->SetHiddenInGame(true, true);
	}

	static FFromLZStep11UndoResult RestoreStep11BooleanResults(UWorld* World, const FString& PressId)
	{
		FFromLZStep11UndoResult Result;
		Result.PressId = PressId;

		if (!World)
		{
			Result.Message = TEXT("World is no longer valid.");
			return Result;
		}

		if (PressId.IsEmpty())
		{
			UE_LOG(LogTemp, Warning, TEXT("Step11: no active press id; skipped press-scoped restore."));
			Result.Message = TEXT("No active Step11 press to undo.");
			return Result;
		}

		const FName GeneratedByTag = Step11GeneratedByPressTag(PressId);
		const FName HiddenByTag = Step11HiddenByPressTag(PressId);
		const FString UndoDiagnosticPath = FPaths::ProjectSavedDir() / TEXT("2DDebug") / PressId / TEXT("11_undo_diagnostics.json");
		Result.UndoDiagnosticPath = UndoDiagnosticPath;
		TSharedRef<FJsonObject> UndoRoot = MakeShared<FJsonObject>();
		UndoRoot->SetNumberField(TEXT("diagnostic_version"), 2);
		UndoRoot->SetStringField(TEXT("press_id"), PressId);
		UndoRoot->SetStringField(TEXT("active_undo_press_id"), GActiveUndoPressId);
		UndoRoot->SetStringField(TEXT("generated_by_tag"), GeneratedByTag.ToString());
		UndoRoot->SetStringField(TEXT("hidden_by_tag"), HiddenByTag.ToString());

		TArray<AActor*> ActorsToDestroy;
		TArray<UProceduralMeshComponent*> GeneratedComponents;
		TArray<UPrimitiveComponent*> HiddenSourceComponents;
		int32 SkippedOtherGeneratedActorCount = 0;
		int32 SkippedOtherGeneratedComponentCount = 0;
		int32 SkippedOtherHiddenSourceComponentCount = 0;
		for (TActorIterator<AActor> It(World); It; ++It)
		{
			AActor* Actor = *It;
			if (!Actor)
			{
				continue;
			}

			const bool bCurrentGeneratedActor = Actor->ActorHasTag(GeneratedByTag);
			if (bCurrentGeneratedActor && ActorHasAnyStep11RuntimeTag(Actor))
			{
				ActorsToDestroy.Add(Actor);
				continue;
			}
			if (!bCurrentGeneratedActor && ActorHasAnyStep11RuntimeTag(Actor))
			{
				++SkippedOtherGeneratedActorCount;
			}

			TArray<UProceduralMeshComponent*> ProceduralComponents;
			Actor->GetComponents<UProceduralMeshComponent>(ProceduralComponents);
			for (UProceduralMeshComponent* Component : ProceduralComponents)
			{
				if (Component &&
					Component->ComponentTags.Contains(Step11BooleanResultTag) &&
					Component->ComponentTags.Contains(GeneratedByTag))
				{
					GeneratedComponents.Add(Component);
				}
				else if (Component &&
					Component->ComponentTags.Contains(Step11BooleanResultTag) &&
					!Component->ComponentTags.Contains(HiddenByTag))
				{
					++SkippedOtherGeneratedComponentCount;
				}
			}

			TArray<UPrimitiveComponent*> PrimitiveComponents;
			Actor->GetComponents<UPrimitiveComponent>(PrimitiveComponents);
			for (UPrimitiveComponent* Component : PrimitiveComponents)
			{
				if (Component &&
					Component->ComponentTags.Contains(Step11HiddenSourceTag) &&
					Component->ComponentTags.Contains(HiddenByTag))
				{
					HiddenSourceComponents.Add(Component);
				}
				else if (Component && Component->ComponentTags.Contains(Step11HiddenSourceTag))
				{
					++SkippedOtherHiddenSourceComponentCount;
				}
			}
		}

		for (UProceduralMeshComponent* Component : GeneratedComponents)
		{
			if (Component)
			{
				Component->DestroyComponent();
			}
		}
		for (AActor* Actor : ActorsToDestroy)
		{
			if (Actor)
			{
				Actor->Destroy();
			}
		}
		for (UPrimitiveComponent* Component : HiddenSourceComponents)
		{
			if (Component)
			{
				Component->ComponentTags.Remove(HiddenByTag);
				if (!ComponentHasAnyStep11HiddenByTag(Component))
				{
					Component->SetVisibility(true, true);
					Component->SetHiddenInGame(false, true);
					Component->ComponentTags.Remove(Step11HiddenSourceTag);
				}
			}
		}
		UndoRoot->SetNumberField(TEXT("destroyed_actor_count"), ActorsToDestroy.Num());
		UndoRoot->SetNumberField(TEXT("destroyed_boolean_component_count"), GeneratedComponents.Num());
		UndoRoot->SetNumberField(TEXT("restored_hidden_source_count"), HiddenSourceComponents.Num());
		UndoRoot->SetNumberField(TEXT("skipped_other_press_actor_count"), SkippedOtherGeneratedActorCount);
		UndoRoot->SetNumberField(TEXT("skipped_other_press_component_count"), SkippedOtherGeneratedComponentCount + SkippedOtherHiddenSourceComponentCount);
		UndoRoot->SetNumberField(TEXT("skipped_other_generated_actor_count"), SkippedOtherGeneratedActorCount);
		UndoRoot->SetNumberField(TEXT("skipped_other_generated_component_count"), SkippedOtherGeneratedComponentCount);
		UndoRoot->SetNumberField(TEXT("skipped_other_hidden_source_component_count"), SkippedOtherHiddenSourceComponentCount);
		IFileManager::Get().MakeDirectory(*FPaths::GetPath(UndoDiagnosticPath), true);
		if (SaveJsonObject(UndoRoot, UndoDiagnosticPath))
		{
			Result.OutputFiles.Add(UndoDiagnosticPath);
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("Step11: failed to save undo diagnostics %s"), *UndoDiagnosticPath);
		}

		Result.bSuccess = true;
		Result.Message = FString::Printf(TEXT("Step11 undo completed for %s."), *PressId);
		Result.DestroyedActorCount = ActorsToDestroy.Num();
		Result.DestroyedBooleanComponentCount = GeneratedComponents.Num();
		Result.RestoredHiddenSourceCount = HiddenSourceComponents.Num();
		UE_LOG(
			LogTemp,
			Log,
			TEXT("Step11: press-scoped restore press=%s restored %d hidden-by source component(s), removed %d generated boolean result component(s), destroyed %d generated runtime actor(s), skipped_other_generated_actors=%d skipped_other_generated_components=%d skipped_other_hidden_sources=%d."),
			*PressId,
			HiddenSourceComponents.Num(),
			GeneratedComponents.Num(),
			ActorsToDestroy.Num(),
			SkippedOtherGeneratedActorCount,
			SkippedOtherGeneratedComponentCount,
			SkippedOtherHiddenSourceComponentCount);
		return Result;
	}

	static void ApplyStep11BooleanOperations(
		UWorld* World,
		const TArray<FReconstructedMesh>& Meshes,
		const FString& PressId,
		const FString& DiagnosticJsonPath,
		TSet<FString>& OutAcceptedCutterActorNames)
	{
		if (!World)
		{
			return;
		}

		OutAcceptedCutterActorNames.Reset();
		TSharedRef<FJsonObject> DiagnosticsRoot = MakeShared<FJsonObject>();
		DiagnosticsRoot->SetNumberField(TEXT("diagnostic_version"), 2);
		DiagnosticsRoot->SetStringField(TEXT("press_id"), PressId);
		DiagnosticsRoot->SetStringField(TEXT("active_undo_press_id"), GActiveUndoPressId);
		DiagnosticsRoot->SetStringField(TEXT("boolean_backend"), FFromLZManifoldBoolean::BackendName());
		DiagnosticsRoot->SetStringField(TEXT("manifold_library_version"), FFromLZManifoldBoolean::LibraryVersion());
		DiagnosticsRoot->SetNumberField(TEXT("excavation_cutter_normal_scale"), ExcavationCutterNormalScale);
		DiagnosticsRoot->SetNumberField(TEXT("excavation_cutter_cap_scale"), ExcavationCutterCapScale);
		DiagnosticsRoot->SetNumberField(TEXT("min_renderable_edge_cm"), Step11BooleanMinRenderableEdgeCm);

		TArray<TSharedPtr<FJsonValue>> InputMeshDiagnostics;
		TArray<TSharedPtr<FJsonValue>> CutterDiagnosticsJson;
		TArray<TSharedPtr<FJsonValue>> TargetDiagnosticsJson;
		TArray<TSharedPtr<FJsonValue>> BuildFailureDiagnostics;

		TArray<const FReconstructedMesh*> Cutters;
		TArray<UE::Geometry::FDynamicMesh3> CutterMeshes;
		TArray<FBox> CutterBounds;
		TArray<FStep11MeshDiagnostics> CutterDiagnostics;
		for (const FReconstructedMesh& Mesh : Meshes)
		{
			TSharedRef<FJsonObject> InputObject = MakeShared<FJsonObject>();
			InputObject->SetStringField(TEXT("actor_name"), Mesh.ActorName);
			InputObject->SetStringField(TEXT("tag"), Mesh.Tag.ToString());
			InputObject->SetBoolField(TEXT("is_excavate_cutter"), Mesh.bIsExcavateCutter);
			InputObject->SetNumberField(TEXT("input_world_vertex_count"), Mesh.VerticesWorld.Num());
			InputObject->SetNumberField(TEXT("input_triangle_index_count"), Mesh.Triangles.Num());
			InputObject->SetNumberField(TEXT("input_triangle_count"), Mesh.Triangles.Num() / 3);
			InputObject->SetArrayField(TEXT("normal"), JsonVector(Mesh.Normal)->AsArray());
			InputObject->SetObjectField(TEXT("input_bounds"), Step11BoxJson(BuildWorldBounds(Mesh.VerticesWorld)));
			InputMeshDiagnostics.Add(MakeShared<FJsonValueObject>(InputObject));

			if (!Mesh.bIsExcavateCutter || Mesh.VerticesWorld.Num() < 3 || Mesh.Triangles.Num() < 3)
			{
				UE_LOG(
					LogTemp,
					Log,
					TEXT("Step11Diag: reconstruction mesh skipped as cutter actor=%s excavate=%d vertices=%d triangle_indices=%d."),
					*Mesh.ActorName,
					Mesh.bIsExcavateCutter ? 1 : 0,
					Mesh.VerticesWorld.Num(),
					Mesh.Triangles.Num());
				continue;
			}

			UE::Geometry::FDynamicMesh3 CutterMesh;
			if (!BuildDynamicMeshFromWorldMesh(Mesh.VerticesWorld, Mesh.Triangles, CutterMesh))
			{
				UE_LOG(LogTemp, Warning, TEXT("Step11: failed to build cutter dynamic mesh for %s."), *Mesh.ActorName);
				TSharedRef<FJsonObject> FailureObject = MakeShared<FJsonObject>();
				FailureObject->SetStringField(TEXT("kind"), TEXT("cutter_build_failed"));
				FailureObject->SetStringField(TEXT("actor_name"), Mesh.ActorName);
				FailureObject->SetNumberField(TEXT("input_world_vertex_count"), Mesh.VerticesWorld.Num());
				FailureObject->SetNumberField(TEXT("input_triangle_index_count"), Mesh.Triangles.Num());
				BuildFailureDiagnostics.Add(MakeShared<FJsonValueObject>(FailureObject));
				continue;
			}

			Cutters.Add(&Mesh);
			CutterBounds.Add(BuildWorldBounds(Mesh.VerticesWorld));
			FStep11MeshDiagnostics Diagnostics = AnalyzeStep11DynamicMesh(CutterMesh, Mesh.ActorName, TEXT("excavate_cutter"));
			CutterDiagnostics.Add(Diagnostics);
			LogStep11MeshDiagnostics(Diagnostics);
			CutterDiagnosticsJson.Add(MakeShared<FJsonValueObject>(Step11MeshDiagnosticsJson(Diagnostics)));
			CutterMeshes.Add(MoveTemp(CutterMesh));
		}

		DiagnosticsRoot->SetArrayField(TEXT("input_reconstruction_meshes"), InputMeshDiagnostics);
		DiagnosticsRoot->SetArrayField(TEXT("cutters"), CutterDiagnosticsJson);
		DiagnosticsRoot->SetArrayField(TEXT("build_failures"), BuildFailureDiagnostics);

		if (Cutters.Num() == 0)
		{
			TSharedRef<FJsonObject> Summary = MakeShared<FJsonObject>();
			Summary->SetNumberField(TEXT("excavate_cutter_count"), 0);
			Summary->SetNumberField(TEXT("updated_static_component_count"), 0);
			Summary->SetNumberField(TEXT("updated_procedural_component_count"), 0);
			Summary->SetNumberField(TEXT("created_boolean_result_actor_count"), 0);
			Summary->SetNumberField(TEXT("failed_boolean_count"), 0);
			Summary->SetStringField(TEXT("status"), TEXT("no_excavate_cutters"));
			DiagnosticsRoot->SetObjectField(TEXT("summary"), Summary);
			DiagnosticsRoot->SetArrayField(TEXT("targets"), TargetDiagnosticsJson);
			if (!DiagnosticJsonPath.IsEmpty() && SaveJsonObject(DiagnosticsRoot, DiagnosticJsonPath))
			{
				UE_LOG(LogTemp, Log, TEXT("Step11Diag: saved diagnostics %s"), *DiagnosticJsonPath);
			}
			return;
		}

		TArray<UProceduralMeshComponent*> ProceduralTargetComponents;
		for (TActorIterator<AActor> It(World); It; ++It)
		{
			AActor* Actor = *It;
			if (!Actor || Actor->IsHidden() ||
				Actor->ActorHasTag(ReconstructedFaceTag) ||
				ActorIsStep11Cutter(Actor))
			{
				continue;
			}

			TArray<UProceduralMeshComponent*> ProceduralComponents;
			Actor->GetComponents<UProceduralMeshComponent>(ProceduralComponents);
			for (UProceduralMeshComponent* Component : ProceduralComponents)
			{
				if (!Component || !Component->IsRegistered() || !Component->IsVisible())
				{
					continue;
				}

				const bool bBooleanResultComponent = Component->ComponentTags.Contains(Step11BooleanResultTag);
				const bool bAttachProceduralComponent =
					Actor->ActorHasTag(Step11ActionAttachTag) ||
					Component->ComponentTags.Contains(Step11ActionAttachTag);
				if (!bBooleanResultComponent && !bAttachProceduralComponent)
				{
					continue;
				}
				if (!ActorHasActiveStep11GeneratedByTag(Actor) && !ComponentHasActiveStep11GeneratedByTag(Component))
				{
					continue;
				}

				ProceduralTargetComponents.Add(Component);
			}
		}

		int32 UpdatedStaticComponentCount = 0;
		int32 UpdatedProceduralComponentCount = 0;
		int32 CreatedBooleanResultActorCount = 0;
		int32 FailedBooleanCount = 0;
		for (UProceduralMeshComponent* Component : ProceduralTargetComponents)
		{
			AActor* Actor = Component ? Component->GetOwner() : nullptr;
			if (!Actor || !Component->IsRegistered() || !Component->IsVisible())
			{
				continue;
			}

			UE::Geometry::FDynamicMesh3 CurrentMesh;
			if (!BuildDynamicMeshFromProceduralMeshComponent(Component, CurrentMesh))
			{
				TSharedRef<FJsonObject> TargetObject = MakeShared<FJsonObject>();
				TargetObject->SetStringField(TEXT("target_actor"), Actor->GetName());
				TargetObject->SetStringField(TEXT("target_component"), Component->GetName());
				TargetObject->SetStringField(TEXT("source_type"), Component->ComponentTags.Contains(Step11BooleanResultTag) ? TEXT("prior_boolean_result") : TEXT("attach_procedural"));
				TargetObject->SetStringField(TEXT("status"), TEXT("build_dynamic_mesh_failed"));
				TargetDiagnosticsJson.Add(MakeShared<FJsonValueObject>(TargetObject));
				UE_LOG(
					LogTemp,
					Warning,
					TEXT("Step11Diag: failed to build procedural target mesh actor=%s component=%s."),
					*Actor->GetName(),
					*Component->GetName());
				continue;
			}

			const FString SourceType = Component->ComponentTags.Contains(Step11BooleanResultTag) ? TEXT("prior_boolean_result") : TEXT("attach_procedural");
			FStep11MeshDiagnostics InitialDiagnostics = AnalyzeStep11DynamicMesh(
				CurrentMesh,
				FString::Printf(TEXT("%s/%s"), *Actor->GetName(), *Component->GetName()),
				SourceType);
			LogStep11MeshDiagnostics(InitialDiagnostics);
			TArray<TSharedPtr<FJsonValue>> OperationDiagnostics;
			TArray<int8> FinalTriangleSourceMeshById;
			TSet<FString> TargetAcceptedCutters;

			bool bEmptyResult = false;
			const bool bModified = ApplyCuttersToDynamicMesh(
				CurrentMesh,
				Actor->GetName(),
				Component->GetName(),
				SourceType,
				Cutters,
				CutterMeshes,
				CutterBounds,
				CutterDiagnostics,
				&OperationDiagnostics,
				FinalTriangleSourceMeshById,
				TargetAcceptedCutters,
				bEmptyResult,
				FailedBooleanCount);

			TSharedRef<FJsonObject> TargetObject = MakeShared<FJsonObject>();
			TargetObject->SetStringField(TEXT("target_actor"), Actor->GetName());
			TargetObject->SetStringField(TEXT("target_component"), Component->GetName());
			TargetObject->SetStringField(TEXT("source_type"), SourceType);
			TargetObject->SetObjectField(TEXT("initial"), Step11MeshDiagnosticsJson(InitialDiagnostics));
			TargetObject->SetBoolField(TEXT("modified"), bModified);
			TargetObject->SetBoolField(TEXT("empty_result"), bEmptyResult);
			TargetObject->SetArrayField(TEXT("operations"), OperationDiagnostics);
			if (!bModified)
			{
				TargetObject->SetStringField(TEXT("status"), TEXT("not_modified"));
				TargetDiagnosticsJson.Add(MakeShared<FJsonValueObject>(TargetObject));
				continue;
			}

			if (bEmptyResult)
			{
				HideStep11SourceComponentForPress(Component, PressId);
				TargetObject->SetStringField(TEXT("status"), TEXT("target_fully_removed_empty_result"));
				MergeStep11StringSet(OutAcceptedCutterActorNames, TargetAcceptedCutters);
				++UpdatedProceduralComponentCount;
			}
			else
			{
				AActor* ResultActor = CreateBooleanResultActor(
					World,
					Component,
					Actor->GetName(),
					Component->GetName(),
					PressId,
					CurrentMesh,
					FinalTriangleSourceMeshById);
				if (ResultActor)
				{
					HideStep11SourceComponentForPress(Component, PressId);
					MergeStep11StringSet(OutAcceptedCutterActorNames, TargetAcceptedCutters);
					TargetObject->SetStringField(TEXT("status"), TEXT("created_boolean_result_actor"));
					TargetObject->SetStringField(TEXT("result_actor"), ResultActor->GetName());
					TargetObject->SetObjectField(
						TEXT("final"),
						Step11MeshDiagnosticsJson(AnalyzeStep11DynamicMesh(CurrentMesh, FString::Printf(TEXT("%s/%s final"), *Actor->GetName(), *Component->GetName()), TEXT("procedural_boolean_result"))));
					++UpdatedProceduralComponentCount;
					++CreatedBooleanResultActorCount;
				}
				else
				{
					TargetObject->SetStringField(TEXT("status"), TEXT("failed_to_create_boolean_result_actor"));
				}
			}
			TargetDiagnosticsJson.Add(MakeShared<FJsonValueObject>(TargetObject));
		}

		for (TActorIterator<AActor> It(World); It; ++It)
		{
			AActor* Actor = *It;
			if (!Actor || Actor->IsHidden() ||
				Actor->ActorHasTag(ReconstructedFaceTag) ||
				ActorHasAnyStep11RuntimeTag(Actor) ||
				!ActorIsTaggedBaseCaptureSubject(Actor))
			{
				continue;
			}

			TArray<UStaticMeshComponent*> StaticComponents;
			Actor->GetComponents<UStaticMeshComponent>(StaticComponents);
			for (UStaticMeshComponent* Component : StaticComponents)
			{
				if (!Component || !Component->IsRegistered() || !Component->IsVisible() || !Component->GetStaticMesh())
				{
					continue;
				}

				UE::Geometry::FDynamicMesh3 CurrentMesh;
				if (!BuildDynamicMeshFromStaticMeshComponent(Component, CurrentMesh))
				{
					TSharedRef<FJsonObject> TargetObject = MakeShared<FJsonObject>();
					TargetObject->SetStringField(TEXT("target_actor"), Actor->GetName());
					TargetObject->SetStringField(TEXT("target_component"), Component->GetName());
					TargetObject->SetStringField(TEXT("source_type"), TEXT("static_mesh"));
					TargetObject->SetStringField(TEXT("status"), TEXT("build_dynamic_mesh_failed"));
					TargetDiagnosticsJson.Add(MakeShared<FJsonValueObject>(TargetObject));
					UE_LOG(
						LogTemp,
						Warning,
						TEXT("Step11Diag: failed to build static target mesh actor=%s component=%s."),
						*Actor->GetName(),
						*Component->GetName());
					continue;
				}

				FStep11MeshDiagnostics InitialDiagnostics = AnalyzeStep11DynamicMesh(
					CurrentMesh,
					FString::Printf(TEXT("%s/%s"), *Actor->GetName(), *Component->GetName()),
					TEXT("static_mesh"));
				LogStep11MeshDiagnostics(InitialDiagnostics);
				TArray<TSharedPtr<FJsonValue>> OperationDiagnostics;
				TArray<int8> FinalTriangleSourceMeshById;
				TSet<FString> TargetAcceptedCutters;

				bool bEmptyResult = false;
				const bool bModified = ApplyCuttersToDynamicMesh(
					CurrentMesh,
					Actor->GetName(),
					Component->GetName(),
					TEXT("static_mesh"),
					Cutters,
					CutterMeshes,
					CutterBounds,
					CutterDiagnostics,
					&OperationDiagnostics,
					FinalTriangleSourceMeshById,
					TargetAcceptedCutters,
					bEmptyResult,
					FailedBooleanCount);

				TSharedRef<FJsonObject> TargetObject = MakeShared<FJsonObject>();
				TargetObject->SetStringField(TEXT("target_actor"), Actor->GetName());
				TargetObject->SetStringField(TEXT("target_component"), Component->GetName());
				TargetObject->SetStringField(TEXT("source_type"), TEXT("static_mesh"));
				TargetObject->SetObjectField(TEXT("initial"), Step11MeshDiagnosticsJson(InitialDiagnostics));
				TargetObject->SetBoolField(TEXT("modified"), bModified);
				TargetObject->SetBoolField(TEXT("empty_result"), bEmptyResult);
				TargetObject->SetArrayField(TEXT("operations"), OperationDiagnostics);
				if (!bModified)
				{
					TargetObject->SetStringField(TEXT("status"), TEXT("not_modified"));
					TargetDiagnosticsJson.Add(MakeShared<FJsonValueObject>(TargetObject));
					continue;
				}

				if (bEmptyResult)
				{
					HideStep11SourceComponentForPress(Component, PressId);
					MergeStep11StringSet(OutAcceptedCutterActorNames, TargetAcceptedCutters);
					TargetObject->SetStringField(TEXT("status"), TEXT("target_fully_removed_empty_result"));
					++UpdatedStaticComponentCount;
				}
				else
				{
					AActor* ResultActor = CreateBooleanResultActor(
						World,
						Component,
						Actor->GetName(),
						Component->GetName(),
						PressId,
						CurrentMesh,
						FinalTriangleSourceMeshById);
					if (ResultActor)
					{
						HideStep11SourceComponentForPress(Component, PressId);
						MergeStep11StringSet(OutAcceptedCutterActorNames, TargetAcceptedCutters);
						TargetObject->SetStringField(TEXT("status"), TEXT("created_boolean_result_actor"));
						TargetObject->SetStringField(TEXT("result_actor"), ResultActor->GetName());
						TargetObject->SetObjectField(
							TEXT("final"),
							Step11MeshDiagnosticsJson(AnalyzeStep11DynamicMesh(CurrentMesh, FString::Printf(TEXT("%s/%s final"), *Actor->GetName(), *Component->GetName()), TEXT("static_mesh_boolean_result"))));
						++UpdatedStaticComponentCount;
						++CreatedBooleanResultActorCount;
					}
					else
					{
						TargetObject->SetStringField(TEXT("status"), TEXT("failed_to_create_boolean_result_actor"));
					}
				}
				TargetDiagnosticsJson.Add(MakeShared<FJsonValueObject>(TargetObject));
			}
		}

		UE_LOG(
			LogTemp,
			Log,
			TEXT("Step11: press=%s applied %d excavate cutter(s), updated %d static mesh component(s), updated %d procedural component(s), created %d boolean result actor(s), failed boolean attempts=%d."),
			*PressId,
			Cutters.Num(),
			UpdatedStaticComponentCount,
			UpdatedProceduralComponentCount,
			CreatedBooleanResultActorCount,
			FailedBooleanCount);

		TSharedRef<FJsonObject> Summary = MakeShared<FJsonObject>();
		Summary->SetNumberField(TEXT("excavate_cutter_count"), Cutters.Num());
		Summary->SetNumberField(TEXT("updated_static_component_count"), UpdatedStaticComponentCount);
		Summary->SetNumberField(TEXT("updated_procedural_component_count"), UpdatedProceduralComponentCount);
		Summary->SetNumberField(TEXT("created_boolean_result_actor_count"), CreatedBooleanResultActorCount);
		Summary->SetNumberField(TEXT("failed_boolean_count"), FailedBooleanCount);
		Summary->SetNumberField(TEXT("target_record_count"), TargetDiagnosticsJson.Num());
		Summary->SetNumberField(TEXT("accepted_cutter_actor_count"), OutAcceptedCutterActorNames.Num());
		Summary->SetStringField(TEXT("status"), FailedBooleanCount > 0 ? TEXT("completed_with_boolean_failures") : TEXT("completed"));
		DiagnosticsRoot->SetObjectField(TEXT("summary"), Summary);
		DiagnosticsRoot->SetArrayField(TEXT("targets"), TargetDiagnosticsJson);
		if (!DiagnosticJsonPath.IsEmpty())
		{
			if (SaveJsonObject(DiagnosticsRoot, DiagnosticJsonPath))
			{
				UE_LOG(LogTemp, Log, TEXT("Step11Diag: saved diagnostics %s"), *DiagnosticJsonPath);
			}
			else
			{
				UE_LOG(LogTemp, Warning, TEXT("Step11Diag: failed to save diagnostics %s"), *DiagnosticJsonPath);
			}
		}
	}

	static FVector UnrealWorldToObjDebugSpace(const FVector& WorldPosition)
	{
		// UE is left-handed Z-up (X forward, Y right, Z up). OBJ viewers normally
		// interpret vertices in a right-handed space; mirror Y for debug export only.
		return FVector(WorldPosition.X, -WorldPosition.Y, WorldPosition.Z);
	}

	static int32 AppendObjWorldMesh(
		FString& Obj,
		int32& VertexOffset,
		const FString& ObjectName,
		const TCHAR* MaterialName,
		const TArray<FVector>& VerticesWorld,
		const TArray<int32>& Triangles)
	{
		if (VerticesWorld.Num() < 3 || Triangles.Num() < 3)
		{
			return 0;
		}

		const int32 BaseVertex = VertexOffset;
		Obj += FString::Printf(TEXT("\no %s\nusemtl %s\n"), *ObjSafeName(ObjectName), MaterialName);
		for (const FVector& V : VerticesWorld)
		{
			const FVector ObjPosition = UnrealWorldToObjDebugSpace(V);
			Obj += FString::Printf(TEXT("v %.6f %.6f %.6f\n"), ObjPosition.X, ObjPosition.Y, ObjPosition.Z);
		}

		int32 FaceCount = 0;
		for (int32 i = 0; i + 2 < Triangles.Num(); i += 3)
		{
			const int32 A = Triangles[i];
			const int32 B = Triangles[i + 1];
			const int32 C = Triangles[i + 2];
			if (!VerticesWorld.IsValidIndex(A) || !VerticesWorld.IsValidIndex(B) || !VerticesWorld.IsValidIndex(C))
			{
				continue;
			}
			Obj += FString::Printf(TEXT("f %d %d %d\n"), BaseVertex + A + 1, BaseVertex + C + 1, BaseVertex + B + 1);
			++FaceCount;
		}
		VertexOffset += VerticesWorld.Num();
		return FaceCount;
	}

	static int32 AppendStaticMeshComponentObj(FString& Obj, int32& VertexOffset, UStaticMeshComponent* Component)
	{
		if (!Component || !Component->IsRegistered() || !Component->IsVisible())
		{
			return 0;
		}

		UStaticMesh* StaticMesh = Component->GetStaticMesh();
		if (!StaticMesh || !StaticMesh->GetRenderData() || StaticMesh->GetRenderData()->LODResources.Num() == 0)
		{
			return 0;
		}

		const FStaticMeshLODResources& LOD = StaticMesh->GetRenderData()->LODResources[0];
		const FPositionVertexBuffer& PositionBuffer = LOD.VertexBuffers.PositionVertexBuffer;
		const int32 NumVertices = PositionBuffer.GetNumVertices();
		if (NumVertices <= 0 || LOD.Sections.Num() == 0)
		{
			return 0;
		}

		const FIndexArrayView Indices = LOD.IndexBuffer.GetArrayView();
		if (Indices.Num() < 3)
		{
			return 0;
		}

		const int32 BaseVertex = VertexOffset;
		const FString OwnerName = Component->GetOwner() ? Component->GetOwner()->GetName() : TEXT("StaticActor");
		Obj += FString::Printf(
			TEXT("\no %s_%s\nusemtl scene_gray\n"),
			*ObjSafeName(OwnerName),
			*ObjSafeName(Component->GetName()));

		const FTransform& ComponentTransform = Component->GetComponentTransform();
		for (int32 VertexIndex = 0; VertexIndex < NumVertices; ++VertexIndex)
		{
			const FVector3f LocalPosition = PositionBuffer.VertexPosition(VertexIndex);
			const FVector WorldPosition = ComponentTransform.TransformPosition(
				FVector(double(LocalPosition.X), double(LocalPosition.Y), double(LocalPosition.Z)));
			const FVector ObjPosition = UnrealWorldToObjDebugSpace(WorldPosition);
			Obj += FString::Printf(TEXT("v %.6f %.6f %.6f\n"), ObjPosition.X, ObjPosition.Y, ObjPosition.Z);
		}

		int32 FaceCount = 0;
		for (const FStaticMeshSection& Section : LOD.Sections)
		{
			for (uint32 TriIndex = 0; TriIndex < Section.NumTriangles; ++TriIndex)
			{
				const uint32 IndexBase = Section.FirstIndex + TriIndex * 3;
				if (IndexBase + 2 >= uint32(Indices.Num()))
				{
					continue;
				}

				const int32 A = int32(Indices[IndexBase]);
				const int32 B = int32(Indices[IndexBase + 1]);
				const int32 C = int32(Indices[IndexBase + 2]);
				if (A < 0 || B < 0 || C < 0 || A >= NumVertices || B >= NumVertices || C >= NumVertices)
				{
					continue;
				}

				Obj += FString::Printf(TEXT("f %d %d %d\n"), BaseVertex + A + 1, BaseVertex + C + 1, BaseVertex + B + 1);
				++FaceCount;
			}
		}

		VertexOffset += NumVertices;
		return FaceCount;
	}

	static int32 AppendProceduralMeshComponentObj(FString& Obj, int32& VertexOffset, UProceduralMeshComponent* Component)
	{
		if (!Component || !Component->IsRegistered() || !Component->IsVisible() || !Component->ComponentTags.Contains(Step11BooleanResultTag))
		{
			return 0;
		}

		const FString OwnerName = Component->GetOwner() ? Component->GetOwner()->GetName() : TEXT("BooleanActor");
		const FTransform& ComponentTransform = Component->GetComponentTransform();
		int32 FaceCount = 0;
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

			const int32 BaseVertex = VertexOffset;
			Obj += FString::Printf(
				TEXT("\no %s_%s_section_%d\nusemtl %s\n"),
				*ObjSafeName(OwnerName),
				*ObjSafeName(Component->GetName()),
				SectionIndex,
				SectionIndex == 0 ? TEXT("scene_gray") : TEXT("boolean_cap_gray"));

			for (const FProcMeshVertex& Vertex : Section->ProcVertexBuffer)
			{
				const FVector WorldPosition = ComponentTransform.TransformPosition(Vertex.Position);
				const FVector ObjPosition = UnrealWorldToObjDebugSpace(WorldPosition);
				Obj += FString::Printf(TEXT("v %.6f %.6f %.6f\n"), ObjPosition.X, ObjPosition.Y, ObjPosition.Z);
			}

			for (int32 i = 0; i + 2 < Section->ProcIndexBuffer.Num(); i += 3)
			{
				const int32 A = Section->ProcIndexBuffer[i];
				const int32 B = Section->ProcIndexBuffer[i + 1];
				const int32 C = Section->ProcIndexBuffer[i + 2];
				if (!Section->ProcVertexBuffer.IsValidIndex(A) ||
					!Section->ProcVertexBuffer.IsValidIndex(B) ||
					!Section->ProcVertexBuffer.IsValidIndex(C))
				{
					continue;
				}

				Obj += FString::Printf(TEXT("f %d %d %d\n"), BaseVertex + A + 1, BaseVertex + C + 1, BaseVertex + B + 1);
				++FaceCount;
			}

			VertexOffset += Section->ProcVertexBuffer.Num();
		}
		return FaceCount;
	}

	static bool ExportReconstructionSceneObj(
		UWorld* World,
		const TArray<FReconstructedMesh>& ReconstructedMeshes,
		const FString& ObjPath)
	{
		if (!World || ObjPath.IsEmpty())
		{
			return false;
		}

		const FString ObjDir = FPaths::GetPath(ObjPath);
		if (!ObjDir.IsEmpty())
		{
			IFileManager::Get().MakeDirectory(*ObjDir, true);
		}

		const FString MtlFilename = FPaths::GetBaseFilename(ObjPath) + TEXT(".mtl");
		const FString MtlPath = ObjDir / MtlFilename;
		const FString Mtl =
			TEXT("# FromLZ Step 10 reconstruction debug materials\n")
			TEXT("newmtl scene_gray\n")
			TEXT("Ka 0.650000 0.650000 0.650000\n")
			TEXT("Kd 0.650000 0.650000 0.650000\n")
			TEXT("Ks 0.000000 0.000000 0.000000\n")
			TEXT("d 1.000000\n\n")
			TEXT("newmtl boolean_cap_gray\n")
			TEXT("Ka 0.700000 0.700000 0.700000\n")
			TEXT("Kd 0.700000 0.700000 0.700000\n")
			TEXT("Ks 0.000000 0.000000 0.000000\n")
			TEXT("d 1.000000\n\n")
			TEXT("newmtl reconstructed_blue\n")
			TEXT("Ka 0.000000 0.250000 1.000000\n")
			TEXT("Kd 0.000000 0.470000 1.000000\n")
			TEXT("Ke 0.000000 0.050000 0.250000\n")
			TEXT("Ks 0.000000 0.000000 0.000000\n")
			TEXT("d 1.000000\n\n")
			TEXT("newmtl reconstructed_cutter_transparent\n")
			TEXT("Ka 0.000000 0.250000 1.000000\n")
			TEXT("Kd 0.000000 0.470000 1.000000\n")
			TEXT("Ke 0.000000 0.050000 0.250000\n")
			TEXT("Ks 0.000000 0.000000 0.000000\n")
			TEXT("d 0.250000\n");

		FString Obj;
		Obj.Reserve(1024 * 1024);
		Obj += TEXT("# FromLZ Step 10 reconstruction scene debug export\n");
		Obj += TEXT("# Coordinates are converted from Unreal left-handed Z-up to OBJ right-handed Z-up.\n");
		Obj += TEXT("# Mapping: obj_x = ue_x, obj_y = -ue_y, obj_z = ue_z. Units are centimeters.\n");
		Obj += FString::Printf(TEXT("mtllib %s\n"), *MtlFilename);

		int32 VertexOffset = 0;
		int32 StaticComponentCount = 0;
		int32 StaticFaceCount = 0;
		int32 BooleanComponentCount = 0;
		int32 BooleanFaceCount = 0;
		for (TActorIterator<AActor> It(World); It; ++It)
		{
			AActor* Actor = *It;
			if (!Actor || Actor->IsHidden() ||
				Actor->ActorHasTag(ReconstructedFaceTag) ||
				Actor->ActorHasTag(ReconstructedSolidTag))
			{
				continue;
			}

			TArray<UStaticMeshComponent*> Components;
			Actor->GetComponents<UStaticMeshComponent>(Components);
			for (UStaticMeshComponent* Component : Components)
			{
				const int32 FaceCount = AppendStaticMeshComponentObj(Obj, VertexOffset, Component);
				if (FaceCount > 0)
				{
					++StaticComponentCount;
					StaticFaceCount += FaceCount;
				}
			}

			TArray<UProceduralMeshComponent*> ProceduralComponents;
			Actor->GetComponents<UProceduralMeshComponent>(ProceduralComponents);
			for (UProceduralMeshComponent* Component : ProceduralComponents)
			{
				const int32 FaceCount = AppendProceduralMeshComponentObj(Obj, VertexOffset, Component);
				if (FaceCount > 0)
				{
					++BooleanComponentCount;
					BooleanFaceCount += FaceCount;
				}
			}
		}

		int32 ReconstructionFaceCount = 0;
		for (const FReconstructedMesh& MeshData : ReconstructedMeshes)
		{
			ReconstructionFaceCount += AppendObjWorldMesh(
				Obj,
				VertexOffset,
				MeshData.ActorName,
				MeshData.bIsExcavateCutter ? TEXT("reconstructed_cutter_transparent") : TEXT("reconstructed_blue"),
				MeshData.VerticesWorld,
				MeshData.Triangles);
		}

		const bool bSavedMtl = FFileHelper::SaveStringToFile(Mtl, *MtlPath);
		const bool bSavedObj = FFileHelper::SaveStringToFile(Obj, *ObjPath);
		if (bSavedMtl && bSavedObj)
		{
			UE_LOG(
				LogTemp,
				Log,
				TEXT("FaceReconstruct: exported debug OBJ %s (static components=%d, static faces=%d, boolean components=%d, boolean faces=%d, reconstruction faces=%d)."),
				*ObjPath,
				StaticComponentCount,
				StaticFaceCount,
				BooleanComponentCount,
				BooleanFaceCount,
				ReconstructionFaceCount);
			return true;
		}

		UE_LOG(LogTemp, Warning, TEXT("FaceReconstruct: failed to export debug OBJ %s or MTL %s."), *ObjPath, *MtlPath);
		return false;
	}

	static void DispatchPressCompletion(
		FFromLZPressCompletionCallback& CompletionCallback,
		const FFromLZPressProcessResult& Result)
	{
		if (!CompletionCallback)
		{
			return;
		}

		FFromLZPressCompletionCallback CallbackToRun = MoveTemp(CompletionCallback);
		AsyncTask(ENamedThreads::GameThread, [CallbackToRun = MoveTemp(CallbackToRun), Result]() mutable
		{
			CallbackToRun(Result);
		});
	}

	static void SpawnMeshesOnGameThread(
		TWeakObjectPtr<UWorld> WorldPtr,
		TArray<FReconstructedMesh> Meshes,
		FString DebugObjPath = FString(),
		FString PressId = FString(),
		int32 SessionGeneration = INDEX_NONE,
		FString PressDir = FString(),
		FString ActionPressDir = FString(),
		bool bPipelineSuccess = true,
		FString CompletionMessage = FString(),
		FFromLZPressCompletionCallback CompletionCallback = nullptr)
	{
		AsyncTask(ENamedThreads::GameThread, [
			WorldPtr,
			Meshes = MoveTemp(Meshes),
			DebugObjPath = MoveTemp(DebugObjPath),
			PressId = MoveTemp(PressId),
			SessionGeneration,
			PressDir = MoveTemp(PressDir),
			ActionPressDir = MoveTemp(ActionPressDir),
			bPipelineSuccess,
			CompletionMessage = MoveTemp(CompletionMessage),
			CompletionCallback = MoveTemp(CompletionCallback)]() mutable
		{
			FFromLZPressProcessResult CompletionResult;
			CompletionResult.bSuccess = bPipelineSuccess;
			CompletionResult.Message = CompletionMessage.IsEmpty()
				? (bPipelineSuccess ? FString(TEXT("Step 10/11 completed.")) : FString(TEXT("Step 10/11 failed.")))
				: CompletionMessage;
			CompletionResult.PressDir = PressDir;
			CompletionResult.ActionPressDir = ActionPressDir;

			if (SessionGeneration != INDEX_NONE && !FFromLZSessionReset::IsSessionGenerationCurrent(SessionGeneration))
			{
				UE_LOG(LogTemp, Log, TEXT("FaceReconstruct: skipped stale runtime mesh spawn on the game thread for press=%s."), *PressId);
				CompletionResult.bSuccess = false;
				CompletionResult.Message = TEXT("Skipped stale runtime mesh spawn because the session generation changed.");
				DispatchPressCompletion(CompletionCallback, CompletionResult);
				return;
			}

			UWorld* World = WorldPtr.Get();
			if (!World)
			{
				UE_LOG(LogTemp, Warning, TEXT("FaceReconstruct: world is no longer valid; skipped runtime mesh spawn."));
				CompletionResult.bSuccess = false;
				CompletionResult.Message = TEXT("World is no longer valid; skipped runtime mesh spawn.");
				DispatchPressCompletion(CompletionCallback, CompletionResult);
				return;
			}

			bool bHasRuntimeMesh = false;
			for (const FReconstructedMesh& MeshData : Meshes)
			{
				if (MeshData.VerticesWorld.Num() >= 3 && MeshData.Triangles.Num() >= 3)
				{
					bHasRuntimeMesh = true;
					break;
				}
			}
			if (bHasRuntimeMesh)
			{
				RegisterActiveStep11Press(PressId);
			}
			const FName GeneratedByTag = Step11GeneratedByPressTag(PressId);
			TSet<FString> AcceptedCutterActorNames;
			const FString Step11DiagnosticJsonPath = DebugObjPath.IsEmpty()
				? FString()
				: FPaths::GetPath(DebugObjPath) / TEXT("11_boolean_diagnostics.json");
			ApplyStep11BooleanOperations(World, Meshes, PressId, Step11DiagnosticJsonPath, AcceptedCutterActorNames);

			UMaterialInterface* VertexColorMaterial = GetReconstructionVertexColorMaterial();
			int32 SpawnedRuntimeActorCount = 0;
			for (const FReconstructedMesh& MeshData : Meshes)
			{
				if (MeshData.VerticesWorld.Num() < 3 || MeshData.Triangles.Num() < 3)
				{
					continue;
				}

				FVector Origin = FVector::ZeroVector;
				for (const FVector& V : MeshData.VerticesWorld)
				{
					Origin += V;
				}
				Origin /= double(MeshData.VerticesWorld.Num());

				FActorSpawnParameters Params;
				Params.Name = FName(*MeshData.ActorName);
				AActor* Actor = World->SpawnActor<AActor>(AActor::StaticClass(), FTransform(Origin), Params);
				if (!Actor)
				{
					UE_LOG(LogTemp, Warning, TEXT("FaceReconstruct: failed to spawn actor %s"), *MeshData.ActorName);
					continue;
				}
				++SpawnedRuntimeActorCount;

				Actor->Tags.AddUnique(MeshData.Tag);
				Actor->Tags.AddUnique(GeneratedByTag);
				Actor->Tags.AddUnique(MeshData.bIsExcavateCutter ? Step11ActionExcavateCutterTag : Step11ActionAttachTag);
#if WITH_EDITOR
				Actor->SetActorLabel(MeshData.ActorName);
#endif

				UProceduralMeshComponent* MeshComponent = NewObject<UProceduralMeshComponent>(Actor, TEXT("ReconstructedFaceMesh"));
				MeshComponent->SetMobility(EComponentMobility::Movable);
				MeshComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
				MeshComponent->bUseAsyncCooking = false;
				MeshComponent->SetCastShadow(false);
				MeshComponent->ComponentTags.AddUnique(GeneratedByTag);
				MeshComponent->ComponentTags.AddUnique(MeshData.bIsExcavateCutter ? Step11ActionExcavateCutterTag : Step11ActionAttachTag);
				Actor->SetRootComponent(MeshComponent);
				Actor->AddInstanceComponent(MeshComponent);
				MeshComponent->RegisterComponent();
				if (!Actor->SetActorLocation(Origin, false, nullptr, ETeleportType::TeleportPhysics))
				{
					MeshComponent->SetWorldLocation(Origin);
				}
				UMaterialInterface* MeshMaterial = nullptr;
				if (MeshData.bIsExcavateCutter)
				{
					MeshMaterial = CreateCutterMaterial(MeshComponent);
				}
				else
				{
					MeshMaterial = ResolveAttachSourceMaterial(World, MeshData, MeshComponent);
					if (!MeshMaterial)
					{
						MeshMaterial = VertexColorMaterial;
					}
				}
				MeshComponent->SetMaterial(0, MeshMaterial);

				TArray<FVector> LocalVertices;
				LocalVertices.Reserve(MeshData.VerticesWorld.Num());
				for (const FVector& V : MeshData.VerticesWorld)
				{
					LocalVertices.Add(V - Origin);
				}

				// Build flat-shaded geometry: each input triangle is expanded into its
				// own three vertices carrying that triangle's outward face normal.
				// We do NOT emit a reversed-winding back triangle here: the solid is
				// closed (cap + reverse cap + sides) so the interior is never visible
				// from outside, and emitting back-faces at the same depth as the front
				// causes z-fighting whenever the assigned material is two-sided
				// (M_Color_Inst), letting the inward-normal back triangle win and
				// producing inverted GBuffer normals (e.g. olive -Z on the top cap).
				//
				// IMPORTANT WINDING NOTE: BuildSolidMeshTriangles emits triangles such
				// that Cross(VB-VA, VC-VA) points OUTWARD (top cap = +Z, side wall =
				// horizontal-outward, source cap = -Z). UE's UProceduralMeshComponent
				// rasterizer treats CCW-from-camera as FRONT face, which means the
				// front face actually ends up on the side OPPOSITE to that math cross
				// direction. Submitting the indices in the original (A,B,C) order
				// therefore renders the inside-out shell (top cap culled when viewed
				// from above, sides showing only their interior face). To make the
				// solid render with its outside visible we submit the REVERSED
				// winding (A,C,B) but keep the vertex normal pointing along the
				// original outward cross direction so the GBuffer Normal pass still
				// encodes the geometric outward direction.
				//
				// This replaces the previous shared-vertex + single MeshData.Normal
				// init pattern, which made every face (cap, reverse cap, sides) share
				// the cap's OrientedNormal and broke GBuffer Normal output /
				// _faces.png flood-fill (side walls inherited the cap's Z-up direction).
				TArray<FVector> ExpandedVertices;
				TArray<FVector> Normals;
				TArray<int32> DrawTriangles;
				const int32 InputTriCount = MeshData.Triangles.Num() / 3;
				const int32 ExpandedCapacity = InputTriCount * 3;
				ExpandedVertices.Reserve(ExpandedCapacity);
				Normals.Reserve(ExpandedCapacity);
				DrawTriangles.Reserve(ExpandedCapacity);

				const FVector FallbackNormal = MeshData.Normal.GetSafeNormal();
				for (int32 ti = 0; ti + 2 < MeshData.Triangles.Num(); ti += 3)
				{
					const int32 IA = MeshData.Triangles[ti];
					const int32 IB = MeshData.Triangles[ti + 1];
					const int32 IC = MeshData.Triangles[ti + 2];
					if (!LocalVertices.IsValidIndex(IA) ||
						!LocalVertices.IsValidIndex(IB) ||
						!LocalVertices.IsValidIndex(IC))
					{
						continue;
					}

					const FVector& VA = LocalVertices[IA];
					const FVector& VB = LocalVertices[IB];
					const FVector& VC = LocalVertices[IC];

					FVector FaceNormal = FVector::CrossProduct(VB - VA, VC - VA).GetSafeNormal();
					if (FaceNormal.IsNearlyZero())
					{
						FaceNormal = FallbackNormal;
					}

					// Emit reversed winding (VA, VC, VB) so UE PMC renders the
					// outward side; vertex normal still encodes the original
					// outward cross direction.
					const int32 BaseFront = ExpandedVertices.Num();
					ExpandedVertices.Add(VA); Normals.Add(FaceNormal);
					ExpandedVertices.Add(VC); Normals.Add(FaceNormal);
					ExpandedVertices.Add(VB); Normals.Add(FaceNormal);
					DrawTriangles.Add(BaseFront);
					DrawTriangles.Add(BaseFront + 1);
					DrawTriangles.Add(BaseFront + 2);
				}

				TArray<FVector2D> UV0;
				UV0.Init(FVector2D::ZeroVector, ExpandedVertices.Num());

				TArray<FColor> Colors;
				Colors.Init(MeshData.Color, ExpandedVertices.Num());

				TArray<FProcMeshTangent> Tangents;
				MeshComponent->CreateMeshSection(0, ExpandedVertices, DrawTriangles, Normals, UV0, Colors, Tangents, false);

				if (MeshData.bIsExcavateCutter && AcceptedCutterActorNames.Contains(MeshData.ActorName))
				{
					Actor->SetActorHiddenInGame(true);
					MeshComponent->SetVisibility(false, true);
					MeshComponent->SetHiddenInGame(true, true);
					UE_LOG(LogTemp, Log, TEXT("Step11Diag: cutter hidden after accepted boolean actor=%s press=%s."), *MeshData.ActorName, *PressId);
				}
			}

			if (!DebugObjPath.IsEmpty())
			{
				ExportReconstructionSceneObj(World, Meshes, DebugObjPath);
			}

			UE_LOG(LogTemp, Log, TEXT("FaceReconstruct: press=%s spawned %d runtime reconstruction actor(s)."), *PressId, Meshes.Num());
			CompletionResult.SpawnedRuntimeActorCount = SpawnedRuntimeActorCount;
			DispatchPressCompletion(CompletionCallback, CompletionResult);
		});
	}

	static void SaveCommonFailureForComponents(
		const TArray<FString>& ComponentNames, const FString& PressDir, const FString& Error)
	{
		TArray<TSharedPtr<FJsonValue>> IndexEntries;
		for (const FString& ComponentName : ComponentNames)
		{
			MakeFailureResult(ComponentName, Error, PressDir / ComponentName);

			FSolidReconstructionResult Solid;
			Solid.ComponentName = ComponentName;
			Solid.ActorName = FString::Printf(TEXT("FromLZ_ReconstructedSolid_%s_%s"), *FPaths::GetCleanFilename(PressDir), *ComponentName);
			const FString SolidError =
				FString::Printf(TEXT("Solid skipped because Step 10 common inputs failed: %s"), *Error);
			PrepareWorldOrthogonalNotAttemptedDebug(
				Solid.CapBBoxRegularization,
				SolidError);
			SaveCapBBoxRegularizationDebug(
				Solid.CapBBoxRegularization,
				PressDir / ComponentName);
			SaveSkippedSolidResult(
				Solid,
				PressDir / ComponentName / TEXT("10_solid_reconstruction.json"),
				SolidError);

			TSharedRef<FJsonObject> Entry = MakeShared<FJsonObject>();
			Entry->SetStringField(TEXT("component"), ComponentName);
			Entry->SetBoolField(TEXT("attempted"), false);
			Entry->SetBoolField(TEXT("applied"), false);
			Entry->SetBoolField(TEXT("fallback_to_original"), true);
			Entry->SetStringField(TEXT("failed_stage"), TEXT("input_validation"));
			Entry->SetStringField(TEXT("reason"), SolidError);
			Entry->SetStringField(
				TEXT("debug_directory"),
				ComponentName / TEXT("10_cap_world_orthogonal_steps"));
			IndexEntries.Add(MakeShared<FJsonValueObject>(Entry));
		}
		TSharedRef<FJsonObject> IndexRoot = MakeShared<FJsonObject>();
		IndexRoot->SetStringField(TEXT("press"), FPaths::GetCleanFilename(PressDir));
		IndexRoot->SetNumberField(TEXT("component_count"), ComponentNames.Num());
		IndexRoot->SetNumberField(TEXT("attempted_count"), 0);
		IndexRoot->SetNumberField(TEXT("applied_count"), 0);
		IndexRoot->SetNumberField(TEXT("fallback_count"), ComponentNames.Num());
		IndexRoot->SetArrayField(TEXT("components"), IndexEntries);
		SaveJsonObject(
			IndexRoot,
			PressDir / TEXT("10_cap_world_orthogonal_index.json"));
	}

	static void SaveWorldOrthogonalPressIndex(
		const FString& PressDir,
		const TArray<FComponentResult>& Results)
	{
		int32 AttemptedCount = 0;
		int32 AppliedCount = 0;
		int32 FallbackCount = 0;
		TArray<TSharedPtr<FJsonValue>> Entries;
		for (const FComponentResult& Result : Results)
		{
			const FCapBBoxRegularizationResult& Debug =
				Result.Solid.CapBBoxRegularization;
			AttemptedCount += Debug.bAttempted ? 1 : 0;
			AppliedCount += Debug.bApplied ? 1 : 0;
			FallbackCount += Debug.bFallbackToOriginal ? 1 : 0;

			FString FailedStage;
			for (const FWorldOrthogonalStageDebug& Stage : Debug.WorldOrthogonalStages)
			{
				if (Stage.Status == TEXT("failed"))
				{
					FailedStage = Stage.Name;
					break;
				}
			}

			TSharedRef<FJsonObject> Entry = MakeShared<FJsonObject>();
			Entry->SetStringField(TEXT("component"), Result.ComponentName);
			Entry->SetStringField(TEXT("action"), Result.Action);
			Entry->SetNumberField(TEXT("selected_face_id"), Result.SelectedFaceId);
			Entry->SetBoolField(TEXT("attempted"), Debug.bAttempted);
			Entry->SetBoolField(TEXT("applied"), Debug.bApplied);
			Entry->SetBoolField(TEXT("fallback_to_original"), Debug.bFallbackToOriginal);
			Entry->SetStringField(TEXT("selected_geometry"), Debug.SelectedGeometry);
			Entry->SetStringField(TEXT("failed_stage"), FailedStage);
			Entry->SetStringField(TEXT("reason"), Debug.RejectionReason);
			Entry->SetNumberField(TEXT("boundary_run_count"), Debug.BoundaryRunCount);
			Entry->SetNumberField(TEXT("primitive_edge_count"), Debug.PrimitiveGeometryEdgeCount);
			Entry->SetNumberField(TEXT("merged_edge_count"), Debug.GeometryEdgeCount);
			Entry->SetStringField(
				TEXT("debug_directory"),
				Result.ComponentName / TEXT("10_cap_world_orthogonal_steps"));
			Entries.Add(MakeShared<FJsonValueObject>(Entry));
		}

		TSharedRef<FJsonObject> Root = MakeShared<FJsonObject>();
		Root->SetStringField(TEXT("press"), FPaths::GetCleanFilename(PressDir));
		Root->SetNumberField(TEXT("component_count"), Results.Num());
		Root->SetNumberField(TEXT("attempted_count"), AttemptedCount);
		Root->SetNumberField(TEXT("applied_count"), AppliedCount);
		Root->SetNumberField(TEXT("fallback_count"), FallbackCount);
		Root->SetArrayField(TEXT("components"), Entries);
		SaveJsonObject(
			Root,
			PressDir / TEXT("10_cap_world_orthogonal_index.json"));
	}
}

bool FFromLZFaceReconstructor::EvaluateCandidateFaces(
	const FString& PressDir,
	int32 SourceWidth,
	int32 SourceHeight,
	const TArray<FFromLZCandidateFaceRequest>& Requests,
	const FFromLZFaceReconstructionParams& Params,
	TArray<FFromLZCandidateFaceEvaluation>& OutEvaluations,
	FString& OutError)
{
	OutEvaluations.Reset();
	OutError.Reset();

	FCommonInputs Inputs;
	if (!LoadCaptureRef(PressDir, Inputs))
	{
		OutError = TEXT("failed to read capture_ref.json or resolve capture/faces paths");
		return false;
	}
	if (!DecodePngToRGBA(Inputs.FacesPngPath, Inputs.FacesRGBA, Inputs.FacesWidth, Inputs.FacesHeight))
	{
		OutError = FString::Printf(TEXT("failed to decode faces png: %s"), *Inputs.FacesPngPath);
		return false;
	}
	if (!LoadFacesJson(Inputs.FacesJsonPath, Inputs.Faces, Inputs.FacesProjection))
	{
		OutError = FString::Printf(TEXT("failed to read faces json: %s"), *Inputs.FacesJsonPath);
		return false;
	}
	FString CameraLoadError;
	if (!LoadCameraJson(Inputs.CaptureJsonPath, Inputs.Camera, CameraLoadError))
	{
		OutError = CameraLoadError;
		return false;
	}
	if (!ValidateProjectionMetadata(Inputs, OutError))
	{
		return false;
	}
	if (!BuildFaceLookups(Inputs, OutError))
	{
		return false;
	}

	OutEvaluations.Reserve(Requests.Num());
	const FString ValidationRoot = PressDir / TEXT("CandidateFaceValidation");
	IFileManager::Get().MakeDirectory(*ValidationRoot, true);
	for (int32 RequestIndex = 0; RequestIndex < Requests.Num(); ++RequestIndex)
	{
		const FFromLZCandidateFaceRequest& Request = Requests[RequestIndex];
		FString SourceLabel = Request.CandidateSource.IsEmpty() ? TEXT("unknown") : Request.CandidateSource;
		SourceLabel.ReplaceInline(TEXT("/"), TEXT("_"));
		SourceLabel.ReplaceInline(TEXT("\\"), TEXT("_"));
		SourceLabel.ReplaceInline(TEXT(" "), TEXT("_"));
		const FString CandidateOutputDir = ValidationRoot / FString::Printf(
			TEXT("Candidate_%03d_%s"), RequestIndex, *SourceLabel);

		const bool bExcavate = Request.Action.Equals(TEXT("excavate"), ESearchCase::IgnoreCase);
		const bool bAttach = Request.Action.Equals(TEXT("attach"), ESearchCase::IgnoreCase);
		if (!bExcavate && !bAttach)
		{
			FFromLZCandidateFaceEvaluation Evaluation;
			Evaluation.bEvaluated = true;
			Evaluation.EvaluationMode = TEXT("unsupported_action");
			Evaluation.SourcePolygonKey = TEXT("cap_polygon");
			Evaluation.RejectReason = FString::Printf(TEXT("unsupported candidate action: %s"), *Request.Action);
			SaveCandidateFaceValidationDebug(
				Request, Evaluation, Params, Inputs, SourceWidth, SourceHeight,
				TArray<uint8>(), TArray<FFaceCandidate>(), CandidateOutputDir);
			OutEvaluations.Add(Evaluation);
			continue;
		}

		const TArray<FVector2D>& SourcePolygon = bExcavate ? Request.CapPolygon : Request.CapPolygonTranslated;
		const FString SourcePolygonKey = bExcavate ? TEXT("cap_polygon") : TEXT("cap_polygon_translated");
		TArray<uint8> Mask;
		TArray<FFaceCandidate> Candidates;
		FFromLZCandidateFaceEvaluation Evaluation = EvaluateSourcePolygonForFaces(
			SourcePolygon,
			Request.SideVectors,
			Request.GreenChains,
			bAttach,
			SourceWidth,
			SourceHeight,
			SourcePolygonKey,
			Inputs,
			&Mask,
			&Candidates);
		if (!Evaluation.bValid && bAttach)
		{
			if (!Params.bEnableAttachSupportPlaneFallback)
			{
				Evaluation.SupportFaceRejectReason = TEXT("attach support-plane fallback disabled by params");
			}
			else
			{
				const double ScaleX = SourceWidth > 0 ? double(Inputs.FacesWidth) / double(SourceWidth) : 1.0;
				const double ScaleY = SourceHeight > 0 ? double(Inputs.FacesHeight) / double(SourceHeight) : 1.0;
				const double SupportFaceVoteMinCoverage = FMath::Clamp(
					double(Params.SupportFaceVoteMinCoverage),
					0.0,
					1.0);
				FString LastRejectReason = TEXT("no green chain candidates were available for support-plane fallback");
				int32 BestAttemptOutputIndex = INDEX_NONE;
				double BestAttemptCameraDistance = TNumericLimits<double>::Max();
				double BestAttemptCoverage = -1.0;
				int32 BestAttemptHitSampleCount = -1;
				double BestAttemptPathLength = -1.0;
				for (int32 ChainIndex = 0; ChainIndex < Request.GreenChains.Num(); ++ChainIndex)
				{
					const FFromLZGreenChainCandidate2D& Chain = Request.GreenChains[ChainIndex];
					const FVector2D ChainStartFace(Chain.Start.X * ScaleX, Chain.Start.Y * ScaleY);
					const FVector2D ChainEndFace(Chain.End.X * ScaleX, Chain.End.Y * ScaleY);
					FFromLZSupportFaceVoteAttempt Attempt;
					Attempt.ChainIndex = ChainIndex;
					Attempt.SeedStrokeId = Chain.SeedStrokeId;
					Attempt.ChainStartFaceSpace = ChainStartFace;
					Attempt.ChainEndFaceSpace = ChainEndFace;
					Attempt.ChainChordLength = Chain.ChordLength > 0.0 ? Chain.ChordLength : (Chain.End - Chain.Start).Size();
					Attempt.ChainPathLength = Chain.PathLength > 0.0 ? Chain.PathLength : Attempt.ChainChordLength;
					FSupportFaceVoteResult Vote;
					if (!VoteSupportFaceForSegment(
						Inputs,
						ChainStartFace,
						ChainEndFace,
						Params.SupportFaceVoteRadiusPx,
						FMath::Max(1.0, double(Params.SupportFaceVoteSampleStepPx)),
						SupportFaceVoteMinCoverage,
						Vote))
					{
						Attempt.SupportFaceCandidates = Vote.FaceCandidates;
						Attempt.SupportVoteCoverage = Vote.Coverage;
						Attempt.SupportVotePixelCoverage = Vote.VotePixelCoverage;
						Attempt.SupportVotePixels = Vote.VotePixels;
						Attempt.SupportConsideredPixels = Vote.ConsideredPixels;
						Attempt.SupportHitSampleCount = Vote.HitSampleCount;
						Attempt.SupportTotalSampleCount = Vote.TotalSampleCount;
						Attempt.SupportFaceWorldZMax = Vote.WorldZMax;
						Attempt.SupportFaceWorldZAverage = Vote.WorldZAverage;
						Attempt.SupportMinCameraDistance = Vote.MinCameraDistance;
						Attempt.SupportWorstPolygonDistancePx = Vote.WorstPolygonDistancePx;
						Attempt.RejectReason = FString::Printf(
							TEXT("chain %d rejected: %s"),
							ChainIndex,
							*Vote.Error);
						LastRejectReason = Attempt.RejectReason;
						Evaluation.SupportFaceVoteAttempts.Add(Attempt);
						continue;
					}
					Attempt.bVoteFound = true;
					Attempt.SupportFaceId = Vote.FaceId;
					Attempt.SupportFaceCandidates = Vote.FaceCandidates;
					Attempt.SupportVotePixels = Vote.VotePixels;
					Attempt.SupportConsideredPixels = Vote.ConsideredPixels;
					Attempt.SupportHitSampleCount = Vote.HitSampleCount;
					Attempt.SupportTotalSampleCount = Vote.TotalSampleCount;
					Attempt.SupportVoteCoverage = Vote.Coverage;
					Attempt.SupportVotePixelCoverage = Vote.VotePixelCoverage;
					Attempt.SupportFaceWorldZMax = Vote.WorldZMax;
					Attempt.SupportFaceWorldZAverage = Vote.WorldZAverage;
					Attempt.SupportMinCameraDistance = Vote.MinCameraDistance;
					Attempt.SupportWorstPolygonDistancePx = Vote.WorstPolygonDistancePx;
					Attempt.bCoveragePass = true;
					Attempt.RejectReason = TEXT("coverage passed; candidate for nearest-camera support face selection");
					const int32 AttemptOutputIndex = Evaluation.SupportFaceVoteAttempts.Add(Attempt);
					const bool bCloser =
						BestAttemptOutputIndex == INDEX_NONE ||
						Attempt.SupportMinCameraDistance < BestAttemptCameraDistance - 1e-6;
					const bool bTieBetterCoverage =
						BestAttemptOutputIndex != INDEX_NONE &&
						FMath::IsNearlyEqual(Attempt.SupportMinCameraDistance, BestAttemptCameraDistance, 1e-6) &&
						Attempt.SupportVoteCoverage > BestAttemptCoverage + 1e-6;
					const bool bTieMoreSamples =
						BestAttemptOutputIndex != INDEX_NONE &&
						FMath::IsNearlyEqual(Attempt.SupportMinCameraDistance, BestAttemptCameraDistance, 1e-6) &&
						FMath::IsNearlyEqual(Attempt.SupportVoteCoverage, BestAttemptCoverage, 1e-6) &&
						Attempt.SupportHitSampleCount > BestAttemptHitSampleCount;
					const bool bTieLongerChain =
						BestAttemptOutputIndex != INDEX_NONE &&
						FMath::IsNearlyEqual(Attempt.SupportMinCameraDistance, BestAttemptCameraDistance, 1e-6) &&
						FMath::IsNearlyEqual(Attempt.SupportVoteCoverage, BestAttemptCoverage, 1e-6) &&
						Attempt.SupportHitSampleCount == BestAttemptHitSampleCount &&
						Attempt.ChainPathLength > BestAttemptPathLength + 1e-6;
					if (bCloser || bTieBetterCoverage || bTieMoreSamples || bTieLongerChain)
					{
						BestAttemptOutputIndex = AttemptOutputIndex;
						BestAttemptCameraDistance = Attempt.SupportMinCameraDistance;
						BestAttemptCoverage = Attempt.SupportVoteCoverage;
						BestAttemptHitSampleCount = Attempt.SupportHitSampleCount;
						BestAttemptPathLength = Attempt.ChainPathLength;
					}
				}
				if (Evaluation.SupportFaceVoteAttempts.IsValidIndex(BestAttemptOutputIndex))
				{
					FFromLZSupportFaceVoteAttempt& BestAttempt = Evaluation.SupportFaceVoteAttempts[BestAttemptOutputIndex];
					BestAttempt.bSelected = true;
					BestAttempt.RejectReason = TEXT("selected nearest-camera support face after per-chain highest-Z selection");
					Evaluation.bAttachSupportPlaneFallbackEligible = true;
					Evaluation.EvaluationMode = TEXT("attach_support_plane_fallback_eligible");
					Evaluation.SupportFaceId = BestAttempt.SupportFaceId;
					Evaluation.SupportGreenChainIndex = BestAttempt.ChainIndex;
					Evaluation.SupportFaceVoteCoverage = BestAttempt.SupportVoteCoverage;
					Evaluation.SupportFaceRejectReason = FString::Printf(
						TEXT("eligible via nearest-camera chain %d: face=%d hit_samples=%d/%d sample_coverage=%.3f vote_pixels=%d considered=%d vote_pixel_coverage=%.3f z_max=%.3f camera_distance=%.3f path_length=%.3f chord_length=%.3f"),
						BestAttempt.ChainIndex,
						BestAttempt.SupportFaceId,
						BestAttempt.SupportHitSampleCount,
						BestAttempt.SupportTotalSampleCount,
						BestAttempt.SupportVoteCoverage,
						BestAttempt.SupportVotePixels,
						BestAttempt.SupportConsideredPixels,
						BestAttempt.SupportVotePixelCoverage,
						BestAttempt.SupportFaceWorldZMax,
						BestAttempt.SupportMinCameraDistance,
						BestAttempt.ChainPathLength,
						BestAttempt.ChainChordLength);
				}
				else if (!Evaluation.bAttachSupportPlaneFallbackEligible)
				{
					Evaluation.SupportFaceRejectReason =
						Request.GreenChains.Num() == 0
							? LastRejectReason
							: FString::Printf(
								TEXT("no green chain passed support face vote threshold %.3f; see support_face_vote_attempts; last_reject=%s"),
								SupportFaceVoteMinCoverage,
								*LastRejectReason);
				}
			}
		}
		SaveCandidateFaceValidationDebug(
			Request, Evaluation, Params, Inputs, SourceWidth, SourceHeight,
			Mask, Candidates, CandidateOutputDir);
		OutEvaluations.Add(MoveTemp(Evaluation));
	}
	return true;
}

void FFromLZFaceReconstructor::ProcessPress(
	const FString& PressDir,
	const FString& ActionPressDir,
	TWeakObjectPtr<UWorld> World,
	int32 SessionGeneration,
	FFromLZPressCompletionCallback CompletionCallback)
{
	ProcessPress(
		PressDir,
		ActionPressDir,
		World,
		FFromLZFaceReconstructionParams(),
		SessionGeneration,
		MoveTemp(CompletionCallback));
}

void FFromLZFaceReconstructor::ProcessPress(
	const FString& PressDir,
	const FString& ActionPressDir,
	TWeakObjectPtr<UWorld> World,
	const FFromLZFaceReconstructionParams& Params,
	int32 SessionGeneration,
	FFromLZPressCompletionCallback CompletionCallback)
{
	if (SessionGeneration != INDEX_NONE && !FFromLZSessionReset::IsSessionGenerationCurrent(SessionGeneration))
	{
		UE_LOG(LogTemp, Log, TEXT("FaceReconstruct: skipped stale press %s because the session generation changed before processing started."), *FPaths::GetCleanFilename(PressDir));
		FFromLZPressProcessResult Result;
		Result.bSuccess = false;
		Result.Message = TEXT("Skipped stale press because the session generation changed before processing started.");
		Result.PressDir = PressDir;
		Result.ActionPressDir = ActionPressDir;
		DispatchPressCompletion(CompletionCallback, Result);
		return;
	}

	const FString PressId = FPaths::GetCleanFilename(PressDir);
	{
		FString ParamsJson;
		ParamsJson += TEXT("{\n");
		ParamsJson += FString::Printf(TEXT("  \"candidate_face_min_overlap_ratio\": %.6f,\n"), Params.CandidateFaceMinOverlapRatio);
		ParamsJson += FString::Printf(TEXT("  \"candidate_face_max_normal_side_angle_degrees\": %.6f,\n"), Params.CandidateFaceMaxNormalSideAngleDegrees);
		ParamsJson += FString::Printf(TEXT("  \"candidate_face_preferred_normal_side_angle_degrees\": %.6f,\n"), Params.CandidateFacePreferredNormalSideAngleDegrees);
		ParamsJson += FString::Printf(TEXT("  \"enable_attach_support_plane_fallback\": %s,\n"), Params.bEnableAttachSupportPlaneFallback ? TEXT("true") : TEXT("false"));
		ParamsJson += FString::Printf(TEXT("  \"support_face_vote_radius_px\": %d,\n"), Params.SupportFaceVoteRadiusPx);
		ParamsJson += FString::Printf(TEXT("  \"support_plane_polygon_tol_px\": %.6f,\n"), Params.SupportPlanePolygonTolPx);
		ParamsJson += TEXT("  \"support_plane_polygon_check_enforced\": false,\n");
		ParamsJson += FString::Printf(TEXT("  \"support_face_vote_sample_step_px\": %.6f,\n"), Params.SupportFaceVoteSampleStepPx);
		ParamsJson += FString::Printf(TEXT("  \"support_face_vote_min_coverage\": %.6f,\n"), Params.SupportFaceVoteMinCoverage);
		ParamsJson += FString::Printf(TEXT("  \"no_penetration_tol_cm\": %.6f,\n"), Params.NoPenetrationTolCm);
		ParamsJson += FString::Printf(TEXT("  \"contact_anchor_tol_px\": %.6f,\n"), Params.ContactAnchorTolPx);
		ParamsJson += FString::Printf(TEXT("  \"attach_path_front_distance_tie_tol_cm\": %.6f,\n"), Params.AttachPathFrontDistanceTieTolCm);
		ParamsJson += FString::Printf(TEXT("  \"attach_path_plane_relation_angle_tol_deg\": %.6f,\n"), Params.AttachPathPlaneRelationAngleTolDeg);
		ParamsJson += FString::Printf(TEXT("  \"attach_path_plane_relation_distance_tol_cm\": %.6f,\n"), Params.AttachPathPlaneRelationDistanceTolCm);
		ParamsJson += FString::Printf(TEXT("  \"support_force_hard_min_green_chord_cm\": %.6f,\n"), Params.SupportForceHardMinGreenChordCm);
		ParamsJson += FString::Printf(TEXT("  \"support_force_preferred_min_green_chord_cm\": %.6f,\n"), Params.SupportForcePreferredMinGreenChordCm);
		ParamsJson += FString::Printf(TEXT("  \"world_ortho_use_per_face_capture\": %s,\n"), Params.bWorldOrthoUsePerFaceCapture ? TEXT("true") : TEXT("false"));
		ParamsJson += FString::Printf(TEXT("  \"world_ortho_per_face_clip_margin_pixels\": %.6f,\n"), Params.WorldOrthoPerFaceClipMarginPixels);
		ParamsJson += FString::Printf(TEXT("  \"world_ortho_pure_red_allow_diagonal_root\": %s,\n"), Params.bWorldOrthoPureRedAllowDiagonalRoot ? TEXT("true") : TEXT("false"));
		ParamsJson += FString::Printf(TEXT("  \"world_ortho_allow_diagonal_supports\": %s,\n"), Params.bWorldOrthoAllowDiagonalSupports ? TEXT("true") : TEXT("false"));
		ParamsJson += FString::Printf(TEXT("  \"world_ortho_max_wrap_gap_fraction\": %.6f,\n"), Params.WorldOrthoMaxWrapGapFraction);
		ParamsJson += FString::Printf(TEXT("  \"world_ortho_allow_topology_repair\": %s,\n"), Params.bWorldOrthoAllowTopologyRepair ? TEXT("true") : TEXT("false"));
		ParamsJson += FString::Printf(TEXT("  \"world_ortho_black_axis_tolerance_degrees\": %.6f,\n"), Params.WorldOrthoBlackAxisToleranceDegrees);
		ParamsJson += FString::Printf(TEXT("  \"world_ortho_diag_threshold_degrees\": %.6f,\n"), Params.WorldOrthoDiagThresholdDegrees);
		ParamsJson += FString::Printf(TEXT("  \"world_ortho_angle_comparison_epsilon_degrees\": %.9f,\n"), Params.WorldOrthoAngleComparisonEpsilonDegrees);
		ParamsJson += FString::Printf(TEXT("  \"world_ortho_black_node_snap_tolerance_pixels\": %.6f,\n"), Params.WorldOrthoBlackNodeSnapTolerancePixels);
		ParamsJson += FString::Printf(TEXT("  \"world_ortho_red_macro_corridor_pixels\": %.6f,\n"), Params.WorldOrthoRedMacroCorridorPixels);
		ParamsJson += FString::Printf(TEXT("  \"world_ortho_red_macro_group_min_length_pixels\": %.6f,\n"), Params.WorldOrthoRedMacroGroupMinLengthPixels);
		ParamsJson += FString::Printf(TEXT("  \"world_ortho_red_primitive_rdp_tolerance_pixels\": %.6f,\n"), Params.WorldOrthoRedPrimitiveRdpTolerancePixels);
		ParamsJson += FString::Printf(TEXT("  \"world_ortho_short_red_edge_length_pixels\": %.6f,\n"), Params.WorldOrthoShortRedEdgeLengthPixels);
		ParamsJson += FString::Printf(TEXT("  \"world_ortho_min_area_ratio\": %.6f,\n"), Params.WorldOrthoMinAreaRatio);
		ParamsJson += FString::Printf(TEXT("  \"world_ortho_pure_red_max_root_candidates\": %d\n"), Params.WorldOrthoPureRedMaxRootCandidates);
		ParamsJson += TEXT("}\n");
		FFileHelper::SaveStringToFile(ParamsJson, *(PressDir / TEXT("10_face_reconstruction_params.json")));
	}

	TArray<FString> ComponentNames;
	IFileManager::Get().IterateDirectory(*PressDir, [&ComponentNames](const TCHAR* InPath, bool bIsDir) -> bool
	{
		if (bIsDir)
		{
			const FString Name = FPaths::GetCleanFilename(FString(InPath));
			if (Name.StartsWith(TEXT("Component_")))
			{
				ComponentNames.Add(Name);
			}
		}
		return true;
	});
	ComponentNames.Sort();

	if (ComponentNames.Num() == 0)
	{
		SpawnMeshesOnGameThread(
			World, TArray<FReconstructedMesh>(), FString(), PressId, SessionGeneration,
			PressDir, ActionPressDir, true,
			TEXT("No component folders were found; Step 10/11 completed with no runtime meshes."),
			MoveTemp(CompletionCallback));
		UE_LOG(LogTemp, Log, TEXT("FaceReconstruct: no component folders found in %s"), *PressDir);
		return;
	}

	FCommonInputs Inputs;
	if (!LoadCaptureRef(PressDir, Inputs))
	{
		SaveCommonFailureForComponents(ComponentNames, PressDir, TEXT("Failed to read capture_ref.json or resolve capture/faces paths"));
		SpawnMeshesOnGameThread(
			World, TArray<FReconstructedMesh>(), FString(), PressId, SessionGeneration,
			PressDir, ActionPressDir, false,
			TEXT("Failed to read capture_ref.json or resolve capture/faces paths."),
			MoveTemp(CompletionCallback));
		return;
	}
	if (!DecodePngToRGBA(Inputs.FacesPngPath, Inputs.FacesRGBA, Inputs.FacesWidth, Inputs.FacesHeight))
	{
		SaveCommonFailureForComponents(ComponentNames, PressDir, FString::Printf(TEXT("Failed to decode faces png: %s"), *Inputs.FacesPngPath));
		SpawnMeshesOnGameThread(
			World, TArray<FReconstructedMesh>(), FString(), PressId, SessionGeneration,
			PressDir, ActionPressDir, false,
			FString::Printf(TEXT("Failed to decode faces png: %s"), *Inputs.FacesPngPath),
			MoveTemp(CompletionCallback));
		return;
	}
	if (!LoadFacesJson(Inputs.FacesJsonPath, Inputs.Faces, Inputs.FacesProjection))
	{
		SaveCommonFailureForComponents(ComponentNames, PressDir, FString::Printf(TEXT("Failed to read faces json: %s"), *Inputs.FacesJsonPath));
		SpawnMeshesOnGameThread(
			World, TArray<FReconstructedMesh>(), FString(), PressId, SessionGeneration,
			PressDir, ActionPressDir, false,
			FString::Printf(TEXT("Failed to read faces json: %s"), *Inputs.FacesJsonPath),
			MoveTemp(CompletionCallback));
		return;
	}
	if (!Inputs.ActorMaterialPngPath.IsEmpty() && !Inputs.ActorMaterialJsonPath.IsEmpty())
	{
		const bool bLoadedActorMaterialPng = DecodePngToRGBA(
			Inputs.ActorMaterialPngPath,
			Inputs.ActorMaterialRGBA,
			Inputs.ActorMaterialWidth,
			Inputs.ActorMaterialHeight);
		const bool bLoadedActorMaterialJson = LoadActorMaterialIdJson(
			Inputs.ActorMaterialJsonPath,
			Inputs.ActorMaterialEntryByColorKey,
			Inputs.ActorMaterialProjection);
		if (!bLoadedActorMaterialPng || !bLoadedActorMaterialJson)
		{
			Inputs.ActorMaterialRGBA.Reset();
			Inputs.ActorMaterialWidth = 0;
			Inputs.ActorMaterialHeight = 0;
			Inputs.ActorMaterialEntryByColorKey.Reset();
			Inputs.ActorMaterialProjection = FOrthographicProjectionMetadata();
			UE_LOG(
				LogTemp,
				Warning,
				TEXT("FaceReconstruct: actor/material id buffer unavailable for press=%s png_ok=%d json_ok=%d png=%s json=%s; attach material will use fallback path."),
				*PressId,
				bLoadedActorMaterialPng ? 1 : 0,
				bLoadedActorMaterialJson ? 1 : 0,
				*Inputs.ActorMaterialPngPath,
				*Inputs.ActorMaterialJsonPath);
		}
	}
	FString CameraLoadError;
	if (!LoadCameraJson(Inputs.CaptureJsonPath, Inputs.Camera, CameraLoadError))
	{
		SaveCommonFailureForComponents(ComponentNames, PressDir, CameraLoadError);
		SpawnMeshesOnGameThread(
			World, TArray<FReconstructedMesh>(), FString(), PressId, SessionGeneration,
			PressDir, ActionPressDir, false,
			CameraLoadError,
			MoveTemp(CompletionCallback));
		return;
	}
	FString ProjectionValidationError;
	if (!ValidateProjectionMetadata(Inputs, ProjectionValidationError))
	{
		SaveCommonFailureForComponents(ComponentNames, PressDir, ProjectionValidationError);
		SpawnMeshesOnGameThread(
			World, TArray<FReconstructedMesh>(), FString(), PressId, SessionGeneration,
			PressDir, ActionPressDir, false,
			ProjectionValidationError,
			MoveTemp(CompletionCallback));
		return;
	}
	UE_LOG(
		LogTemp,
		Log,
		TEXT("FaceReconstruct: projection validated press=%s source=%s matrix=(%.17g, %.17g, %.17g, %.17g) span=(%.6f, %.6f)"),
		*PressId,
		*Inputs.Camera.ProjectionMatrixSource,
		Inputs.Camera.ProjectionMatrix.M[0][0],
		Inputs.Camera.ProjectionMatrix.M[1][1],
		Inputs.Camera.ProjectionMatrix.M[3][0],
		Inputs.Camera.ProjectionMatrix.M[3][1],
		2.0 / FMath::Abs(Inputs.Camera.ProjectionMatrix.M[0][0]),
		2.0 / FMath::Abs(Inputs.Camera.ProjectionMatrix.M[1][1]));
	FString FaceLookupError;
	if (!BuildFaceLookups(Inputs, FaceLookupError))
	{
		SaveCommonFailureForComponents(ComponentNames, PressDir, FaceLookupError);
		SpawnMeshesOnGameThread(
			World, TArray<FReconstructedMesh>(), FString(), PressId, SessionGeneration,
			PressDir, ActionPressDir, false,
			FaceLookupError,
			MoveTemp(CompletionCallback));
		return;
	}

	FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));

	TArray<FComponentResult> Results;
	Results.SetNum(ComponentNames.Num());
	FromLZProcessing::LimitedParallelFor(ComponentNames.Num(), [&](int32 Index)
	{
		Results[Index] = ProcessComponent(ComponentNames[Index], PressDir, ActionPressDir, Inputs, Params);
	});
	SaveWorldOrthogonalPressIndex(PressDir, Results);

	TArray<FReconstructedMesh> MeshesToSpawn;
	for (const FComponentResult& Result : Results)
	{
		if (!Result.bSuccess)
		{
			continue;
		}

		if (Result.Solid.bSuccess)
		{
			FReconstructedMesh SolidMesh;
			SolidMesh.ActorName = Result.Solid.ActorName;
			SolidMesh.Tag = ReconstructedSolidTag;
			SolidMesh.VerticesWorld = Result.Solid.MeshVerticesWorld;
			SolidMesh.Triangles = Result.Solid.MeshTriangles;
			SolidMesh.Normal = Result.Solid.MeshNormal;
			SolidMesh.PressId = PressId;
			SolidMesh.bIsExcavateCutter = Result.Solid.Action.Equals(TEXT("excavate"), ESearchCase::IgnoreCase);
			SolidMesh.SourceFaceId = Result.Solid.SelectedFaceId;
			SolidMesh.SourcePlanePoint = Result.Solid.SourcePlanePoint;
			SolidMesh.SourcePlaneNormal = Result.Solid.SourcePlaneNormal;
			SolidMesh.SourceFaceVerticesWorld = Result.Solid.SourceFaceVerticesWorld;
			SolidMesh.SourceMaterialProbePointsWorld = Result.Solid.SourceMaterialProbePointsWorld;
			SolidMesh.AttachMaterialId = Result.Solid.AttachMaterialId;
			if (SolidMesh.bIsExcavateCutter)
			{
				ScaleVerticesAlongAxis(SolidMesh.VerticesWorld, SolidMesh.Normal, ExcavationCutterNormalScale);
				ScaleVerticesPerpendicularToAxis(SolidMesh.VerticesWorld, SolidMesh.Normal, ExcavationCutterCapScale);
			}
			SolidMesh.Color = SolidMesh.bIsExcavateCutter ? FColor(0, 120, 255, 80) : ReconstructedDebugBlue;
			MeshesToSpawn.Add(MoveTemp(SolidMesh));
		}
	}

	if (SessionGeneration != INDEX_NONE && !FFromLZSessionReset::IsSessionGenerationCurrent(SessionGeneration))
	{
		UE_LOG(LogTemp, Log, TEXT("FaceReconstruct: skipped stale runtime spawn for press %s because the session generation changed after reconstruction."), *PressId);
		FFromLZPressProcessResult Result;
		Result.bSuccess = false;
		Result.Message = TEXT("Skipped stale runtime spawn because the session generation changed after reconstruction.");
		Result.PressDir = PressDir;
		Result.ActionPressDir = ActionPressDir;
		DispatchPressCompletion(CompletionCallback, Result);
		return;
	}

	SpawnMeshesOnGameThread(
		World,
		MoveTemp(MeshesToSpawn),
		PressDir / TEXT("10_reconstruction_scene.obj"),
		PressId,
		SessionGeneration,
		PressDir,
		ActionPressDir,
		true,
		TEXT("Step 10/11 completed."),
		MoveTemp(CompletionCallback));
	UE_LOG(LogTemp, Log, TEXT("FaceReconstruct: processed %d component(s) for %s."), ComponentNames.Num(), *PressDir);
}

bool FFromLZFaceReconstructor::IsStep11RuntimeActor(const AActor* Actor)
{
	return ActorHasAnyStep11RuntimeTag(Actor);
}

bool FFromLZFaceReconstructor::IsStep11RuntimeActorActiveForCapture(const AActor* Actor)
{
	if (!Actor)
	{
		return false;
	}

	if (Actor->ActorHasTag(Step11ActionExcavateCutterTag))
	{
		return false;
	}

	const bool bCaptureRuntimeActor =
		Actor->ActorHasTag(Step11ActionAttachTag) ||
		Actor->ActorHasTag(Step11BooleanResultTag);
	return bCaptureRuntimeActor && ActorHasActiveStep11GeneratedByTag(Actor);
}

void FFromLZFaceReconstructor::RestoreStep11RuntimeBooleans(
	TWeakObjectPtr<UWorld> World,
	FFromLZStep11UndoCompletionCallback CompletionCallback)
{
	AsyncTask(ENamedThreads::GameThread, [World, CompletionCallback = MoveTemp(CompletionCallback)]() mutable
	{
		auto DispatchCompletion = [&CompletionCallback](const FFromLZStep11UndoResult& Result)
		{
			if (CompletionCallback)
			{
				CompletionCallback(Result);
			}
		};

		UWorld* WorldPtr = World.Get();
		if (!WorldPtr)
		{
			UE_LOG(LogTemp, Warning, TEXT("Step11: world is no longer valid; skipped restore."));
			FFromLZStep11UndoResult Result;
			Result.Message = TEXT("World is no longer valid.");
			DispatchCompletion(Result);
			return;
		}

		const FString PressId = PopActiveStep11Press();
		if (PressId.IsEmpty())
		{
			UE_LOG(LogTemp, Warning, TEXT("Step11: no active press id; skipped restore."));
			FFromLZStep11UndoResult Result;
			Result.Message = TEXT("No active Step11 press to undo.");
			DispatchCompletion(Result);
			return;
		}

		DispatchCompletion(RestoreStep11BooleanResults(WorldPtr, PressId));
	});
}

void FFromLZFaceReconstructor::ResetAllRuntimeState(TWeakObjectPtr<UWorld> World)
{
	UWorld* WorldPtr = World.Get();
	if (!WorldPtr)
	{
		return;
	}

	TArray<FString> ActivePresses = GActiveStep11PressStack;
	for (int32 Index = ActivePresses.Num() - 1; Index >= 0; --Index)
	{
		RestoreStep11BooleanResults(WorldPtr, ActivePresses[Index]);
	}

	TArray<AActor*> RuntimeActorsToDestroy;
	for (TActorIterator<AActor> It(WorldPtr); It; ++It)
	{
		AActor* Actor = *It;
		if (Actor && ActorHasAnyStep11RuntimeTag(Actor))
		{
			RuntimeActorsToDestroy.Add(Actor);
		}
	}
	for (AActor* Actor : RuntimeActorsToDestroy)
	{
		if (Actor)
		{
			Actor->Destroy();
		}
	}

	for (TActorIterator<AActor> It(WorldPtr); It; ++It)
	{
		AActor* Actor = *It;
		if (!Actor)
		{
			continue;
		}

		TArray<UPrimitiveComponent*> PrimitiveComponents;
		Actor->GetComponents<UPrimitiveComponent>(PrimitiveComponents);
		for (UPrimitiveComponent* Component : PrimitiveComponents)
		{
			if (!Component)
			{
				continue;
			}
			if (Component->ComponentTags.Contains(Step11HiddenSourceTag))
			{
				Component->SetVisibility(true, true);
				Component->SetHiddenInGame(false, true);
				Component->ComponentTags.Remove(Step11HiddenSourceTag);
			}

			for (int32 TagIndex = Component->ComponentTags.Num() - 1; TagIndex >= 0; --TagIndex)
			{
				const FString TagString = Component->ComponentTags[TagIndex].ToString();
				if (TagString.StartsWith(TEXT("FromLZ_HiddenBy_Press_"), ESearchCase::CaseSensitive) ||
					TagString.StartsWith(TEXT("FromLZ_GeneratedBy_Press_"), ESearchCase::CaseSensitive))
				{
					Component->ComponentTags.RemoveAt(TagIndex);
				}
			}
		}

		for (int32 TagIndex = Actor->Tags.Num() - 1; TagIndex >= 0; --TagIndex)
		{
			const FString TagString = Actor->Tags[TagIndex].ToString();
			if (TagString.StartsWith(TEXT("FromLZ_GeneratedBy_Press_"), ESearchCase::CaseSensitive))
			{
				Actor->Tags.RemoveAt(TagIndex);
			}
		}
	}

	GActiveStep11PressStack.Reset();
	RefreshActiveUndoPressId();
	UE_LOG(LogTemp, Log, TEXT("Step11: reset all runtime state."));
}
