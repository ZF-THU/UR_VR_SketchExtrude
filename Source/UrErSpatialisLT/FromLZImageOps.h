#pragma once

#include "CoreMinimal.h"

// Low-level single-channel (8-bit) image operations used by the 2D sketch
// pipeline. Masks are stored row-major in a TArray<uint8> of size Width*Height,
// with foreground = 255 and background = 0 unless noted otherwise.
namespace FromLZImageOps
{
	// Foreground = any pixel that is NOT near-white. Replaces the Python
	// grayscale+Gaussian+Otsu path for the colored composite (black/red/green/blue
	// lines on white). WhiteThreshold: a pixel counts as white when all of R,G,B
	// are strictly greater than WhiteThreshold.
	void BinarizeNonWhite(const TArray<uint8>& RGBA, int32 Width, int32 Height, uint8 WhiteThreshold, TArray<uint8>& OutBin);

	// Morphological dilation with a rectangular structuring element spanning
	// [OffMinX..OffMaxX] x [OffMinY..OffMaxY] relative to each pixel.
	void Dilate(const TArray<uint8>& In, int32 Width, int32 Height, int32 OffMinX, int32 OffMaxX, int32 OffMinY, int32 OffMaxY, TArray<uint8>& Out);

	// Morphological erosion with the same rectangular structuring element.
	void Erode(const TArray<uint8>& In, int32 Width, int32 Height, int32 OffMinX, int32 OffMaxX, int32 OffMinY, int32 OffMaxY, TArray<uint8>& Out);

	// Morphological close (dilate then erode) with a square KxK kernel, repeated Iterations times.
	void MorphClose(TArray<uint8>& InOut, int32 Width, int32 Height, int32 Kernel, int32 Iterations);

	// Square 2x2 dilation (matches the Python dilate(np.ones((2,2)))), repeated Iterations times.
	void Dilate2x2(TArray<uint8>& InOut, int32 Width, int32 Height, int32 Iterations);

	// Zero out 8-connected foreground components whose pixel area is below MinArea.
	void RemoveSmallComponents(TArray<uint8>& InOut, int32 Width, int32 Height, int32 MinArea);

	// Zhang-Suen thinning (port of the Python zhang_suen_thinning fallback).
	// Input/Output foreground = 255.
	void ZhangSuenThinning(const TArray<uint8>& In, int32 Width, int32 Height, TArray<uint8>& OutSkel, int32 MaxIter = 100);

	// Step 3 skeleton gap repair:
	//   1. On the original full skeleton, estimate every endpoint's outward direction by
	//      tracing about 5px inward along its incident edge. Connect strict mutual-nearest
	//      endpoint pairs (60/60 degrees), then endpoint-to-segment targets (source 60),
	//      then relaxed endpoint pairs (source 60, target 100). Connector color follows
	//      the source endpoint's incident-edge color.
	//   2. Extract a red/black graph from the first-pass result. Red degree-1 endpoints
	//      run the same strict endpoint, segment/topology-node, and relaxed endpoint
	//      direction rules against one fixed snapshot. These second-pass connectors are red.
	//   3. Backfill all connectors into the full skeleton, preserve every green connector,
	//      prune only connectors that close a small loop, then trim short dangling branches
	//      by Euclidean path length. Connector pixels participate in branch geometry.
	// All masks are foreground = 255. OutConnected is the first-pass full-skeleton
	// result; OutReconnected includes the second red-dead-end reconnect; the remaining
	// masks are the small-loop-pruned and final branch-cleaned stages. OutEffectiveColorMap
	// contains the surviving synthetic connector colors for downstream stroke coloring.
	void CleanupSkeletonEndpoints(
		const TArray<uint8>& Skel, int32 Width, int32 Height,
		float GapTol, int32 ConnectThickness,
		float SmallLoopBboxAreaThresh, float BranchPruneMaxPixels,
		const TArray<uint8>& SourceColorMap, int32 SourceColorSampleRadius,
		const FString& RedBlackConnectorsDebugPngPath, const FString& RedBlackReconnectedDebugPngPath,
		const FString& ConnectorPruneDebugJsonPath,
		TArray<uint8>& OutConnected, TArray<uint8>& OutReconnected,
		TArray<uint8>& OutSmallLoopPruned, TArray<uint8>& OutCleaned,
		TArray<uint8>& OutEffectiveColorMap);

	// A traced stroke is an ordered polyline of pixel coordinates (x, y).
	using FStroke = TArray<FVector2D>;

	// Color class of a stroke / pixel, derived from the input composite.
	// Red/Green/Blue are the hand-drawn primary strokes; Black is the 3D
	// line-art render; None is background (white) or unclassified.
	enum class EStrokeColor : uint8 { None = 0, Black = 1, Red = 2, Green = 3, Blue = 4 };

	const TCHAR* StrokeColorToString(EStrokeColor Color);

	// A stroke annotated with a color class plus how many of its points fell on
	// synthetic / background pixels (gap-repair connections) in the composite.
	struct FColoredStroke
	{
		FStroke Points;
		EStrokeColor Color = EStrokeColor::None;
		int32 ConnectionPointCount = 0; // points that were originally None (gap-repair / drift)

		// Step 7 geometry metrics (valid only when bHasMetrics is true).
		bool bHasMetrics = false;
		double Arc = 0.0;             // polyline arc length
		double Chord = 0.0;           // endpoint-to-endpoint distance
		double Straightness = 0.0;    // chord / arc (1 = perfectly straight)
		double P90PcaError = 0.0;     // 90th-percentile distance to PCA line
		double PcaRmsError = 0.0;     // RMS distance to PCA line
		double P90ChordDev = 0.0;     // 90th-percentile distance to chord line
		double ChordDevRatio = 0.0;   // P90ChordDev / chord
		FVector2D Direction = FVector2D::ZeroVector; // PCA principal axis
	};

	// Classify one RGB sample into a color class.
	EStrokeColor ClassifyRGB(uint8 R, uint8 G, uint8 B);
	EStrokeColor ClassifyRGB(uint8 R, uint8 G, uint8 B, uint8 WhiteCutoff, int32 DominanceMargin);

	// Per-pixel color-class map (values are EStrokeColor cast to uint8) for the composite.
	void BuildColorClassMap(const TArray<uint8>& RGBA, int32 Width, int32 Height, TArray<uint8>& OutMap);
	void BuildColorClassMap(const TArray<uint8>& RGBA, int32 Width, int32 Height, uint8 WhiteCutoff, int32 DominanceMargin, TArray<uint8>& OutMap);

	// Dominant non-None color class within a square window of the given radius (None if empty).
	EStrokeColor SampleColorAt(const TArray<uint8>& ColorMap, int32 Width, int32 Height, int32 X, int32 Y, int32 Radius);

	// Assign a color class to every traced stroke by sampling the composite color map,
	// then split each stroke at color-class boundaries into mono-color pieces. Synthetic
	// gap-repair runs (None) are reclassified by their neighbors:
	//   same color -> that color; primary vs primary -> nearest-aligned neighbor;
	//   red|black -> black; green|black -> green; blue|black -> blue.
	// RGB pieces of different colors are therefore never merged.
	// Short color runs whose arc length is below MinRunArc (e.g. black "blips" where a
	// colored stroke crosses a black render line) are absorbed into their neighbors so
	// each emitted piece is a single clean color.
	void ColorizeAndSplitStrokes(
		const TArray<FStroke>& In, const TArray<uint8>& ColorMap, int32 Width, int32 Height,
		int32 SampleRadius, float MinRunArc, TArray<FColoredStroke>& Out);

	// Corner-split colored strokes, preserving each piece's color class (same algorithm as
	// SplitStrokesAtCorners but per colored stroke).
	void SplitColoredStrokesAtCorners(
		const TArray<FColoredStroke>& In, float AngleThresh, int32 MinPixels,
		float SegmentArc, float SplitPeakMinDistance, int32 MaxIters,
		TArray<FColoredStroke>& Out);

	// Step 6 same-color merge (port of Python merge_post_corner_split_strokes, restricted
	// to identical color class): merge near-collinear fragments by endpoint gap + PCA axis
	// angle, protecting true junctions where a third stroke endpoint sits at the merge point.
	// Only strokes of the SAME color class are ever merged.
	void MergeColoredStrokesSameColor(
		const TArray<FColoredStroke>& In, float MaxGap, float MaxAngle, int32 MaxIters,
		float ProtectJunctionRadius, TArray<FColoredStroke>& Out);

	// Step 7 metrics (port of Python build_stroke_infos / stroke_linearity_metrics): fills
	// Arc/Chord/Straightness/PCA-line errors/Direction on each stroke (bHasMetrics = true).
	void ComputeStrokeMetrics(TArray<FColoredStroke>& InOut);

	// Step 8 enclosed-region mask (port of Python make_input_enclosed_region_mask, endpoint-
	// nearest-connect + flood variant): rasterize strokes (thickness), connect each endpoint
	// to its nearest endpoint to close gaps, then flood from the borders; pixels not reached
	// and not on the barrier are the enclosed interior. OutMask foreground = 255.
	void ComputeEnclosedRegionMask(
		const TArray<FColoredStroke>& Strokes, int32 Width, int32 Height,
		int32 Thickness, TArray<uint8>& OutMask, TArray<uint8>& OutBarrier);

	// Step 9 cap-extrusion recovery result.
	struct FCapBoundaryRun
	{
		int32 StrokeId = INDEX_NONE;
		EStrokeColor Color = EStrokeColor::None;
		bool bSynthetic = false;
		bool bReversed = false;
		int32 StartNodeId = INDEX_NONE;
		int32 EndNodeId = INDEX_NONE;
		FVector2D StartNodePosition = FVector2D::ZeroVector;
		FVector2D EndNodePosition = FVector2D::ZeroVector;
		FStroke Points;
		double ArcLengthPixels = 0.0;
		double ChordLengthPixels = 0.0;
		double Straightness = 0.0;
	};

	struct FCapExtrusionResult
	{
		bool bFound = false;
		bool bUsedBlack = false;          // loop required black strokes to close
		bool bHasInteriorGreen = false;   // compatibility: true when Action == "excavate"
		FString Action;                  // attach, excavate, or skip
		FString ActionDecisionReason;
		int32 LocalGreenStrokeCount = 0;
		double GreenInsideTotalLength = 0.0;
		double GreenOutsideTotalLength = 0.0;
		int32 InteriorGreenStrokeId = -1; // compatibility/debug: best local green by inside length
		int32 InteriorGreenInsidePoints = 0;
		int32 InteriorGreenTotalPoints = 0;
		double InteriorGreenInsideRatio = 0.0;
		double InteriorGreenInsideLength = 0.0;
		double InteriorGreenStrokeLength = 0.0;
		TArray<int32> CapStrokeIds;       // source/connector stroke indices forming the cap loop
		FString CandidateSource;          // red_only, local_black, or fallback_trace
		int32 CandidateAnchorStrokeId = -1;
		int32 SideStrokeId = -1;          // local green seed selected for the extrusion side
		FVector2D SideVector = FVector2D::ZeroVector; // traced ChainEnd - ChainStart
		double SideLength = 0.0;          // endpoint displacement used for cap translation
		double SideChainPathLength = 0.0; // full traced arc plus endpoint connection gaps
		FVector2D SideChainStart = FVector2D::ZeroVector;
		FVector2D SideChainEnd = FVector2D::ZeroVector;
		FVector2D SideSeedDirection = FVector2D::ZeroVector;
		TArray<int32> SideChainStrokeIds;
		double SideTraceTotalGap = 0.0;
		FString SideTraceStopReason;
		FStroke CapPolygon;               // ordered closed loop points
		FStroke CapPolygonTranslated;     // CapPolygon + SideVector
		TArray<FVector2D> CapNodes;       // ordered loop vertices (junction points)
		TArray<FCapBoundaryRun> OrderedBoundaryRuns; // loop order; connectors retain topology only

		// Selected full green chain. Candidates are compared by path length, then the
		// selected chain is oriented from its cap-near endpoint to its cap-far endpoint.
		TArray<FVector2D> SideCandidateVectors;
		TArray<FVector2D> SideCandidateStarts;
		TArray<FVector2D> SideCandidateEnds;

		bool bFaceEvaluationValid = false;
		FString FaceEvaluationSourcePolygon;
		FString FaceEvaluationRejectReason;
		int32 FaceEvaluationCapMaskPixels = 0;
		int32 PreselectedFaceId = -1;
		int32 PreselectedFaceOverlapPixels = 0;
		double PreselectedFaceOverlapRatio = 0.0;
		double PreselectedFaceNormalSideAngleDegrees = -1.0;
		double PreselectedFaceDistanceToCamera = 0.0;
	};

	// Step 9: detect every red cap loop in one pipeline run and recover its extrusion.
	// Real red/red and red/black intersections are planarized first, then exact-coordinate red
	// degree-1 nodes enter a two-stage repair. Stage A resolves black contact within
	// 2px, then globally pairs compatible red dead ends within ConnectorTol; a black
	// candidate vetoes a pair only when it is more than 2px closer. The Stage A
	// connectors are planarized with the real geometry and an exact mixed red/black
	// graph identifies unresolved original endpoints. Stage B independently searches
	// real red-segment interiors and black segments in each source endpoint's forward
	// half-plane; synthetic connectors participate in topology but are not red targets.
	// Connector paths that hit unrelated real geometry or existing connectors first
	// are rejected. A final connector-aware planarization precedes graph extraction.
	// Cycle anchors must be real red strokes with at least 20px arc length. Synthetic
	// red connectors may participate in paths but cannot anchor a search. Each anchor
	// retains one shortest-edge valid cycle, or at most two when multiple valid cycles
	// tie at that minimum edge count; ties prefer longer real-red arc, shorter black
	// arc, then smaller polygon area.
	// Components are red-driven: red-only loops are selected first, then local black
	// closures, then fallback red/black traces. Before consuming red strokes, every
	// candidate must have a valid local-green action and a source polygon that covers
	// at least 25% of one captured face whose projected normal is within 30 degrees
	// of the unoriented green side vector and whose plane can be intersected. Conflicting
	// red-only loops prefer the larger cap area. All loop sources currently reject
	// candidates below 500 px^2 bounding-box area.
	// Black strokes never define the initial component split.
	// Each selected loop writes its 09a/09b/09 debug into PressDir/Component_%%/. For each cap
	// an Action.json is written to ActionPressDir/Component_%%/: local green stroke pixels are
	// accumulated inside/outside the cap loop; inside > outside -> "excavate", outside > inside
	// -> "attach", and ties or missing local green -> "skip". Returns the number of caps found
	// and fills OutResults (one per Component_%%, in folder order).
	// After planarization and connector insertion, graph endpoints within 5px are
	// merged into shared nodes before the existing component extraction begins.
	int32 RecoverCapExtrusionsPerComponent(const TArray<FColoredStroke>& Strokes, float ConnectorTol, float BlackSelectTol, int32 Width, int32 Height, const FString& PressDir, const FString& ActionPressDir, TArray<FCapExtrusionResult>& OutResults);

	// Debug render of the recovered extrusion: cap loop, translated cap, and side connectors.
	bool SaveCapExtrusionPng(const TArray<FColoredStroke>& Strokes, const FCapExtrusionResult& Res, int32 Width, int32 Height, const FString& Path, int32 Thickness = 2);

	// JSON describing the recovered cap/side/translation.
	bool SaveCapExtrusionJson(const FCapExtrusionResult& Res, const FString& Path);

	// Render colored strokes using their class color (red/green/blue/black, None=gray) on white.
	bool SaveColoredStrokesPng(const TArray<FColoredStroke>& Strokes, int32 Width, int32 Height, const FString& Path, int32 Thickness = 2);

	// Write a JSON file describing every colored stroke: id, color, point coordinates,
	// endpoints, connection-point count, and neighbor ids (strokes sharing an endpoint
	// within EndpointTol pixels).
	bool SaveColoredStrokesJson(const TArray<FColoredStroke>& Strokes, int32 Width, int32 Height, const FString& Path, float EndpointTol);

	// Step 4 stroke tracing (port of Python trace_strokes): classify skeleton
	// pixels by crossing number, then walk 8-connected polylines between true
	// endpoints/branches (plus a second pass for pure cycles). Strokes shorter
	// than MinPixels points are discarded.
	void TraceStrokes(const TArray<uint8>& Skel, int32 Width, int32 Height, int32 MinPixels, TArray<FStroke>& OutStrokes);

	// Step 5 corner splitting (port of Python split_stroke_at_corners): split each
	// stroke at points where the unoriented PCA axis angle between the left/right
	// arc-length windows exceeds AngleThresh (degrees). Iterative non-maximum
	// suppression with arc-distance conflict resolution. AngleThresh <= 0 disables.
	void SplitStrokesAtCorners(
		const TArray<FStroke>& In, float AngleThresh, int32 MinPixels,
		float SegmentArc, float SplitPeakMinDistance, int32 MaxIters,
		TArray<FStroke>& Out);

	// Render strokes as distinct-colored polylines on white for debug, then write a PNG.
	bool SaveStrokesPng(const TArray<FStroke>& Strokes, int32 Width, int32 Height, const FString& Path, int32 Thickness = 2);

	// Encode an 8-bit single-channel mask as a grayscale-on-white-style RGB PNG and write it.
	// Foreground (255) is drawn black on white background for readable debug output when bInvertForDisplay is true;
	// otherwise the raw mask value is written to R=G=B.
	bool SaveMaskPng(const TArray<uint8>& Mask, int32 Width, int32 Height, const FString& Path, bool bInvertForDisplay);
}
