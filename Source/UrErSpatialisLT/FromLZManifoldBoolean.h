#pragma once

#include "CoreMinimal.h"
#include "DynamicMesh/DynamicMesh3.h"

struct FFromLZManifoldBooleanDiagnostics
{
	FString BooleanBackend = TEXT("Manifold3D");
	FString LibraryVersion = TEXT("3.5.0+37125da");
	bool bTargetManifoldInputValid = false;
	bool bCutterManifoldInputValid = false;
	bool bManifoldDifferenceSuccess = false;
	bool bOutputMeshBuilt = false;
	bool bTargetOrientationReversedForManifold = false;
	bool bCutterOrientationReversedForManifold = false;
	bool bResultOrientationReversedToTargetSign = false;
	bool bCapSectionUnavailable = true;
	FString ManifoldErrorMessage;
	int32 TargetInputTriangles = 0;
	int32 CutterInputTriangles = 0;
	int32 ResultOutputTriangles = 0;
	double TargetSignedVolumeForManifold = 0.0;
	double CutterSignedVolumeForManifold = 0.0;
	double ResultSignedVolumeBeforeRenderFix = 0.0;
};

class FFromLZManifoldBoolean
{
public:
	static const TCHAR* BackendName();
	static const TCHAR* LibraryVersion();

	static bool Difference(
		const UE::Geometry::FDynamicMesh3& TargetMesh,
		const UE::Geometry::FDynamicMesh3& CutterMesh,
		int32 TargetRenderSign,
		UE::Geometry::FDynamicMesh3& OutResultMesh,
		FFromLZManifoldBooleanDiagnostics& OutDiagnostics);
};
