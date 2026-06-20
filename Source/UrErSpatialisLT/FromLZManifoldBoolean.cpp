#include "FromLZManifoldBoolean.h"

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4390 4723)
#endif
THIRD_PARTY_INCLUDES_START
#include "manifold/manifold.h"
THIRD_PARTY_INCLUDES_END
#ifdef _MSC_VER
#pragma warning(pop)
#endif

namespace
{
	constexpr double ManifoldInputToleranceCm = 0.001;
	constexpr double ManifoldWeldToleranceCm = 0.01;

	static double SignedVolume(const UE::Geometry::FDynamicMesh3& Mesh)
	{
		double Volume = 0.0;
		for (int32 TriangleId : Mesh.TriangleIndicesItr())
		{
			const UE::Geometry::FIndex3i Tri = Mesh.GetTriangle(TriangleId);
			if (!Mesh.IsVertex(Tri.A) || !Mesh.IsVertex(Tri.B) || !Mesh.IsVertex(Tri.C))
			{
				continue;
			}

			const FVector3d A = Mesh.GetVertex(Tri.A);
			const FVector3d B = Mesh.GetVertex(Tri.B);
			const FVector3d C = Mesh.GetVertex(Tri.C);
			Volume += FVector3d::DotProduct(A, FVector3d::CrossProduct(B, C)) / 6.0;
		}
		return Volume;
	}

	static int32 SignedVolumeSign(double Volume)
	{
		if (Volume > 1e-6)
		{
			return 1;
		}
		if (Volume < -1e-6)
		{
			return -1;
		}
		return 0;
	}

	static FString ManifoldQuantizedVertexKey(const FVector3d& Position)
	{
		const int64 X = FMath::RoundToInt64(Position.X / ManifoldWeldToleranceCm);
		const int64 Y = FMath::RoundToInt64(Position.Y / ManifoldWeldToleranceCm);
		const int64 Z = FMath::RoundToInt64(Position.Z / ManifoldWeldToleranceCm);
		return FString::Printf(TEXT("%lld_%lld_%lld"), X, Y, Z);
	}

	static bool DynamicMeshToMeshGL64(
		const UE::Geometry::FDynamicMesh3& Mesh,
		manifold::MeshGL64& OutMesh,
		FString& OutError)
	{
		OutMesh = manifold::MeshGL64();
		OutMesh.numProp = 3;
		OutMesh.tolerance = ManifoldInputToleranceCm;
		if (Mesh.VertexCount() <= 0 || Mesh.TriangleCount() <= 0)
		{
			OutError = TEXT("empty_dynamic_mesh");
			return false;
		}

		TMap<int32, uint64_t> VertexToIndex;
		VertexToIndex.Reserve(Mesh.VertexCount());
		OutMesh.vertProperties.reserve(size_t(Mesh.VertexCount()) * 3);
		for (int32 VertexId : Mesh.VertexIndicesItr())
		{
			const FVector3d V = Mesh.GetVertex(VertexId);
			if (!FMath::IsFinite(V.X) || !FMath::IsFinite(V.Y) || !FMath::IsFinite(V.Z))
			{
				OutError = TEXT("non_finite_vertex");
				return false;
			}

			const uint64_t NewIndex = uint64_t(OutMesh.vertProperties.size() / 3);
			VertexToIndex.Add(VertexId, NewIndex);
			OutMesh.vertProperties.push_back(V.X);
			OutMesh.vertProperties.push_back(V.Y);
			OutMesh.vertProperties.push_back(V.Z);
		}

		OutMesh.triVerts.reserve(size_t(Mesh.TriangleCount()) * 3);
		for (int32 TriangleId : Mesh.TriangleIndicesItr())
		{
			const UE::Geometry::FIndex3i Tri = Mesh.GetTriangle(TriangleId);
			const uint64_t* A = VertexToIndex.Find(Tri.A);
			const uint64_t* B = VertexToIndex.Find(Tri.B);
			const uint64_t* C = VertexToIndex.Find(Tri.C);
			if (!A || !B || !C || *A == *B || *B == *C || *C == *A)
			{
				continue;
			}

			OutMesh.triVerts.push_back(*A);
			OutMesh.triVerts.push_back(*B);
			OutMesh.triVerts.push_back(*C);
		}

		if (OutMesh.NumTri() <= 0)
		{
			OutError = TEXT("no_valid_triangles");
			return false;
		}

		OutMesh.Merge();
		return true;
	}

	static bool MeshGL64ToDynamicMesh(
		const manifold::MeshGL64& Mesh,
		UE::Geometry::FDynamicMesh3& OutMesh,
		FString& OutError)
	{
		OutMesh = UE::Geometry::FDynamicMesh3();
		if (Mesh.NumTri() <= 0)
		{
			return true;
		}

		TMap<FString, int32> WeldedVertexByPosition;
		TArray<int32> ManifoldToDynamic;
		ManifoldToDynamic.SetNum(int32(Mesh.NumVert()));
		for (uint64_t VertexIndex = 0; VertexIndex < Mesh.NumVert(); ++VertexIndex)
		{
			const uint64_t Offset = VertexIndex * Mesh.numProp;
			if (Offset + 2 >= Mesh.vertProperties.size())
			{
				OutError = TEXT("output_vertex_property_out_of_bounds");
				return false;
			}

			const FVector3d V(Mesh.vertProperties[Offset], Mesh.vertProperties[Offset + 1], Mesh.vertProperties[Offset + 2]);
			if (!FMath::IsFinite(V.X) || !FMath::IsFinite(V.Y) || !FMath::IsFinite(V.Z))
			{
				OutError = TEXT("output_non_finite_vertex");
				return false;
			}

			const FString Key = ManifoldQuantizedVertexKey(V);
			if (const int32* Existing = WeldedVertexByPosition.Find(Key))
			{
				ManifoldToDynamic[int32(VertexIndex)] = *Existing;
			}
			else
			{
				const int32 DynamicId = OutMesh.AppendVertex(V);
				WeldedVertexByPosition.Add(Key, DynamicId);
				ManifoldToDynamic[int32(VertexIndex)] = DynamicId;
			}
		}

		int32 AddedTriangles = 0;
		for (uint64_t TriOffset = 0; TriOffset + 2 < Mesh.triVerts.size(); TriOffset += 3)
		{
			const uint64_t IA = Mesh.triVerts[TriOffset];
			const uint64_t IB = Mesh.triVerts[TriOffset + 1];
			const uint64_t IC = Mesh.triVerts[TriOffset + 2];
			if (!ManifoldToDynamic.IsValidIndex(int32(IA)) ||
				!ManifoldToDynamic.IsValidIndex(int32(IB)) ||
				!ManifoldToDynamic.IsValidIndex(int32(IC)))
			{
				continue;
			}

			const int32 A = ManifoldToDynamic[int32(IA)];
			const int32 B = ManifoldToDynamic[int32(IB)];
			const int32 C = ManifoldToDynamic[int32(IC)];
			if (A == B || B == C || C == A)
			{
				continue;
			}

			if (OutMesh.AppendTriangle(A, B, C) >= 0)
			{
				++AddedTriangles;
			}
		}

		if (AddedTriangles <= 0)
		{
			OutError = TEXT("output_no_valid_triangles");
			return false;
		}
		return true;
	}

	static FString ManifoldErrorToFString(manifold::Manifold::Error Error)
	{
		switch (Error)
		{
		case manifold::Manifold::Error::NoError:
			return TEXT("No Error");
		case manifold::Manifold::Error::NonFiniteVertex:
			return TEXT("Non Finite Vertex");
		case manifold::Manifold::Error::NotManifold:
			return TEXT("Not Manifold");
		case manifold::Manifold::Error::VertexOutOfBounds:
			return TEXT("Vertex Out Of Bounds");
		case manifold::Manifold::Error::PropertiesWrongLength:
			return TEXT("Properties Wrong Length");
		case manifold::Manifold::Error::MissingPositionProperties:
			return TEXT("Missing Position Properties");
		case manifold::Manifold::Error::MergeVectorsDifferentLengths:
			return TEXT("Merge Vectors Different Lengths");
		case manifold::Manifold::Error::MergeIndexOutOfBounds:
			return TEXT("Merge Index Out Of Bounds");
		case manifold::Manifold::Error::TransformWrongLength:
			return TEXT("Transform Wrong Length");
		case manifold::Manifold::Error::RunIndexWrongLength:
			return TEXT("Run Index Wrong Length");
		case manifold::Manifold::Error::FaceIDWrongLength:
			return TEXT("Face ID Wrong Length");
		case manifold::Manifold::Error::InvalidConstruction:
			return TEXT("Invalid Construction");
		case manifold::Manifold::Error::ResultTooLarge:
			return TEXT("Result Too Large");
		case manifold::Manifold::Error::InvalidTangents:
			return TEXT("Invalid Tangents");
		case manifold::Manifold::Error::Cancelled:
			return TEXT("Cancelled");
		default:
			return TEXT("Unknown Error");
		}
	}
}

const TCHAR* FFromLZManifoldBoolean::BackendName()
{
	return TEXT("Manifold3D");
}

const TCHAR* FFromLZManifoldBoolean::LibraryVersion()
{
	return TEXT("3.5.0+37125da");
}

bool FFromLZManifoldBoolean::Difference(
	const UE::Geometry::FDynamicMesh3& TargetMesh,
	const UE::Geometry::FDynamicMesh3& CutterMesh,
	int32 TargetRenderSign,
	UE::Geometry::FDynamicMesh3& OutResultMesh,
	FFromLZManifoldBooleanDiagnostics& OutDiagnostics)
{
	OutDiagnostics = FFromLZManifoldBooleanDiagnostics();
	OutResultMesh = UE::Geometry::FDynamicMesh3();
	OutDiagnostics.TargetInputTriangles = TargetMesh.TriangleCount();
	OutDiagnostics.CutterInputTriangles = CutterMesh.TriangleCount();

	UE::Geometry::FDynamicMesh3 TargetWork = TargetMesh;
	UE::Geometry::FDynamicMesh3 CutterWork = CutterMesh;
	double TargetVolume = SignedVolume(TargetWork);
	double CutterVolume = SignedVolume(CutterWork);
	if (SignedVolumeSign(TargetVolume) < 0)
	{
		TargetWork.ReverseOrientation(false);
		OutDiagnostics.bTargetOrientationReversedForManifold = true;
		TargetVolume = SignedVolume(TargetWork);
	}
	if (SignedVolumeSign(CutterVolume) < 0)
	{
		CutterWork.ReverseOrientation(false);
		OutDiagnostics.bCutterOrientationReversedForManifold = true;
		CutterVolume = SignedVolume(CutterWork);
	}
	OutDiagnostics.TargetSignedVolumeForManifold = TargetVolume;
	OutDiagnostics.CutterSignedVolumeForManifold = CutterVolume;

	FString ConvertError;
	manifold::MeshGL64 TargetGL;
	if (!DynamicMeshToMeshGL64(TargetWork, TargetGL, ConvertError))
	{
		OutDiagnostics.ManifoldErrorMessage = TEXT("target_meshgl_conversion_failed: ") + ConvertError;
		return false;
	}
	OutDiagnostics.bTargetManifoldInputValid = true;

	manifold::MeshGL64 CutterGL;
	if (!DynamicMeshToMeshGL64(CutterWork, CutterGL, ConvertError))
	{
		OutDiagnostics.ManifoldErrorMessage = TEXT("cutter_meshgl_conversion_failed: ") + ConvertError;
		return false;
	}
	OutDiagnostics.bCutterManifoldInputValid = true;

	manifold::ManifoldParams().suppressErrors = true;
	const manifold::Manifold TargetManifold(TargetGL);
	const manifold::Manifold::Error TargetStatus = TargetManifold.Status();
	if (TargetStatus != manifold::Manifold::Error::NoError)
	{
		OutDiagnostics.bTargetManifoldInputValid = false;
		OutDiagnostics.ManifoldErrorMessage = TEXT("target_manifold_status: ") + ManifoldErrorToFString(TargetStatus);
		return false;
	}

	const manifold::Manifold CutterManifold(CutterGL);
	const manifold::Manifold::Error CutterStatus = CutterManifold.Status();
	if (CutterStatus != manifold::Manifold::Error::NoError)
	{
		OutDiagnostics.bCutterManifoldInputValid = false;
		OutDiagnostics.ManifoldErrorMessage = TEXT("cutter_manifold_status: ") + ManifoldErrorToFString(CutterStatus);
		return false;
	}

	const manifold::Manifold ResultManifold = TargetManifold - CutterManifold;
	const manifold::Manifold::Error ResultStatus = ResultManifold.Status();
	if (ResultStatus != manifold::Manifold::Error::NoError)
	{
		OutDiagnostics.ManifoldErrorMessage = TEXT("difference_status: ") + ManifoldErrorToFString(ResultStatus);
		return false;
	}
	OutDiagnostics.bManifoldDifferenceSuccess = true;

	const manifold::MeshGL64 ResultGL = ResultManifold.GetMeshGL64();
	OutDiagnostics.ResultOutputTriangles = int32(ResultGL.NumTri());
	if (ResultGL.NumTri() <= 0)
	{
		OutDiagnostics.bOutputMeshBuilt = true;
		return true;
	}

	if (!MeshGL64ToDynamicMesh(ResultGL, OutResultMesh, ConvertError))
	{
		OutDiagnostics.ManifoldErrorMessage = TEXT("result_dynamic_mesh_conversion_failed: ") + ConvertError;
		return false;
	}

	OutDiagnostics.ResultSignedVolumeBeforeRenderFix = SignedVolume(OutResultMesh);
	const int32 ResultSign = SignedVolumeSign(OutDiagnostics.ResultSignedVolumeBeforeRenderFix);
	if (TargetRenderSign != 0 && ResultSign != 0 && ResultSign != TargetRenderSign)
	{
		OutResultMesh.ReverseOrientation(false);
		OutDiagnostics.bResultOrientationReversedToTargetSign = true;
	}

	OutDiagnostics.bOutputMeshBuilt = true;
	return true;
}
