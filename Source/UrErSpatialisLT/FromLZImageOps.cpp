#include "FromLZImageOps.h"
#include "FromLZFaceReconstructor.h"
#include "FromLZProcessingLimits.h"

#include "Algo/Reverse.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformTime.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Misc/FileHelper.h"
#include "Modules/ModuleManager.h"

namespace FromLZImageOps
{
	void BinarizeNonWhite(const TArray<uint8>& RGBA, int32 Width, int32 Height, uint8 WhiteThreshold, TArray<uint8>& OutBin)
	{
		const int32 N = Width * Height;
		OutBin.SetNumUninitialized(N);
		for (int32 i = 0; i < N; ++i)
		{
			const int32 Off = i * 4;
			const uint8 R = RGBA[Off + 0];
			const uint8 G = RGBA[Off + 1];
			const uint8 B = RGBA[Off + 2];
			const bool bWhite = R > WhiteThreshold && G > WhiteThreshold && B > WhiteThreshold;
			OutBin[i] = bWhite ? 0 : 255;
		}
	}

	void Dilate(const TArray<uint8>& In, int32 Width, int32 Height, int32 OffMinX, int32 OffMaxX, int32 OffMinY, int32 OffMaxY, TArray<uint8>& Out)
	{
		Out.SetNumUninitialized(Width * Height);
		for (int32 y = 0; y < Height; ++y)
		{
			for (int32 x = 0; x < Width; ++x)
			{
				uint8 v = 0;
				for (int32 oy = OffMinY; oy <= OffMaxY && v == 0; ++oy)
				{
					const int32 yy = y + oy;
					if (yy < 0 || yy >= Height)
					{
						continue;
					}
					for (int32 ox = OffMinX; ox <= OffMaxX; ++ox)
					{
						const int32 xx = x + ox;
						if (xx < 0 || xx >= Width)
						{
							continue;
						}
						if (In[yy * Width + xx] > 0)
						{
							v = 255;
							break;
						}
					}
				}
				Out[y * Width + x] = v;
			}
		}
	}

	void Erode(const TArray<uint8>& In, int32 Width, int32 Height, int32 OffMinX, int32 OffMaxX, int32 OffMinY, int32 OffMaxY, TArray<uint8>& Out)
	{
		Out.SetNumUninitialized(Width * Height);
		for (int32 y = 0; y < Height; ++y)
		{
			for (int32 x = 0; x < Width; ++x)
			{
				uint8 v = 255;
				for (int32 oy = OffMinY; oy <= OffMaxY && v == 255; ++oy)
				{
					const int32 yy = y + oy;
					for (int32 ox = OffMinX; ox <= OffMaxX; ++ox)
					{
						const int32 xx = x + ox;
						const bool bInside = xx >= 0 && xx < Width && yy >= 0 && yy < Height;
						if (!bInside || In[yy * Width + xx] == 0)
						{
							v = 0;
							break;
						}
					}
				}
				Out[y * Width + x] = v;
			}
		}
	}

	void MorphClose(TArray<uint8>& InOut, int32 Width, int32 Height, int32 Kernel, int32 Iterations)
	{
		if (Kernel <= 1 || Iterations <= 0)
		{
			return;
		}
		const int32 r = Kernel / 2;
		TArray<uint8> Tmp;
		for (int32 it = 0; it < Iterations; ++it)
		{
			Dilate(InOut, Width, Height, -r, r, -r, r, Tmp);
			Erode(Tmp, Width, Height, -r, r, -r, r, InOut);
		}
	}

	void Dilate2x2(TArray<uint8>& InOut, int32 Width, int32 Height, int32 Iterations)
	{
		if (Iterations <= 0)
		{
			return;
		}
		TArray<uint8> Tmp;
		for (int32 it = 0; it < Iterations; ++it)
		{
			// 2x2 structuring element with anchor at top-left: offsets {0,1} in x and y.
			Dilate(InOut, Width, Height, 0, 1, 0, 1, Tmp);
			InOut = Tmp;
		}
	}

	void RemoveSmallComponents(TArray<uint8>& InOut, int32 Width, int32 Height, int32 MinArea)
	{
		if (MinArea <= 1)
		{
			return;
		}
		const int32 N = Width * Height;
		TArray<int32> Label;
		Label.Init(-1, N);

		TArray<int32> Stack;
		Stack.Reserve(1024);

		static const int32 DX[8] = { 1, -1, 0, 0, 1, 1, -1, -1 };
		static const int32 DY[8] = { 0, 0, 1, -1, 1, -1, 1, -1 };

		TArray<int32> CompPixels;
		CompPixels.Reserve(1024);

		for (int32 start = 0; start < N; ++start)
		{
			if (InOut[start] == 0 || Label[start] != -1)
			{
				continue;
			}

			CompPixels.Reset();
			Stack.Reset();
			Stack.Push(start);
			Label[start] = start;

			while (Stack.Num() > 0)
			{
				const int32 p = Stack.Pop(EAllowShrinking::No);
				CompPixels.Push(p);
				const int32 px = p % Width;
				const int32 py = p / Width;
				for (int32 k = 0; k < 8; ++k)
				{
					const int32 nx = px + DX[k];
					const int32 ny = py + DY[k];
					if (nx < 0 || nx >= Width || ny < 0 || ny >= Height)
					{
						continue;
					}
					const int32 np = ny * Width + nx;
					if (InOut[np] > 0 && Label[np] == -1)
					{
						Label[np] = start;
						Stack.Push(np);
					}
				}
			}

			if (CompPixels.Num() < MinArea)
			{
				for (int32 p : CompPixels)
				{
					InOut[p] = 0;
				}
			}
		}
	}

	void ZhangSuenThinning(const TArray<uint8>& In, int32 Width, int32 Height, TArray<uint8>& OutSkel, int32 MaxIter)
	{
		const int32 N = Width * Height;
		TArray<uint8> Img;
		Img.SetNumUninitialized(N);
		for (int32 i = 0; i < N; ++i)
		{
			Img[i] = In[i] > 0 ? 1 : 0;
		}

		auto At = [&](int32 x, int32 y) -> uint8
		{
			if (x < 0 || x >= Width || y < 0 || y >= Height)
			{
				return 0;
			}
			return Img[y * Width + x];
		};

		TArray<uint8> Marker;
		Marker.SetNumUninitialized(N);

		for (int32 iter = 0; iter < MaxIter; ++iter)
		{
			bool bChanged = false;

			for (int32 step = 0; step < 2; ++step)
			{
				FMemory::Memzero(Marker.GetData(), Marker.Num());

				for (int32 y = 0; y < Height; ++y)
				{
					for (int32 x = 0; x < Width; ++x)
					{
						if (Img[y * Width + x] != 1)
						{
							continue;
						}

						const uint8 P2 = At(x, y - 1);
						const uint8 P3 = At(x + 1, y - 1);
						const uint8 P4 = At(x + 1, y);
						const uint8 P5 = At(x + 1, y + 1);
						const uint8 P6 = At(x, y + 1);
						const uint8 P7 = At(x - 1, y + 1);
						const uint8 P8 = At(x - 1, y);
						const uint8 P9 = At(x - 1, y - 1);

						const int32 B = P2 + P3 + P4 + P5 + P6 + P7 + P8 + P9;
						if (B < 2 || B > 6)
						{
							continue;
						}

						int32 A = 0;
						A += (P2 == 0 && P3 == 1) ? 1 : 0;
						A += (P3 == 0 && P4 == 1) ? 1 : 0;
						A += (P4 == 0 && P5 == 1) ? 1 : 0;
						A += (P5 == 0 && P6 == 1) ? 1 : 0;
						A += (P6 == 0 && P7 == 1) ? 1 : 0;
						A += (P7 == 0 && P8 == 1) ? 1 : 0;
						A += (P8 == 0 && P9 == 1) ? 1 : 0;
						A += (P9 == 0 && P2 == 1) ? 1 : 0;
						if (A != 1)
						{
							continue;
						}

						if (step == 0)
						{
							if ((P2 * P4 * P6) == 0 && (P4 * P6 * P8) == 0)
							{
								Marker[y * Width + x] = 1;
							}
						}
						else
						{
							if ((P2 * P4 * P8) == 0 && (P2 * P6 * P8) == 0)
							{
								Marker[y * Width + x] = 1;
							}
						}
					}
				}

				for (int32 i = 0; i < N; ++i)
				{
					if (Marker[i])
					{
						Img[i] = 0;
						bChanged = true;
					}
				}
			}

			if (!bChanged)
			{
				break;
			}
		}

		OutSkel.SetNumUninitialized(N);
		for (int32 i = 0; i < N; ++i)
		{
			OutSkel[i] = Img[i] ? 255 : 0;
		}
	}

	// ====================================================================
	// Step 3 helpers: skeleton graph topology (ported from the Python
	// get_neighbors / crossing_number / skeleton_node_type primitives).
	// All operate on a foreground=255 mask laid out row-major.
	// ====================================================================
	namespace SkelGraph
	{
		FORCEINLINE bool Fg(const uint8* M, int32 W, int32 H, int32 x, int32 y)
		{
			return x >= 0 && x < W && y >= 0 && y < H && M[y * W + x] > 0;
		}

		// "Safe" 8-neighbors: 4-neighbors always; a diagonal only when neither of
		// its two orthogonal sides is foreground (avoids corner-cutting shortcuts).
		// Fills OutN (capacity 8) and returns the count.
		int32 SafeNeighbors(const uint8* M, int32 W, int32 H, int32 x, int32 y, int32 OutN[8])
		{
			int32 Count = 0;
			static const int32 OD[4][2] = { {1, 0}, {-1, 0}, {0, 1}, {0, -1} };
			for (int32 k = 0; k < 4; ++k)
			{
				const int32 xx = x + OD[k][0];
				const int32 yy = y + OD[k][1];
				if (Fg(M, W, H, xx, yy))
				{
					OutN[Count++] = yy * W + xx;
				}
			}
			static const int32 DD[4][2] = { {1, 1}, {1, -1}, {-1, 1}, {-1, -1} };
			for (int32 k = 0; k < 4; ++k)
			{
				const int32 dx = DD[k][0];
				const int32 dy = DD[k][1];
				const int32 xx = x + dx;
				const int32 yy = y + dy;
				if (!Fg(M, W, H, xx, yy))
				{
					continue;
				}
				const bool bSide1 = Fg(M, W, H, x + dx, y);
				const bool bSide2 = Fg(M, W, H, x, y + dy);
				if (!bSide1 && !bSide2)
				{
					OutN[Count++] = yy * W + xx;
				}
			}
			return Count;
		}

		// Crossing number over the raw 8-neighborhood (P2..P9 = N,NE,E,SE,S,SW,W,NW).
		int32 CrossingNumber(const uint8* M, int32 W, int32 H, int32 x, int32 y)
		{
			const int32 V[8] = {
				Fg(M, W, H, x,     y - 1) ? 1 : 0,
				Fg(M, W, H, x + 1, y - 1) ? 1 : 0,
				Fg(M, W, H, x + 1, y)     ? 1 : 0,
				Fg(M, W, H, x + 1, y + 1) ? 1 : 0,
				Fg(M, W, H, x,     y + 1) ? 1 : 0,
				Fg(M, W, H, x - 1, y + 1) ? 1 : 0,
				Fg(M, W, H, x - 1, y)     ? 1 : 0,
				Fg(M, W, H, x - 1, y - 1) ? 1 : 0,
			};
			int32 Transitions = 0;
			for (int32 i = 0; i < 8; ++i)
			{
				if (V[i] != V[(i + 1) % 8])
				{
					++Transitions;
				}
			}
			return Transitions / 2;
		}

		enum class ENode : uint8 { Isolated, Endpoint, Branch, Regular };

		ENode NodeType(const uint8* M, int32 W, int32 H, int32 x, int32 y)
		{
			int32 Tmp[8];
			const int32 Deg = SafeNeighbors(M, W, H, x, y, Tmp);
			if (Deg == 0)
			{
				return ENode::Isolated;
			}
			if (Deg == 1)
			{
				return ENode::Endpoint;
			}
			if (CrossingNumber(M, W, H, x, y) >= 3)
			{
				return ENode::Branch;
			}
			return ENode::Regular;
		}

		void FindEndpoints(const uint8* M, int32 W, int32 H, TArray<FIntPoint>& Out)
		{
			Out.Reset();
			for (int32 y = 0; y < H; ++y)
			{
				for (int32 x = 0; x < W; ++x)
				{
					if (M[y * W + x] > 0 && NodeType(M, W, H, x, y) == ENode::Endpoint)
					{
						Out.Emplace(x, y);
					}
				}
			}
		}

		// Bresenham 8-connected line points (matches cv2.LINE_8), inclusive of both ends.
		void LinePoints(FIntPoint P0, FIntPoint P1, TArray<FIntPoint>& Out)
		{
			Out.Reset();
			int32 x0 = P0.X, y0 = P0.Y;
			const int32 x1 = P1.X, y1 = P1.Y;
			const int32 dx = FMath::Abs(x1 - x0);
			const int32 dy = -FMath::Abs(y1 - y0);
			const int32 sx = x0 < x1 ? 1 : -1;
			const int32 sy = y0 < y1 ? 1 : -1;
			int32 err = dx + dy;
			for (;;)
			{
				Out.Emplace(x0, y0);
				if (x0 == x1 && y0 == y1)
				{
					break;
				}
				const int32 e2 = 2 * err;
				if (e2 >= dy)
				{
					err += dy;
					x0 += sx;
				}
				if (e2 <= dx)
				{
					err += dx;
					y0 += sy;
				}
			}
		}

		// Shortest pixel path (BFS over safe neighbors) from Start to Goal; both must be
		// foreground. Returns true and fills OutPath (Start..Goal inclusive) when reachable.
		bool ShortestPath(const uint8* M, int32 W, int32 H, FIntPoint Start, FIntPoint Goal, TArray<FIntPoint>& OutPath)
		{
			OutPath.Reset();
			if (!Fg(M, W, H, Start.X, Start.Y) || !Fg(M, W, H, Goal.X, Goal.Y))
			{
				return false;
			}
			const int32 N = W * H;
			const int32 StartIdx = Start.Y * W + Start.X;
			const int32 GoalIdx = Goal.Y * W + Goal.X;

			TArray<int32> Parent;
			Parent.Init(-2, N); // -2 = unvisited, -1 = root
			TArray<int32> Queue;
			Queue.Reserve(256);
			Queue.Add(StartIdx);
			Parent[StartIdx] = -1;

			int32 Head = 0;
			bool bFound = (StartIdx == GoalIdx);
			while (Head < Queue.Num() && !bFound)
			{
				const int32 Cur = Queue[Head++];
				const int32 cx = Cur % W;
				const int32 cy = Cur / W;
				int32 Nb[8];
				const int32 Deg = SafeNeighbors(M, W, H, cx, cy, Nb);
				for (int32 i = 0; i < Deg; ++i)
				{
					const int32 NI = Nb[i];
					if (Parent[NI] != -2)
					{
						continue;
					}
					Parent[NI] = Cur;
					if (NI == GoalIdx)
					{
						bFound = true;
						break;
					}
					Queue.Add(NI);
				}
			}

			if (!bFound)
			{
				return false;
			}

			for (int32 Cur = GoalIdx; Cur != -1; Cur = Parent[Cur])
			{
				OutPath.Emplace(Cur % W, Cur / W);
			}
			Algo::Reverse(OutPath);
			return true;
		}

		void BranchStopPoints(const uint8* M, int32 W, int32 H, TSet<int32>& Out)
		{
			Out.Reset();
			for (int32 y = 0; y < H; ++y)
			{
				for (int32 x = 0; x < W; ++x)
				{
					if (M[y * W + x] > 0 && NodeType(M, W, H, x, y) == ENode::Branch)
					{
						Out.Add(y * W + x);
					}
				}
			}
		}

		// Pick the candidate that best preserves direction (max dot of unit vectors).
		int32 ChooseBestContinuation(FIntPoint Prev, FIntPoint Cur, const int32* Cands, int32 NumCands, int32 W)
		{
			if (NumCands == 0)
			{
				return -1;
			}
			if (NumCands == 1)
			{
				return Cands[0];
			}
			FVector2D PrevV(Cur.X - Prev.X, Cur.Y - Prev.Y);
			if (PrevV.Size() < 1e-8)
			{
				return Cands[0];
			}
			PrevV.Normalize();
			int32 Best = -1;
			double BestScore = -1e9;
			for (int32 i = 0; i < NumCands; ++i)
			{
				const int32 qx = Cands[i] % W;
				const int32 qy = Cands[i] / W;
				FVector2D NextV(qx - Cur.X, qy - Cur.Y);
				if (NextV.Size() < 1e-8)
				{
					continue;
				}
				NextV.Normalize();
				const double Score = FVector2D::DotProduct(PrevV, NextV);
				if (Score > BestScore)
				{
					BestScore = Score;
					Best = Cands[i];
				}
			}
			return Best;
		}

		// Trace one dangling path from an endpoint until the first branch/endpoint
		// (or a stop node). Returns coords including Start and the terminal node.
		void TraceBranchPath(const uint8* M, int32 W, int32 H, FIntPoint Start, const TSet<int32>& StopNodes, TArray<FIntPoint>& OutPath)
		{
			OutPath.Reset();
			OutPath.Add(Start);
			FIntPoint Prev(-1, -1);
			bool bHasPrev = false;
			FIntPoint Cur = Start;
			int32 Safety = 0;
			for (;;)
			{
				if (++Safety > 20000)
				{
					break;
				}
				int32 Nb[8];
				const int32 Deg = SafeNeighbors(M, W, H, Cur.X, Cur.Y, Nb);
				int32 Cands[8];
				int32 NumCands = 0;
				const int32 PrevIdx = bHasPrev ? (Prev.Y * W + Prev.X) : -1;
				for (int32 i = 0; i < Deg; ++i)
				{
					if (Nb[i] != PrevIdx)
					{
						Cands[NumCands++] = Nb[i];
					}
				}
				if (NumCands == 0)
				{
					break;
				}
				if (OutPath.Num() > 1)
				{
					const ENode T = NodeType(M, W, H, Cur.X, Cur.Y);
					if (T == ENode::Endpoint || T == ENode::Branch)
					{
						break;
					}
				}
				const FIntPoint Ref = bHasPrev ? Prev : Cur;
				const int32 NextIdx = ChooseBestContinuation(Ref, Cur, Cands, NumCands, W);
				if (NextIdx < 0)
				{
					break;
				}
				Prev = Cur;
				bHasPrev = true;
				Cur = FIntPoint(NextIdx % W, NextIdx / W);
				OutPath.Add(Cur);
				if (StopNodes.Contains(NextIdx))
				{
					break;
				}
				const ENode T = NodeType(M, W, H, Cur.X, Cur.Y);
				if (T == ENode::Endpoint || T == ENode::Branch)
				{
					break;
				}
			}
		}

		FVector2D EndpointOutwardDirection(
			const uint8* M,
			int32 W,
			int32 H,
			const FIntPoint& Endpoint,
			double InteriorArcPixels,
			TArray<int32>* OutIncidentEdgePixels = nullptr)
		{
			if (OutIncidentEdgePixels)
			{
				OutIncidentEdgePixels->Reset();
			}
			if (!Fg(M, W, H, Endpoint.X, Endpoint.Y) ||
				NodeType(M, W, H, Endpoint.X, Endpoint.Y) != ENode::Endpoint)
			{
				return FVector2D::ZeroVector;
			}

			const int32 EndpointIdx = Endpoint.Y * W + Endpoint.X;
			if (OutIncidentEdgePixels)
			{
				OutIncidentEdgePixels->Add(EndpointIdx);
			}

			FIntPoint Prev = Endpoint;
			FIntPoint Cur = Endpoint;
			bool bHasPrev = false;
			double Arc = 0.0;
			int32 Safety = 0;
			while (++Safety <= 20000)
			{
				int32 Neighbors[8];
				const int32 Degree = SafeNeighbors(M, W, H, Cur.X, Cur.Y, Neighbors);
				int32 Candidates[8];
				int32 CandidateCount = 0;
				const int32 PrevIdx = bHasPrev ? Prev.Y * W + Prev.X : INDEX_NONE;
				for (int32 i = 0; i < Degree; ++i)
				{
					if (Neighbors[i] != PrevIdx)
					{
						Candidates[CandidateCount++] = Neighbors[i];
					}
				}
				if (CandidateCount == 0)
				{
					break;
				}

				const FIntPoint Reference = bHasPrev ? Prev : Cur;
				const int32 NextIdx = ChooseBestContinuation(Reference, Cur, Candidates, CandidateCount, W);
				if (NextIdx < 0)
				{
					break;
				}

				const FIntPoint Next(NextIdx % W, NextIdx / W);
				Arc += FVector2D::Distance(
					FVector2D(Cur.X, Cur.Y),
					FVector2D(Next.X, Next.Y));
				Prev = Cur;
				Cur = Next;
				bHasPrev = true;
				if (OutIncidentEdgePixels)
				{
					OutIncidentEdgePixels->Add(NextIdx);
				}

				if (Arc >= InteriorArcPixels)
				{
					break;
				}
				const ENode Type = NodeType(M, W, H, Cur.X, Cur.Y);
				if (Type == ENode::Endpoint || Type == ENode::Branch)
				{
					break;
				}
			}

			return (FVector2D(Endpoint.X, Endpoint.Y) - FVector2D(Cur.X, Cur.Y)).GetSafeNormal();
		}
	} // namespace SkelGraph

	void CleanupSkeletonEndpoints(
		const TArray<uint8>& Skel, int32 Width, int32 Height,
		float GapTol, int32 ConnectThickness,
		float SmallLoopBboxAreaThresh, float BranchPruneMaxPixels,
		const TArray<uint8>& SourceColorMap, int32 SourceColorSampleRadius,
		const FString& RedBlackConnectorsDebugPngPath, const FString& RedBlackReconnectedDebugPngPath,
		const FString& ConnectorPruneDebugJsonPath,
		TArray<uint8>& OutConnected, TArray<uint8>& OutReconnected,
		TArray<uint8>& OutSmallLoopPruned, TArray<uint8>& OutCleaned,
		TArray<uint8>& OutEffectiveColorMap)
	{
		const int32 N = Width * Height;
		const bool bHasSourceColorMap = SourceColorMap.Num() >= N;
		const int32 SampleRadius = FMath::Max(0, SourceColorSampleRadius);
		const double Tol2 = double(GapTol) * double(GapTol);
		constexpr double EndpointDirectionTracePixels = 5.0;
		constexpr double EndpointForwardCosThreshold = 0.5; // cos(60 degrees)
		constexpr double RelaxedTargetCosThreshold = -0.17364817766693033; // cos(100 degrees)
		(void)ConnectThickness;

		TArray<uint8> Work;
		Work.SetNumUninitialized(N);
		for (int32 i = 0; i < N; ++i)
		{
			Work[i] = Skel[i] > 0 ? 255 : 0;
		}

		auto SampleMapColorClass = [&](const TArray<uint8>& Map, const FIntPoint& P) -> EStrokeColor
		{
			if (Map.Num() < N)
			{
				return EStrokeColor::None;
			}

			int32 Counts[5] = { 0, 0, 0, 0, 0 };
			const int32 MinY = FMath::Max(0, P.Y - SampleRadius);
			const int32 MaxY = FMath::Min(Height - 1, P.Y + SampleRadius);
			const int32 MinX = FMath::Max(0, P.X - SampleRadius);
			const int32 MaxX = FMath::Min(Width - 1, P.X + SampleRadius);
			for (int32 y = MinY; y <= MaxY; ++y)
			{
				for (int32 x = MinX; x <= MaxX; ++x)
				{
					const uint8 C = Map[y * Width + x];
					if (C > uint8(EStrokeColor::None) && C <= uint8(EStrokeColor::Blue))
					{
						++Counts[C];
					}
				}
			}

			int32 Best = int32(EStrokeColor::None);
			int32 BestCount = 0;
			for (int32 C = 1; C <= 4; ++C)
			{
				if (Counts[C] > BestCount)
				{
					Best = C;
					BestCount = Counts[C];
				}
			}
			return EStrokeColor(Best);
		};

		auto SampleSourceColorClass = [&](const FIntPoint& P) -> EStrokeColor
		{
			return bHasSourceColorMap
				? SampleMapColorClass(SourceColorMap, P)
				: EStrokeColor::None;
		};

		struct FConn
		{
			FString Source = TEXT("all_skeleton");
			EStrokeColor Color = EStrokeColor::Black;
			FIntPoint P0;
			FIntPoint P1;
			TArray<FIntPoint> LinePts;
			TArray<FIntPoint> AddedPts;
			double GapLength = 0.0;
			double SourceAngleDegrees = -1.0;
			double TargetAngleDegrees = -1.0;
			FString TargetType = TEXT("endpoint");
			bool bRbAltPath = false;
			int32 RbAltPathPixels = 0;
			int64 RbBboxArea = -1;
			bool bProtectedKeep = false;
			bool bFullSkippedByProtection = false;
			bool bFullAltPath = false;
			int32 FullAltPathPixels = 0;
			int64 FullBboxArea = -1;
			int32 FullAddedPixels = 0;
			bool bKept = true;
			FString Decision = TEXT("not_processed");
		};
		TArray<FConn> Connections;

		auto AngleDegreesFromDot = [](double Dot) -> double
		{
			return FMath::RadiansToDegrees(FMath::Acos(FMath::Clamp(Dot, -1.0, 1.0)));
		};
		auto ConnectorKey = [](const FIntPoint& A, const FIntPoint& B) -> FString
		{
			return (A.X < B.X || (A.X == B.X && A.Y <= B.Y))
				? FString::Printf(TEXT("%d,%d-%d,%d"), A.X, A.Y, B.X, B.Y)
				: FString::Printf(TEXT("%d,%d-%d,%d"), B.X, B.Y, A.X, A.Y);
		};
		auto ColorBit = [](EStrokeColor Color) -> uint8
		{
			return Color == EStrokeColor::None ? 0 : uint8(1u << uint8(Color));
		};
		auto PreferredColorFromMask = [](uint8 Mask) -> EStrokeColor
		{
			const EStrokeColor Priority[] =
			{
				EStrokeColor::Red,
				EStrokeColor::Black,
				EStrokeColor::Green,
				EStrokeColor::Blue
			};
			for (EStrokeColor Color : Priority)
			{
				if ((Mask & uint8(1u << uint8(Color))) != 0)
				{
					return Color;
				}
			}
			return EStrokeColor::None;
		};

		TArray<FIntPoint> EndpointsBefore;
		SkelGraph::FindEndpoints(Work.GetData(), Width, Height, EndpointsBefore);
		const int32 E = EndpointsBefore.Num();
		TArray<FVector2D> EndpointForward;
		TArray<TSet<int32>> EndpointIncidentPixels;
		TArray<EStrokeColor> EndpointColors;
		EndpointForward.Init(FVector2D::ZeroVector, E);
		EndpointIncidentPixels.SetNum(E);
		EndpointColors.Init(EStrokeColor::Black, E);
		for (int32 i = 0; i < E; ++i)
		{
			TArray<int32> IncidentPixels;
			EndpointForward[i] = SkelGraph::EndpointOutwardDirection(
				Work.GetData(),
				Width,
				Height,
				EndpointsBefore[i],
				EndpointDirectionTracePixels,
				&IncidentPixels);
			for (int32 Idx : IncidentPixels)
			{
				EndpointIncidentPixels[i].Add(Idx);
			}

			int32 ColorCounts[5] = { 0, 0, 0, 0, 0 };
			for (int32 Idx : IncidentPixels)
			{
				const EStrokeColor Color = SampleSourceColorClass(FIntPoint(Idx % Width, Idx / Width));
				if (Color != EStrokeColor::None)
				{
					++ColorCounts[uint8(Color)];
				}
			}
			int32 BestColor = int32(EStrokeColor::Black);
			int32 BestCount = 0;
			for (int32 Color = 1; Color <= 4; ++Color)
			{
				if (ColorCounts[Color] > BestCount)
				{
					BestCount = ColorCounts[Color];
					BestColor = Color;
				}
			}
			EndpointColors[i] = EStrokeColor(BestColor);
		}

		TArray<bool> Used;
		Used.Init(false, E);
		TArray<int32> NearestJ;
		TArray<double> NearestD2;
		NearestJ.Init(INDEX_NONE, E);
		NearestD2.Init(TNumericLimits<double>::Max(), E);
		if (GapTol > 0.0f)
		{
			for (int32 i = 0; i < E; ++i)
			{
				for (int32 j = 0; j < E; ++j)
				{
					if (i == j || EndpointForward[i].IsNearlyZero() || EndpointForward[j].IsNearlyZero())
					{
						continue;
					}
					const FVector2D Delta(
						double(EndpointsBefore[j].X - EndpointsBefore[i].X),
						double(EndpointsBefore[j].Y - EndpointsBefore[i].Y));
					const double D2 = Delta.SizeSquared();
					if (D2 <= 1e-8 || D2 > Tol2)
					{
						continue;
					}
					const FVector2D ToTarget = Delta.GetSafeNormal();
					if (FVector2D::DotProduct(EndpointForward[i], ToTarget) < EndpointForwardCosThreshold ||
						FVector2D::DotProduct(EndpointForward[j], -ToTarget) < EndpointForwardCosThreshold)
					{
						continue;
					}
					if (D2 < NearestD2[i])
					{
						NearestD2[i] = D2;
						NearestJ[i] = j;
					}
				}
			}

			for (int32 i = 0; i < E; ++i)
			{
				const int32 j = NearestJ[i];
				if (j < 0 || Used[i] || Used[j] || NearestJ[j] != i)
				{
					continue;
				}
				FConn Conn;
				Conn.Source = TEXT("strict_endpoint_to_endpoint");
				Conn.Color = EndpointColors[i];
				Conn.P0 = EndpointsBefore[i];
				Conn.P1 = EndpointsBefore[j];
				const FVector2D ToTarget = (
					FVector2D(Conn.P1.X, Conn.P1.Y) -
					FVector2D(Conn.P0.X, Conn.P0.Y)).GetSafeNormal();
				Conn.SourceAngleDegrees = AngleDegreesFromDot(
					FVector2D::DotProduct(EndpointForward[i], ToTarget));
				Conn.TargetAngleDegrees = AngleDegreesFromDot(
					FVector2D::DotProduct(EndpointForward[j], -ToTarget));
				Connections.Add(MoveTemp(Conn));
				Used[i] = true;
				Used[j] = true;
			}

			for (int32 i = 0; i < E; ++i)
			{
				if (Used[i] || EndpointForward[i].IsNearlyZero())
				{
					continue;
				}

				const FIntPoint Endpoint = EndpointsBefore[i];
				const FVector2D EndpointPos(Endpoint.X, Endpoint.Y);
				double BestD2 = TNumericLimits<double>::Max();
				FVector2D BestPoint = FVector2D::ZeroVector;
				const int32 SearchRadius = FMath::CeilToInt(GapTol);
				const int32 MinX = FMath::Max(0, Endpoint.X - SearchRadius);
				const int32 MaxX = FMath::Min(Width - 1, Endpoint.X + SearchRadius);
				const int32 MinY = FMath::Max(0, Endpoint.Y - SearchRadius);
				const int32 MaxY = FMath::Min(Height - 1, Endpoint.Y + SearchRadius);
				for (int32 y = MinY; y <= MaxY; ++y)
				{
					for (int32 x = MinX; x <= MaxX; ++x)
					{
						const int32 AIdx = y * Width + x;
						if (Work[AIdx] == 0)
						{
							continue;
						}
						int32 SegmentNeighbors[8];
						const int32 SegmentDegree = SkelGraph::SafeNeighbors(
							Work.GetData(), Width, Height, x, y, SegmentNeighbors);
						for (int32 n = 0; n < SegmentDegree; ++n)
						{
							const int32 BIdx = SegmentNeighbors[n];
							if (BIdx <= AIdx ||
								EndpointIncidentPixels[i].Contains(AIdx) ||
								EndpointIncidentPixels[i].Contains(BIdx))
							{
								continue;
							}
							const FVector2D SegmentStart(x, y);
							const FVector2D SegmentEnd(BIdx % Width, BIdx / Width);
							const FVector2D Segment = SegmentEnd - SegmentStart;
							const double SegmentLen2 = Segment.SizeSquared();
							if (SegmentLen2 <= 1e-8)
							{
								continue;
							}
							const double T = FMath::Clamp(
								FVector2D::DotProduct(EndpointPos - SegmentStart, Segment) / SegmentLen2,
								0.0,
								1.0);
							const FVector2D Candidate = SegmentStart + Segment * T;
							const FVector2D Delta = Candidate - EndpointPos;
							const double D2 = Delta.SizeSquared();
							if (D2 <= 1.0 || D2 > Tol2 || D2 >= BestD2)
							{
								continue;
							}
							const FVector2D Direction = Delta.GetSafeNormal();
							if (Direction.IsNearlyZero() ||
								FVector2D::DotProduct(EndpointForward[i], Direction) < EndpointForwardCosThreshold)
							{
								continue;
							}
							BestD2 = D2;
							BestPoint = Candidate;
						}
					}
				}

				if (BestD2 < TNumericLimits<double>::Max())
				{
					FConn Conn;
					Conn.Source = TEXT("strict_endpoint_to_segment");
					Conn.Color = EndpointColors[i];
					Conn.P0 = Endpoint;
					Conn.P1 = FIntPoint(FMath::RoundToInt(BestPoint.X), FMath::RoundToInt(BestPoint.Y));
					Conn.GapLength = FMath::Sqrt(BestD2);
					Conn.SourceAngleDegrees = AngleDegreesFromDot(
						FVector2D::DotProduct(
							EndpointForward[i],
							(BestPoint - EndpointPos).GetSafeNormal()));
					Conn.TargetType = TEXT("segment");
					Connections.Add(MoveTemp(Conn));
					Used[i] = true;
				}
			}

			for (int32 i = 0; i < E; ++i)
			{
				if (Used[i] || EndpointForward[i].IsNearlyZero())
				{
					continue;
				}
				int32 BestJ = INDEX_NONE;
				double BestD2 = TNumericLimits<double>::Max();
				double BestSourceAngle = -1.0;
				double BestTargetAngle = -1.0;
				for (int32 j = 0; j < E; ++j)
				{
					if (i == j || Used[j] || EndpointForward[j].IsNearlyZero())
					{
						continue;
					}
					const FVector2D Delta(
						double(EndpointsBefore[j].X - EndpointsBefore[i].X),
						double(EndpointsBefore[j].Y - EndpointsBefore[i].Y));
					const double D2 = Delta.SizeSquared();
					if (D2 <= 1e-8 || D2 > Tol2 || D2 >= BestD2)
					{
						continue;
					}
					const FVector2D ToTarget = Delta.GetSafeNormal();
					const double SourceDot = FVector2D::DotProduct(EndpointForward[i], ToTarget);
					const double TargetDot = FVector2D::DotProduct(EndpointForward[j], -ToTarget);
					if (SourceDot < EndpointForwardCosThreshold ||
						TargetDot < RelaxedTargetCosThreshold)
					{
						continue;
					}
					BestJ = j;
					BestD2 = D2;
					BestSourceAngle = AngleDegreesFromDot(SourceDot);
					BestTargetAngle = AngleDegreesFromDot(TargetDot);
				}
				if (BestJ >= 0)
				{
					FConn Conn;
					Conn.Source = TEXT("relaxed_endpoint_to_endpoint");
					Conn.Color = EndpointColors[i];
					Conn.P0 = EndpointsBefore[i];
					Conn.P1 = EndpointsBefore[BestJ];
					Conn.GapLength = FMath::Sqrt(BestD2);
					Conn.SourceAngleDegrees = BestSourceAngle;
					Conn.TargetAngleDegrees = BestTargetAngle;
					Connections.Add(MoveTemp(Conn));
					Used[i] = true;
					Used[BestJ] = true;
				}
			}
		}

		TArray<uint8> Connected = Work;
		TArray<uint8> ConnectorColorMask;
		ConnectorColorMask.Init(0, N);
		TArray<int32> ConnectorOwnerCount;
		ConnectorOwnerCount.Init(0, N);
		TArray<int32> RedBlackOwnerCount;
		RedBlackOwnerCount.Init(0, N);
		TSet<FString> ConnectorKeys;
		auto RasterizeConnector = [&](FConn& C)
		{
			if (C.GapLength <= 0.0)
			{
				C.GapLength = FVector2D::Distance(
					FVector2D(C.P0.X, C.P0.Y),
					FVector2D(C.P1.X, C.P1.Y));
			}
			SkelGraph::LinePoints(C.P0, C.P1, C.LinePts);
			ConnectorKeys.Add(ConnectorKey(C.P0, C.P1));
			for (const FIntPoint& P : C.LinePts)
			{
				if (P.X >= 0 && P.X < Width && P.Y >= 0 && P.Y < Height)
				{
					const int32 Idx = P.Y * Width + P.X;
					if (Work[Idx] == 0)
					{
						C.AddedPts.Add(P);
						ConnectorColorMask[Idx] |= ColorBit(C.Color);
						++ConnectorOwnerCount[Idx];
						if (C.Color == EStrokeColor::Red || C.Color == EStrokeColor::Black)
						{
							++RedBlackOwnerCount[Idx];
						}
					}
					Connected[Idx] = 255;
				}
			}
		};
		for (FConn& C : Connections)
		{
			RasterizeConnector(C);
		}
		OutConnected = Connected;

		auto EffectiveColorAt = [&](const FIntPoint& P) -> EStrokeColor
		{
			const int32 Idx = P.Y * Width + P.X;
			if (Work[Idx] == 0 && ConnectorColorMask[Idx] != 0)
			{
				return PreferredColorFromMask(ConnectorColorMask[Idx]);
			}
			return SampleSourceColorClass(P);
		};

		TArray<uint8> RedBlackCandidate;
		RedBlackCandidate.Init(0, N);
		TArray<uint8> RedBlackDebugKind;
		RedBlackDebugKind.Init(0, N);
		auto RebuildRedBlackCandidate = [&]()
		{
			for (int32 Idx = 0; Idx < N; ++Idx)
			{
				RedBlackCandidate[Idx] = 0;
				RedBlackDebugKind[Idx] = 0;
				if (Connected[Idx] == 0)
				{
					continue;
				}
				const FIntPoint P(Idx % Width, Idx / Width);
				const EStrokeColor Color = EffectiveColorAt(P);
				if (Color == EStrokeColor::Red || Color == EStrokeColor::Black)
				{
					RedBlackCandidate[Idx] = 255;
					RedBlackDebugKind[Idx] = Color == EStrokeColor::Black ? 1 : 2;
				}
			}
		};
		RebuildRedBlackCandidate();

		auto SaveRedBlackDebug = [&](const FString& Path)
		{
			if (Path.IsEmpty())
			{
				return;
			}
			TArray<uint8> RGBA;
			RGBA.Init(255, N * 4);
			for (int32 i = 0; i < N; ++i)
			{
				const int32 Off = i * 4;
				if (RedBlackDebugKind[i] == 1)
				{
					RGBA[Off + 0] = 0;
					RGBA[Off + 1] = 0;
					RGBA[Off + 2] = 0;
				}
				else if (RedBlackDebugKind[i] == 2)
				{
					RGBA[Off + 0] = 220;
					RGBA[Off + 1] = 20;
					RGBA[Off + 2] = 60;
				}
				RGBA[Off + 3] = 255;
			}
			IImageWrapperModule& IWM = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
			TSharedPtr<IImageWrapper> IW = IWM.CreateImageWrapper(EImageFormat::PNG);
			if (IW.IsValid())
			{
				IW->SetRaw(RGBA.GetData(), RGBA.Num(), Width, Height, ERGBFormat::RGBA, 8);
				const TArray64<uint8>& Compressed = IW->GetCompressed();
				FFileHelper::SaveArrayToFile(
					TArrayView<const uint8>(Compressed.GetData(), static_cast<int32>(Compressed.Num())),
					*Path);
			}
		};
		SaveRedBlackDebug(RedBlackConnectorsDebugPngPath);
		const TArray<uint8> RedBlackSearchSnapshot = RedBlackCandidate;

		struct FRbEndpoint
		{
			FIntPoint Pos;
			FVector2D Forward = FVector2D::ZeroVector;
			EStrokeColor Color = EStrokeColor::None;
			TSet<int32> IncidentPixels;
		};
		TArray<FIntPoint> RbEndpointPoints;
		SkelGraph::FindEndpoints(RedBlackCandidate.GetData(), Width, Height, RbEndpointPoints);
		TArray<FRbEndpoint> RbEndpoints;
		RbEndpoints.SetNum(RbEndpointPoints.Num());
		for (int32 i = 0; i < RbEndpointPoints.Num(); ++i)
		{
			FRbEndpoint& Endpoint = RbEndpoints[i];
			Endpoint.Pos = RbEndpointPoints[i];
			TArray<int32> IncidentPixels;
			Endpoint.Forward = SkelGraph::EndpointOutwardDirection(
				RedBlackSearchSnapshot.GetData(),
				Width,
				Height,
				Endpoint.Pos,
				EndpointDirectionTracePixels,
				&IncidentPixels);
			int32 RedCount = 0;
			int32 BlackCount = 0;
			for (int32 Idx : IncidentPixels)
			{
				Endpoint.IncidentPixels.Add(Idx);
				const EStrokeColor Color = EffectiveColorAt(FIntPoint(Idx % Width, Idx / Width));
				RedCount += Color == EStrokeColor::Red ? 1 : 0;
				BlackCount += Color == EStrokeColor::Black ? 1 : 0;
			}
			Endpoint.Color = RedCount > BlackCount ? EStrokeColor::Red : EStrokeColor::Black;
		}

		struct FRbProposal
		{
			int32 SourceEndpoint = INDEX_NONE;
			int32 TargetEndpoint = INDEX_NONE;
			FIntPoint TargetPoint;
			double D2 = TNumericLimits<double>::Max();
			double SourceAngle = -1.0;
			double TargetAngle = -1.0;
			FString Source;
			FString TargetType = TEXT("endpoint");
		};
		auto ProposalLess = [](const FRbProposal& A, const FRbProposal& B)
		{
			if (!FMath::IsNearlyEqual(A.D2, B.D2, 1e-8))
			{
				return A.D2 < B.D2;
			}
			if (A.SourceEndpoint != B.SourceEndpoint)
			{
				return A.SourceEndpoint < B.SourceEndpoint;
			}
			if (A.TargetEndpoint != B.TargetEndpoint)
			{
				return A.TargetEndpoint < B.TargetEndpoint;
			}
			if (A.TargetPoint.Y != B.TargetPoint.Y)
			{
				return A.TargetPoint.Y < B.TargetPoint.Y;
			}
			return A.TargetPoint.X < B.TargetPoint.X;
		};

		TArray<bool> RbUsed;
		RbUsed.Init(false, RbEndpoints.Num());
		auto AppendRbProposal = [&](const FRbProposal& Proposal)
		{
			if (!RbEndpoints.IsValidIndex(Proposal.SourceEndpoint))
			{
				return false;
			}
			const FIntPoint P0 = RbEndpoints[Proposal.SourceEndpoint].Pos;
			const FIntPoint P1 = Proposal.TargetPoint;
			const FString Key = ConnectorKey(P0, P1);
			if (P0 == P1 || ConnectorKeys.Contains(Key))
			{
				return false;
			}
			FConn Conn;
			Conn.Source = Proposal.Source;
			Conn.Color = EStrokeColor::Red;
			Conn.P0 = P0;
			Conn.P1 = P1;
			Conn.GapLength = FMath::Sqrt(Proposal.D2);
			Conn.SourceAngleDegrees = Proposal.SourceAngle;
			Conn.TargetAngleDegrees = Proposal.TargetAngle;
			Conn.TargetType = Proposal.TargetType;
			RasterizeConnector(Conn);
			for (const FIntPoint& P : Conn.LinePts)
			{
				if (P.X >= 0 && P.X < Width && P.Y >= 0 && P.Y < Height)
				{
					const int32 Idx = P.Y * Width + P.X;
					RedBlackCandidate[Idx] = 255;
					RedBlackDebugKind[Idx] = 2;
				}
			}
			Connections.Add(MoveTemp(Conn));
			return true;
		};

		if (GapTol > 0.0f)
		{
			TArray<FRbProposal> StrictEndpointProposals;
			TSet<FString> StrictEndpointPairKeys;
			for (int32 i = 0; i < RbEndpoints.Num(); ++i)
			{
				if (RbEndpoints[i].Color != EStrokeColor::Red ||
					RbEndpoints[i].Forward.IsNearlyZero())
				{
					continue;
				}
				for (int32 j = 0; j < RbEndpoints.Num(); ++j)
				{
					if (i == j || RbEndpoints[j].Forward.IsNearlyZero())
					{
						continue;
					}
					const FVector2D Delta(
						double(RbEndpoints[j].Pos.X - RbEndpoints[i].Pos.X),
						double(RbEndpoints[j].Pos.Y - RbEndpoints[i].Pos.Y));
					const double D2 = Delta.SizeSquared();
					if (D2 <= 1e-8 || D2 > Tol2)
					{
						continue;
					}
					const FVector2D Direction = Delta.GetSafeNormal();
					const double SourceDot = FVector2D::DotProduct(RbEndpoints[i].Forward, Direction);
					const double TargetDot = FVector2D::DotProduct(RbEndpoints[j].Forward, -Direction);
					if (SourceDot < EndpointForwardCosThreshold ||
						TargetDot < EndpointForwardCosThreshold)
					{
						continue;
					}
					const FString PairKey = ConnectorKey(RbEndpoints[i].Pos, RbEndpoints[j].Pos);
					if (StrictEndpointPairKeys.Contains(PairKey))
					{
						continue;
					}
					StrictEndpointPairKeys.Add(PairKey);
					FRbProposal Proposal;
					Proposal.SourceEndpoint = i;
					Proposal.TargetEndpoint = j;
					Proposal.TargetPoint = RbEndpoints[j].Pos;
					Proposal.D2 = D2;
					Proposal.SourceAngle = AngleDegreesFromDot(SourceDot);
					Proposal.TargetAngle = AngleDegreesFromDot(TargetDot);
					Proposal.Source = TEXT("red_reconnect_strict_endpoint_to_endpoint");
					StrictEndpointProposals.Add(MoveTemp(Proposal));
				}
			}
			StrictEndpointProposals.Sort(ProposalLess);
			for (const FRbProposal& Proposal : StrictEndpointProposals)
			{
				if (RbUsed[Proposal.SourceEndpoint] || RbUsed[Proposal.TargetEndpoint])
				{
					continue;
				}
				if (AppendRbProposal(Proposal))
				{
					RbUsed[Proposal.SourceEndpoint] = true;
					RbUsed[Proposal.TargetEndpoint] = true;
				}
			}

			for (int32 i = 0; i < RbEndpoints.Num(); ++i)
			{
				if (RbUsed[i] ||
					RbEndpoints[i].Color != EStrokeColor::Red ||
					RbEndpoints[i].Forward.IsNearlyZero())
				{
					continue;
				}
				const FVector2D SourcePos(RbEndpoints[i].Pos.X, RbEndpoints[i].Pos.Y);
				FRbProposal Best;
				Best.SourceEndpoint = i;
				Best.Source = TEXT("red_reconnect_strict_endpoint_to_segment");
				Best.TargetType = TEXT("segment");

				for (int32 y = 0; y < Height; ++y)
				{
					for (int32 x = 0; x < Width; ++x)
					{
						const int32 AIdx = y * Width + x;
						if (RedBlackSearchSnapshot[AIdx] == 0)
						{
							continue;
						}
						int32 Neighbors[8];
						const int32 Degree = SkelGraph::SafeNeighbors(
							RedBlackSearchSnapshot.GetData(), Width, Height, x, y, Neighbors);
						if (SkelGraph::NodeType(RedBlackSearchSnapshot.GetData(), Width, Height, x, y) == SkelGraph::ENode::Branch)
						{
							const FVector2D Delta = FVector2D(x, y) - SourcePos;
							const double D2 = Delta.SizeSquared();
							if (D2 > 1.0 && D2 <= Tol2 && D2 < Best.D2)
							{
								const FVector2D Direction = Delta.GetSafeNormal();
								const double SourceDot = FVector2D::DotProduct(RbEndpoints[i].Forward, Direction);
								if (SourceDot >= EndpointForwardCosThreshold)
								{
									Best.TargetPoint = FIntPoint(x, y);
									Best.D2 = D2;
									Best.SourceAngle = AngleDegreesFromDot(SourceDot);
									Best.TargetType = TEXT("topology_node");
								}
							}
						}
						for (int32 n = 0; n < Degree; ++n)
						{
							const int32 BIdx = Neighbors[n];
							if (BIdx <= AIdx ||
								RbEndpoints[i].IncidentPixels.Contains(AIdx) ||
								RbEndpoints[i].IncidentPixels.Contains(BIdx))
							{
								continue;
							}
							const FVector2D A(x, y);
							const FVector2D B(BIdx % Width, BIdx / Width);
							const FVector2D Segment = B - A;
							const double SegmentLen2 = Segment.SizeSquared();
							if (SegmentLen2 <= 1e-8)
							{
								continue;
							}
							const double T = FMath::Clamp(
								FVector2D::DotProduct(SourcePos - A, Segment) / SegmentLen2,
								0.0,
								1.0);
							if (T <= 1e-6 || T >= 1.0 - 1e-6)
							{
								continue;
							}
							const FVector2D Candidate = A + Segment * T;
							const FVector2D Delta = Candidate - SourcePos;
							const double D2 = Delta.SizeSquared();
							if (D2 <= 1.0 || D2 > Tol2 || D2 >= Best.D2)
							{
								continue;
							}
							const FVector2D Direction = Delta.GetSafeNormal();
							const double SourceDot = FVector2D::DotProduct(RbEndpoints[i].Forward, Direction);
							if (SourceDot < EndpointForwardCosThreshold)
							{
								continue;
							}
							Best.TargetPoint = FIntPoint(
								FMath::RoundToInt(Candidate.X),
								FMath::RoundToInt(Candidate.Y));
							Best.D2 = D2;
							Best.SourceAngle = AngleDegreesFromDot(SourceDot);
							Best.TargetType = TEXT("segment");
						}
					}
				}
				if (Best.D2 < TNumericLimits<double>::Max() && AppendRbProposal(Best))
				{
					RbUsed[i] = true;
				}
			}

			TArray<FRbProposal> RelaxedEndpointProposals;
			TSet<FString> RelaxedEndpointPairKeys;
			for (int32 i = 0; i < RbEndpoints.Num(); ++i)
			{
				if (RbUsed[i] ||
					RbEndpoints[i].Color != EStrokeColor::Red ||
					RbEndpoints[i].Forward.IsNearlyZero())
				{
					continue;
				}
				for (int32 j = 0; j < RbEndpoints.Num(); ++j)
				{
					if (i == j || RbUsed[j] || RbEndpoints[j].Forward.IsNearlyZero())
					{
						continue;
					}
					const FVector2D Delta(
						double(RbEndpoints[j].Pos.X - RbEndpoints[i].Pos.X),
						double(RbEndpoints[j].Pos.Y - RbEndpoints[i].Pos.Y));
					const double D2 = Delta.SizeSquared();
					if (D2 <= 1e-8 || D2 > Tol2)
					{
						continue;
					}
					const FVector2D Direction = Delta.GetSafeNormal();
					const double SourceDot = FVector2D::DotProduct(RbEndpoints[i].Forward, Direction);
					const double TargetDot = FVector2D::DotProduct(RbEndpoints[j].Forward, -Direction);
					if (SourceDot < EndpointForwardCosThreshold ||
						TargetDot < RelaxedTargetCosThreshold)
					{
						continue;
					}
					const FString PairKey = ConnectorKey(RbEndpoints[i].Pos, RbEndpoints[j].Pos);
					if (RelaxedEndpointPairKeys.Contains(PairKey))
					{
						continue;
					}
					RelaxedEndpointPairKeys.Add(PairKey);
					FRbProposal Proposal;
					Proposal.SourceEndpoint = i;
					Proposal.TargetEndpoint = j;
					Proposal.TargetPoint = RbEndpoints[j].Pos;
					Proposal.D2 = D2;
					Proposal.SourceAngle = AngleDegreesFromDot(SourceDot);
					Proposal.TargetAngle = AngleDegreesFromDot(TargetDot);
					Proposal.Source = TEXT("red_reconnect_relaxed_endpoint_to_endpoint");
					RelaxedEndpointProposals.Add(MoveTemp(Proposal));
				}
			}
			RelaxedEndpointProposals.Sort(ProposalLess);
			for (const FRbProposal& Proposal : RelaxedEndpointProposals)
			{
				if (RbUsed[Proposal.SourceEndpoint] || RbUsed[Proposal.TargetEndpoint])
				{
					continue;
				}
				if (AppendRbProposal(Proposal))
				{
					RbUsed[Proposal.SourceEndpoint] = true;
					RbUsed[Proposal.TargetEndpoint] = true;
				}
			}
		}

		OutReconnected = Connected;
		SaveRedBlackDebug(RedBlackReconnectedDebugPngPath);
		const float LoopThresh = FMath::Max(0.0f, SmallLoopBboxAreaThresh);
		auto LoopBboxArea = [](const TArray<FIntPoint>& Path, const TArray<FIntPoint>& AddedPts) -> int64
		{
			if (Path.Num() == 0 && AddedPts.Num() == 0)
			{
				return -1;
			}
			int32 MinX = MAX_int32;
			int32 MinY = MAX_int32;
			int32 MaxX = MIN_int32;
			int32 MaxY = MIN_int32;
			auto Accumulate = [&](const FIntPoint& P)
			{
				MinX = FMath::Min(MinX, P.X);
				MinY = FMath::Min(MinY, P.Y);
				MaxX = FMath::Max(MaxX, P.X);
				MaxY = FMath::Max(MaxY, P.Y);
			};
			for (const FIntPoint& P : Path) { Accumulate(P); }
			for (const FIntPoint& P : AddedPts) { Accumulate(P); }
			return int64(MaxX - MinX + 1) * int64(MaxY - MinY + 1);
		};

		TArray<uint8> ProtectedConnectorPixels;
		ProtectedConnectorPixels.Init(0, N);
		for (FConn& C : Connections)
		{
			if (C.Color == EStrokeColor::Green)
			{
				C.bProtectedKeep = true;
				C.Decision = TEXT("keep_green_connector");
				for (const FIntPoint& P : C.AddedPts)
				{
					ProtectedConnectorPixels[P.Y * Width + P.X] = 255;
				}
				continue;
			}
			if (LoopThresh <= 0.0f ||
				(C.Color != EStrokeColor::Red && C.Color != EStrokeColor::Black) ||
				C.AddedPts.Num() == 0)
			{
				continue;
			}

			TArray<uint8> Probe = RedBlackCandidate;
			for (const FIntPoint& P : C.AddedPts)
			{
				const int32 Idx = P.Y * Width + P.X;
				if (RedBlackOwnerCount[Idx] <= 1)
				{
					Probe[Idx] = 0;
				}
			}

			TArray<FIntPoint> RbPath;
			C.bRbAltPath = SkelGraph::ShortestPath(
				Probe.GetData(), Width, Height, C.P0, C.P1, RbPath);
			if (C.bRbAltPath)
			{
				C.RbAltPathPixels = RbPath.Num();
				C.RbBboxArea = LoopBboxArea(RbPath, C.AddedPts);
				if (double(C.RbBboxArea) >= double(LoopThresh))
				{
					C.bProtectedKeep = true;
					for (const FIntPoint& P : C.AddedPts)
					{
						ProtectedConnectorPixels[P.Y * Width + P.X] = 255;
					}
				}
			}
		}

		TArray<uint8> Current = Connected;
		TArray<int32> ActiveOwnerCount = ConnectorOwnerCount;
		if (LoopThresh > 0.0f)
		{
			for (FConn& C : Connections)
			{
				if (C.bProtectedKeep)
				{
					C.bFullSkippedByProtection = true;
					C.bKept = true;
					if (C.Color != EStrokeColor::Green)
					{
						C.Decision = TEXT("keep_red_black_large_loop_protected");
					}
					continue;
				}

				TArray<FIntPoint> RemovedPixels;
				for (const FIntPoint& P : C.AddedPts)
				{
					const int32 Idx = P.Y * Width + P.X;
					ActiveOwnerCount[Idx] = FMath::Max(0, ActiveOwnerCount[Idx] - 1);
					if (ActiveOwnerCount[Idx] == 0 &&
						ProtectedConnectorPixels[Idx] == 0 &&
						Current[Idx] > 0)
					{
						Current[Idx] = 0;
						RemovedPixels.Add(P);
					}
				}
				C.FullAddedPixels = RemovedPixels.Num();
				if (RemovedPixels.Num() == 0)
				{
					C.bKept = true;
					C.Decision = C.AddedPts.Num() == 0 ? TEXT("skip_no_added_pixels") : TEXT("keep_no_unprotected_added_pixels");
					for (const FIntPoint& P : C.AddedPts)
					{
						++ActiveOwnerCount[P.Y * Width + P.X];
					}
					continue;
				}

				TArray<FIntPoint> Path;
				C.bFullAltPath = SkelGraph::ShortestPath(Current.GetData(), Width, Height, C.P0, C.P1, Path);
				if (!C.bFullAltPath)
				{
					for (const FIntPoint& P : C.AddedPts)
					{
						const int32 Idx = P.Y * Width + P.X;
						++ActiveOwnerCount[Idx];
						Current[Idx] = 255;
					}
					C.bKept = true;
					C.Decision = TEXT("keep_no_full_alt_path");
					continue;
				}

				C.FullAltPathPixels = Path.Num();
				C.FullBboxArea = LoopBboxArea(Path, C.AddedPts);

				if (double(C.FullBboxArea) < double(LoopThresh))
				{
					C.bKept = false;
					C.Decision = TEXT("delete_full_small_loop");
				}
				else
				{
					for (const FIntPoint& P : C.AddedPts)
					{
						const int32 Idx = P.Y * Width + P.X;
						++ActiveOwnerCount[Idx];
						Current[Idx] = 255;
					}
					C.bKept = true;
					C.Decision = TEXT("keep_full_large_loop");
				}
			}
		}
		OutSmallLoopPruned = Current;

		auto BuildEffectiveColorMap = [&](const TArray<uint8>& Skeleton, TArray<uint8>& OutMap)
		{
			if (bHasSourceColorMap)
			{
				OutMap = SourceColorMap;
			}
			else
			{
				OutMap.Init(uint8(EStrokeColor::None), N);
			}
			for (const FConn& C : Connections)
			{
				if (!C.bKept)
				{
					continue;
				}
				for (const FIntPoint& P : C.AddedPts)
				{
					const int32 Idx = P.Y * Width + P.X;
					if (Skeleton[Idx] > 0)
					{
						OutMap[Idx] = uint8(C.Color);
					}
				}
			}
		};

		float MaxPixels = -1.0f;
		if (BranchPruneMaxPixels > 0.0f)
		{
			MaxPixels = BranchPruneMaxPixels;
		}
		else if (LoopThresh > 0.0f)
		{
			MaxPixels = FMath::Max(30.0f, 3.0f * GapTol);
		}

		TArray<uint8> EffectiveBeforeBranchPrune;
		BuildEffectiveColorMap(Current, EffectiveBeforeBranchPrune);
		TArray<FIntPoint> PrunePoints;
		SkelGraph::FindEndpoints(Current.GetData(), Width, Height, PrunePoints);
		TSet<int32> StopNodes;
		SkelGraph::BranchStopPoints(Current.GetData(), Width, Height, StopNodes);

		auto SamplesAsProtectedSourceColor = [&](const FIntPoint& P) -> bool
		{
			int32 ProtectedSamples = 0;
			int32 ColoredSamples = 0;
			const int32 MinY = FMath::Max(0, P.Y - SampleRadius);
			const int32 MaxY = FMath::Min(Height - 1, P.Y + SampleRadius);
			const int32 MinX = FMath::Max(0, P.X - SampleRadius);
			const int32 MaxX = FMath::Min(Width - 1, P.X + SampleRadius);
			for (int32 yy = MinY; yy <= MaxY; ++yy)
			{
				for (int32 xx = MinX; xx <= MaxX; ++xx)
				{
					const uint8 C = EffectiveBeforeBranchPrune[yy * Width + xx];
					if (C == uint8(EStrokeColor::None))
					{
						continue;
					}
					++ColoredSamples;
					if (C == uint8(EStrokeColor::Red) || C == uint8(EStrokeColor::Black))
					{
						++ProtectedSamples;
					}
				}
			}

			return ProtectedSamples > 0 && ProtectedSamples * 2 >= ColoredSamples;
		};

		auto BranchHasProtectedSourceColor = [&](const TArray<FIntPoint>& BranchPath) -> bool
		{
			// The prune operation removes all but the terminal node, so classify only
			// pixels that would actually be deleted.
			const int32 DeleteCount = FMath::Max(0, BranchPath.Num() - 1);
			if (DeleteCount == 0)
			{
				return false;
			}

			int32 ProtectedPathSamples = 0;
			for (int32 k = 0; k < DeleteCount; ++k)
			{
				if (SamplesAsProtectedSourceColor(BranchPath[k]))
				{
					++ProtectedPathSamples;
				}
			}

			const int32 Needed = FMath::Max(1, (DeleteCount + 1) / 2);
			return ProtectedPathSamples >= Needed;
		};

		auto PathArcLength = [](const TArray<FIntPoint>& BranchPath) -> double
		{
			double Arc = 0.0;
			for (int32 i = 1; i < BranchPath.Num(); ++i)
			{
				Arc += FVector2D::Distance(
					FVector2D(BranchPath[i - 1].X, BranchPath[i - 1].Y),
					FVector2D(BranchPath[i].X, BranchPath[i].Y));
			}
			return Arc;
		};

		TArray<uint8> Cleaned = Current;
		TArray<FIntPoint> Path;
		for (const FIntPoint& P : PrunePoints)
		{
			if (P.X < 0 || P.X >= Width || P.Y < 0 || P.Y >= Height)
			{
				continue;
			}
			const int32 Idx = P.Y * Width + P.X;
			if (Cleaned[Idx] == 0)
			{
				continue;
			}
			if (SkelGraph::NodeType(Cleaned.GetData(), Width, Height, P.X, P.Y) != SkelGraph::ENode::Endpoint)
			{
				continue;
			}
			SkelGraph::TraceBranchPath(Cleaned.GetData(), Width, Height, P, StopNodes, Path);
			if (Path.Num() == 0)
			{
				continue;
			}
			if (MaxPixels > 0.0f && PathArcLength(Path) > MaxPixels)
			{
				continue;
			}
			if (BranchHasProtectedSourceColor(Path))
			{
				continue;
			}
			for (int32 k = 0; k < Path.Num() - 1; ++k)
			{
				Cleaned[Path[k].Y * Width + Path[k].X] = 0;
			}
			const FIntPoint End = Path.Last();
			const int32 EndIdx = End.Y * Width + End.X;
			if (!StopNodes.Contains(EndIdx) &&
				SkelGraph::NodeType(Cleaned.GetData(), Width, Height, End.X, End.Y) == SkelGraph::ENode::Endpoint)
			{
				Cleaned[EndIdx] = 0;
			}
		}

		OutCleaned = MoveTemp(Cleaned);
		BuildEffectiveColorMap(OutCleaned, OutEffectiveColorMap);

		if (!ConnectorPruneDebugJsonPath.IsEmpty())
		{
			FString Json;
			Json += TEXT("{\n");
			Json += FString::Printf(TEXT("  \"gap_tol\": %.3f,\n"), GapTol);
			Json += FString::Printf(TEXT("  \"endpoint_direction_trace_pixels\": %.3f,\n"), EndpointDirectionTracePixels);
			Json += FString::Printf(TEXT("  \"small_loop_bbox_area_thresh\": %.3f,\n"), SmallLoopBboxAreaThresh);
			Json += FString::Printf(TEXT("  \"red_black_source_sample_radius\": %d,\n"), SampleRadius);
			Json += FString::Printf(TEXT("  \"connector_count\": %d,\n"), Connections.Num());
			Json += TEXT("  \"connectors\": [\n");
			for (int32 i = 0; i < Connections.Num(); ++i)
			{
				const FConn& C = Connections[i];
				Json += TEXT("    {\n");
				Json += FString::Printf(TEXT("      \"id\": %d,\n"), i + 1);
				Json += FString::Printf(TEXT("      \"source\": \"%s\",\n"), *C.Source);
				Json += FString::Printf(TEXT("      \"color\": \"%s\",\n"), StrokeColorToString(C.Color));
				Json += FString::Printf(TEXT("      \"p0\": [%d, %d],\n"), C.P0.X, C.P0.Y);
				Json += FString::Printf(TEXT("      \"p1\": [%d, %d],\n"), C.P1.X, C.P1.Y);
				Json += FString::Printf(TEXT("      \"gap_length\": %.3f,\n"), C.GapLength);
				Json += FString::Printf(TEXT("      \"target_type\": \"%s\",\n"), *C.TargetType);
				Json += FString::Printf(TEXT("      \"source_angle_degrees\": %.3f,\n"), C.SourceAngleDegrees);
				Json += FString::Printf(TEXT("      \"target_angle_degrees\": %.3f,\n"), C.TargetAngleDegrees);
				Json += FString::Printf(TEXT("      \"line_pixels\": %d,\n"), C.LinePts.Num());
				Json += FString::Printf(TEXT("      \"added_pixels\": %d,\n"), C.AddedPts.Num());
				Json += FString::Printf(TEXT("      \"rb_alt_path_found\": %s,\n"), C.bRbAltPath ? TEXT("true") : TEXT("false"));
				Json += FString::Printf(TEXT("      \"rb_alt_path_pixels\": %d,\n"), C.RbAltPathPixels);
				Json += FString::Printf(TEXT("      \"rb_bbox_area\": %lld,\n"), C.RbBboxArea);
				Json += FString::Printf(TEXT("      \"protected_keep\": %s,\n"), C.bProtectedKeep ? TEXT("true") : TEXT("false"));
				Json += FString::Printf(TEXT("      \"full_skipped_by_protection\": %s,\n"), C.bFullSkippedByProtection ? TEXT("true") : TEXT("false"));
				Json += FString::Printf(TEXT("      \"full_added_pixels_checked\": %d,\n"), C.FullAddedPixels);
				Json += FString::Printf(TEXT("      \"full_alt_path_found\": %s,\n"), C.bFullAltPath ? TEXT("true") : TEXT("false"));
				Json += FString::Printf(TEXT("      \"full_alt_path_pixels\": %d,\n"), C.FullAltPathPixels);
				Json += FString::Printf(TEXT("      \"full_bbox_area\": %lld,\n"), C.FullBboxArea);
				Json += FString::Printf(TEXT("      \"kept\": %s,\n"), C.bKept ? TEXT("true") : TEXT("false"));
				Json += FString::Printf(TEXT("      \"decision\": \"%s\"\n"), *C.Decision);
				Json += FString::Printf(TEXT("    }%s\n"), (i + 1 < Connections.Num()) ? TEXT(",") : TEXT(""));
			}
			Json += TEXT("  ]\n");
			Json += TEXT("}\n");
			FFileHelper::SaveStringToFile(Json, *ConnectorPruneDebugJsonPath);
		}
	}

	// ====================================================================
	// Step 4: stroke tracing (port of Python trace_strokes).
	// ====================================================================
	void TraceStrokes(const TArray<uint8>& Skel, int32 Width, int32 Height, int32 MinPixels, TArray<FStroke>& OutStrokes)
	{
		OutStrokes.Reset();
		const uint8* M = Skel.GetData();

		auto EdgeKey = [Width](int32 a, int32 b) -> uint64
		{
			const uint32 lo = uint32(FMath::Min(a, b));
			const uint32 hi = uint32(FMath::Max(a, b));
			return (uint64(lo) << 32) | uint64(hi);
		};

		// True endpoints / branches are the topological nodes we trace between.
		TSet<int32> Nodes;
		for (int32 y = 0; y < Height; ++y)
		{
			for (int32 x = 0; x < Width; ++x)
			{
				if (M[y * Width + x] == 0)
				{
					continue;
				}
				const SkelGraph::ENode T = SkelGraph::NodeType(M, Width, Height, x, y);
				if (T == SkelGraph::ENode::Endpoint || T == SkelGraph::ENode::Branch)
				{
					Nodes.Add(y * Width + x);
				}
			}
		}

		TSet<uint64> VisitedEdges;

		auto IdxToPoint = [Width](int32 Idx) -> FVector2D
		{
			return FVector2D(double(Idx % Width), double(Idx / Width));
		};

		// Walk one polyline starting along edge (Start -> First). bStopAtNodes mirrors
		// the first Python pass (break when reaching another node); the second pass
		// instead closes pure cycles back onto Start.
		auto WalkPath = [&](int32 Start, int32 First, bool bStopAtNodes, TArray<int32>& OutPath)
		{
			OutPath.Reset();
			OutPath.Add(Start);
			int32 Prev = Start;
			int32 Cur = First;
			int32 Safety = 0;
			for (;;)
			{
				if (++Safety > 20000)
				{
					break;
				}
				OutPath.Add(Cur);

				if (bStopAtNodes && Nodes.Contains(Cur) && Cur != Start)
				{
					break;
				}

				const int32 cx = Cur % Width;
				const int32 cy = Cur / Width;
				int32 Nb[8];
				const int32 Deg = SkelGraph::SafeNeighbors(M, Width, Height, cx, cy, Nb);

				int32 Cands[8];
				int32 NumCands = 0;
				for (int32 i = 0; i < Deg; ++i)
				{
					if (Nb[i] == Prev)
					{
						continue;
					}
					if (VisitedEdges.Contains(EdgeKey(Cur, Nb[i])))
					{
						continue;
					}
					Cands[NumCands++] = Nb[i];
				}
				if (NumCands == 0)
				{
					break;
				}

				const FVector2D PrevPt = IdxToPoint(Prev);
				const FVector2D CurPt = IdxToPoint(Cur);
				const int32 Next = SkelGraph::ChooseBestContinuation(
					FIntPoint(int32(PrevPt.X), int32(PrevPt.Y)),
					FIntPoint(int32(CurPt.X), int32(CurPt.Y)),
					Cands, NumCands, Width);
				if (Next < 0)
				{
					break;
				}

				if (!bStopAtNodes && Next == Start)
				{
					VisitedEdges.Add(EdgeKey(Cur, Next));
					OutPath.Add(Start);
					break;
				}

				VisitedEdges.Add(EdgeKey(Cur, Next));
				Prev = Cur;
				Cur = Next;
			}
		};

		auto EmitPath = [&](const TArray<int32>& Path)
		{
			if (Path.Num() < MinPixels)
			{
				return;
			}
			FStroke S;
			S.Reserve(Path.Num());
			for (int32 Idx : Path)
			{
				S.Add(IdxToPoint(Idx));
			}
			OutStrokes.Add(MoveTemp(S));
		};

		TArray<int32> Path;

		// Pass 1: trace from true nodes (raster order for determinism).
		TArray<int32> NodeList = Nodes.Array();
		NodeList.Sort();
		for (int32 Start : NodeList)
		{
			const int32 sx = Start % Width;
			const int32 sy = Start / Width;
			int32 Nb[8];
			const int32 Deg = SkelGraph::SafeNeighbors(M, Width, Height, sx, sy, Nb);
			for (int32 i = 0; i < Deg; ++i)
			{
				const uint64 EK = EdgeKey(Start, Nb[i]);
				if (VisitedEdges.Contains(EK))
				{
					continue;
				}
				VisitedEdges.Add(EK);
				WalkPath(Start, Nb[i], /*bStopAtNodes*/ true, Path);
				EmitPath(Path);
			}
		}

		// Pass 2: remaining pure cycles / unvisited chains.
		for (int32 y = 0; y < Height; ++y)
		{
			for (int32 x = 0; x < Width; ++x)
			{
				const int32 P = y * Width + x;
				if (M[P] == 0)
				{
					continue;
				}
				int32 Nb[8];
				const int32 Deg = SkelGraph::SafeNeighbors(M, Width, Height, x, y, Nb);
				for (int32 i = 0; i < Deg; ++i)
				{
					const uint64 EK = EdgeKey(P, Nb[i]);
					if (VisitedEdges.Contains(EK))
					{
						continue;
					}
					VisitedEdges.Add(EK);
					WalkPath(P, Nb[i], /*bStopAtNodes*/ false, Path);
					EmitPath(Path);
				}
			}
		}
	}

	// ====================================================================
	// Step 5: corner splitting (port of Python split_stroke_at_corners).
	// ====================================================================
	namespace StrokeMath
	{
		// Principal (largest-eigenvalue) PCA axis direction of S[a..b] inclusive.
		// Returns a normalized vector, or (0,0) when degenerate.
		FVector2D PcaDir(const FStroke& S, int32 a, int32 b)
		{
			const int32 Cnt = b - a + 1;
			if (Cnt < 2)
			{
				return FVector2D::ZeroVector;
			}
			FVector2D Mean(0, 0);
			for (int32 k = a; k <= b; ++k)
			{
				Mean += S[k];
			}
			Mean /= double(Cnt);
			double Sxx = 0, Sxy = 0, Syy = 0;
			for (int32 k = a; k <= b; ++k)
			{
				const double dx = S[k].X - Mean.X;
				const double dy = S[k].Y - Mean.Y;
				Sxx += dx * dx; Sxy += dx * dy; Syy += dy * dy;
			}
			const double Tr = Sxx + Syy;
			const double Det = Sxx * Syy - Sxy * Sxy;
			const double Tmp = FMath::Sqrt(FMath::Max(0.0, Tr * Tr * 0.25 - Det));
			const double L1 = Tr * 0.5 + Tmp; // largest eigenvalue
			FVector2D Dir;
			if (FMath::Abs(Sxy) > 1e-12)
			{
				Dir = FVector2D(Sxy, L1 - Sxx); // eigenvector for L1
			}
			else
			{
				Dir = (Sxx >= Syy) ? FVector2D(1, 0) : FVector2D(0, 1);
			}
			if (Dir.Size() < 1e-12)
			{
				return FVector2D::ZeroVector;
			}
			Dir.Normalize();
			return Dir;
		}

		// Unoriented angle (degrees, 0..90) between two segment PCA axes.
		double SegAxisAngleDeg(const FStroke& S, int32 la, int32 lb, int32 ra, int32 rb)
		{
			const FVector2D D1 = PcaDir(S, la, lb);
			const FVector2D D2 = PcaDir(S, ra, rb);
			if (D1.IsNearlyZero() || D2.IsNearlyZero())
			{
				return 0.0;
			}
			double C = FMath::Abs(FVector2D::DotProduct(D1, D2));
			C = FMath::Clamp(C, 0.0, 1.0);
			return FMath::RadiansToDegrees(FMath::Acos(C));
		}

		int32 WalkLeftByArc(const FStroke& S, int32 i, int32 StopI, double MaxArc)
		{
			int32 Start = i;
			double Total = 0.0;
			for (int32 k = i; k > StopI; --k)
			{
				Total += (S[k] - S[k - 1]).Size();
				Start = k - 1;
				if (Total >= MaxArc)
				{
					break;
				}
			}
			return Start;
		}

		// Exclusive right end index. StopI < 0 means "no next split" (-> n-1).
		int32 WalkRightByArc(const FStroke& S, int32 i, int32 StopI, double MaxArc)
		{
			const int32 N = S.Num();
			if (StopI < 0)
			{
				StopI = N - 1;
			}
			StopI = FMath::Clamp(StopI, i, N - 1);
			int32 End = i + 1;
			double Total = 0.0;
			for (int32 k = i; k < StopI; ++k)
			{
				Total += (S[k + 1] - S[k]).Size();
				End = k + 2;
				if (Total >= MaxArc)
				{
					break;
				}
			}
			return End;
		}
	} // namespace StrokeMath

	// Compute the accepted corner-split indices for one stroke. Returns sorted indices.
	static void ComputeCornerSplitIndices(
		const FStroke& S, float AngleThresh, int32 MinPixels,
		float SegmentArc, float SplitPeakMinDistance, int32 MaxIters,
		TArray<int32>& OutSplits)
	{
		using namespace StrokeMath;
		OutSplits.Reset();
		const int32 N = S.Num();
		if (AngleThresh <= 0.0f || N < FMath::Max(3, MinPixels * 2 + 1))
		{
			return;
		}
		const double SegArc = FMath::Max(1.0, double(SegmentArc));
		const double PeakMinDist = FMath::Max(0.0, double(SplitPeakMinDistance));
		const int32 OptIters = FMath::Max(1, MaxIters);

		// Arc-length prefix sums.
		TArray<double> Arc;
		Arc.SetNumZeroed(N);
		for (int32 k = 1; k < N; ++k)
		{
			Arc[k] = Arc[k - 1] + (S[k] - S[k - 1]).Size();
		}
		auto ArcDist = [&](int32 i, int32 j) -> double
		{
			i = FMath::Clamp(i, 0, N - 1);
			j = FMath::Clamp(j, 0, N - 1);
			return FMath::Abs(Arc[i] - Arc[j]);
		};

		// Segment-angle for split index i given left bound PrevSplit and right bound
		// NextSplit (-1 = none). Fills window lengths.
		auto SegAngleAt = [&](int32 i, int32 PrevSplit, int32 NextSplit, int32& OutLeftLen, int32& OutRightLen) -> double
		{
			const int32 LeftStart = WalkLeftByArc(S, i, PrevSplit, SegArc);
			const int32 RightEnd = WalkRightByArc(S, i, NextSplit, SegArc);
			OutLeftLen = i - LeftStart + 1;
			OutRightLen = (RightEnd - 1) - i + 1;
			if (OutLeftLen < 2 || OutRightLen < 2)
			{
				return 0.0;
			}
			return SegAxisAngleDeg(S, LeftStart, i, i, RightEnd - 1);
		};

		// Dynamic-neighbor evaluation (context = current selected minus i).
		auto Evaluate = [&](int32 i, const TArray<int32>& SortedContext, double& OutScore) -> bool
		{
			OutScore = 0.0;
			int32 PrevSplit = 0;
			bool bHasPrev = false;
			int32 NextSplit = -1;
			for (int32 C : SortedContext)
			{
				if (C < i) { PrevSplit = C; bHasPrev = true; }
				else if (C > i) { NextSplit = C; break; }
			}

			if (i - PrevSplit < MinPixels)
			{
				return false;
			}
			if (NextSplit < 0)
			{
				if (N - i < MinPixels)
				{
					return false;
				}
			}
			else if (NextSplit - i < MinPixels)
			{
				return false;
			}
			if (bHasPrev && PeakMinDist > 0.0 && ArcDist(i, PrevSplit) < PeakMinDist)
			{
				return false;
			}
			if (NextSplit >= 0 && PeakMinDist > 0.0 && ArcDist(i, NextSplit) < PeakMinDist)
			{
				return false;
			}
			int32 LL = 0, RL = 0;
			const double Angle = SegAngleAt(i, PrevSplit, NextSplit, LL, RL);
			if (LL < 2 || RL < 2)
			{
				return false;
			}
			OutScore = Angle;
			return Angle >= double(AngleThresh);
		};

		// Fixed-bound scan high-score test (segment bounds PrevSplit/NextSplit, both indices).
		auto ScanHigh = [&](int32 i, int32 PrevSplit, int32 NextSplit, double& OutAngle) -> bool
		{
			OutAngle = 0.0;
			if (i - PrevSplit < MinPixels)
			{
				return false;
			}
			if (NextSplit - i < MinPixels)
			{
				return false;
			}
			if (PrevSplit > 0 && PeakMinDist > 0.0 && ArcDist(i, PrevSplit) < PeakMinDist)
			{
				return false;
			}
			if (NextSplit < N - 1 && PeakMinDist > 0.0 && ArcDist(i, NextSplit) < PeakMinDist)
			{
				return false;
			}
			int32 LL = 0, RL = 0;
			const double Angle = SegAngleAt(i, PrevSplit, NextSplit, LL, RL);
			OutAngle = Angle;
			if (LL < 2 || RL < 2)
			{
				return false;
			}
			return Angle >= double(AngleThresh);
		};

		TSet<int32> Selected;
		TSet<FString> SeenSets;
		SeenSets.Add(TEXT("")); // empty set already "seen"

		auto EncodeSet = [](const TArray<int32>& Sorted) -> FString
		{
			FString Key;
			for (int32 V : Sorted)
			{
				Key += FString::Printf(TEXT("%d,"), V);
			}
			return Key;
		};

		for (int32 Pass = 0; Pass < OptIters; ++Pass)
		{
			TArray<int32> SelSorted = Selected.Array();
			SelSorted.Sort();

			// --- scan candidate peaks under current segmentation ---
			TArray<int32> Interior;
			for (int32 V : SelSorted)
			{
				if (V > 0 && V < N - 1)
				{
					Interior.Add(V);
				}
			}
			TArray<int32> Bounds;
			Bounds.Add(0);
			Bounds.Append(Interior);
			Bounds.Add(N - 1);

			TArray<int32> CandidatePeaks;
			for (int32 b = 0; b + 1 < Bounds.Num(); ++b)
			{
				const int32 PrevSplit = Bounds[b];
				const int32 NextSplit = Bounds[b + 1];

				// Walk i in (PrevSplit+1 .. NextSplit-1), grouping consecutive high-score points.
				TArray<int32> GroupIdx;
				TArray<double> GroupAngle;
				auto FlushGroup = [&]()
				{
					if (GroupIdx.Num() == 0)
					{
						return;
					}
					// Local maxima of segment angle within the consecutive group.
					TArray<int32> Peaks;
					for (int32 j = 0; j < GroupIdx.Num(); ++j)
					{
						const double Sc = GroupAngle[j];
						const bool bGEleft = (j == 0) || (Sc >= GroupAngle[j - 1]);
						const bool bGEright = (j + 1 >= GroupIdx.Num()) || (Sc >= GroupAngle[j + 1]);
						if (bGEleft && bGEright)
						{
							Peaks.Add(GroupIdx[j]);
						}
					}
					if (Peaks.Num() == 0)
					{
						int32 BestJ = 0;
						for (int32 j = 1; j < GroupIdx.Num(); ++j)
						{
							if (GroupAngle[j] > GroupAngle[BestJ])
							{
								BestJ = j;
							}
						}
						Peaks.Add(GroupIdx[BestJ]);
					}
					CandidatePeaks.Append(Peaks);
					GroupIdx.Reset();
					GroupAngle.Reset();
				};

				for (int32 i = PrevSplit + 1; i < NextSplit; ++i)
				{
					double Angle = 0.0;
					const bool bHigh = ScanHigh(i, PrevSplit, NextSplit, Angle);
					if (bHigh)
					{
						if (GroupIdx.Num() > 0 && i == GroupIdx.Last() + 1)
						{
							GroupIdx.Add(i);
							GroupAngle.Add(Angle);
						}
						else
						{
							FlushGroup();
							GroupIdx.Add(i);
							GroupAngle.Add(Angle);
						}
					}
					else
					{
						FlushGroup();
					}
				}
				FlushGroup();
			}

			// --- evaluate candidate pool (peaks + carried selected) ---
			TSet<int32> Pool;
			for (int32 P : CandidatePeaks)
			{
				Pool.Add(P);
			}
			for (int32 Si : Selected)
			{
				if (Si > 0 && Si < N - 1)
				{
					Pool.Add(Si);
				}
			}

			TArray<int32> PoolSorted = Pool.Array();
			PoolSorted.Sort();

			TArray<int32> Proposed;
			TMap<int32, double> ScoreMap;
			for (int32 Si : PoolSorted)
			{
				TArray<int32> Context = SelSorted;
				Context.Remove(Si);
				double Score = 0.0;
				const bool bOk = Evaluate(Si, Context, Score);
				ScoreMap.Add(Si, Score);
				if (bOk)
				{
					Proposed.Add(Si);
				}
			}

			// --- resolve close-split conflicts (greedy, strongest first) ---
			Proposed.Sort([&](int32 A, int32 B)
			{
				const double Sa = ScoreMap[A];
				const double Sb = ScoreMap[B];
				if (Sa != Sb)
				{
					return Sa > Sb; // higher score first
				}
				return A < B; // tie: smaller index first
			});

			TArray<int32> Kept;
			for (int32 Idx : Proposed)
			{
				bool bTooClose = false;
				if (PeakMinDist > 0.0)
				{
					for (int32 K : Kept)
					{
						if (ArcDist(Idx, K) < PeakMinDist)
						{
							bTooClose = true;
							break;
						}
					}
				}
				if (!bTooClose)
				{
					Kept.Add(Idx);
				}
			}

			TSet<int32> NewSelected;
			NewSelected.Append(Kept);

			// --- convergence check ---
			TArray<int32> NewSorted = NewSelected.Array();
			NewSorted.Sort();
			const FString Key = EncodeSet(NewSorted);
			const bool bSame = (NewSelected.Num() == Selected.Num()) && !NewSelected.Difference(Selected).Num();
			Selected = MoveTemp(NewSelected);
			if (bSame || SeenSets.Contains(Key))
			{
				break;
			}
			SeenSets.Add(Key);
		}

		// Final acceptance: keep selected indices that still pass evaluation
		// against the final segmentation.
		TArray<int32> FinalSel = Selected.Array();
		FinalSel.Sort();
		for (int32 Si : FinalSel)
		{
			TArray<int32> Context = FinalSel;
			Context.Remove(Si);
			double Score = 0.0;
			if (Evaluate(Si, Context, Score))
			{
				OutSplits.Add(Si);
			}
		}
	}

	void SplitStrokesAtCorners(
		const TArray<FStroke>& In, float AngleThresh, int32 MinPixels,
		float SegmentArc, float SplitPeakMinDistance, int32 MaxIters,
		TArray<FStroke>& Out)
	{
		Out.Reset();
		if (AngleThresh <= 0.0f)
		{
			Out = In;
			return;
		}

		TArray<int32> Splits;
		for (const FStroke& S : In)
		{
			ComputeCornerSplitIndices(S, AngleThresh, MinPixels, SegmentArc, SplitPeakMinDistance, MaxIters, Splits);

			if (Splits.Num() == 0)
			{
				Out.Add(S);
				continue;
			}

			int32 Added = 0;
			int32 Start = 0;
			for (int32 SplitI : Splits)
			{
				const int32 Count = SplitI - Start + 1; // inclusive of split point
				if (Count >= MinPixels)
				{
					FStroke Piece(S.GetData() + Start, Count);
					Out.Add(MoveTemp(Piece));
					++Added;
				}
				Start = SplitI;
			}
			const int32 TailCount = S.Num() - Start;
			if (TailCount >= MinPixels)
			{
				FStroke Tail(S.GetData() + Start, TailCount);
				Out.Add(MoveTemp(Tail));
				++Added;
			}
			if (Added == 0)
			{
				Out.Add(S); // never drop a stroke entirely
			}
		}
	}

	bool SaveStrokesPng(const TArray<FStroke>& Strokes, int32 Width, int32 Height, const FString& Path, int32 Thickness)
	{
		const int32 N = Width * Height;
		TArray<uint8> RGBA;
		RGBA.Init(255, N * 4); // white background

		static const uint8 Palette[][3] = {
			{220,  20,  60}, { 30, 144, 255}, { 34, 139,  34}, {255, 140,   0},
			{148,   0, 211}, {  0, 191, 191}, {199,  21, 133}, {139,  69,  19},
			{ 70, 130, 180}, {255,  20, 147}, {107, 142,  35}, {  0,   0,   0},
		};
		const int32 PaletteCount = UE_ARRAY_COUNT(Palette);
		const int32 R = FMath::Max(0, Thickness / 2);

		auto Plot = [&](int32 x, int32 y, const uint8* Col)
		{
			for (int32 oy = -R; oy <= R; ++oy)
			{
				for (int32 ox = -R; ox <= R; ++ox)
				{
					const int32 xx = x + ox;
					const int32 yy = y + oy;
					if (xx < 0 || xx >= Width || yy < 0 || yy >= Height)
					{
						continue;
					}
					const int32 Off = (yy * Width + xx) * 4;
					RGBA[Off + 0] = Col[0];
					RGBA[Off + 1] = Col[1];
					RGBA[Off + 2] = Col[2];
					RGBA[Off + 3] = 255;
				}
			}
		};

		TArray<FIntPoint> LineBuf;
		for (int32 s = 0; s < Strokes.Num(); ++s)
		{
			const uint8* Col = Palette[s % PaletteCount];
			const FStroke& Stroke = Strokes[s];
			for (int32 k = 0; k + 1 < Stroke.Num(); ++k)
			{
				const FIntPoint P0(FMath::RoundToInt(Stroke[k].X), FMath::RoundToInt(Stroke[k].Y));
				const FIntPoint P1(FMath::RoundToInt(Stroke[k + 1].X), FMath::RoundToInt(Stroke[k + 1].Y));
				SkelGraph::LinePoints(P0, P1, LineBuf);
				for (const FIntPoint& P : LineBuf)
				{
					Plot(P.X, P.Y, Col);
				}
			}
		}

		IImageWrapperModule& IWM = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
		TSharedPtr<IImageWrapper> IW = IWM.CreateImageWrapper(EImageFormat::PNG);
		if (!IW.IsValid())
		{
			return false;
		}
		IW->SetRaw(RGBA.GetData(), RGBA.Num(), Width, Height, ERGBFormat::RGBA, 8);
		const TArray64<uint8>& C = IW->GetCompressed();
		return FFileHelper::SaveArrayToFile(TArrayView<const uint8>(C.GetData(), static_cast<int32>(C.Num())), *Path);
	}

	// ====================================================================
	// Color classification + color-aware stroke splitting.
	// ====================================================================
	const TCHAR* StrokeColorToString(EStrokeColor Color)
	{
		switch (Color)
		{
		case EStrokeColor::Black: return TEXT("black");
		case EStrokeColor::Red:   return TEXT("red");
		case EStrokeColor::Green: return TEXT("green");
		case EStrokeColor::Blue:  return TEXT("blue");
		default:                  return TEXT("none");
		}
	}

	EStrokeColor ClassifyRGB(uint8 R, uint8 G, uint8 B)
	{
		return ClassifyRGB(R, G, B, /*WhiteCutoff*/ 220, /*DominanceMargin*/ 30);
	}

	EStrokeColor ClassifyRGB(uint8 R, uint8 G, uint8 B, uint8 WhiteCutoff, int32 DominanceMargin)
	{
		// Near-white background.
		if (R > WhiteCutoff && G > WhiteCutoff && B > WhiteCutoff)
		{
			return EStrokeColor::None;
		}
		const int32 r = R, g = G, b = B;
		const int32 Dom = FMath::Max(0, DominanceMargin);
		if (r >= g + Dom && r >= b + Dom)
		{
			return EStrokeColor::Red;
		}
		if (g >= r + Dom && g >= b + Dom)
		{
			return EStrokeColor::Green;
		}
		if (b >= r + Dom && b >= g + Dom)
		{
			return EStrokeColor::Blue;
		}
		// Dark or neutral (no clear hue) -> the black line-art render.
		return EStrokeColor::Black;
	}

	void BuildColorClassMap(const TArray<uint8>& RGBA, int32 Width, int32 Height, TArray<uint8>& OutMap)
	{
		BuildColorClassMap(RGBA, Width, Height, /*WhiteCutoff*/ 220, /*DominanceMargin*/ 30, OutMap);
	}

	void BuildColorClassMap(const TArray<uint8>& RGBA, int32 Width, int32 Height, uint8 WhiteCutoff, int32 DominanceMargin, TArray<uint8>& OutMap)
	{
		const int32 N = Width * Height;
		OutMap.SetNumUninitialized(N);
		for (int32 i = 0; i < N; ++i)
		{
			const int32 Off = i * 4;
			OutMap[i] = uint8(ClassifyRGB(RGBA[Off + 0], RGBA[Off + 1], RGBA[Off + 2], WhiteCutoff, DominanceMargin));
		}
	}

	EStrokeColor SampleColorAt(const TArray<uint8>& ColorMap, int32 Width, int32 Height, int32 X, int32 Y, int32 Radius)
	{
		int32 Counts[5] = { 0, 0, 0, 0, 0 };
		for (int32 oy = -Radius; oy <= Radius; ++oy)
		{
			const int32 yy = Y + oy;
			if (yy < 0 || yy >= Height)
			{
				continue;
			}
			for (int32 ox = -Radius; ox <= Radius; ++ox)
			{
				const int32 xx = X + ox;
				if (xx < 0 || xx >= Width)
				{
					continue;
				}
				const uint8 C = ColorMap[yy * Width + xx];
				if (C != uint8(EStrokeColor::None))
				{
					++Counts[C];
				}
			}
		}
		int32 Best = int32(EStrokeColor::None);
		int32 BestCount = 0;
		for (int32 c = 1; c <= 4; ++c)
		{
			if (Counts[c] > BestCount)
			{
				BestCount = Counts[c];
				Best = c;
			}
		}
		return EStrokeColor(Best);
	}

	namespace ColorSplit
	{
		FORCEINLINE bool IsPrimary(EStrokeColor C)
		{
			return C == EStrokeColor::Red || C == EStrokeColor::Green || C == EStrokeColor::Blue;
		}

		// Unoriented direction of S[a..b]; falls back to PCA, then zero.
		FVector2D RunDir(const FStroke& S, int32 a, int32 b)
		{
			if (a == b)
			{
				return FVector2D::ZeroVector;
			}
			FVector2D D = S[b] - S[a];
			if (D.Size() > 1e-6)
			{
				D.Normalize();
				return D;
			}
			return StrokeMath::PcaDir(S, FMath::Min(a, b), FMath::Max(a, b));
		}

		double UnorientedAngleDeg(const FVector2D& A, const FVector2D& B)
		{
			if (A.IsNearlyZero() || B.IsNearlyZero())
			{
				return 90.0; // treat unknown as worst alignment
			}
			double C = FMath::Abs(FVector2D::DotProduct(A, B));
			C = FMath::Clamp(C, 0.0, 1.0);
			return FMath::RadiansToDegrees(FMath::Acos(C));
		}

		// Classify a gap-repair (None) run from its neighbor colors and directions.
		EStrokeColor ClassifyConnection(
			EStrokeColor Left, EStrokeColor Right,
			const FVector2D& ConnDir, const FVector2D& LeftDir, const FVector2D& RightDir)
		{
			if (Left == EStrokeColor::None && Right == EStrokeColor::None)
			{
				return EStrokeColor::Black;
			}
			if (Left == EStrokeColor::None)
			{
				return Right;
			}
			if (Right == EStrokeColor::None)
			{
				return Left;
			}
			if (Left == Right)
			{
				return Left;
			}

			// Both colored but different.
			if (IsPrimary(Left) && IsPrimary(Right))
			{
				const double AngLeft = UnorientedAngleDeg(ConnDir, LeftDir);
				const double AngRight = UnorientedAngleDeg(ConnDir, RightDir);
				return (AngRight <= AngLeft) ? Right : Left; // nearest-aligned neighbor wins
			}

			// One side is Black, the other a primary.
			const EStrokeColor Primary = (Left == EStrokeColor::Black) ? Right : Left;
			switch (Primary)
			{
			case EStrokeColor::Red:   return EStrokeColor::Black; // red | black -> black
			case EStrokeColor::Green: return EStrokeColor::Green; // green | black -> green
			case EStrokeColor::Blue:  return EStrokeColor::Blue;  // blue | black -> blue
			default:                  return EStrokeColor::Black;
			}
		}
	} // namespace ColorSplit

	void ColorizeAndSplitStrokes(
		const TArray<FStroke>& In, const TArray<uint8>& ColorMap, int32 Width, int32 Height,
		int32 SampleRadius, float MinRunArc, TArray<FColoredStroke>& Out)
	{
		using namespace ColorSplit;
		Out.Reset();

		// Build maximal equal-color runs of a per-point label array as [Start,End] index pairs.
		auto BuildRuns = [](const TArray<EStrokeColor>& Lab, TArray<int32>& RunStart, TArray<int32>& RunEnd, TArray<EStrokeColor>& RunCol)
		{
			RunStart.Reset(); RunEnd.Reset(); RunCol.Reset();
			const int32 N = Lab.Num();
			for (int32 i = 0; i < N; )
			{
				int32 j = i;
				while (j + 1 < N && Lab[j + 1] == Lab[i])
				{
					++j;
				}
				RunStart.Add(i); RunEnd.Add(j); RunCol.Add(Lab[i]);
				i = j + 1;
			}
		};

		for (const FStroke& S : In)
		{
			const int32 N = S.Num();
			if (N == 0)
			{
				continue;
			}

			// Per-point color class + original "synthetic" (None) flag.
			TArray<EStrokeColor> Cls;
			Cls.SetNumUninitialized(N);
			TArray<uint8> WasNone;
			WasNone.SetNumUninitialized(N);
			for (int32 k = 0; k < N; ++k)
			{
				Cls[k] = SampleColorAt(ColorMap, Width, Height, FMath::RoundToInt(S[k].X), FMath::RoundToInt(S[k].Y), SampleRadius);
				WasNone[k] = (Cls[k] == EStrokeColor::None) ? 1 : 0;
			}

			// Arc-length prefix sums for run-length thresholds.
			TArray<double> Arc;
			Arc.SetNumZeroed(N);
			for (int32 k = 1; k < N; ++k)
			{
				Arc[k] = Arc[k - 1] + (S[k] - S[k - 1]).Size();
			}

			TArray<int32> RunStart, RunEnd;
			TArray<EStrokeColor> RunCol;
			TArray<EStrokeColor> Final = Cls;

			// 1) Reclassify synthetic (None) runs from their neighbor color/direction.
			BuildRuns(Final, RunStart, RunEnd, RunCol);
			for (int32 r = 0; r < RunCol.Num(); ++r)
			{
				if (RunCol[r] != EStrokeColor::None)
				{
					continue;
				}
				const EStrokeColor LeftC = (r > 0) ? RunCol[r - 1] : EStrokeColor::None;
				const EStrokeColor RightC = (r + 1 < RunCol.Num()) ? RunCol[r + 1] : EStrokeColor::None;
				const FVector2D ConnDir = RunDir(S, RunStart[r], RunEnd[r]);
				const FVector2D LeftDir = (r > 0) ? RunDir(S, RunStart[r - 1], RunEnd[r - 1]) : FVector2D::ZeroVector;
				const FVector2D RightDir = (r + 1 < RunCol.Num()) ? RunDir(S, RunStart[r + 1], RunEnd[r + 1]) : FVector2D::ZeroVector;
				const EStrokeColor NC = ClassifyConnection(LeftC, RightC, ConnDir, LeftDir, RightDir);
				for (int32 k = RunStart[r]; k <= RunEnd[r]; ++k)
				{
					Final[k] = NC;
				}
			}

			// 2) Absorb short runs (e.g. black blips at colored/black crossings) into neighbors.
			for (;;)
			{
				BuildRuns(Final, RunStart, RunEnd, RunCol);
				const int32 RunCount = RunCol.Num();
				if (RunCount <= 1)
				{
					break;
				}
				auto RunArc = [&](int32 i) -> double { return Arc[RunEnd[i]] - Arc[RunStart[i]]; };

				// Sub-threshold runs, shortest first.
				TArray<int32> Cand;
				for (int32 i = 0; i < RunCount; ++i)
				{
					if (RunArc(i) < double(MinRunArc))
					{
						Cand.Add(i);
					}
				}
				Cand.Sort([&](int32 A, int32 B) { return RunArc(A) < RunArc(B); });

				bool bApplied = false;
				for (int32 Idx : Cand)
				{
					const EStrokeColor LeftC = (Idx > 0) ? RunCol[Idx - 1] : EStrokeColor::None;
					const EStrokeColor RightC = (Idx + 1 < RunCount) ? RunCol[Idx + 1] : EStrokeColor::None;
					EStrokeColor NC = RunCol[Idx];
					if (LeftC != EStrokeColor::None && RightC != EStrokeColor::None)
					{
						NC = (LeftC == RightC) ? LeftC
							: (RunArc(Idx - 1) >= RunArc(Idx + 1) ? LeftC : RightC);
					}
					else if (LeftC != EStrokeColor::None)
					{
						NC = LeftC;
					}
					else if (RightC != EStrokeColor::None)
					{
						NC = RightC;
					}

					if (NC != RunCol[Idx])
					{
						for (int32 k = RunStart[Idx]; k <= RunEnd[Idx]; ++k)
						{
							Final[k] = NC;
						}
						bApplied = true;
						break;
					}
				}
				if (!bApplied)
				{
					break;
				}
			}

			// 3) Emit one mono-color piece per final run.
			BuildRuns(Final, RunStart, RunEnd, RunCol);
			for (int32 r = 0; r < RunCol.Num(); ++r)
			{
				const int32 StartIdx = RunStart[r];
				const int32 EndIdx = RunEnd[r];
				FColoredStroke CS;
				CS.Color = RunCol[r];
				CS.Points = FStroke(S.GetData() + StartIdx, EndIdx - StartIdx + 1);
				int32 ConnPts = 0;
				for (int32 k = StartIdx; k <= EndIdx; ++k)
				{
					ConnPts += WasNone[k];
				}
				CS.ConnectionPointCount = ConnPts;
				Out.Add(MoveTemp(CS));
			}
		}
	}

	void SplitColoredStrokesAtCorners(
		const TArray<FColoredStroke>& In, float AngleThresh, int32 MinPixels,
		float SegmentArc, float SplitPeakMinDistance, int32 MaxIters,
		TArray<FColoredStroke>& Out)
	{
		Out.Reset();
		TArray<int32> Splits;
		for (const FColoredStroke& CS : In)
		{
			if (AngleThresh <= 0.0f)
			{
				Out.Add(CS);
				continue;
			}
			ComputeCornerSplitIndices(CS.Points, AngleThresh, MinPixels, SegmentArc, SplitPeakMinDistance, MaxIters, Splits);
			if (Splits.Num() == 0)
			{
				Out.Add(CS);
				continue;
			}

			int32 Added = 0;
			int32 Start = 0;
			auto Emit = [&](int32 S0, int32 Count)
			{
				if (Count < MinPixels)
				{
					return;
				}
				FColoredStroke Piece;
				Piece.Color = CS.Color;
				Piece.Points = FStroke(CS.Points.GetData() + S0, Count);
				Out.Add(MoveTemp(Piece));
				++Added;
			};
			for (int32 SplitI : Splits)
			{
				Emit(Start, SplitI - Start + 1);
				Start = SplitI;
			}
			Emit(Start, CS.Points.Num() - Start);
			if (Added == 0)
			{
				Out.Add(CS);
			}
		}
	}

	// ====================================================================
	// Step 6: same-color merge of corner-split fragments.
	// ====================================================================
	namespace MergeOps
	{
		FVector2D EndpointChordAxis(const FStroke& S)
		{
			if (S.Num() < 2)
			{
				return FVector2D::ZeroVector;
			}
			FVector2D D = S.Last() - S[0];
			if (D.Size() < 1e-8)
			{
				return FVector2D::ZeroVector;
			}
			D.Normalize();
			return D;
		}

		FStroke MergePolylineByEndpoints(const FStroke& S1, int32 End1, const FStroke& S2, int32 End2)
		{
			FStroke A = S1;
			if (End1 == 0)
			{
				Algo::Reverse(A);
			}
			FStroke B = S2;
			if (End2 == 1)
			{
				Algo::Reverse(B);
			}
			FStroke Out = A;
			int32 StartB = 0;
			if (Out.Num() > 0 && B.Num() > 0 && (Out.Last() - B[0]).Size() < 1e-6)
			{
				StartB = 1;
			}
			for (int32 k = StartB; k < B.Num(); ++k)
			{
				Out.Add(B[k]);
			}
			return Out;
		}

		struct FMergeInfo
		{
			bool bOk = false;
			int32 End1 = 0;
			int32 End2 = 0;
			double Gap = 0.0;
			double Angle = 0.0;
			FVector2D MergePoint = FVector2D::ZeroVector;
		};

		FMergeInfo CanPostSplitMerge(const FStroke& A, const FStroke& B, float MaxGap, float MaxAngle)
		{
			FMergeInfo R;
			if (A.Num() < 2 || B.Num() < 2)
			{
				return R;
			}
			const FVector2D D1 = StrokeMath::PcaDir(A, 0, A.Num() - 1);
			const FVector2D D2 = StrokeMath::PcaDir(B, 0, B.Num() - 1);
			if (D1.IsNearlyZero() || D2.IsNearlyZero())
			{
				return R;
			}
			const double Angle = ColorSplit::UnorientedAngleDeg(D1, D2);
			if (Angle > double(MaxAngle))
			{
				return R;
			}
			const FVector2D C1 = EndpointChordAxis(A);
			const FVector2D C2 = EndpointChordAxis(B);
			if (C1.IsNearlyZero() || C2.IsNearlyZero())
			{
				return R;
			}

			double BestGap = TNumericLimits<double>::Max();
			for (int32 E1 = 0; E1 <= 1; ++E1)
			{
				const FVector2D P1 = (E1 == 0) ? A[0] : A.Last();
				for (int32 E2 = 0; E2 <= 1; ++E2)
				{
					const FVector2D P2 = (E2 == 0) ? B[0] : B.Last();
					const double Gap = FVector2D::Distance(P1, P2);
					if (Gap > double(MaxGap))
					{
						continue;
					}
					const FStroke Merged = MergePolylineByEndpoints(A, E1, B, E2);
					const FVector2D MC = EndpointChordAxis(Merged);
					if (MC.IsNearlyZero())
					{
						continue;
					}
					const double MergedEndpointAngle = FMath::Max(
						ColorSplit::UnorientedAngleDeg(MC, C1),
						ColorSplit::UnorientedAngleDeg(MC, C2));
					if (MergedEndpointAngle > double(MaxAngle))
					{
						continue;
					}
					if (Gap < BestGap)
					{
						BestGap = Gap;
						R.bOk = true;
						R.End1 = E1;
						R.End2 = E2;
						R.Gap = Gap;
						R.Angle = Angle;
						R.MergePoint = (P1 + P2) * 0.5;
					}
				}
			}
			return R;
		}

		bool ThirdEndpointNearMergePoint(const TArray<FColoredStroke>& Strokes, int32 I, int32 J, const FVector2D& MergePoint, float Radius)
		{
			if (Radius <= 0.0f)
			{
				return false;
			}
			const double R2 = double(Radius) * double(Radius);
			for (int32 k = 0; k < Strokes.Num(); ++k)
			{
				if (k == I || k == J || Strokes[k].Points.Num() == 0)
				{
					continue;
				}
				if (FVector2D::DistSquared(Strokes[k].Points[0], MergePoint) <= R2 ||
					FVector2D::DistSquared(Strokes[k].Points.Last(), MergePoint) <= R2)
				{
					return true;
				}
			}
			return false;
		}
	} // namespace MergeOps

	void MergeColoredStrokesSameColor(
		const TArray<FColoredStroke>& In, float MaxGap, float MaxAngle, int32 MaxIters,
		float ProtectJunctionRadius, TArray<FColoredStroke>& Out)
	{
		using namespace MergeOps;
		Out = In;

		for (int32 Iter = 0; Iter < MaxIters; ++Iter)
		{
			int32 BestI = -1, BestJ = -1;
			FMergeInfo BestInfo;
			double BestCost = TNumericLimits<double>::Max();

			for (int32 i = 0; i < Out.Num(); ++i)
			{
				for (int32 j = i + 1; j < Out.Num(); ++j)
				{
					if (Out[i].Color != Out[j].Color) // same color class only
					{
						continue;
					}
					const FMergeInfo Info = CanPostSplitMerge(Out[i].Points, Out[j].Points, MaxGap, MaxAngle);
					if (!Info.bOk)
					{
						continue;
					}
					if (ThirdEndpointNearMergePoint(Out, i, j, Info.MergePoint, ProtectJunctionRadius))
					{
						continue; // protect true junction
					}
					const double Cost = Info.Gap + 0.1 * Info.Angle;
					if (Cost < BestCost)
					{
						BestCost = Cost;
						BestI = i;
						BestJ = j;
						BestInfo = Info;
					}
				}
			}

			if (BestI < 0)
			{
				break;
			}

			FColoredStroke Merged;
			Merged.Color = Out[BestI].Color;
			Merged.Points = MergePolylineByEndpoints(Out[BestI].Points, BestInfo.End1, Out[BestJ].Points, BestInfo.End2);
			Merged.ConnectionPointCount = Out[BestI].ConnectionPointCount + Out[BestJ].ConnectionPointCount;

			Out.RemoveAt(BestJ); // BestJ > BestI, remove higher index first
			Out.RemoveAt(BestI);
			Out.Add(MoveTemp(Merged));
		}
	}

	// ====================================================================
	// Step 7: stroke geometry metrics.
	// ====================================================================
	static double Percentile90(TArray<double>& V)
	{
		const int32 N = V.Num();
		if (N == 0)
		{
			return 0.0;
		}
		V.Sort();
		if (N == 1)
		{
			return V[0];
		}
		const double Rank = 0.9 * double(N - 1);
		const int32 Lo = FMath::FloorToInt(Rank);
		const int32 Hi = FMath::Min(Lo + 1, N - 1);
		const double Frac = Rank - double(Lo);
		return V[Lo] + (V[Hi] - V[Lo]) * Frac;
	}

	void ComputeStrokeMetrics(TArray<FColoredStroke>& InOut)
	{
		for (FColoredStroke& CS : InOut)
		{
			const FStroke& P = CS.Points;
			const int32 N = P.Num();
			CS.bHasMetrics = true;
			if (N < 2)
			{
				CS.Arc = 0; CS.Chord = 0; CS.Straightness = 0;
				CS.P90PcaError = 0; CS.PcaRmsError = 0; CS.P90ChordDev = 0; CS.ChordDevRatio = 0;
				CS.Direction = FVector2D::ZeroVector;
				continue;
			}

			double Arc = 0.0;
			for (int32 k = 1; k < N; ++k)
			{
				Arc += (P[k] - P[k - 1]).Size();
			}
			const double Chord = (P[N - 1] - P[0]).Size();
			CS.Arc = Arc;
			CS.Chord = Chord;
			CS.Straightness = (Arc < 1e-8) ? 0.0 : Chord / Arc;

			// PCA principal axis + line (normal form a*x + b*y + c = 0, |(a,b)| = 1).
			const FVector2D Dir = StrokeMath::PcaDir(P, 0, N - 1);
			CS.Direction = Dir;
			FVector2D Center(0, 0);
			for (const FVector2D& Pt : P)
			{
				Center += Pt;
			}
			Center /= double(N);

			TArray<double> PcaDist;
			PcaDist.SetNumUninitialized(N);
			if (!Dir.IsNearlyZero())
			{
				const FVector2D Normal(-Dir.Y, Dir.X);
				const double C = -FVector2D::DotProduct(Normal, Center);
				for (int32 k = 0; k < N; ++k)
				{
					PcaDist[k] = FMath::Abs(Normal.X * P[k].X + Normal.Y * P[k].Y + C);
				}
			}
			else
			{
				for (int32 k = 0; k < N; ++k)
				{
					PcaDist[k] = 0.0;
				}
			}
			double SumSq = 0.0;
			for (double d : PcaDist)
			{
				SumSq += d * d;
			}
			CS.PcaRmsError = FMath::Sqrt(SumSq / double(N));
			CS.P90PcaError = Percentile90(PcaDist);

			// Chord-line deviation.
			FVector2D V = P[N - 1] - P[0];
			if (V.Size() < 1e-8 || Chord < 1e-8)
			{
				CS.P90ChordDev = 0.0;
				CS.ChordDevRatio = 1e9; // "inf"
			}
			else
			{
				const double Vn = V.Size();
				const FVector2D NormalChord(-V.Y / Vn, V.X / Vn);
				const double CC = -FVector2D::DotProduct(NormalChord, P[0]);
				TArray<double> ChordDist;
				ChordDist.SetNumUninitialized(N);
				for (int32 k = 0; k < N; ++k)
				{
					ChordDist[k] = FMath::Abs(NormalChord.X * P[k].X + NormalChord.Y * P[k].Y + CC);
				}
				CS.P90ChordDev = Percentile90(ChordDist);
				CS.ChordDevRatio = CS.P90ChordDev / Chord;
			}
		}
	}

	// ====================================================================
	// Step 8: enclosed-region mask (endpoint-nearest-connect + flood).
	// ====================================================================
	void ComputeEnclosedRegionMask(
		const TArray<FColoredStroke>& Strokes, int32 Width, int32 Height,
		int32 Thickness, TArray<uint8>& OutMask, TArray<uint8>& OutBarrier)
	{
		const int32 N = Width * Height;
		TArray<uint8> Barrier;
		Barrier.Init(0, N);
		const int32 Rad = FMath::Max(1, Thickness / 2);

		auto Stamp = [&](int32 x, int32 y)
		{
			for (int32 oy = -Rad; oy <= Rad; ++oy)
			{
				for (int32 ox = -Rad; ox <= Rad; ++ox)
				{
					const int32 xx = x + ox;
					const int32 yy = y + oy;
					if (xx >= 0 && xx < Width && yy >= 0 && yy < Height)
					{
						Barrier[yy * Width + xx] = 255;
					}
				}
			}
		};

		TArray<FIntPoint> LineBuf;
		auto DrawSeg = [&](const FVector2D& A, const FVector2D& B)
		{
			SkelGraph::LinePoints(
				FIntPoint(FMath::RoundToInt(A.X), FMath::RoundToInt(A.Y)),
				FIntPoint(FMath::RoundToInt(B.X), FMath::RoundToInt(B.Y)), LineBuf);
			for (const FIntPoint& P : LineBuf)
			{
				Stamp(P.X, P.Y);
			}
		};

		// Rasterize all strokes.
		for (const FColoredStroke& CS : Strokes)
		{
			for (int32 k = 0; k + 1 < CS.Points.Num(); ++k)
			{
				DrawSeg(CS.Points[k], CS.Points[k + 1]);
			}
		}

		// Connect each stroke endpoint to its nearest other endpoint to close gaps.
		TArray<FVector2D> Endpoints;
		for (const FColoredStroke& CS : Strokes)
		{
			if (CS.Points.Num() > 0)
			{
				Endpoints.Add(CS.Points[0]);
				Endpoints.Add(CS.Points.Last());
			}
		}
		for (int32 i = 0; i < Endpoints.Num(); ++i)
		{
			int32 Best = -1;
			double BestD2 = TNumericLimits<double>::Max();
			for (int32 j = 0; j < Endpoints.Num(); ++j)
			{
				if (j == i)
				{
					continue;
				}
				const double D2 = FVector2D::DistSquared(Endpoints[i], Endpoints[j]);
				if (D2 < BestD2)
				{
					BestD2 = D2;
					Best = j;
				}
			}
			if (Best >= 0)
			{
				DrawSeg(Endpoints[i], Endpoints[Best]);
			}
		}

		// Flood the background (barrier == 0) from all border pixels.
		TArray<uint8> Reachable;
		Reachable.Init(0, N);
		TArray<int32> Stack;
		Stack.Reserve(1024);
		auto TryPush = [&](int32 x, int32 y)
		{
			if (x < 0 || x >= Width || y < 0 || y >= Height)
			{
				return;
			}
			const int32 Idx = y * Width + x;
			if (Barrier[Idx] == 0 && Reachable[Idx] == 0)
			{
				Reachable[Idx] = 1;
				Stack.Push(Idx);
			}
		};
		for (int32 x = 0; x < Width; ++x)
		{
			TryPush(x, 0);
			TryPush(x, Height - 1);
		}
		for (int32 y = 0; y < Height; ++y)
		{
			TryPush(0, y);
			TryPush(Width - 1, y);
		}
		while (Stack.Num() > 0)
		{
			const int32 P = Stack.Pop(EAllowShrinking::No);
			const int32 px = P % Width;
			const int32 py = P / Width;
			TryPush(px + 1, py);
			TryPush(px - 1, py);
			TryPush(px, py + 1);
			TryPush(px, py - 1);
		}

		// Interior = background not reached from the borders.
		OutMask.Init(0, N);
		for (int32 i = 0; i < N; ++i)
		{
			if (Barrier[i] == 0 && Reachable[i] == 0)
			{
				OutMask[i] = 255;
			}
		}
		OutBarrier = MoveTemp(Barrier);
	}

	// ====================================================================
	// Step 9: red cap-loop detection + longest-green side + translate-copy.
	// ====================================================================
	namespace CapOps
	{
		// Endpoint graph. Real gaps are bridged by explicit connector strokes first;
		// this snap tolerance is only for nearly coincident endpoints.
		struct FGraph
		{
			TArray<int32> NodeU;   // per edge: start node
			TArray<int32> NodeV;   // per edge: end node
			TArray<int32> StrokeId;// per edge: index into the source stroke array
			TArray<uint8> bBlack;  // per edge: 1 if black, 0 if red
			TArray<uint8> bSynthetic; // per edge: 1 if generated connector stroke
			TArray<FVector2D> NodePos; // representative position per node
			int32 NumNodes = 0;
		};

		int32 Find(TArray<int32>& Parent, int32 X)
		{
			while (Parent[X] != X)
			{
				Parent[X] = Parent[Parent[X]];
				X = Parent[X];
			}
			return X;
		}

		// Build a graph over the given edge strokes (by index into Strokes), snapping
		// only nearly coincident endpoints. Wider red/black gaps are represented by
		// explicit connector strokes before this graph is built.
		void BuildGraph(const TArray<FColoredStroke>& Strokes, const TArray<int32>& EdgeStrokes, float NodeSnapTol, int32 FirstSyntheticStrokeId, FGraph& G)
		{
			const int32 E = EdgeStrokes.Num();
			TArray<FVector2D> Pts; // 2 endpoints per edge
			Pts.Reserve(E * 2);
			for (int32 s : EdgeStrokes)
			{
				const FStroke& P = Strokes[s].Points;
				Pts.Add(P.Num() > 0 ? P[0] : FVector2D::ZeroVector);
				Pts.Add(P.Num() > 0 ? P.Last() : FVector2D::ZeroVector);
			}

			const int32 NP = Pts.Num();
			TArray<int32> Parent;
			Parent.SetNumUninitialized(NP);
			for (int32 i = 0; i < NP; ++i)
			{
				Parent[i] = i;
			}
			const double Tol2 = double(NodeSnapTol) * double(NodeSnapTol);
			for (int32 i = 0; i < NP; ++i)
			{
				for (int32 j = i + 1; j < NP; ++j)
				{
					if (FVector2D::DistSquared(Pts[i], Pts[j]) <= Tol2)
					{
						Parent[Find(Parent, j)] = Find(Parent, i);
					}
				}
			}

			// Compact node ids.
			TMap<int32, int32> Remap;
			TArray<FVector2D> Accum;
			TArray<int32> AccumN;
			auto NodeOf = [&](int32 PtIdx) -> int32
			{
				const int32 Root = Find(Parent, PtIdx);
				if (int32* Found = Remap.Find(Root))
				{
					Accum[*Found] += Pts[PtIdx];
					AccumN[*Found] += 1;
					return *Found;
				}
				const int32 NewId = Accum.Num();
				Remap.Add(Root, NewId);
				Accum.Add(Pts[PtIdx]);
				AccumN.Add(1);
				return NewId;
			};

			G = FGraph();
			for (int32 e = 0; e < E; ++e)
			{
				const int32 U = NodeOf(2 * e);
				const int32 V = NodeOf(2 * e + 1);
				G.NodeU.Add(U);
				G.NodeV.Add(V);
				G.StrokeId.Add(EdgeStrokes[e]);
				G.bBlack.Add(Strokes[EdgeStrokes[e]].Color == EStrokeColor::Black ? 1 : 0);
				G.bSynthetic.Add(EdgeStrokes[e] >= FirstSyntheticStrokeId ? 1 : 0);
			}
			G.NumNodes = Accum.Num();
			G.NodePos.SetNumUninitialized(G.NumNodes);
			for (int32 n = 0; n < G.NumNodes; ++n)
			{
				G.NodePos[n] = Accum[n] / double(FMath::Max(1, AccumN[n]));
			}
		}

		// BFS shortest edge-path from Src to Dst over the subset of edges flagged usable,
		// excluding edge index ExcludeEdge. EdgeUsable[e]!=0 means the edge may be traversed.
		// Returns the ordered edge indices (into G arrays).
		bool BfsPath(const FGraph& G, int32 Src, int32 Dst, int32 ExcludeEdge, const TArray<uint8>& EdgeUsable, TArray<int32>& OutEdgePath)
		{
			OutEdgePath.Reset();
			if (Src == Dst)
			{
				return true; // empty path
			}
			TArray<int32> ParentNode, ParentEdge;
			ParentNode.Init(-2, G.NumNodes);
			ParentEdge.Init(-1, G.NumNodes);
			TArray<int32> Q;
			Q.Add(Src);
			ParentNode[Src] = -1;
			int32 Head = 0;
			bool bFound = false;
			while (Head < Q.Num() && !bFound)
			{
				const int32 Cur = Q[Head++];
				for (int32 e = 0; e < G.StrokeId.Num(); ++e)
				{
					if (e == ExcludeEdge)
					{
						continue;
					}
					if (!EdgeUsable[e])
					{
						continue;
					}
					int32 Other = -1;
					if (G.NodeU[e] == Cur) { Other = G.NodeV[e]; }
					else if (G.NodeV[e] == Cur) { Other = G.NodeU[e]; }
					else { continue; }
					if (ParentNode[Other] != -2)
					{
						continue;
					}
					ParentNode[Other] = Cur;
					ParentEdge[Other] = e;
					if (Other == Dst)
					{
						bFound = true;
						break;
					}
					Q.Add(Other);
				}
			}
			if (!bFound)
			{
				return false;
			}
			for (int32 N = Dst; N != Src; N = ParentNode[N])
			{
				OutEdgePath.Add(ParentEdge[N]);
			}
			Algo::Reverse(OutEdgePath);
			return true;
		}

		// Find the smallest loop anchored on a red edge, traversing only EdgeUsable edges.
		// Returns ordered edge indices (cycle).
		bool FindRedCycle(const FGraph& G, const TArray<uint8>& bRedEdge, const TArray<uint8>& EdgeUsable, TArray<int32>& OutCycle)
		{
			OutCycle.Reset();
			int32 BestLen = MAX_int32;
			for (int32 e = 0; e < G.StrokeId.Num(); ++e)
			{
				if (!bRedEdge[e])
				{
					continue; // cycle must include a red edge; anchor on red
				}
				const int32 U = G.NodeU[e];
				const int32 V = G.NodeV[e];
				if (U == V)
				{
					// A single self-closed red stroke is NOT treated as a cap loop;
					// the loop must connect at least two strokes end-to-end.
					continue;
				}
				TArray<int32> Path;
				if (BfsPath(G, U, V, /*ExcludeEdge*/ e, EdgeUsable, Path))
				{
					const int32 Len = Path.Num() + 1;
					if (Len < BestLen)
					{
						BestLen = Len;
						OutCycle = Path;
						OutCycle.Add(e);
					}
				}
			}
			return OutCycle.Num() > 0;
		}

		// Walk an unordered cycle (edge indices) into an ordered node + point sequence.
		void BuildPolygon(const FColoredStroke* StrokesData, const FGraph& G, const TArray<int32>& Cycle,
			FStroke& OutPolygon, TArray<FVector2D>& OutNodes, TArray<FCapBoundaryRun>* OutRuns = nullptr)
		{
			OutPolygon.Reset();
			OutNodes.Reset();
			if (OutRuns)
			{
				OutRuns->Reset();
			}
			if (Cycle.Num() == 0)
			{
				return;
			}

			// Order edges head-to-tail starting from one endpoint of the first edge.
			TArray<int32> Remaining = Cycle;
			int32 CurNode = G.NodeU[Cycle[0]];
			const int32 StartNode = CurNode;

			while (Remaining.Num() > 0)
			{
				int32 PickPos = -1;
				bool bForward = true;
				for (int32 r = 0; r < Remaining.Num(); ++r)
				{
					const int32 e = Remaining[r];
					if (G.NodeU[e] == CurNode) { PickPos = r; bForward = true; break; }
					if (G.NodeV[e] == CurNode) { PickPos = r; bForward = false; break; }
				}
				if (PickPos < 0)
				{
					break; // disconnected (shouldn't happen for a valid cycle)
				}
				const int32 e = Remaining[PickPos];
				Remaining.RemoveAt(PickPos);

				const FStroke& SP = StrokesData[G.StrokeId[e]].Points;
				OutNodes.Add(G.NodePos[CurNode]);
				FCapBoundaryRun Run;
				Run.StrokeId = G.StrokeId[e];
				Run.Color = G.bBlack[e] ? EStrokeColor::Black : EStrokeColor::Red;
				Run.bSynthetic = G.bSynthetic.IsValidIndex(e) && G.bSynthetic[e] != 0;
				Run.bReversed = !bForward;
				Run.StartNodeId = CurNode;
				Run.StartNodePosition = G.NodePos[CurNode];

				if (bForward)
				{
					for (int32 k = 0; k < SP.Num(); ++k)
					{
						if (OutPolygon.Num() == 0 || (OutPolygon.Last() - SP[k]).SizeSquared() > 1e-6)
						{
							OutPolygon.Add(SP[k]);
						}
						if (Run.Points.Num() == 0 || (Run.Points.Last() - SP[k]).SizeSquared() > 1e-6)
						{
							Run.Points.Add(SP[k]);
						}
					}
					CurNode = G.NodeV[e];
				}
				else
				{
					for (int32 k = SP.Num() - 1; k >= 0; --k)
					{
						if (OutPolygon.Num() == 0 || (OutPolygon.Last() - SP[k]).SizeSquared() > 1e-6)
						{
							OutPolygon.Add(SP[k]);
						}
						if (Run.Points.Num() == 0 || (Run.Points.Last() - SP[k]).SizeSquared() > 1e-6)
						{
							Run.Points.Add(SP[k]);
						}
					}
					CurNode = G.NodeU[e];
				}
				Run.EndNodeId = CurNode;
				Run.EndNodePosition = G.NodePos[CurNode];
				Run.ArcLengthPixels = StrokesData[Run.StrokeId].Arc;
				Run.ChordLengthPixels = StrokesData[Run.StrokeId].Chord;
				Run.Straightness = StrokesData[Run.StrokeId].Straightness;
				if (OutRuns)
				{
					OutRuns->Add(MoveTemp(Run));
				}
			}

			// Close the polygon back to the start node.
			if (OutPolygon.Num() > 0)
			{
				OutPolygon.Add(G.NodePos[StartNode]);
			}
		}

		// Render a graph: real red/black edges keep their source colors; synthetic
		// connector edges are orange (red connector) or gray (black connector).
		// Only edges with Kept[e]!=0 are drawn (pass an all-ones array to draw everything).
		bool SaveGraphPng(const TArray<FColoredStroke>& Strokes, const FGraph& G, const TArray<uint8>& Kept,
			int32 Width, int32 Height, const FString& Path)
		{
			TArray<uint8> RGBA;
			RGBA.Init(255, Width * Height * 4);
			auto Plot = [&](int32 x, int32 y, uint8 r, uint8 g, uint8 b)
			{
				if (x < 0 || x >= Width || y < 0 || y >= Height) { return; }
				const int32 Off = (y * Width + x) * 4;
				RGBA[Off + 0] = r; RGBA[Off + 1] = g; RGBA[Off + 2] = b; RGBA[Off + 3] = 255;
			};
			TArray<FIntPoint> LineBuf;
			for (int32 e = 0; e < G.StrokeId.Num(); ++e)
			{
				if (!Kept.IsValidIndex(e) || !Kept[e]) { continue; }
				const bool bSynthetic = G.bSynthetic.IsValidIndex(e) && G.bSynthetic[e] != 0;
				const uint8 r = bSynthetic ? (G.bBlack[e] ? 120 : 255) : (G.bBlack[e] ? 0 : 220);
				const uint8 g = bSynthetic ? (G.bBlack[e] ? 120 : 140) : (G.bBlack[e] ? 0 : 20);
				const uint8 b = bSynthetic ? (G.bBlack[e] ? 120 : 0) : (G.bBlack[e] ? 0 : 60);
				const FStroke& SP = Strokes[G.StrokeId[e]].Points;
				for (int32 k = 0; k + 1 < SP.Num(); ++k)
				{
					SkelGraph::LinePoints(
						FIntPoint(FMath::RoundToInt(SP[k].X), FMath::RoundToInt(SP[k].Y)),
						FIntPoint(FMath::RoundToInt(SP[k + 1].X), FMath::RoundToInt(SP[k + 1].Y)), LineBuf);
					for (const FIntPoint& P : LineBuf) { Plot(P.X, P.Y, r, g, b); }
				}
			}
			for (int32 n = 0; n < G.NumNodes; ++n)
			{
				const int32 nx = FMath::RoundToInt(G.NodePos[n].X);
				const int32 ny = FMath::RoundToInt(G.NodePos[n].Y);
				for (int32 oy = -2; oy <= 2; ++oy)
				{
					for (int32 ox = -2; ox <= 2; ++ox)
					{
						Plot(nx + ox, ny + oy, 30, 80, 220);
					}
				}
			}
			IImageWrapperModule& IWM = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
			TSharedPtr<IImageWrapper> IW = IWM.CreateImageWrapper(EImageFormat::PNG);
			if (!IW.IsValid()) { return false; }
			IW->SetRaw(RGBA.GetData(), RGBA.Num(), Width, Height, ERGBFormat::RGBA, 8);
			const TArray64<uint8>& C = IW->GetCompressed();
			return FFileHelper::SaveArrayToFile(TArrayView<const uint8>(C.GetData(), static_cast<int32>(C.Num())), *Path);
		}

		bool SaveGraphJson(const TArray<FColoredStroke>& Strokes, const FGraph& G, const TArray<uint8>& Kept, const FString& Path)
		{
			FString Json;
			Json += TEXT("{\n  \"nodes\": [\n");
			for (int32 n = 0; n < G.NumNodes; ++n)
			{
				Json += FString::Printf(TEXT("    {\"id\": %d, \"pos\": [%.2f, %.2f]}%s\n"),
					n, G.NodePos[n].X, G.NodePos[n].Y, (n + 1 < G.NumNodes ? TEXT(",") : TEXT("")));
			}
			Json += TEXT("  ],\n  \"edges\": [\n");
			const int32 E = G.StrokeId.Num();
			for (int32 e = 0; e < E; ++e)
			{
				const bool bKept = Kept.IsValidIndex(e) && Kept[e] != 0;
				const bool bSynthetic = G.bSynthetic.IsValidIndex(e) && G.bSynthetic[e] != 0;
				Json += FString::Printf(TEXT("    {\"stroke_id\": %d, \"u\": %d, \"v\": %d, \"color\": \"%s\", \"synthetic\": %s, \"kept\": %s}%s\n"),
					G.StrokeId[e], G.NodeU[e], G.NodeV[e],
					G.bBlack[e] ? TEXT("black") : TEXT("red"),
					bSynthetic ? TEXT("true") : TEXT("false"),
					bKept ? TEXT("true") : TEXT("false"),
					(e + 1 < E ? TEXT(",") : TEXT("")));
			}
			Json += TEXT("  ]\n}\n");
			return FFileHelper::SaveStringToFile(Json, *Path);
		}
	} // namespace CapOps

	// After real intersections and synthetic endpoint connectors are added, merge
	// the blue graph endpoints in 09_all_red_black_graph.png within this radius.
	static constexpr float CapLoopGraphNodeSnapTol = 5.0f;
	static constexpr double CapGeometryEpsilon = 1e-6;
	static constexpr double CapEndpointDirectionArc = 5.0;
	static constexpr double CapBlackContactDistancePx = 2.0;
	static constexpr double CapBlackPreferenceBiasPx = 2.0;
	static constexpr double CapRedOnlyLoopMinBboxArea = 500.0;
	static constexpr double CapBorrowedLoopMinBboxArea = 500.0;
	static constexpr double CapCycleAnchorMinRealRedLengthPx = 20.0;
	static constexpr int32 CapCycleCandidatesPerAnchor = 2;
	static constexpr double CapMixedSelectionTimeBudgetSeconds = 5.0;
	static constexpr double CapInteriorRedMaxLineLengthPx = 20.0;

	struct FCapStrokeSplit
	{
		int32 SegmentIndex = -1;
		double T = 0.0;
		FVector2D Pos = FVector2D::ZeroVector;
	};

	struct FCapSegmentHit
	{
		double T = 0.0;
		double U = 0.0;
		FVector2D Pos = FVector2D::ZeroVector;
	};

	static double Cross2D(const FVector2D& A, const FVector2D& B)
	{
		return A.X * B.Y - A.Y * B.X;
	}

	static void AddSegmentHit(
		TArray<FCapSegmentHit>& OutHits,
		double T,
		double U,
		const FVector2D& Pos)
	{
		for (const FCapSegmentHit& Existing : OutHits)
		{
			if (FVector2D::DistSquared(Existing.Pos, Pos) <= CapGeometryEpsilon * CapGeometryEpsilon)
			{
				return;
			}
		}

		FCapSegmentHit Hit;
		Hit.T = FMath::Clamp(T, 0.0, 1.0);
		Hit.U = FMath::Clamp(U, 0.0, 1.0);
		Hit.Pos = Pos;
		OutHits.Add(Hit);
	}

	static void FindSegmentIntersections(
		const FVector2D& A,
		const FVector2D& B,
		const FVector2D& C,
		const FVector2D& D,
		TArray<FCapSegmentHit>& OutHits)
	{
		OutHits.Reset();
		const FVector2D R = B - A;
		const FVector2D S = D - C;
		const double RLen2 = R.SizeSquared();
		const double SLen2 = S.SizeSquared();
		if (RLen2 <= CapGeometryEpsilon || SLen2 <= CapGeometryEpsilon)
		{
			return;
		}

		const FVector2D CA = C - A;
		const double RCrossS = Cross2D(R, S);
		const double CACrossR = Cross2D(CA, R);
		if (FMath::Abs(RCrossS) > CapGeometryEpsilon)
		{
			const double T = Cross2D(CA, S) / RCrossS;
			const double U = CACrossR / RCrossS;
			if (T >= -CapGeometryEpsilon && T <= 1.0 + CapGeometryEpsilon &&
				U >= -CapGeometryEpsilon && U <= 1.0 + CapGeometryEpsilon)
			{
				const double ClampedT = FMath::Clamp(T, 0.0, 1.0);
				AddSegmentHit(OutHits, ClampedT, U, A + R * ClampedT);
			}
			return;
		}

		if (FMath::Abs(CACrossR) > CapGeometryEpsilon)
		{
			return;
		}

		// Collinear overlap: its boundaries are sufficient to planarize both polylines.
		const double T0 = FVector2D::DotProduct(C - A, R) / RLen2;
		const double T1 = FVector2D::DotProduct(D - A, R) / RLen2;
		const double Lo = FMath::Max(0.0, FMath::Min(T0, T1));
		const double Hi = FMath::Min(1.0, FMath::Max(T0, T1));
		if (Lo > Hi + CapGeometryEpsilon)
		{
			return;
		}

		auto AddCollinearBoundary = [&](double T)
		{
			const FVector2D Pos = A + R * T;
			const double U = FVector2D::DotProduct(Pos - C, S) / SLen2;
			AddSegmentHit(OutHits, T, U, Pos);
		};
		AddCollinearBoundary(Lo);
		if (Hi - Lo > CapGeometryEpsilon)
		{
			AddCollinearBoundary(Hi);
		}
	}

	static void AddStrokeSplit(
		TArray<TArray<FCapStrokeSplit>>& SplitsByStroke,
		int32 StrokeId,
		int32 SegmentIndex,
		double T,
		const FVector2D& Pos)
	{
		if (!SplitsByStroke.IsValidIndex(StrokeId))
		{
			return;
		}
		TArray<FCapStrokeSplit>& Splits = SplitsByStroke[StrokeId];
		for (const FCapStrokeSplit& Existing : Splits)
		{
			if (FVector2D::DistSquared(Existing.Pos, Pos) <= CapGeometryEpsilon * CapGeometryEpsilon)
			{
				return;
			}
		}

		FCapStrokeSplit Split;
		Split.SegmentIndex = SegmentIndex;
		Split.T = FMath::Clamp(T, 0.0, 1.0);
		Split.Pos = Pos;
		Splits.Add(Split);
	}

	static void AppendPlanarStrokeFragments(
		const FColoredStroke& Source,
		const TArray<FCapStrokeSplit>& SourceSplits,
		TArray<FColoredStroke>& OutStrokes)
	{
		if (Source.Points.Num() < 2 || SourceSplits.Num() == 0)
		{
			OutStrokes.Add(Source);
			return;
		}

		TArray<FCapStrokeSplit> Splits = SourceSplits;
		Splits.Sort([](const FCapStrokeSplit& A, const FCapStrokeSplit& B)
		{
			const double AKey = double(A.SegmentIndex) + A.T;
			const double BKey = double(B.SegmentIndex) + B.T;
			return AKey < BKey;
		});

		TArray<TArray<FCapStrokeSplit>> SegmentSplits;
		SegmentSplits.SetNum(Source.Points.Num() - 1);
		for (const FCapStrokeSplit& Split : Splits)
		{
			if (SegmentSplits.IsValidIndex(Split.SegmentIndex))
			{
				SegmentSplits[Split.SegmentIndex].Add(Split);
			}
		}

		auto AppendFragment = [&](const FStroke& Points)
		{
			if (Points.Num() < 2)
			{
				return;
			}
			double Arc = 0.0;
			for (int32 i = 1; i < Points.Num(); ++i)
			{
				Arc += (Points[i] - Points[i - 1]).Size();
			}
			if (Arc <= CapGeometryEpsilon)
			{
				return;
			}

			FColoredStroke Fragment = Source;
			Fragment.Points = Points;
			Fragment.ConnectionPointCount = 0;
			Fragment.bHasMetrics = false;
			OutStrokes.Add(MoveTemp(Fragment));
		};

		FStroke Current;
		Current.Add(Source.Points[0]);
		for (int32 SegmentIndex = 0; SegmentIndex + 1 < Source.Points.Num(); ++SegmentIndex)
		{
			for (const FCapStrokeSplit& Split : SegmentSplits[SegmentIndex])
			{
				if (FVector2D::DistSquared(Current.Last(), Split.Pos) > CapGeometryEpsilon * CapGeometryEpsilon)
				{
					Current.Add(Split.Pos);
				}
				AppendFragment(Current);
				Current.Reset();
				Current.Add(Split.Pos);
			}

			const FVector2D& SegmentEnd = Source.Points[SegmentIndex + 1];
			if (FVector2D::DistSquared(Current.Last(), SegmentEnd) > CapGeometryEpsilon * CapGeometryEpsilon)
			{
				Current.Add(SegmentEnd);
			}
		}
		AppendFragment(Current);
	}

	static FVector2D GetEndpointOutwardDirection(const FStroke& Points, int32 EndpointIndex)
	{
		if (Points.Num() < 2)
		{
			return FVector2D::ZeroVector;
		}

		const bool bStart = EndpointIndex == 0;
		const int32 EndpointPointIndex = bStart ? 0 : Points.Num() - 1;
		const int32 Step = bStart ? 1 : -1;
		FVector2D Interior = Points[EndpointPointIndex];
		double Arc = 0.0;
		for (int32 i = EndpointPointIndex + Step; i >= 0 && i < Points.Num(); i += Step)
		{
			Arc += (Points[i] - Interior).Size();
			Interior = Points[i];
			if (Arc >= CapEndpointDirectionArc)
			{
				break;
			}
		}
		return (Points[EndpointPointIndex] - Interior).GetSafeNormal();
	}

	struct FCapRedDeadEnd
	{
		int32 DeadEndId = INDEX_NONE;
		FVector2D Pos = FVector2D::ZeroVector;
		FVector2D Forward = FVector2D::ZeroVector;
		int32 SourceStrokeId = INDEX_NONE;
		int32 TopologyStrokeId = INDEX_NONE;
		int32 EndpointIndex = 0;
		bool bSynthetic = false;
	};

	struct FCapPlanarStrokeMeta
	{
		int32 SourceStrokeId = INDEX_NONE;
		int32 ConnectorId = INDEX_NONE;
		bool bSynthetic = false;
	};

	struct FCapSegmentCandidate
	{
		bool bValid = false;
		int32 TargetStrokeId = INDEX_NONE;
		int32 TargetSegmentIndex = INDEX_NONE;
		double TargetT = 0.0;
		double Distance = TNumericLimits<double>::Max();
		FVector2D Point = FVector2D::ZeroVector;
	};

	enum class ECapConnectorKind : uint8
	{
		BlackContact,
		RedEndpointPair,
		RedSegmentFallback,
		BlackSegmentFallback,
		BlackEndpointRepair
	};

	struct FCapConnectorProposal
	{
		ECapConnectorKind Kind = ECapConnectorKind::RedSegmentFallback;
		EStrokeColor Color = EStrokeColor::Red;
		int32 SourceDeadEndId = INDEX_NONE;
		int32 TargetDeadEndId = INDEX_NONE;
		int32 TargetStrokeId = INDEX_NONE;
		FVector2D Start = FVector2D::ZeroVector;
		FVector2D End = FVector2D::ZeroVector;
		double Distance = 0.0;
	};

	struct FCapRedTopologyNode
	{
		FVector2D Pos = FVector2D::ZeroVector;
		int32 Degree = 0;
		int32 IncidentStrokeId = INDEX_NONE;
		int32 IncidentEndpointIndex = 0;
	};

	static void CollectPlanarRedDeadEnds(
		const TArray<FColoredStroke>& PlanarStrokes,
		const TArray<FCapPlanarStrokeMeta>& PlanarMeta,
		TArray<FCapRedDeadEnd>& OutDeadEnds)
	{
		OutDeadEnds.Reset();

		TArray<FCapRedTopologyNode> Nodes;
		const double TopologyTol2 = CapGeometryEpsilon * CapGeometryEpsilon;
		for (int32 StrokeId = 0; StrokeId < PlanarStrokes.Num(); ++StrokeId)
		{
			if (PlanarStrokes[StrokeId].Color != EStrokeColor::Red)
			{
				continue;
			}
			const FStroke& Points = PlanarStrokes[StrokeId].Points;
			if (Points.Num() < 2)
			{
				continue;
			}

			for (int32 EndpointIndex = 0; EndpointIndex < 2; ++EndpointIndex)
			{
				const FVector2D Endpoint = EndpointIndex == 0 ? Points[0] : Points.Last();
				int32 NodeId = INDEX_NONE;
				for (int32 CandidateNodeId = 0; CandidateNodeId < Nodes.Num(); ++CandidateNodeId)
				{
					if (FVector2D::DistSquared(Endpoint, Nodes[CandidateNodeId].Pos) <= TopologyTol2)
					{
						NodeId = CandidateNodeId;
						break;
					}
				}

				if (NodeId == INDEX_NONE)
				{
					NodeId = Nodes.AddDefaulted();
					Nodes[NodeId].Pos = Endpoint;
				}

				FCapRedTopologyNode& Node = Nodes[NodeId];
				++Node.Degree;
				Node.IncidentStrokeId = StrokeId;
				Node.IncidentEndpointIndex = EndpointIndex;
			}
		}

		for (const FCapRedTopologyNode& Node : Nodes)
		{
			if (Node.Degree != 1 || !PlanarStrokes.IsValidIndex(Node.IncidentStrokeId))
			{
				continue;
			}

			const FVector2D Forward = GetEndpointOutwardDirection(
				PlanarStrokes[Node.IncidentStrokeId].Points,
				Node.IncidentEndpointIndex);
			if (Forward.IsNearlyZero())
			{
				continue;
			}

			FCapRedDeadEnd DeadEnd;
			DeadEnd.Pos = Node.Pos;
			DeadEnd.Forward = Forward;
			DeadEnd.SourceStrokeId = PlanarMeta.IsValidIndex(Node.IncidentStrokeId) ? PlanarMeta[Node.IncidentStrokeId].SourceStrokeId : INDEX_NONE;
			DeadEnd.TopologyStrokeId = Node.IncidentStrokeId;
			DeadEnd.EndpointIndex = Node.IncidentEndpointIndex;
			DeadEnd.bSynthetic = PlanarMeta.IsValidIndex(Node.IncidentStrokeId) && PlanarMeta[Node.IncidentStrokeId].bSynthetic;
			OutDeadEnds.Add(DeadEnd);
		}

		OutDeadEnds.Sort([](const FCapRedDeadEnd& A, const FCapRedDeadEnd& B)
		{
			if (A.SourceStrokeId != B.SourceStrokeId) { return A.SourceStrokeId < B.SourceStrokeId; }
			if (A.TopologyStrokeId != B.TopologyStrokeId) { return A.TopologyStrokeId < B.TopologyStrokeId; }
			if (A.EndpointIndex != B.EndpointIndex) { return A.EndpointIndex < B.EndpointIndex; }
			if (!FMath::IsNearlyEqual(A.Pos.X, B.Pos.X, CapGeometryEpsilon)) { return A.Pos.X < B.Pos.X; }
			return A.Pos.Y < B.Pos.Y;
		});
		for (int32 DeadEndId = 0; DeadEndId < OutDeadEnds.Num(); ++DeadEndId)
		{
			OutDeadEnds[DeadEndId].DeadEndId = DeadEndId;
		}
	}

	static bool ClosestPointOnSegmentInForwardHalfPlane(
		const FVector2D& Endpoint,
		const FVector2D& Forward,
		const FVector2D& A,
		const FVector2D& B,
		double& OutT,
		FVector2D& OutPoint)
	{
		const FVector2D Segment = B - A;
		const double SegmentLen2 = Segment.SizeSquared();
		if (SegmentLen2 <= CapGeometryEpsilon)
		{
			return false;
		}

		const double DA = FVector2D::DotProduct(A - Endpoint, Forward);
		const double DB = FVector2D::DotProduct(B - Endpoint, Forward);
		if (DA < -CapGeometryEpsilon && DB < -CapGeometryEpsilon)
		{
			return false;
		}

		double MinT = 0.0;
		double MaxT = 1.0;
		if (DA < -CapGeometryEpsilon)
		{
			MinT = FMath::Clamp(-DA / (DB - DA), 0.0, 1.0);
		}
		else if (DB < -CapGeometryEpsilon)
		{
			MaxT = FMath::Clamp(-DA / (DB - DA), 0.0, 1.0);
		}
		if (MinT > MaxT + CapGeometryEpsilon)
		{
			return false;
		}

		const double ProjectionT = FVector2D::DotProduct(Endpoint - A, Segment) / SegmentLen2;
		OutT = FMath::Clamp(ProjectionT, MinT, MaxT);
		OutPoint = A + Segment * OutT;
		return FVector2D::DotProduct(OutPoint - Endpoint, Forward) >= -CapGeometryEpsilon;
	}

	static FString RoundedConnectorKey(const FVector2D& A, const FVector2D& B)
	{
		const int32 Ax = FMath::RoundToInt(A.X);
		const int32 Ay = FMath::RoundToInt(A.Y);
		const int32 Bx = FMath::RoundToInt(B.X);
		const int32 By = FMath::RoundToInt(B.Y);
		return (Ax < Bx || (Ax == Bx && Ay <= By))
			? FString::Printf(TEXT("%d,%d-%d,%d"), Ax, Ay, Bx, By)
			: FString::Printf(TEXT("%d,%d-%d,%d"), Bx, By, Ax, Ay);
	}

	static bool IsCapTopologyColor(EStrokeColor Color)
	{
		return Color == EStrokeColor::Red || Color == EStrokeColor::Black;
	}

	static void BuildRepairPlanarGeometry(
		const TArray<FColoredStroke>& SourceStrokes,
		const TArray<FColoredStroke>& SyntheticConnectors,
		TArray<FColoredStroke>& OutPlanarStrokes,
		TArray<FCapPlanarStrokeMeta>& OutPlanarMeta,
		int32& OutFirstSyntheticStrokeId,
		int32* OutRealRedBlackIntersectionCount = nullptr,
		int32* OutRealRedRedIntersectionCount = nullptr)
	{
		TArray<FColoredStroke> Combined = SourceStrokes;
		Combined.Append(SyntheticConnectors);
		TArray<TArray<FCapStrokeSplit>> SplitsByStroke;
		SplitsByStroke.SetNum(Combined.Num());
		int32 RealRedBlackIntersectionCount = 0;
		int32 RealRedRedIntersectionCount = 0;
		TArray<FCapSegmentHit> Hits;

		for (int32 AId = 0; AId < Combined.Num(); ++AId)
		{
			if (!IsCapTopologyColor(Combined[AId].Color))
			{
				continue;
			}
			const bool bASynthetic = AId >= SourceStrokes.Num();
			const FStroke& APoints = Combined[AId].Points;
			for (int32 BId = AId + 1; BId < Combined.Num(); ++BId)
			{
				if (!IsCapTopologyColor(Combined[BId].Color))
				{
					continue;
				}
				const bool bBSynthetic = BId >= SourceStrokes.Num();
				const bool bRealRedBlack =
					!bASynthetic &&
					!bBSynthetic &&
					Combined[AId].Color != Combined[BId].Color;
				const bool bRealRedRed =
					!bASynthetic &&
					!bBSynthetic &&
					Combined[AId].Color == EStrokeColor::Red &&
					Combined[BId].Color == EStrokeColor::Red;
				if (!bASynthetic && !bBSynthetic && !bRealRedBlack && !bRealRedRed)
				{
					continue;
				}

				const FStroke& BPoints = Combined[BId].Points;
				for (int32 ASegment = 0; ASegment + 1 < APoints.Num(); ++ASegment)
				{
					for (int32 BSegment = 0; BSegment + 1 < BPoints.Num(); ++BSegment)
					{
						FindSegmentIntersections(
							APoints[ASegment], APoints[ASegment + 1],
							BPoints[BSegment], BPoints[BSegment + 1],
							Hits);
						for (const FCapSegmentHit& Hit : Hits)
						{
							const int32 BeforeA = SplitsByStroke[AId].Num();
							AddStrokeSplit(SplitsByStroke, AId, ASegment, Hit.T, Hit.Pos);
							AddStrokeSplit(SplitsByStroke, BId, BSegment, Hit.U, Hit.Pos);
							if (bRealRedBlack && SplitsByStroke[AId].Num() > BeforeA)
							{
								++RealRedBlackIntersectionCount;
							}
							else if (bRealRedRed && SplitsByStroke[AId].Num() > BeforeA)
							{
								++RealRedRedIntersectionCount;
							}
						}
					}
				}
			}
		}

		OutPlanarStrokes.Reset();
		OutPlanarMeta.Reset();
		auto AppendStroke = [&](
			const FColoredStroke& Stroke,
			const TArray<FCapStrokeSplit>& Splits,
			int32 SourceStrokeId,
			int32 ConnectorId,
			bool bSynthetic)
		{
			const int32 Before = OutPlanarStrokes.Num();
			AppendPlanarStrokeFragments(Stroke, Splits, OutPlanarStrokes);
			for (int32 FragmentId = Before; FragmentId < OutPlanarStrokes.Num(); ++FragmentId)
			{
				FCapPlanarStrokeMeta Meta;
				Meta.SourceStrokeId = SourceStrokeId;
				Meta.ConnectorId = ConnectorId;
				Meta.bSynthetic = bSynthetic;
				OutPlanarMeta.Add(Meta);
			}
		};

		for (int32 SourceStrokeId = 0; SourceStrokeId < SourceStrokes.Num(); ++SourceStrokeId)
		{
			AppendStroke(
				SourceStrokes[SourceStrokeId],
				SplitsByStroke[SourceStrokeId],
				SourceStrokeId,
				INDEX_NONE,
				false);
		}
		OutFirstSyntheticStrokeId = OutPlanarStrokes.Num();
		for (int32 ConnectorId = 0; ConnectorId < SyntheticConnectors.Num(); ++ConnectorId)
		{
			const int32 CombinedId = SourceStrokes.Num() + ConnectorId;
			AppendStroke(
				SyntheticConnectors[ConnectorId],
				SplitsByStroke[CombinedId],
				INDEX_NONE,
				ConnectorId,
				true);
		}

		if (OutRealRedBlackIntersectionCount)
		{
			*OutRealRedBlackIntersectionCount = RealRedBlackIntersectionCount;
		}
		if (OutRealRedRedIntersectionCount)
		{
			*OutRealRedRedIntersectionCount = RealRedRedIntersectionCount;
		}
	}

	static bool ConnectorCrossesRealGeometry(
		const FVector2D& Start,
		const FVector2D& End,
		const TArray<FColoredStroke>& SourceStrokes)
	{
		if (FVector2D::DistSquared(Start, End) <= CapGeometryEpsilon * CapGeometryEpsilon)
		{
			return false;
		}

		TArray<FCapSegmentHit> Hits;
		static constexpr double ParamEpsilon = 1e-6;
		for (const FColoredStroke& Stroke : SourceStrokes)
		{
			if (!IsCapTopologyColor(Stroke.Color))
			{
				continue;
			}
			for (int32 SegmentIndex = 0; SegmentIndex + 1 < Stroke.Points.Num(); ++SegmentIndex)
			{
				FindSegmentIntersections(
					Start,
					End,
					Stroke.Points[SegmentIndex],
					Stroke.Points[SegmentIndex + 1],
					Hits);
				for (const FCapSegmentHit& Hit : Hits)
				{
					if (Hit.T > ParamEpsilon && Hit.T < 1.0 - ParamEpsilon)
					{
						return true;
					}
				}
			}
		}
		return false;
	}

	static bool ConnectorIntersectsAcceptedConnector(
		const FVector2D& Start,
		const FVector2D& End,
		const TArray<FColoredStroke>& AcceptedConnectors)
	{
		TArray<FCapSegmentHit> Hits;
		static constexpr double ParamEpsilon = 1e-6;
		for (const FColoredStroke& Existing : AcceptedConnectors)
		{
			if (Existing.Points.Num() < 2)
			{
				continue;
			}
			FindSegmentIntersections(
				Start,
				End,
				Existing.Points[0],
				Existing.Points.Last(),
				Hits);
			for (const FCapSegmentHit& Hit : Hits)
			{
				const bool bProposalEndpoint = Hit.T <= ParamEpsilon || Hit.T >= 1.0 - ParamEpsilon;
				const bool bExistingEndpoint = Hit.U <= ParamEpsilon || Hit.U >= 1.0 - ParamEpsilon;
				if (!bProposalEndpoint || !bExistingEndpoint)
				{
					return true;
				}
			}
		}
		return false;
	}

	static bool IsStrictInteriorTarget(const FStroke& Points, int32 SegmentIndex, double T)
	{
		static constexpr double ParamEpsilon = 1e-6;
		if (Points.Num() < 2)
		{
			return false;
		}
		if (SegmentIndex == 0 && T <= ParamEpsilon)
		{
			return false;
		}
		if (SegmentIndex + 1 == Points.Num() - 1 && T >= 1.0 - ParamEpsilon)
		{
			return false;
		}
		return true;
	}

	static FCapSegmentCandidate FindBestForwardSegmentCandidate(
		const TArray<FColoredStroke>& SourceStrokes,
		const TArray<int32>& TargetStrokeIds,
		const FCapRedDeadEnd& DeadEnd,
		double MaxDistance,
		bool bStrictInteriorTarget,
		bool bAllowExactContactWithoutForward,
		const TArray<FColoredStroke>& ExistingConnectors)
	{
		const double MaxDistance2 = MaxDistance * MaxDistance;
		const double GeometryEpsilon2 = CapGeometryEpsilon * CapGeometryEpsilon;
		FCapSegmentCandidate Best;

		for (int32 TargetStrokeId : TargetStrokeIds)
		{
			if (!SourceStrokes.IsValidIndex(TargetStrokeId) ||
				TargetStrokeId == DeadEnd.SourceStrokeId)
			{
				continue;
			}
			const FStroke& Points = SourceStrokes[TargetStrokeId].Points;
			for (int32 SegmentIndex = 0; SegmentIndex + 1 < Points.Num(); ++SegmentIndex)
			{
				const FVector2D Segment = Points[SegmentIndex + 1] - Points[SegmentIndex];
				const double SegmentLen2 = Segment.SizeSquared();
				if (SegmentLen2 <= CapGeometryEpsilon)
				{
					continue;
				}

				double T = FMath::Clamp(
					FVector2D::DotProduct(DeadEnd.Pos - Points[SegmentIndex], Segment) / SegmentLen2,
					0.0,
					1.0);
				FVector2D Point = Points[SegmentIndex] + Segment * T;
				double D2 = FVector2D::DistSquared(DeadEnd.Pos, Point);
				if (!(bAllowExactContactWithoutForward && D2 <= GeometryEpsilon2))
				{
					if (!ClosestPointOnSegmentInForwardHalfPlane(
						DeadEnd.Pos,
						DeadEnd.Forward,
						Points[SegmentIndex],
						Points[SegmentIndex + 1],
						T,
						Point))
					{
						continue;
					}
					D2 = FVector2D::DistSquared(DeadEnd.Pos, Point);
				}

				if (D2 > MaxDistance2 ||
					(!bAllowExactContactWithoutForward && D2 <= GeometryEpsilon2) ||
					(bStrictInteriorTarget && !IsStrictInteriorTarget(Points, SegmentIndex, T)))
				{
					continue;
				}
				if (D2 > GeometryEpsilon2 &&
					(ConnectorCrossesRealGeometry(DeadEnd.Pos, Point, SourceStrokes) ||
					 ConnectorIntersectsAcceptedConnector(DeadEnd.Pos, Point, ExistingConnectors)))
				{
					continue;
				}

				const double Distance = FMath::Sqrt(D2);
				const bool bBetter =
					!Best.bValid ||
					Distance < Best.Distance - CapGeometryEpsilon ||
					(FMath::IsNearlyEqual(Distance, Best.Distance, CapGeometryEpsilon) &&
						(TargetStrokeId < Best.TargetStrokeId ||
							(TargetStrokeId == Best.TargetStrokeId &&
								(SegmentIndex < Best.TargetSegmentIndex ||
									(SegmentIndex == Best.TargetSegmentIndex && T < Best.TargetT)))));
				if (!bBetter)
				{
					continue;
				}

				Best.bValid = true;
				Best.TargetStrokeId = TargetStrokeId;
				Best.TargetSegmentIndex = SegmentIndex;
				Best.TargetT = T;
				Best.Distance = Distance;
				Best.Point = Point;
			}
		}
		return Best;
	}

	static bool ConnectorProposalLess(
		const FCapConnectorProposal& A,
		const FCapConnectorProposal& B)
	{
		if (!FMath::IsNearlyEqual(A.Distance, B.Distance, CapGeometryEpsilon))
		{
			return A.Distance < B.Distance;
		}
		if (A.SourceDeadEndId != B.SourceDeadEndId)
		{
			return A.SourceDeadEndId < B.SourceDeadEndId;
		}
		if (A.TargetDeadEndId != B.TargetDeadEndId)
		{
			return A.TargetDeadEndId < B.TargetDeadEndId;
		}
		if (A.TargetStrokeId != B.TargetStrokeId)
		{
			return A.TargetStrokeId < B.TargetStrokeId;
		}
		return static_cast<uint8>(A.Kind) < static_cast<uint8>(B.Kind);
	}

	static bool TryAppendConnectorProposal(
		const FCapConnectorProposal& Proposal,
		const TArray<FColoredStroke>& SourceStrokes,
		TArray<FColoredStroke>& AcceptedConnectors,
		TSet<FString>& ConnectorKeys)
	{
		if (Proposal.Distance <= CapGeometryEpsilon)
		{
			return true;
		}
		const FString Key = RoundedConnectorKey(Proposal.Start, Proposal.End);
		if (ConnectorKeys.Contains(Key) ||
			ConnectorCrossesRealGeometry(Proposal.Start, Proposal.End, SourceStrokes) ||
			ConnectorIntersectsAcceptedConnector(Proposal.Start, Proposal.End, AcceptedConnectors))
		{
			return false;
		}

		FColoredStroke Connector;
		Connector.Points.Add(Proposal.Start);
		Connector.Points.Add(Proposal.End);
		Connector.Color = Proposal.Color;
		Connector.ConnectionPointCount = Connector.Points.Num();
		AcceptedConnectors.Add(MoveTemp(Connector));
		ConnectorKeys.Add(Key);
		return true;
	}

	static void CollectUnresolvedInitialRedDeadEnds(
		const TArray<FColoredStroke>& PlanarStrokes,
		int32 FirstSyntheticStrokeId,
		const TArray<FCapRedDeadEnd>& InitialDeadEnds,
		TArray<FCapRedDeadEnd>& OutUnresolved)
	{
		using namespace CapOps;
		OutUnresolved.Reset();
		TArray<int32> EdgeIds;
		for (int32 StrokeId = 0; StrokeId < PlanarStrokes.Num(); ++StrokeId)
		{
			if (IsCapTopologyColor(PlanarStrokes[StrokeId].Color))
			{
				EdgeIds.Add(StrokeId);
			}
		}

		FGraph G;
		BuildGraph(PlanarStrokes, EdgeIds, static_cast<float>(CapGeometryEpsilon), FirstSyntheticStrokeId, G);
		TArray<int32> Degree;
		Degree.Init(0, G.NumNodes);
		for (int32 EdgeId = 0; EdgeId < G.StrokeId.Num(); ++EdgeId)
		{
			if (Degree.IsValidIndex(G.NodeU[EdgeId])) { ++Degree[G.NodeU[EdgeId]]; }
			if (Degree.IsValidIndex(G.NodeV[EdgeId])) { ++Degree[G.NodeV[EdgeId]]; }
		}

		const double Tol2 = CapGeometryEpsilon * CapGeometryEpsilon;
		for (const FCapRedDeadEnd& DeadEnd : InitialDeadEnds)
		{
			int32 NodeId = INDEX_NONE;
			for (int32 CandidateNodeId = 0; CandidateNodeId < G.NodePos.Num(); ++CandidateNodeId)
			{
				if (FVector2D::DistSquared(DeadEnd.Pos, G.NodePos[CandidateNodeId]) <= Tol2)
				{
					NodeId = CandidateNodeId;
					break;
				}
			}
			if (!Degree.IsValidIndex(NodeId) || Degree[NodeId] <= 1)
			{
				OutUnresolved.Add(DeadEnd);
			}
		}
	}

	static void BuildPlanarEndpointConnectorStrokes(
		const TArray<FColoredStroke>& SourceStrokes,
		const TArray<int32>& RedIdx,
		const TArray<int32>& BlackIdx,
		float ConnectorTol,
		TArray<FColoredStroke>& OutStrokes,
		int32& OutFirstSyntheticStrokeId)
	{
		TArray<FColoredStroke> SyntheticConnectors;
		TSet<FString> ConnectorKeys;

		TArray<FColoredStroke> InitialPlanarStrokes;
		TArray<FCapPlanarStrokeMeta> InitialPlanarMeta;
		int32 InitialFirstSyntheticStrokeId = INDEX_NONE;
		int32 RealRedBlackIntersectionCount = 0;
		int32 RealRedRedIntersectionCount = 0;
		BuildRepairPlanarGeometry(
			SourceStrokes,
			SyntheticConnectors,
			InitialPlanarStrokes,
			InitialPlanarMeta,
			InitialFirstSyntheticStrokeId,
			&RealRedBlackIntersectionCount,
			&RealRedRedIntersectionCount);

		TArray<FCapRedDeadEnd> InitialRedDeadEnds;
		CollectPlanarRedDeadEnds(InitialPlanarStrokes, InitialPlanarMeta, InitialRedDeadEnds);
		InitialRedDeadEnds.RemoveAll([](const FCapRedDeadEnd& DeadEnd)
		{
			return DeadEnd.bSynthetic;
		});
		for (int32 DeadEndId = 0; DeadEndId < InitialRedDeadEnds.Num(); ++DeadEndId)
		{
			InitialRedDeadEnds[DeadEndId].DeadEndId = DeadEndId;
		}
		const int32 InitialRedDeadEndCount = InitialRedDeadEnds.Num();
		const double ConnectorMaxDistance = FMath::Max(0.0, static_cast<double>(ConnectorTol));

		// Stage A. Exact/near black contacts are resolved first. A zero-distance
		// contact needs no connector because real red/black planarization already
		// created the shared node.
		TArray<uint8> bStageAResolved;
		bStageAResolved.Init(0, InitialRedDeadEnds.Num());
		TArray<FCapConnectorProposal> BlackContactProposals;
		int32 ExactBlackContactCount = 0;
		for (const FCapRedDeadEnd& DeadEnd : InitialRedDeadEnds)
		{
			const FCapSegmentCandidate Contact = FindBestForwardSegmentCandidate(
				SourceStrokes,
				BlackIdx,
				DeadEnd,
				CapBlackContactDistancePx,
				false,
				true,
				SyntheticConnectors);
			if (!Contact.bValid)
			{
				continue;
			}
			if (Contact.Distance <= CapGeometryEpsilon)
			{
				bStageAResolved[DeadEnd.DeadEndId] = 1;
				++ExactBlackContactCount;
				continue;
			}

			FCapConnectorProposal Proposal;
			Proposal.Kind = ECapConnectorKind::BlackContact;
			Proposal.Color = EStrokeColor::Red;
			Proposal.SourceDeadEndId = DeadEnd.DeadEndId;
			Proposal.TargetStrokeId = Contact.TargetStrokeId;
			Proposal.Start = DeadEnd.Pos;
			Proposal.End = Contact.Point;
			Proposal.Distance = Contact.Distance;
			BlackContactProposals.Add(Proposal);
		}
		BlackContactProposals.Sort(ConnectorProposalLess);
		int32 BlackContactConnectorCount = 0;
		for (const FCapConnectorProposal& Proposal : BlackContactProposals)
		{
			if (!bStageAResolved.IsValidIndex(Proposal.SourceDeadEndId) ||
				bStageAResolved[Proposal.SourceDeadEndId])
			{
				continue;
			}
			if (TryAppendConnectorProposal(Proposal, SourceStrokes, SyntheticConnectors, ConnectorKeys))
			{
				bStageAResolved[Proposal.SourceDeadEndId] = 1;
				++BlackContactConnectorCount;
			}
		}

		// Every remaining endpoint independently records its nearest viable black
		// segment. That distance can veto an endpoint pair only when black is
		// clearly closer by the configured bias.
		TArray<FCapSegmentCandidate> BestBlackByDeadEnd;
		BestBlackByDeadEnd.SetNum(InitialRedDeadEnds.Num());
		for (const FCapRedDeadEnd& DeadEnd : InitialRedDeadEnds)
		{
			if (bStageAResolved[DeadEnd.DeadEndId])
			{
				continue;
			}
			BestBlackByDeadEnd[DeadEnd.DeadEndId] = FindBestForwardSegmentCandidate(
				SourceStrokes,
				BlackIdx,
				DeadEnd,
				ConnectorMaxDistance,
				false,
				false,
				SyntheticConnectors);
		}

		TArray<FCapConnectorProposal> EndpointPairProposals;
		for (int32 AId = 0; AId < InitialRedDeadEnds.Num(); ++AId)
		{
			if (bStageAResolved[AId])
			{
				continue;
			}
			const FCapRedDeadEnd& A = InitialRedDeadEnds[AId];
			for (int32 BId = AId + 1; BId < InitialRedDeadEnds.Num(); ++BId)
			{
				if (bStageAResolved[BId])
				{
					continue;
				}
				const FCapRedDeadEnd& B = InitialRedDeadEnds[BId];
				const FVector2D Delta = B.Pos - A.Pos;
				const double Distance = Delta.Size();
				if (Distance <= CapGeometryEpsilon || Distance > ConnectorMaxDistance)
				{
					continue;
				}
				const bool bASeesB = FVector2D::DotProduct(Delta, A.Forward) >= -CapGeometryEpsilon;
				const bool bBSeesA = FVector2D::DotProduct(-Delta, B.Forward) >= -CapGeometryEpsilon;
				if (!bASeesB && !bBSeesA)
				{
					continue;
				}
				const FCapSegmentCandidate& ABlack = BestBlackByDeadEnd[AId];
				const FCapSegmentCandidate& BBlack = BestBlackByDeadEnd[BId];
				if ((ABlack.bValid && ABlack.Distance + CapBlackPreferenceBiasPx < Distance) ||
					(BBlack.bValid && BBlack.Distance + CapBlackPreferenceBiasPx < Distance))
				{
					continue;
				}
				if (ConnectorCrossesRealGeometry(A.Pos, B.Pos, SourceStrokes) ||
					ConnectorIntersectsAcceptedConnector(A.Pos, B.Pos, SyntheticConnectors))
				{
					continue;
				}
				FCapConnectorProposal Proposal;
				Proposal.Kind = ECapConnectorKind::RedEndpointPair;
				Proposal.Color = EStrokeColor::Red;
				Proposal.SourceDeadEndId = AId;
				Proposal.TargetDeadEndId = BId;
				Proposal.Start = A.Pos;
				Proposal.End = B.Pos;
				Proposal.Distance = Distance;
				EndpointPairProposals.Add(Proposal);
			}
		}
		EndpointPairProposals.Sort(ConnectorProposalLess);
		TArray<uint8> bEndpointOccupied = bStageAResolved;
		int32 RedEndpointPairConnectorCount = 0;
		for (const FCapConnectorProposal& Proposal : EndpointPairProposals)
		{
			if (bEndpointOccupied[Proposal.SourceDeadEndId] ||
				bEndpointOccupied[Proposal.TargetDeadEndId])
			{
				continue;
			}
			if (TryAppendConnectorProposal(Proposal, SourceStrokes, SyntheticConnectors, ConnectorKeys))
			{
				bEndpointOccupied[Proposal.SourceDeadEndId] = 1;
				bEndpointOccupied[Proposal.TargetDeadEndId] = 1;
				++RedEndpointPairConnectorCount;
			}
		}

		// Commit Stage A and use an exact mixed red/black graph to identify which
		// original red dead ends still have only one incident edge.
		TArray<FColoredStroke> StageAPlanarStrokes;
		TArray<FCapPlanarStrokeMeta> StageAPlanarMeta;
		int32 StageAFirstSyntheticStrokeId = INDEX_NONE;
		BuildRepairPlanarGeometry(
			SourceStrokes,
			SyntheticConnectors,
			StageAPlanarStrokes,
			StageAPlanarMeta,
			StageAFirstSyntheticStrokeId);

		TArray<FCapRedDeadEnd> UnresolvedAfterStageA;
		CollectUnresolvedInitialRedDeadEnds(
			StageAPlanarStrokes,
			StageAFirstSyntheticStrokeId,
			InitialRedDeadEnds,
			UnresolvedAfterStageA);

		// Stage B. Red-segment and black-segment searches are independent and use
		// the same Stage A snapshot. Synthetic red connectors participate in the
		// temporary graph, but are never active red-segment targets.
		TArray<FCapConnectorProposal> FallbackProposals;
		for (const FCapRedDeadEnd& DeadEnd : UnresolvedAfterStageA)
		{
			const FCapSegmentCandidate RedCandidate = FindBestForwardSegmentCandidate(
				SourceStrokes,
				RedIdx,
				DeadEnd,
				ConnectorMaxDistance,
				true,
				false,
				SyntheticConnectors);
			const FCapSegmentCandidate BlackCandidate = FindBestForwardSegmentCandidate(
				SourceStrokes,
				BlackIdx,
				DeadEnd,
				ConnectorMaxDistance,
				false,
				false,
				SyntheticConnectors);

			const FCapSegmentCandidate* Selected = nullptr;
			ECapConnectorKind Kind = ECapConnectorKind::RedSegmentFallback;
			if (RedCandidate.bValid && BlackCandidate.bValid)
			{
				if (BlackCandidate.Distance + CapBlackPreferenceBiasPx < RedCandidate.Distance)
				{
					Selected = &BlackCandidate;
					Kind = ECapConnectorKind::BlackSegmentFallback;
				}
				else
				{
					Selected = &RedCandidate;
				}
			}
			else if (RedCandidate.bValid)
			{
				Selected = &RedCandidate;
			}
			else if (BlackCandidate.bValid)
			{
				Selected = &BlackCandidate;
				Kind = ECapConnectorKind::BlackSegmentFallback;
			}
			if (!Selected)
			{
				continue;
			}

			FCapConnectorProposal Proposal;
			Proposal.Kind = Kind;
			Proposal.Color = EStrokeColor::Red;
			Proposal.SourceDeadEndId = DeadEnd.DeadEndId;
			Proposal.TargetStrokeId = Selected->TargetStrokeId;
			Proposal.Start = DeadEnd.Pos;
			Proposal.End = Selected->Point;
			Proposal.Distance = Selected->Distance;
			FallbackProposals.Add(Proposal);
		}

		FallbackProposals.Sort(ConnectorProposalLess);
		int32 RedSegmentConnectorCount = 0;
		int32 BlackSegmentConnectorCount = 0;
		for (const FCapConnectorProposal& Proposal : FallbackProposals)
		{
			if (!TryAppendConnectorProposal(Proposal, SourceStrokes, SyntheticConnectors, ConnectorKeys))
			{
				continue;
			}
			if (Proposal.Kind == ECapConnectorKind::BlackSegmentFallback)
			{
				++BlackSegmentConnectorCount;
			}
			else
			{
				++RedSegmentConnectorCount;
			}
		}

		TArray<FColoredStroke> StageBPlanarStrokes;
		TArray<FCapPlanarStrokeMeta> StageBPlanarMeta;
		int32 StageBFirstSyntheticStrokeId = INDEX_NONE;
		BuildRepairPlanarGeometry(
			SourceStrokes,
			SyntheticConnectors,
			StageBPlanarStrokes,
			StageBPlanarMeta,
			StageBFirstSyntheticStrokeId);

		// Preserve the existing black endpoint repair pass, but run it against the
		// topology produced by both red-repair stages and commit it as one batch.
		struct FEndpointRef
		{
			FVector2D Pos = FVector2D::ZeroVector;
			EStrokeColor Color = EStrokeColor::None;
			int32 SourceStrokeId = INDEX_NONE;
			int32 LocalStrokeId = INDEX_NONE;
			int32 EndpointIndex = 0;
			int32 NodeId = INDEX_NONE;
			bool bSynthetic = false;
		};

		TArray<int32> StageBEdgeIds;
		for (int32 StrokeId = 0; StrokeId < StageBPlanarStrokes.Num(); ++StrokeId)
		{
			if (IsCapTopologyColor(StageBPlanarStrokes[StrokeId].Color))
			{
				StageBEdgeIds.Add(StrokeId);
			}
		}
		CapOps::FGraph StageBGraph;
		CapOps::BuildGraph(
			StageBPlanarStrokes,
			StageBEdgeIds,
			static_cast<float>(CapGeometryEpsilon),
			StageBFirstSyntheticStrokeId,
			StageBGraph);
		TArray<int32> StageBDegree;
		StageBDegree.Init(0, StageBGraph.NumNodes);
		for (int32 EdgeId = 0; EdgeId < StageBGraph.StrokeId.Num(); ++EdgeId)
		{
			if (StageBDegree.IsValidIndex(StageBGraph.NodeU[EdgeId])) { ++StageBDegree[StageBGraph.NodeU[EdgeId]]; }
			if (StageBDegree.IsValidIndex(StageBGraph.NodeV[EdgeId])) { ++StageBDegree[StageBGraph.NodeV[EdgeId]]; }
		}

		TArray<FEndpointRef> EndpointRefs;
		const double ExactTol2 = CapGeometryEpsilon * CapGeometryEpsilon;
		for (int32 StrokeId = 0; StrokeId < StageBPlanarStrokes.Num(); ++StrokeId)
		{
			const FColoredStroke& Stroke = StageBPlanarStrokes[StrokeId];
			if (!IsCapTopologyColor(Stroke.Color) || Stroke.Points.Num() < 2)
			{
				continue;
			}
			for (int32 EndpointIndex = 0; EndpointIndex < 2; ++EndpointIndex)
			{
				FEndpointRef Ref;
				Ref.Pos = EndpointIndex == 0 ? Stroke.Points[0] : Stroke.Points.Last();
				Ref.Color = Stroke.Color;
				Ref.LocalStrokeId = StrokeId;
				Ref.EndpointIndex = EndpointIndex;
				if (StageBPlanarMeta.IsValidIndex(StrokeId))
				{
					Ref.SourceStrokeId = StageBPlanarMeta[StrokeId].SourceStrokeId;
					Ref.bSynthetic = StageBPlanarMeta[StrokeId].bSynthetic;
				}
				for (int32 NodeId = 0; NodeId < StageBGraph.NodePos.Num(); ++NodeId)
				{
					if (FVector2D::DistSquared(Ref.Pos, StageBGraph.NodePos[NodeId]) <= ExactTol2)
					{
						Ref.NodeId = NodeId;
						break;
					}
				}
				EndpointRefs.Add(Ref);
			}
		}

		TArray<FCapConnectorProposal> BlackEndpointProposals;
		int32 BlackDeadEndCount = 0;
		int32 BlackDeadEndId = 0;
		const double ConnectorMaxDistance2 = ConnectorMaxDistance * ConnectorMaxDistance;
		for (const FEndpointRef& DeadEnd : EndpointRefs)
		{
			if (DeadEnd.Color != EStrokeColor::Black ||
				DeadEnd.bSynthetic ||
				!StageBDegree.IsValidIndex(DeadEnd.NodeId) ||
				StageBDegree[DeadEnd.NodeId] != 1 ||
				!StageBPlanarStrokes.IsValidIndex(DeadEnd.LocalStrokeId))
			{
				continue;
			}
			++BlackDeadEndCount;
			const FVector2D Forward = GetEndpointOutwardDirection(
				StageBPlanarStrokes[DeadEnd.LocalStrokeId].Points,
				DeadEnd.EndpointIndex);
			if (Forward.IsNearlyZero())
			{
				++BlackDeadEndId;
				continue;
			}

			int32 BestTargetRefId = INDEX_NONE;
			double BestD2 = TNumericLimits<double>::Max();
			for (int32 CandidateRefId = 0; CandidateRefId < EndpointRefs.Num(); ++CandidateRefId)
			{
				const FEndpointRef& Candidate = EndpointRefs[CandidateRefId];
				if (!IsCapTopologyColor(Candidate.Color) ||
					(!Candidate.bSynthetic && Candidate.SourceStrokeId == DeadEnd.SourceStrokeId))
				{
					continue;
				}
				const FVector2D Delta = Candidate.Pos - DeadEnd.Pos;
				const double D2 = Delta.SizeSquared();
				if (D2 <= ExactTol2 ||
					D2 > ConnectorMaxDistance2 ||
					FVector2D::DotProduct(Delta, Forward) < -CapGeometryEpsilon ||
					ConnectorCrossesRealGeometry(DeadEnd.Pos, Candidate.Pos, SourceStrokes) ||
					ConnectorIntersectsAcceptedConnector(DeadEnd.Pos, Candidate.Pos, SyntheticConnectors))
				{
					continue;
				}
				if (D2 < BestD2 - CapGeometryEpsilon ||
					(FMath::IsNearlyEqual(D2, BestD2, CapGeometryEpsilon) && CandidateRefId < BestTargetRefId))
				{
					BestD2 = D2;
					BestTargetRefId = CandidateRefId;
				}
			}
			if (EndpointRefs.IsValidIndex(BestTargetRefId))
			{
				FCapConnectorProposal Proposal;
				Proposal.Kind = ECapConnectorKind::BlackEndpointRepair;
				Proposal.Color = EStrokeColor::Black;
				Proposal.SourceDeadEndId = BlackDeadEndId;
				Proposal.TargetDeadEndId = BestTargetRefId;
				Proposal.Start = DeadEnd.Pos;
				Proposal.End = EndpointRefs[BestTargetRefId].Pos;
				Proposal.Distance = FMath::Sqrt(BestD2);
				BlackEndpointProposals.Add(Proposal);
			}
			++BlackDeadEndId;
		}

		BlackEndpointProposals.Sort(ConnectorProposalLess);
		int32 BlackEndpointConnectorCount = 0;
		for (const FCapConnectorProposal& Proposal : BlackEndpointProposals)
		{
			if (TryAppendConnectorProposal(Proposal, SourceStrokes, SyntheticConnectors, ConnectorKeys))
			{
				++BlackEndpointConnectorCount;
			}
		}

		TArray<FCapPlanarStrokeMeta> FinalPlanarMeta;
		BuildRepairPlanarGeometry(
			SourceStrokes,
			SyntheticConnectors,
			OutStrokes,
			FinalPlanarMeta,
			OutFirstSyntheticStrokeId);
		ComputeStrokeMetrics(OutStrokes);

		UE_LOG(LogTemp, Log,
			TEXT("Sketch2D Step 9 topology: %d real red/red intersection(s), %d real red/black intersection(s), %d initial red dead-end(s), %d exact black contact(s), %d black-contact connector(s), %d endpoint-pair connector(s), %d unresolved after Stage A, %d red-segment connector(s), %d black-segment connector(s), %d black dead-end(s), %d black endpoint connector(s), %d planar stroke fragment(s)"),
			RealRedRedIntersectionCount,
			RealRedBlackIntersectionCount,
			InitialRedDeadEndCount,
			ExactBlackContactCount,
			BlackContactConnectorCount,
			RedEndpointPairConnectorCount,
			UnresolvedAfterStageA.Num(),
			RedSegmentConnectorCount,
			BlackSegmentConnectorCount,
			BlackDeadEndCount,
			BlackEndpointConnectorCount,
			OutStrokes.Num());
	}

	struct FLoopCandidate
	{
		FString Source;
		FString Key;
		FString RejectReason;
		int32 Priority = 0;
		int32 AnchorStrokeId = -1;
		int32 EdgeCount = 0;
		double LoopArea = 0.0;
		double LoopBboxArea = 0.0;
		double SyntheticMaxLength = 0.0;
		double BlackTotalLength = 0.0;
		bool bHasValidLocalGreenAction = false;
		FString GreenPrefilterReason;
		bool bHasValidFaceEvaluation = false;
		FString FaceEvaluationReason;
		bool bRejectedByInteriorRed = false;
		FString InteriorRedRejectReason;
		TArray<int32> LocalGreenStrokeIds;
		double GreenBoundaryLength = 0.0;
		TArray<int32> StrokeIds;
		TArray<int32> RedStrokeIds;
		TArray<int32> RealRedStrokeIds;
		TArray<int32> BlackStrokeIds;
		TArray<int32> SyntheticStrokeIds;
		FCapExtrusionResult Result;
		bool bSelected = false;
		FString SelectionClass;
		FString SelectionPhase;
	};

	struct FMixedSelectionScore
	{
		double CoveredRedArcLength = 0.0;
		double BorrowedBlackArcLength = 0.0;
		int32 GraphEdgeCount = 0;
		int32 CoveredRedStrokeCount = 0;
		int32 CandidateCount = 0;
		int32 LocalBlackCandidateCount = 0;
		FString StableKey;
	};

	struct FLoopSelectionDiagnostics
	{
		double TimeBudgetSeconds = CapMixedSelectionTimeBudgetSeconds;
		double ElapsedSeconds = 0.0;
		int64 VisitedStateCount = 0;
		int32 EligibleRedOnlyCount = 0;
		int32 EligibleMixedCount = 0;
		int32 MixedBlockedByRedOnlyCount = 0;
		int32 RedOnlySharedStrokeWarningCount = 0;
		bool bTimedOut = false;
		bool bOptimalityProven = true;
		FMixedSelectionScore BestMixedScore;
	};

	static void SortUniqueInts(TArray<int32>& Values)
	{
		Values.Sort();
		for (int32 i = Values.Num() - 1; i > 0; --i)
		{
			if (Values[i] == Values[i - 1])
			{
				Values.RemoveAt(i);
			}
		}
	}

	static FString JoinInts(const TArray<int32>& Values)
	{
		FString Out;
		for (int32 i = 0; i < Values.Num(); ++i)
		{
			Out += FString::Printf(TEXT("%s%d"), (i == 0 ? TEXT("") : TEXT(",")), Values[i]);
		}
		return Out;
	}

	static FString StrokeSetKey(TArray<int32> StrokeIds)
	{
		SortUniqueInts(StrokeIds);
		return JoinInts(StrokeIds);
	}

	static double PolygonAbsArea(const FStroke& Poly)
	{
		if (Poly.Num() < 3)
		{
			return 0.0;
		}
		double Sum = 0.0;
		for (int32 i = 0; i < Poly.Num(); ++i)
		{
			const FVector2D& A = Poly[i];
			const FVector2D& B = Poly[(i + 1) % Poly.Num()];
			Sum += A.X * B.Y - B.X * A.Y;
		}
		return FMath::Abs(0.5 * Sum);
	}

	static double PolygonBboxArea(const FStroke& Poly)
	{
		if (Poly.Num() == 0)
		{
			return 0.0;
		}

		double MinX = TNumericLimits<double>::Max();
		double MinY = TNumericLimits<double>::Max();
		double MaxX = -TNumericLimits<double>::Max();
		double MaxY = -TNumericLimits<double>::Max();
		for (const FVector2D& P : Poly)
		{
			MinX = FMath::Min(MinX, double(P.X));
			MinY = FMath::Min(MinY, double(P.Y));
			MaxX = FMath::Max(MaxX, double(P.X));
			MaxY = FMath::Max(MaxY, double(P.Y));
		}

		const int32 MinXi = FMath::FloorToInt(MinX);
		const int32 MinYi = FMath::FloorToInt(MinY);
		const int32 MaxXi = FMath::CeilToInt(MaxX);
		const int32 MaxYi = FMath::CeilToInt(MaxY);
		return double(MaxXi - MinXi + 1) * double(MaxYi - MinYi + 1);
	}

	static bool IsValidLoopPolygon(const FStroke& Poly)
	{
		return Poly.Num() >= 4 && PolygonAbsArea(Poly) > 1.0;
	}

	static double StrokeArcLength(const FColoredStroke& Stroke);

	static bool MakeLoopCandidateFromCycle(
		const TArray<FColoredStroke>& Strokes,
		const CapOps::FGraph& G,
		const TArray<int32>& Cycle,
		const TCHAR* Source,
		int32 Priority,
		int32 AnchorStrokeId,
		FLoopCandidate& Out)
	{
		using namespace CapOps;
		Out = FLoopCandidate();
		if (Cycle.Num() < 2)
		{
			return false;
		}

		Out.Source = Source;
		Out.Priority = Priority;
		Out.AnchorStrokeId = AnchorStrokeId;
		Out.EdgeCount = Cycle.Num();
		Out.Result = FCapExtrusionResult();
		Out.Result.bFound = true;
		Out.Result.CandidateSource = Source;
		Out.Result.CandidateAnchorStrokeId = AnchorStrokeId;

		for (int32 e : Cycle)
		{
			if (!G.StrokeId.IsValidIndex(e))
			{
				return false;
			}
			const int32 StrokeId = G.StrokeId[e];
			const bool bSynthetic = G.bSynthetic.IsValidIndex(e) && G.bSynthetic[e] != 0;
			Out.StrokeIds.Add(StrokeId);
			Out.Result.CapStrokeIds.Add(StrokeId);
			if (bSynthetic)
			{
				Out.SyntheticStrokeIds.Add(StrokeId);
			}
			if (G.bBlack[e])
			{
				Out.BlackStrokeIds.Add(StrokeId);
			}
			else
			{
				Out.RedStrokeIds.Add(StrokeId);
				if (!bSynthetic)
				{
					Out.RealRedStrokeIds.Add(StrokeId);
				}
			}
		}
		SortUniqueInts(Out.StrokeIds);
		SortUniqueInts(Out.RedStrokeIds);
		SortUniqueInts(Out.RealRedStrokeIds);
		SortUniqueInts(Out.BlackStrokeIds);
		SortUniqueInts(Out.SyntheticStrokeIds);
		if (Out.RealRedStrokeIds.Num() == 0)
		{
			return false;
		}

		Out.Result.bUsedBlack = Out.BlackStrokeIds.Num() > 0;
		for (int32 StrokeId : Out.SyntheticStrokeIds)
		{
			if (Strokes.IsValidIndex(StrokeId))
			{
				Out.SyntheticMaxLength = FMath::Max(
					Out.SyntheticMaxLength,
					StrokeArcLength(Strokes[StrokeId]));
			}
		}
		for (int32 StrokeId : Out.BlackStrokeIds)
		{
			if (Strokes.IsValidIndex(StrokeId))
			{
				Out.BlackTotalLength += StrokeArcLength(Strokes[StrokeId]);
			}
		}
		BuildPolygon(
			Strokes.GetData(),
			G,
			Cycle,
			Out.Result.CapPolygon,
			Out.Result.CapNodes,
			&Out.Result.OrderedBoundaryRuns);
		if (!IsValidLoopPolygon(Out.Result.CapPolygon))
		{
			return false;
		}
		Out.LoopArea = PolygonAbsArea(Out.Result.CapPolygon);
		Out.LoopBboxArea = PolygonBboxArea(Out.Result.CapPolygon);
		Out.Key = StrokeSetKey(Out.StrokeIds);
		return !Out.Key.IsEmpty();
	}

	static void AddUniqueCandidate(TArray<FLoopCandidate>& Candidates, TSet<FString>& SeenKeys, const FLoopCandidate& Candidate)
	{
		if (SeenKeys.Contains(Candidate.Key))
		{
			return;
		}
		SeenKeys.Add(Candidate.Key);
		Candidates.Add(Candidate);
	}

	static double CandidateRealRedArcLength(
		const FLoopCandidate& Candidate,
		const TArray<FColoredStroke>& Strokes)
	{
		double TotalLength = 0.0;
		for (int32 StrokeId : Candidate.RealRedStrokeIds)
		{
			if (Strokes.IsValidIndex(StrokeId))
			{
				TotalLength += StrokeArcLength(Strokes[StrokeId]);
			}
		}
		return TotalLength;
	}

	static bool CandidateGenerationLess(
		const FLoopCandidate& A,
		const FLoopCandidate& B,
		const TArray<FColoredStroke>& Strokes)
	{
		if (A.EdgeCount != B.EdgeCount)
		{
			return A.EdgeCount < B.EdgeCount;
		}
		const double ARealRedLength = CandidateRealRedArcLength(A, Strokes);
		const double BRealRedLength = CandidateRealRedArcLength(B, Strokes);
		if (!FMath::IsNearlyEqual(ARealRedLength, BRealRedLength, 1e-6))
		{
			return ARealRedLength > BRealRedLength;
		}
		if (!FMath::IsNearlyEqual(A.BlackTotalLength, B.BlackTotalLength, 1e-6))
		{
			return A.BlackTotalLength < B.BlackTotalLength;
		}
		if (!FMath::IsNearlyEqual(A.LoopArea, B.LoopArea, 1e-6))
		{
			return A.LoopArea < B.LoopArea;
		}
		return FCString::Strcmp(*A.Key, *B.Key) < 0;
	}

	// Per-anchor tie-break used by `local_black` and `fallback_trace` passes.
	// Ordering (lexicographic):
	//   1. RealRedArcLength desc — favors candidates that explain the most genuine
	//      red ink (excludes synthetic connectors; cycle edges are already 2-core
	//      survivors by construction). Equality tolerance 1.0 px so sub-pixel noise
	//      falls through to the LoopArea decision.
	//   2. LoopArea asc — among red-equivalent candidates, prefer the smaller cap.
	//   3. BlackTotalLength asc — less black perimeter consumed.
	//   4. EdgeCount asc — fewer graph edges.
	//   5. Key lex asc — deterministic floor.
	static bool CandidateMinAreaLess(
		const FLoopCandidate& A,
		const FLoopCandidate& B,
		const TArray<FColoredStroke>& Strokes)
	{
		const double ARealRedLength = CandidateRealRedArcLength(A, Strokes);
		const double BRealRedLength = CandidateRealRedArcLength(B, Strokes);
		if (!FMath::IsNearlyEqual(ARealRedLength, BRealRedLength, 1.0))
		{
			return ARealRedLength > BRealRedLength;
		}
		if (!FMath::IsNearlyEqual(A.LoopArea, B.LoopArea, 1e-6))
		{
			return A.LoopArea < B.LoopArea;
		}
		if (!FMath::IsNearlyEqual(A.BlackTotalLength, B.BlackTotalLength, 1e-6))
		{
			return A.BlackTotalLength < B.BlackTotalLength;
		}
		if (A.EdgeCount != B.EdgeCount)
		{
			return A.EdgeCount < B.EdgeCount;
		}
		return FCString::Strcmp(*A.Key, *B.Key) < 0;
	}

	static bool FindNearestLocalGreenForStroke(
		const TArray<FColoredStroke>& Strokes,
		int32 StrokeId,
		const TArray<int32>& AllGreen,
		double GreenSelectToleranceSquared,
		int32* OutNearestGreenStrokeId = nullptr,
		double* OutNearestDistanceSquared = nullptr)
	{
		if (OutNearestGreenStrokeId)
		{
			*OutNearestGreenStrokeId = INDEX_NONE;
		}
		if (OutNearestDistanceSquared)
		{
			*OutNearestDistanceSquared = TNumericLimits<double>::Max();
		}
		if (!Strokes.IsValidIndex(StrokeId))
		{
			return false;
		}

		const FStroke& StrokePoints = Strokes[StrokeId].Points;
		if (StrokePoints.Num() == 0)
		{
			return false;
		}

		bool bFound = false;
		double BestD2 = TNumericLimits<double>::Max();
		int32 BestGreenStrokeId = INDEX_NONE;
		for (int32 GreenStrokeId : AllGreen)
		{
			if (!Strokes.IsValidIndex(GreenStrokeId))
			{
				continue;
			}
			const FStroke& GreenPoints = Strokes[GreenStrokeId].Points;
			if (GreenPoints.Num() == 0)
			{
				continue;
			}

			for (const FVector2D& StrokePoint : StrokePoints)
			{
				const double StartD2 = FVector2D::DistSquared(StrokePoint, GreenPoints[0]);
				if (StartD2 < BestD2)
				{
					BestD2 = StartD2;
					BestGreenStrokeId = GreenStrokeId;
				}
				const double EndD2 = FVector2D::DistSquared(StrokePoint, GreenPoints.Last());
				if (EndD2 < BestD2)
				{
					BestD2 = EndD2;
					BestGreenStrokeId = GreenStrokeId;
				}
			}
		}

		if (BestD2 <= GreenSelectToleranceSquared)
		{
			bFound = true;
		}
		if (OutNearestGreenStrokeId && bFound)
		{
			*OutNearestGreenStrokeId = BestGreenStrokeId;
		}
		if (OutNearestDistanceSquared && bFound)
		{
			*OutNearestDistanceSquared = BestD2;
		}
		return bFound;
	}

	static void CollectLocalGreenStrokeIdsForRedStrokes(
		const TArray<FColoredStroke>& Strokes,
		const TArray<int32>& RedStrokeIds,
		const TArray<int32>& AllGreen,
		double GreenSelectToleranceSquared,
		TArray<int32>& OutLocalGreenStrokeIds)
	{
		OutLocalGreenStrokeIds.Reset();
		for (int32 GreenStrokeId : AllGreen)
		{
			if (!Strokes.IsValidIndex(GreenStrokeId))
			{
				continue;
			}
			const FStroke& GreenPoints = Strokes[GreenStrokeId].Points;
			if (GreenPoints.Num() == 0)
			{
				continue;
			}

			bool bNear = false;
			for (int32 RedStrokeId : RedStrokeIds)
			{
				if (!Strokes.IsValidIndex(RedStrokeId))
				{
					continue;
				}
				for (const FVector2D& RedPoint : Strokes[RedStrokeId].Points)
				{
					if (FVector2D::DistSquared(RedPoint, GreenPoints[0]) <= GreenSelectToleranceSquared ||
						FVector2D::DistSquared(RedPoint, GreenPoints.Last()) <= GreenSelectToleranceSquared)
					{
						bNear = true;
						break;
					}
				}
				if (bNear)
				{
					break;
				}
			}
			if (bNear)
			{
				OutLocalGreenStrokeIds.Add(GreenStrokeId);
			}
		}
	}

	static void BuildCycleCoreEdgeMask(const CapOps::FGraph& G, TArray<uint8>& OutEdgeUsable)
	{
		const int32 EdgeCount = G.StrokeId.Num();
		OutEdgeUsable.Init(1, EdgeCount);
		TArray<TArray<int32>> IncidentEdges;
		IncidentEdges.SetNum(G.NumNodes);
		TArray<int32> Degree;
		Degree.Init(0, G.NumNodes);

		for (int32 EdgeIndex = 0; EdgeIndex < EdgeCount; ++EdgeIndex)
		{
			if (!G.NodeU.IsValidIndex(EdgeIndex) ||
				!G.NodeV.IsValidIndex(EdgeIndex) ||
				G.NodeU[EdgeIndex] < 0 ||
				G.NodeV[EdgeIndex] < 0 ||
				G.NodeU[EdgeIndex] >= G.NumNodes ||
				G.NodeV[EdgeIndex] >= G.NumNodes ||
				G.NodeU[EdgeIndex] == G.NodeV[EdgeIndex])
			{
				OutEdgeUsable[EdgeIndex] = 0;
				continue;
			}

			const int32 U = G.NodeU[EdgeIndex];
			const int32 V = G.NodeV[EdgeIndex];
			IncidentEdges[U].Add(EdgeIndex);
			IncidentEdges[V].Add(EdgeIndex);
			++Degree[U];
			++Degree[V];
		}

		TArray<int32> Queue;
		TArray<uint8> bQueued;
		bQueued.Init(0, G.NumNodes);
		for (int32 NodeIndex = 0; NodeIndex < G.NumNodes; ++NodeIndex)
		{
			if (Degree[NodeIndex] <= 1)
			{
				Queue.Add(NodeIndex);
				bQueued[NodeIndex] = 1;
			}
		}

		for (int32 Head = 0; Head < Queue.Num(); ++Head)
		{
			const int32 NodeIndex = Queue[Head];
			bQueued[NodeIndex] = 0;
			if (Degree[NodeIndex] > 1)
			{
				continue;
			}

			for (int32 EdgeIndex : IncidentEdges[NodeIndex])
			{
				if (!OutEdgeUsable.IsValidIndex(EdgeIndex) || !OutEdgeUsable[EdgeIndex])
				{
					continue;
				}

				OutEdgeUsable[EdgeIndex] = 0;
				const int32 U = G.NodeU[EdgeIndex];
				const int32 V = G.NodeV[EdgeIndex];
				const int32 OtherNode = U == NodeIndex ? V : U;
				if (Degree.IsValidIndex(U))
				{
					--Degree[U];
				}
				if (Degree.IsValidIndex(V))
				{
					--Degree[V];
				}
				if (Degree.IsValidIndex(OtherNode) &&
					Degree[OtherNode] <= 1 &&
					!bQueued[OtherNode])
				{
					Queue.Add(OtherNode);
					bQueued[OtherNode] = 1;
				}
			}
		}
	}

	static bool HasUsableBlackEdge(const CapOps::FGraph& G, const TArray<uint8>& EdgeUsableMask)
	{
		for (int32 EdgeIndex = 0; EdgeIndex < G.StrokeId.Num(); ++EdgeIndex)
		{
			if (EdgeUsableMask.IsValidIndex(EdgeIndex) &&
				EdgeUsableMask[EdgeIndex] &&
				G.bBlack.IsValidIndex(EdgeIndex) &&
				G.bBlack[EdgeIndex])
			{
				return true;
			}
		}
		return false;
	}

	struct FLoopAnchorDebugEntry
	{
		int32 RedGroupIndex = INDEX_NONE;
		int32 EdgeIndex = INDEX_NONE;
		int32 StrokeId = INDEX_NONE;
		int32 NearestGreenStrokeId = INDEX_NONE;
		double ArcLength = 0.0;
		double GreenDistanceSquared = TNumericLimits<double>::Max();
	};

	struct FLoopAnchorSearchOptions
	{
		const TArray<uint8>* EdgeUsableMask = nullptr;
		const TArray<int32>* GreenStrokeIds = nullptr;
		double GreenSelectToleranceSquared = 0.0;
		bool bRequireAnchorNearGreen = false;
		bool bSortAnchorsByLength = false;
		bool bPreferMinAreaCandidate = false;
		int32 MaxCandidatesPerAnchor = CapCycleCandidatesPerAnchor;
		int32 RedGroupIndex = INDEX_NONE;
		TArray<FLoopAnchorDebugEntry>* AnchorDebugEntries = nullptr;
	};

	static void CollectTopCycleCandidatesForAnchor(
		const TArray<FColoredStroke>& Strokes,
		const CapOps::FGraph& G,
		const TArray<TArray<int32>>& Adjacency,
		int32 AnchorEdge,
		const TCHAR* Source,
		int32 Priority,
		bool bRequireBlack,
		bool bPreferMinAreaCandidate,
		int32 MaxCandidatesPerAnchor,
		TArray<FLoopCandidate>& OutCandidates)
	{
		using namespace CapOps;
		OutCandidates.Reset();

		const int32 StartNode = G.NodeU[AnchorEdge];
		const int32 TargetNode = G.NodeV[AnchorEdge];

		TArray<int32> MinHopsToTarget;
		MinHopsToTarget.Init(MAX_int32, G.NumNodes);
		TArray<int32> Queue;
		Queue.Add(TargetNode);
		MinHopsToTarget[TargetNode] = 0;
		for (int32 Head = 0; Head < Queue.Num(); ++Head)
		{
			const int32 CurrentNode = Queue[Head];
			for (int32 EdgeIndex : Adjacency[CurrentNode])
			{
				if (EdgeIndex == AnchorEdge)
				{
					continue;
				}
				const int32 OtherNode =
					G.NodeU[EdgeIndex] == CurrentNode ? G.NodeV[EdgeIndex] :
					(G.NodeV[EdgeIndex] == CurrentNode ? G.NodeU[EdgeIndex] : INDEX_NONE);
				if (OtherNode == INDEX_NONE || MinHopsToTarget[OtherNode] != MAX_int32)
				{
					continue;
				}
				MinHopsToTarget[OtherNode] = MinHopsToTarget[CurrentNode] + 1;
				Queue.Add(OtherNode);
			}
		}

		if (!MinHopsToTarget.IsValidIndex(StartNode) || MinHopsToTarget[StartNode] == MAX_int32)
		{
			return;
		}

		TSet<FString> SeenAnchorKeys;
		TArray<uint8> VisitedNodes;
		VisitedNodes.Init(0, G.NumNodes);
		VisitedNodes[StartNode] = 1;
		TArray<int32> PathEdges;

		const int32 ShortestPathEdgeCount = MinHopsToTarget[StartNode];
		const int32 MaxSimplePathEdgeCount = FMath::Max(0, G.NumNodes - 1);
		for (int32 PathEdgeLimit = ShortestPathEdgeCount;
			PathEdgeLimit <= MaxSimplePathEdgeCount;
			++PathEdgeLimit)
		{
			TFunction<void(int32, int32)> EnumeratePathsAtDepth = [&](int32 CurrentNode, int32 RemainingEdges)
			{
				if (RemainingEdges == 0)
				{
					if (CurrentNode != TargetNode)
					{
						return;
					}
					TArray<int32> Cycle = PathEdges;
					Cycle.Add(AnchorEdge);
					FLoopCandidate Candidate;
					if (!MakeLoopCandidateFromCycle(
						Strokes,
						G,
						Cycle,
						Source,
						Priority,
						G.StrokeId[AnchorEdge],
						Candidate))
					{
						return;
					}
					if (bRequireBlack && Candidate.BlackStrokeIds.Num() == 0)
					{
						return;
					}
					if (!SeenAnchorKeys.Contains(Candidate.Key))
					{
						SeenAnchorKeys.Add(Candidate.Key);
						OutCandidates.Add(MoveTemp(Candidate));
					}
					return;
				}

				if (CurrentNode == TargetNode ||
					!MinHopsToTarget.IsValidIndex(CurrentNode) ||
					MinHopsToTarget[CurrentNode] > RemainingEdges)
				{
					return;
				}

				for (int32 EdgeIndex : Adjacency[CurrentNode])
				{
					if (EdgeIndex == AnchorEdge)
					{
						continue;
					}
					const int32 OtherNode =
						G.NodeU[EdgeIndex] == CurrentNode ? G.NodeV[EdgeIndex] :
						(G.NodeV[EdgeIndex] == CurrentNode ? G.NodeU[EdgeIndex] : INDEX_NONE);
					if (OtherNode == INDEX_NONE ||
						VisitedNodes[OtherNode] ||
						!MinHopsToTarget.IsValidIndex(OtherNode) ||
						MinHopsToTarget[OtherNode] > RemainingEdges - 1)
					{
						continue;
					}
					if (OtherNode == TargetNode && RemainingEdges != 1)
					{
						continue;
					}

					VisitedNodes[OtherNode] = 1;
					PathEdges.Add(EdgeIndex);
					EnumeratePathsAtDepth(OtherNode, RemainingEdges - 1);
					PathEdges.Pop();
					VisitedNodes[OtherNode] = 0;
				}
			};

			EnumeratePathsAtDepth(StartNode, PathEdgeLimit);
			if (OutCandidates.Num() > 0)
			{
				break;
			}
		}

		OutCandidates.Sort([&Strokes, bPreferMinAreaCandidate](const FLoopCandidate& A, const FLoopCandidate& B)
		{
			return bPreferMinAreaCandidate
				? CandidateMinAreaLess(A, B, Strokes)
				: CandidateGenerationLess(A, B, Strokes);
		});
		if (MaxCandidatesPerAnchor >= 0 && OutCandidates.Num() > MaxCandidatesPerAnchor)
		{
			OutCandidates.SetNum(MaxCandidatesPerAnchor);
		}
	}

	static void CollectCycleCandidatesFromGraph(
		const TArray<FColoredStroke>& Strokes,
		const CapOps::FGraph& G,
		const TCHAR* Source,
		int32 Priority,
		bool bRequireBlack,
		TArray<FLoopCandidate>& Candidates,
		TSet<FString>& SeenKeys,
		const FLoopAnchorSearchOptions& Options = FLoopAnchorSearchOptions())
	{
		using namespace CapOps;
		TArray<TArray<int32>> Adjacency;
		Adjacency.SetNum(G.NumNodes);
		for (int32 EdgeIndex = 0; EdgeIndex < G.StrokeId.Num(); ++EdgeIndex)
		{
			if (Options.EdgeUsableMask &&
				(!Options.EdgeUsableMask->IsValidIndex(EdgeIndex) ||
				 !(*Options.EdgeUsableMask)[EdgeIndex]))
			{
				continue;
			}
			if (G.NodeU.IsValidIndex(EdgeIndex))
			{
				Adjacency[G.NodeU[EdgeIndex]].Add(EdgeIndex);
			}
			if (G.NodeV.IsValidIndex(EdgeIndex) && G.NodeV[EdgeIndex] != G.NodeU[EdgeIndex])
			{
				Adjacency[G.NodeV[EdgeIndex]].Add(EdgeIndex);
			}
		}

		struct FAnchorRef
		{
			int32 EdgeIndex = INDEX_NONE;
			int32 StrokeId = INDEX_NONE;
			int32 NearestGreenStrokeId = INDEX_NONE;
			double ArcLength = 0.0;
			double GreenDistanceSquared = TNumericLimits<double>::Max();
		};

		TArray<FAnchorRef> Anchors;
		for (int32 e = 0; e < G.StrokeId.Num(); ++e)
		{
			if (Options.EdgeUsableMask &&
				(!Options.EdgeUsableMask->IsValidIndex(e) ||
				 !(*Options.EdgeUsableMask)[e]))
			{
				continue;
			}
			if (G.bBlack[e] ||
				(G.bSynthetic.IsValidIndex(e) && G.bSynthetic[e]) ||
				!Strokes.IsValidIndex(G.StrokeId[e]))
			{
				continue;
			}
			if (G.NodeU[e] == G.NodeV[e])
			{
				continue;
			}

			const int32 StrokeId = G.StrokeId[e];
			const double AnchorArcLength = StrokeArcLength(Strokes[StrokeId]);
			if (!Options.bRequireAnchorNearGreen &&
				AnchorArcLength < CapCycleAnchorMinRealRedLengthPx)
			{
				continue;
			}

			int32 NearestGreenStrokeId = INDEX_NONE;
			double GreenDistanceSquared = TNumericLimits<double>::Max();
			if (Options.bRequireAnchorNearGreen)
			{
				if (!Options.GreenStrokeIds ||
					!FindNearestLocalGreenForStroke(
						Strokes,
						StrokeId,
						*Options.GreenStrokeIds,
						Options.GreenSelectToleranceSquared,
						&NearestGreenStrokeId,
						&GreenDistanceSquared))
				{
					continue;
				}
			}

			FAnchorRef Anchor;
			Anchor.EdgeIndex = e;
			Anchor.StrokeId = StrokeId;
			Anchor.NearestGreenStrokeId = NearestGreenStrokeId;
			Anchor.ArcLength = AnchorArcLength;
			Anchor.GreenDistanceSquared = GreenDistanceSquared;
			Anchors.Add(Anchor);

			if (Options.AnchorDebugEntries)
			{
				FLoopAnchorDebugEntry DebugEntry;
				DebugEntry.RedGroupIndex = Options.RedGroupIndex;
				DebugEntry.EdgeIndex = e;
				DebugEntry.StrokeId = StrokeId;
				DebugEntry.NearestGreenStrokeId = NearestGreenStrokeId;
				DebugEntry.ArcLength = AnchorArcLength;
				DebugEntry.GreenDistanceSquared = GreenDistanceSquared;
				Options.AnchorDebugEntries->Add(MoveTemp(DebugEntry));
			}
		}

		if (Options.bSortAnchorsByLength)
		{
			Anchors.Sort([](const FAnchorRef& A, const FAnchorRef& B)
			{
				if (!FMath::IsNearlyEqual(A.ArcLength, B.ArcLength, 1e-6))
				{
					return A.ArcLength > B.ArcLength;
				}
				if (!FMath::IsNearlyEqual(A.GreenDistanceSquared, B.GreenDistanceSquared, 1e-6))
				{
					return A.GreenDistanceSquared < B.GreenDistanceSquared;
				}
				if (A.StrokeId != B.StrokeId)
				{
					return A.StrokeId < B.StrokeId;
				}
				return A.EdgeIndex < B.EdgeIndex;
			});
		}

		for (const FAnchorRef& Anchor : Anchors)
		{
			TArray<FLoopCandidate> AnchorCandidates;
			CollectTopCycleCandidatesForAnchor(
				Strokes,
				G,
				Adjacency,
				Anchor.EdgeIndex,
				Source,
				Priority,
				bRequireBlack,
				Options.bPreferMinAreaCandidate,
				Options.MaxCandidatesPerAnchor,
				AnchorCandidates);
			for (const FLoopCandidate& Candidate : AnchorCandidates)
			{
				AddUniqueCandidate(Candidates, SeenKeys, Candidate);
			}
		}
	}

	static void GroupRedStrokeComponents(
		const TArray<FColoredStroke>& Strokes,
		const TArray<int32>& RedIdx,
		float NodeSnapTol,
		int32 FirstSyntheticStrokeId,
		TArray<TArray<int32>>& OutGroups)
	{
		using namespace CapOps;
		OutGroups.Reset();
		if (RedIdx.Num() == 0)
		{
			return;
		}

		FGraph G;
		BuildGraph(Strokes, RedIdx, NodeSnapTol, FirstSyntheticStrokeId, G);
		if (G.NumNodes <= 0)
		{
			return;
		}

		TArray<int32> Parent;
		Parent.SetNumUninitialized(G.NumNodes);
		for (int32 n = 0; n < G.NumNodes; ++n)
		{
			Parent[n] = n;
		}
		for (int32 e = 0; e < G.StrokeId.Num(); ++e)
		{
			Parent[Find(Parent, G.NodeV[e])] = Find(Parent, G.NodeU[e]);
		}

		TMap<int32, int32> GroupByRoot;
		for (int32 e = 0; e < G.StrokeId.Num(); ++e)
		{
			const int32 Root = Find(Parent, G.NodeU[e]);
			int32 GroupIndex = INDEX_NONE;
			if (int32* Existing = GroupByRoot.Find(Root))
			{
				GroupIndex = *Existing;
			}
			else
			{
				GroupIndex = OutGroups.Num();
				GroupByRoot.Add(Root, GroupIndex);
				OutGroups.AddDefaulted();
			}
			OutGroups[GroupIndex].Add(G.StrokeId[e]);
		}

		for (TArray<int32>& Group : OutGroups)
		{
			SortUniqueInts(Group);
		}
	}

	static bool SaveGreenAnchorDebugPng(
		const TArray<FColoredStroke>& Strokes,
		const TArray<FLoopAnchorDebugEntry>& Anchors,
		bool bColorByRedGroup,
		int32 Width,
		int32 Height,
		const FString& Path)
	{
		if (Width <= 0 || Height <= 0)
		{
			return false;
		}

		TArray<uint8> RGBA;
		RGBA.Init(255, Width * Height * 4);

		auto Plot = [&](int32 x, int32 y, uint8 R, uint8 G, uint8 B, int32 Radius)
		{
			for (int32 oy = -Radius; oy <= Radius; ++oy)
			{
				for (int32 ox = -Radius; ox <= Radius; ++ox)
				{
					const int32 xx = x + ox;
					const int32 yy = y + oy;
					if (xx < 0 || xx >= Width || yy < 0 || yy >= Height)
					{
						continue;
					}
					const int32 Off = (yy * Width + xx) * 4;
					RGBA[Off + 0] = R;
					RGBA[Off + 1] = G;
					RGBA[Off + 2] = B;
					RGBA[Off + 3] = 255;
				}
			}
		};

		auto DrawStroke = [&](int32 StrokeId, uint8 R, uint8 G, uint8 B, int32 Radius)
		{
			if (!Strokes.IsValidIndex(StrokeId))
			{
				return;
			}
			TArray<FIntPoint> LineBuf;
			const FStroke& Points = Strokes[StrokeId].Points;
			for (int32 k = 0; k + 1 < Points.Num(); ++k)
			{
				const FIntPoint P0(FMath::RoundToInt(Points[k].X), FMath::RoundToInt(Points[k].Y));
				const FIntPoint P1(FMath::RoundToInt(Points[k + 1].X), FMath::RoundToInt(Points[k + 1].Y));
				SkelGraph::LinePoints(P0, P1, LineBuf);
				for (const FIntPoint& P : LineBuf)
				{
					Plot(P.X, P.Y, R, G, B, Radius);
				}
			}
		};

		for (int32 StrokeId = 0; StrokeId < Strokes.Num(); ++StrokeId)
		{
			switch (Strokes[StrokeId].Color)
			{
			case EStrokeColor::Red:
				DrawStroke(StrokeId, 255, 205, 215, 1);
				break;
			case EStrokeColor::Black:
				DrawStroke(StrokeId, 155, 155, 155, 1);
				break;
			case EStrokeColor::Green:
				DrawStroke(StrokeId, 90, 190, 90, 1);
				break;
			default:
				break;
			}
		}

		static const uint8 GroupPalette[][3] = {
			{230,  25,  75}, { 60, 180,  75}, {  0, 130, 200}, {245, 130,  48},
			{145,  30, 180}, { 70, 240, 240}, {240,  50, 230}, {210, 245,  60},
			{250, 190, 190}, {  0, 128, 128}, {230, 190, 255}, {170, 110,  40},
		};
		const int32 PaletteCount = UE_ARRAY_COUNT(GroupPalette);
		for (const FLoopAnchorDebugEntry& Anchor : Anchors)
		{
			const int32 ColorIndex = bColorByRedGroup
				? FMath::Abs(Anchor.RedGroupIndex) % PaletteCount
				: 4;
			const uint8* Color = GroupPalette[ColorIndex];
			DrawStroke(Anchor.StrokeId, Color[0], Color[1], Color[2], 3);
		}

		IImageWrapperModule& IWM = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
		TSharedPtr<IImageWrapper> IW = IWM.CreateImageWrapper(EImageFormat::PNG);
		if (!IW.IsValid())
		{
			return false;
		}
		IW->SetRaw(RGBA.GetData(), RGBA.Num(), Width, Height, ERGBFormat::RGBA, 8);
		const TArray64<uint8>& Compressed = IW->GetCompressed();
		return FFileHelper::SaveArrayToFile(
			TArrayView<const uint8>(Compressed.GetData(), static_cast<int32>(Compressed.Num())),
			*Path);
	}

	static void CollectRedFirstLoopCandidates(
		const TArray<FColoredStroke>& Strokes,
		const TArray<int32>& RedIdx,
		const TArray<int32>& BlackIdx,
		const TArray<int32>& GreenIdx,
		float NodeSnapTol,
		int32 FirstSyntheticStrokeId,
		double GreenSelectToleranceSquared,
		int32 Width,
		int32 Height,
		const FString& PressDir,
		TArray<FLoopCandidate>& OutCandidates)
	{
		using namespace CapOps;
		OutCandidates.Reset();
		TSet<FString> SeenKeys;
		TArray<FLoopAnchorDebugEntry> LocalBlackAnchorDebugEntries;
		TArray<FLoopAnchorDebugEntry> FallbackAnchorDebugEntries;

		// Phase 1: all red-only endpoint cycles. These get first selection priority.
		{
			FGraph RedGraph;
			BuildGraph(Strokes, RedIdx, NodeSnapTol, FirstSyntheticStrokeId, RedGraph);
			CollectCycleCandidatesFromGraph(
				Strokes, RedGraph, TEXT("red_only"), /*Priority*/ 0, /*bRequireBlack*/ false,
				OutCandidates, SeenKeys);
		}

		// Phase 2: each red-only component closes its own endpoints through the global black pool.
		TArray<TArray<int32>> RedGroups;
		GroupRedStrokeComponents(Strokes, RedIdx, NodeSnapTol, FirstSyntheticStrokeId, RedGroups);
		for (int32 RedGroupIndex = 0; RedGroupIndex < RedGroups.Num(); ++RedGroupIndex)
		{
			const TArray<int32>& RedGroup = RedGroups[RedGroupIndex];
			TArray<int32> EdgeIds = RedGroup;
			EdgeIds.Append(BlackIdx);
			FGraph LocalGraph;
			BuildGraph(Strokes, EdgeIds, NodeSnapTol, FirstSyntheticStrokeId, LocalGraph);
			TArray<uint8> LocalEdgeUsableMask;
			BuildCycleCoreEdgeMask(LocalGraph, LocalEdgeUsableMask);
			if (!HasUsableBlackEdge(LocalGraph, LocalEdgeUsableMask))
			{
				continue;
			}

			FLoopAnchorSearchOptions Options;
			Options.EdgeUsableMask = &LocalEdgeUsableMask;
			Options.GreenStrokeIds = &GreenIdx;
			Options.GreenSelectToleranceSquared = GreenSelectToleranceSquared;
			Options.bRequireAnchorNearGreen = true;
			Options.bSortAnchorsByLength = true;
			Options.bPreferMinAreaCandidate = true;
			Options.MaxCandidatesPerAnchor = 1;
			Options.RedGroupIndex = RedGroupIndex;
			Options.AnchorDebugEntries = &LocalBlackAnchorDebugEntries;
			CollectCycleCandidatesFromGraph(
				Strokes, LocalGraph, TEXT("local_black"), /*Priority*/ 1, /*bRequireBlack*/ true,
				OutCandidates, SeenKeys, Options);
		}

		// Phase 3: fallback trace over all remaining-style red combinations plus all black.
		// Selection happens later, so red conflicts with stronger candidates will be rejected.
		{
			TArray<int32> EdgeIds = RedIdx;
			EdgeIds.Append(BlackIdx);
			FGraph FallbackGraph;
			BuildGraph(Strokes, EdgeIds, NodeSnapTol, FirstSyntheticStrokeId, FallbackGraph);
			TArray<uint8> FallbackEdgeUsableMask;
			BuildCycleCoreEdgeMask(FallbackGraph, FallbackEdgeUsableMask);
			FLoopAnchorSearchOptions Options;
			Options.EdgeUsableMask = &FallbackEdgeUsableMask;
			Options.GreenStrokeIds = &GreenIdx;
			Options.GreenSelectToleranceSquared = GreenSelectToleranceSquared;
			Options.bRequireAnchorNearGreen = true;
			Options.bSortAnchorsByLength = true;
			Options.bPreferMinAreaCandidate = true;
			Options.MaxCandidatesPerAnchor = 1;
			Options.AnchorDebugEntries = &FallbackAnchorDebugEntries;
			CollectCycleCandidatesFromGraph(
				Strokes, FallbackGraph, TEXT("fallback_trace"), /*Priority*/ 2, /*bRequireBlack*/ false,
				OutCandidates, SeenKeys, Options);
		}

		SaveGreenAnchorDebugPng(
			Strokes,
			LocalBlackAnchorDebugEntries,
			/*bColorByRedGroup*/ true,
			Width,
			Height,
			PressDir / TEXT("09_local_black_green_anchors.png"));
		SaveGreenAnchorDebugPng(
			Strokes,
			FallbackAnchorDebugEntries,
			/*bColorByRedGroup*/ false,
			Width,
			Height,
			PressDir / TEXT("09_fallback_green_anchors.png"));
	}

	static bool CandidatePassesSelectionValidation(const FLoopCandidate& Candidate, FString& OutReason)
	{
		if (!Candidate.bHasValidLocalGreenAction)
		{
			OutReason = Candidate.GreenPrefilterReason.IsEmpty()
				? TEXT("candidate has no valid local green action")
				: Candidate.GreenPrefilterReason;
			return false;
		}
		const bool bActuallyRedOnly = Candidate.BlackStrokeIds.Num() == 0;
		const double MinBboxArea = bActuallyRedOnly ? CapRedOnlyLoopMinBboxArea : CapBorrowedLoopMinBboxArea;
		if (Candidate.LoopBboxArea < MinBboxArea)
		{
			OutReason = bActuallyRedOnly
				? FString::Printf(TEXT("red-only loop bbox area %.3f below %.3f"), Candidate.LoopBboxArea, MinBboxArea)
				: FString::Printf(TEXT("loop bbox area %.3f below %.3f"), Candidate.LoopBboxArea, MinBboxArea);
			return false;
		}
		if (Candidate.bRejectedByInteriorRed)
		{
			OutReason = Candidate.InteriorRedRejectReason.IsEmpty()
				? TEXT("candidate rejected: interior red line exceeds threshold")
				: Candidate.InteriorRedRejectReason;
			return false;
		}
		if (!Candidate.bHasValidFaceEvaluation)
		{
			OutReason = Candidate.FaceEvaluationReason.IsEmpty()
				? TEXT("candidate has no valid base face")
				: Candidate.FaceEvaluationReason;
			return false;
		}
		OutReason = TEXT("passed_all_candidate_validations");
		return true;
	}

	static bool IsBetterMixedScore(const FMixedSelectionScore& A, const FMixedSelectionScore& B)
	{
		if (!FMath::IsNearlyEqual(A.CoveredRedArcLength, B.CoveredRedArcLength, 1e-6))
		{
			return A.CoveredRedArcLength > B.CoveredRedArcLength;
		}
		if (!FMath::IsNearlyEqual(A.BorrowedBlackArcLength, B.BorrowedBlackArcLength, 1e-6))
		{
			return A.BorrowedBlackArcLength < B.BorrowedBlackArcLength;
		}
		if (A.GraphEdgeCount != B.GraphEdgeCount) { return A.GraphEdgeCount < B.GraphEdgeCount; }
		if (A.CoveredRedStrokeCount != B.CoveredRedStrokeCount) { return A.CoveredRedStrokeCount > B.CoveredRedStrokeCount; }
		if (A.CandidateCount != B.CandidateCount) { return A.CandidateCount < B.CandidateCount; }
		if (A.LocalBlackCandidateCount != B.LocalBlackCandidateCount) { return A.LocalBlackCandidateCount > B.LocalBlackCandidateCount; }
		return FCString::Strcmp(*A.StableKey, *B.StableKey) < 0;
	}

	static FMixedSelectionScore ScoreMixedSelection(
		const TArray<FLoopCandidate>& Candidates,
		const TArray<FColoredStroke>& Strokes,
		const TArray<int32>& SelectedIndices)
	{
		FMixedSelectionScore Score;
		TSet<int32> CoveredRedStrokeIds;
		TArray<FString> Keys;
		for (int32 CandidateIndex : SelectedIndices)
		{
			const FLoopCandidate& Candidate = Candidates[CandidateIndex];
			Score.BorrowedBlackArcLength += Candidate.BlackTotalLength;
			Score.GraphEdgeCount += Candidate.EdgeCount;
			Score.CandidateCount++;
			Score.LocalBlackCandidateCount += Candidate.Source == TEXT("local_black") ? 1 : 0;
			Keys.Add(Candidate.Key);
			for (int32 RedStrokeId : Candidate.RealRedStrokeIds)
			{
				if (!CoveredRedStrokeIds.Contains(RedStrokeId))
				{
					CoveredRedStrokeIds.Add(RedStrokeId);
					if (Strokes.IsValidIndex(RedStrokeId))
					{
						Score.CoveredRedArcLength += StrokeArcLength(Strokes[RedStrokeId]);
					}
				}
			}
		}
		Score.CoveredRedStrokeCount = CoveredRedStrokeIds.Num();
		Keys.Sort();
		Score.StableKey = FString::Join(Keys, TEXT("|"));
		return Score;
	}

	static bool CandidateConflictsWithRedSet(const FLoopCandidate& Candidate, const TSet<int32>& RedStrokeIds)
	{
		for (int32 RedStrokeId : Candidate.RealRedStrokeIds)
		{
			if (RedStrokeIds.Contains(RedStrokeId))
			{
				return true;
			}
		}
		return false;
	}

	static void SelectLoopCandidates(
		TArray<FLoopCandidate>& Candidates,
		const TArray<FColoredStroke>& Strokes,
		TArray<int32>& OutSelectedIndices,
		FLoopSelectionDiagnostics& OutDiagnostics)
	{
		OutSelectedIndices.Reset();
		OutDiagnostics = FLoopSelectionDiagnostics();
		TSet<int32> LockedRedStrokeIds;
		TMap<int32, int32> RedOnlyStrokeOwner;
		TArray<int32> MixedPool;

		for (int32 CandidateIndex = 0; CandidateIndex < Candidates.Num(); ++CandidateIndex)
		{
			FLoopCandidate& Candidate = Candidates[CandidateIndex];
			Candidate.bSelected = false;
			Candidate.RejectReason.Reset();
			Candidate.SelectionClass = Candidate.BlackStrokeIds.Num() == 0 ? TEXT("authoritative_red_only") : TEXT("mixed");
			Candidate.SelectionPhase = TEXT("validation");

			FString ValidationReason;
			if (!CandidatePassesSelectionValidation(Candidate, ValidationReason))
			{
				Candidate.RejectReason = ValidationReason;
				continue;
			}

			if (Candidate.BlackStrokeIds.Num() == 0)
			{
				++OutDiagnostics.EligibleRedOnlyCount;
				Candidate.bSelected = true;
				Candidate.SelectionPhase = TEXT("red_only_locked");
				Candidate.RejectReason = TEXT("selected_authoritative_red_only");
				OutSelectedIndices.Add(CandidateIndex);
				for (int32 RedStrokeId : Candidate.RealRedStrokeIds)
				{
					if (const int32* ExistingOwner = RedOnlyStrokeOwner.Find(RedStrokeId))
					{
						++OutDiagnostics.RedOnlySharedStrokeWarningCount;
						UE_LOG(LogTemp, Warning,
							TEXT("Step9 selection: authoritative red-only candidates %d and %d unexpectedly share red stroke %d; both remain selected."),
							*ExistingOwner, CandidateIndex, RedStrokeId);
					}
					else
					{
						RedOnlyStrokeOwner.Add(RedStrokeId, CandidateIndex);
					}
					LockedRedStrokeIds.Add(RedStrokeId);
				}
			}
			else
			{
				++OutDiagnostics.EligibleMixedCount;
				Candidate.SelectionPhase = TEXT("mixed_pool");
				MixedPool.Add(CandidateIndex);
			}
		}

		TArray<int32> SearchPool;
		for (int32 CandidateIndex : MixedPool)
		{
			FLoopCandidate& Candidate = Candidates[CandidateIndex];
			if (CandidateConflictsWithRedSet(Candidate, LockedRedStrokeIds))
			{
				++OutDiagnostics.MixedBlockedByRedOnlyCount;
				Candidate.SelectionPhase = TEXT("blocked_by_red_only");
				Candidate.RejectReason = TEXT("mixed candidate uses red stroke locked by authoritative red-only loop");
				continue;
			}
			SearchPool.Add(CandidateIndex);
		}

		SearchPool.Sort([&Candidates, &Strokes](int32 AIndex, int32 BIndex)
		{
			const FLoopCandidate& A = Candidates[AIndex];
			const FLoopCandidate& B = Candidates[BIndex];
			double ARedLength = 0.0;
			double BRedLength = 0.0;
			for (int32 Id : A.RealRedStrokeIds) { if (Strokes.IsValidIndex(Id)) { ARedLength += StrokeArcLength(Strokes[Id]); } }
			for (int32 Id : B.RealRedStrokeIds) { if (Strokes.IsValidIndex(Id)) { BRedLength += StrokeArcLength(Strokes[Id]); } }
			// Aligned with per-anchor `CandidateMinAreaLess`: red-coverage primary
			// (1.0 px tolerance), then smaller LoopArea, then less black, then fewer
			// edges, then existing deterministic tiebreakers.
			if (!FMath::IsNearlyEqual(ARedLength, BRedLength, 1.0)) { return ARedLength > BRedLength; }
			if (!FMath::IsNearlyEqual(A.LoopArea, B.LoopArea, 1e-6)) { return A.LoopArea < B.LoopArea; }
			if (!FMath::IsNearlyEqual(A.BlackTotalLength, B.BlackTotalLength, 1e-6)) { return A.BlackTotalLength < B.BlackTotalLength; }
			if (A.EdgeCount != B.EdgeCount) { return A.EdgeCount < B.EdgeCount; }
			if (A.RealRedStrokeIds.Num() != B.RealRedStrokeIds.Num()) { return A.RealRedStrokeIds.Num() > B.RealRedStrokeIds.Num(); }
			if (A.Source != B.Source) { return A.Source == TEXT("local_black"); }
			return FCString::Strcmp(*A.Key, *B.Key) < 0;
		});

		TArray<int32> BestMixed;
		TSet<int32> GreedyUsedRed;
		for (int32 CandidateIndex : SearchPool)
		{
			if (!CandidateConflictsWithRedSet(Candidates[CandidateIndex], GreedyUsedRed))
			{
				BestMixed.Add(CandidateIndex);
				for (int32 RedStrokeId : Candidates[CandidateIndex].RealRedStrokeIds) { GreedyUsedRed.Add(RedStrokeId); }
			}
		}
		FMixedSelectionScore BestScore = ScoreMixedSelection(Candidates, Strokes, BestMixed);

		TArray<double> RemainingRedUpperBound;
		RemainingRedUpperBound.SetNumZeroed(SearchPool.Num() + 1);
		for (int32 Position = SearchPool.Num() - 1; Position >= 0; --Position)
		{
			double CandidateRedLength = 0.0;
			for (int32 RedStrokeId : Candidates[SearchPool[Position]].RealRedStrokeIds)
			{
				if (Strokes.IsValidIndex(RedStrokeId)) { CandidateRedLength += StrokeArcLength(Strokes[RedStrokeId]); }
			}
			RemainingRedUpperBound[Position] = RemainingRedUpperBound[Position + 1] + CandidateRedLength;
		}

		const double SearchStartSeconds = FPlatformTime::Seconds();
		TArray<int32> CurrentMixed;
		TSet<int32> CurrentUsedRed;
		TFunction<void(int32, double)> Search = [&](int32 Position, double CurrentCoveredLength)
		{
			if (OutDiagnostics.bTimedOut) { return; }
			++OutDiagnostics.VisitedStateCount;
			if (FPlatformTime::Seconds() - SearchStartSeconds >= CapMixedSelectionTimeBudgetSeconds)
			{
				OutDiagnostics.bTimedOut = true;
				OutDiagnostics.bOptimalityProven = false;
				return;
			}
			if (Position >= SearchPool.Num())
			{
				const FMixedSelectionScore Score = ScoreMixedSelection(Candidates, Strokes, CurrentMixed);
				if (IsBetterMixedScore(Score, BestScore))
				{
					BestScore = Score;
					BestMixed = CurrentMixed;
				}
				return;
			}
			if (CurrentCoveredLength + RemainingRedUpperBound[Position] + 1e-6 < BestScore.CoveredRedArcLength)
			{
				return;
			}

			const int32 CandidateIndex = SearchPool[Position];
			const FLoopCandidate& Candidate = Candidates[CandidateIndex];
			if (!CandidateConflictsWithRedSet(Candidate, CurrentUsedRed))
			{
				double AddedLength = 0.0;
				for (int32 RedStrokeId : Candidate.RealRedStrokeIds)
				{
					CurrentUsedRed.Add(RedStrokeId);
					if (Strokes.IsValidIndex(RedStrokeId)) { AddedLength += StrokeArcLength(Strokes[RedStrokeId]); }
				}
				CurrentMixed.Add(CandidateIndex);
				Search(Position + 1, CurrentCoveredLength + AddedLength);
				CurrentMixed.Pop();
				for (int32 RedStrokeId : Candidate.RealRedStrokeIds) { CurrentUsedRed.Remove(RedStrokeId); }
			}
			Search(Position + 1, CurrentCoveredLength);
		};
		Search(0, 0.0);

		OutDiagnostics.ElapsedSeconds = FPlatformTime::Seconds() - SearchStartSeconds;
		OutDiagnostics.BestMixedScore = BestScore;
		TSet<int32> BestMixedSet;
		for (int32 CandidateIndex : BestMixed)
		{
			BestMixedSet.Add(CandidateIndex);
		}
		for (int32 CandidateIndex : SearchPool)
		{
			FLoopCandidate& Candidate = Candidates[CandidateIndex];
			if (BestMixedSet.Contains(CandidateIndex))
			{
				Candidate.bSelected = true;
				Candidate.SelectionPhase = OutDiagnostics.bTimedOut ? TEXT("mixed_best_found_timeout") : TEXT("mixed_exact_optimum");
				Candidate.RejectReason = OutDiagnostics.bTimedOut ? TEXT("selected_best_found_before_timeout") : TEXT("selected_exact_mixed_optimum");
				OutSelectedIndices.Add(CandidateIndex);
			}
			else
			{
				Candidate.SelectionPhase = TEXT("mixed_not_selected");
				Candidate.RejectReason = OutDiagnostics.bTimedOut
					? TEXT("not in best mixed combination found before timeout")
					: TEXT("not in exact optimal mixed combination");
			}
		}
	}

	static FString JsonEscaped(FString Text)
	{
		Text.ReplaceInline(TEXT("\\"), TEXT("\\\\"));
		Text.ReplaceInline(TEXT("\""), TEXT("\\\""));
		return Text;
	}

	static void AppendIntArrayJson(FString& Json, const TCHAR* Key, const TArray<int32>& Values, bool bTrailingComma)
	{
		Json += FString::Printf(TEXT("    \"%s\": ["), Key);
		for (int32 i = 0; i < Values.Num(); ++i)
		{
			Json += FString::Printf(TEXT("%s%d"), (i == 0 ? TEXT("") : TEXT(", ")), Values[i]);
		}
		Json += bTrailingComma ? TEXT("],\n") : TEXT("]\n");
	}

	static bool SaveLoopCandidatesJson(
		const TArray<FLoopCandidate>& Candidates,
		const FLoopSelectionDiagnostics& Diagnostics,
		const FString& Path)
	{
		FString Json;
		Json += TEXT("{\n");
		Json += FString::Printf(TEXT("  \"candidate_count\": %d,\n"), Candidates.Num());
		int32 SelectedCount = 0;
		for (const FLoopCandidate& Candidate : Candidates)
		{
			if (Candidate.bSelected)
			{
				++SelectedCount;
			}
		}
		Json += FString::Printf(TEXT("  \"selected_count\": %d,\n"), SelectedCount);
		Json += TEXT("  \"selection\": {\n");
		Json += TEXT("    \"red_only_policy\": \"select_all_valid_and_lock_real_red_strokes\",\n");
		Json += TEXT("    \"mixed_conflict_policy\": \"shared_real_red_stroke_only\",\n");
		Json += FString::Printf(TEXT("    \"mixed_time_budget_seconds\": %.3f,\n"), Diagnostics.TimeBudgetSeconds);
		Json += FString::Printf(TEXT("    \"mixed_elapsed_seconds\": %.6f,\n"), Diagnostics.ElapsedSeconds);
		Json += FString::Printf(TEXT("    \"mixed_visited_state_count\": %lld,\n"), Diagnostics.VisitedStateCount);
		Json += FString::Printf(TEXT("    \"mixed_timed_out\": %s,\n"), Diagnostics.bTimedOut ? TEXT("true") : TEXT("false"));
		Json += FString::Printf(TEXT("    \"mixed_optimality_proven\": %s,\n"), Diagnostics.bOptimalityProven ? TEXT("true") : TEXT("false"));
		Json += FString::Printf(TEXT("    \"eligible_red_only_count\": %d,\n"), Diagnostics.EligibleRedOnlyCount);
		Json += FString::Printf(TEXT("    \"eligible_mixed_count\": %d,\n"), Diagnostics.EligibleMixedCount);
		Json += FString::Printf(TEXT("    \"mixed_blocked_by_red_only_count\": %d,\n"), Diagnostics.MixedBlockedByRedOnlyCount);
		Json += FString::Printf(TEXT("    \"red_only_shared_stroke_warning_count\": %d,\n"), Diagnostics.RedOnlySharedStrokeWarningCount);
		Json += FString::Printf(TEXT("    \"best_mixed_covered_red_arc_length\": %.6f,\n"), Diagnostics.BestMixedScore.CoveredRedArcLength);
		Json += FString::Printf(TEXT("    \"best_mixed_borrowed_black_arc_length\": %.6f,\n"), Diagnostics.BestMixedScore.BorrowedBlackArcLength);
		Json += FString::Printf(TEXT("    \"best_mixed_graph_edge_count\": %d,\n"), Diagnostics.BestMixedScore.GraphEdgeCount);
		Json += FString::Printf(TEXT("    \"best_mixed_covered_red_stroke_count\": %d,\n"), Diagnostics.BestMixedScore.CoveredRedStrokeCount);
		Json += FString::Printf(TEXT("    \"best_mixed_candidate_count\": %d,\n"), Diagnostics.BestMixedScore.CandidateCount);
		Json += FString::Printf(TEXT("    \"best_mixed_local_black_candidate_count\": %d\n"), Diagnostics.BestMixedScore.LocalBlackCandidateCount);
		Json += TEXT("  },\n");
		Json += TEXT("  \"candidates\": [\n");
		for (int32 i = 0; i < Candidates.Num(); ++i)
		{
			const FLoopCandidate& C = Candidates[i];
			Json += TEXT("  {\n");
			Json += FString::Printf(TEXT("    \"index\": %d,\n"), i);
			Json += FString::Printf(TEXT("    \"source\": \"%s\",\n"), *JsonEscaped(C.Source));
			Json += FString::Printf(TEXT("    \"priority\": %d,\n"), C.Priority);
			Json += FString::Printf(TEXT("    \"anchor_stroke_id\": %d,\n"), C.AnchorStrokeId);
			Json += FString::Printf(TEXT("    \"selected\": %s,\n"), C.bSelected ? TEXT("true") : TEXT("false"));
			Json += FString::Printf(TEXT("    \"reason\": \"%s\",\n"), *JsonEscaped(C.RejectReason));
			Json += FString::Printf(TEXT("    \"selection_class\": \"%s\",\n"), *JsonEscaped(C.SelectionClass));
			Json += FString::Printf(TEXT("    \"selection_phase\": \"%s\",\n"), *JsonEscaped(C.SelectionPhase));
			Json += FString::Printf(TEXT("    \"edge_count\": %d,\n"), C.EdgeCount);
			Json += FString::Printf(TEXT("    \"loop_area\": %.3f,\n"), C.LoopArea);
			Json += FString::Printf(TEXT("    \"loop_bbox_area\": %.3f,\n"), C.LoopBboxArea);
			Json += FString::Printf(TEXT("    \"synthetic_max_length\": %.3f,\n"), C.SyntheticMaxLength);
			Json += FString::Printf(TEXT("    \"black_total_length\": %.3f,\n"), C.BlackTotalLength);
			Json += FString::Printf(TEXT("    \"has_valid_local_green_action\": %s,\n"), C.bHasValidLocalGreenAction ? TEXT("true") : TEXT("false"));
			Json += FString::Printf(TEXT("    \"green_prefilter_reason\": \"%s\",\n"), *JsonEscaped(C.GreenPrefilterReason));
			Json += FString::Printf(TEXT("    \"face_evaluation_valid\": %s,\n"), C.bHasValidFaceEvaluation ? TEXT("true") : TEXT("false"));
			Json += FString::Printf(TEXT("    \"face_evaluation_reason\": \"%s\",\n"), *JsonEscaped(C.FaceEvaluationReason));
			Json += FString::Printf(TEXT("    \"rejected_by_interior_red\": %s,\n"), C.bRejectedByInteriorRed ? TEXT("true") : TEXT("false"));
			Json += FString::Printf(TEXT("    \"interior_red_reject_reason\": \"%s\",\n"), *JsonEscaped(C.InteriorRedRejectReason));
			Json += FString::Printf(TEXT("    \"face_source_polygon\": \"%s\",\n"), *JsonEscaped(C.Result.FaceEvaluationSourcePolygon));
			Json += FString::Printf(TEXT("    \"face_cap_mask_pixels\": %d,\n"), C.Result.FaceEvaluationCapMaskPixels);
			Json += FString::Printf(TEXT("    \"preselected_face_id\": %d,\n"), C.Result.PreselectedFaceId);
			Json += FString::Printf(TEXT("    \"preselected_face_overlap_pixels\": %d,\n"), C.Result.PreselectedFaceOverlapPixels);
			Json += FString::Printf(TEXT("    \"preselected_face_overlap_ratio\": %.6f,\n"), C.Result.PreselectedFaceOverlapRatio);
			Json += FString::Printf(TEXT("    \"candidate_face_overlap_threshold\": %.6f,\n"), FFromLZFaceReconstructor::CandidateFaceMinOverlapRatio);
			Json += FString::Printf(TEXT("    \"preselected_face_normal_side_angle_degrees\": %.6f,\n"), C.Result.PreselectedFaceNormalSideAngleDegrees);
			Json += FString::Printf(TEXT("    \"candidate_face_max_normal_side_angle_degrees\": %.6f,\n"), FFromLZFaceReconstructor::CandidateFaceMaxNormalSideAngleDegrees);
			Json += FString::Printf(TEXT("    \"preselected_face_distance_to_camera\": %.6f,\n"), C.Result.PreselectedFaceDistanceToCamera);
			Json += FString::Printf(TEXT("    \"prefiltered_action\": \"%s\",\n"), *JsonEscaped(C.Result.Action));
			Json += FString::Printf(TEXT("    \"prefiltered_action_reason\": \"%s\",\n"), *JsonEscaped(C.Result.ActionDecisionReason));
			Json += FString::Printf(TEXT("    \"prefiltered_side_length\": %.3f,\n"), C.Result.SideLength);
			Json += FString::Printf(TEXT("    \"prefiltered_inside_green_total\": %.3f,\n"), C.Result.GreenInsideTotalLength);
			Json += FString::Printf(TEXT("    \"prefiltered_outside_green_total\": %.3f,\n"), C.Result.GreenOutsideTotalLength);
			Json += FString::Printf(TEXT("    \"used_black\": %s,\n"), C.BlackStrokeIds.Num() > 0 ? TEXT("true") : TEXT("false"));
			AppendIntArrayJson(Json, TEXT("local_green_stroke_ids"), C.LocalGreenStrokeIds, true);
			AppendIntArrayJson(Json, TEXT("stroke_ids"), C.StrokeIds, true);
			AppendIntArrayJson(Json, TEXT("red_stroke_ids"), C.RedStrokeIds, true);
			AppendIntArrayJson(Json, TEXT("real_red_stroke_ids"), C.RealRedStrokeIds, true);
			AppendIntArrayJson(Json, TEXT("synthetic_stroke_ids"), C.SyntheticStrokeIds, true);
			AppendIntArrayJson(Json, TEXT("black_stroke_ids"), C.BlackStrokeIds, false);
			Json += FString::Printf(TEXT("  }%s\n"), (i + 1 < Candidates.Num() ? TEXT(",") : TEXT("")));
		}
		Json += TEXT("  ]\n");
		Json += TEXT("}\n");
		return FFileHelper::SaveStringToFile(Json, *Path);
	}

	// Even-odd ray-cast point-in-polygon test (polygon is the closed cap loop).
	static bool PointInPolygon(const FStroke& Poly, const FVector2D& P)
	{
		const int32 N = Poly.Num();
		if (N < 3) { return false; }
		bool bIn = false;
		for (int32 i = 0, j = N - 1; i < N; j = i++)
		{
			const FVector2D& A = Poly[i];
			const FVector2D& B = Poly[j];
			if (((A.Y > P.Y) != (B.Y > P.Y)) &&
				(P.X < (B.X - A.X) * (P.Y - A.Y) / (B.Y - A.Y) + A.X))
			{
				bIn = !bIn;
			}
		}
		return bIn;
	}

	// Reject a candidate when any contiguous run of a red stroke inside the cap
	// polygon exceeds CapInteriorRedMaxLineLengthPx pixels.
	static void CheckCapInteriorRedLines(
		FLoopCandidate& Candidate,
		const TArray<FColoredStroke>& Strokes,
		const TArray<int32>& AllRed)
	{
		Candidate.bRejectedByInteriorRed = false;
		Candidate.InteriorRedRejectReason.Reset();

		const FStroke& CapPoly = Candidate.Result.CapPolygon;
		if (CapPoly.Num() < 3)
		{
			return;
		}

		// Build a set of the candidate's own red stroke ids (cap boundary) to exclude.
		TSet<int32> SelfRedIds;
		for (int32 Sid : Candidate.RedStrokeIds)
		{
			SelfRedIds.Add(Sid);
		}

		for (int32 RedStrokeId : AllRed)
		{
			// Skip red strokes that form the candidate's own cap boundary —
			// they naturally lie on/inside their own polygon and must not
			// trigger a false-positive rejection.
			if (SelfRedIds.Contains(RedStrokeId))
			{
				continue;
			}
			if (!Strokes.IsValidIndex(RedStrokeId))
			{
				continue;
			}
			const FStroke& Points = Strokes[RedStrokeId].Points;
			if (Points.Num() < 2)
			{
				continue;
			}

			double ContiguousInsideLength = 0.0;
			bool bPrevInside = PointInPolygon(CapPoly, Points[0]);

			for (int32 k = 1; k < Points.Num(); ++k)
			{
				const bool bInside = PointInPolygon(CapPoly, Points[k]);
				if (bPrevInside && bInside)
				{
					ContiguousInsideLength += (Points[k] - Points[k - 1]).Size();
				}
				else if (!bInside)
				{
					ContiguousInsideLength = 0.0;
				}
				else
				{
					// bInside && !bPrevInside: start a new inside run from this point.
					ContiguousInsideLength = 0.0;
				}

				if (ContiguousInsideLength > CapInteriorRedMaxLineLengthPx)
				{
					Candidate.bRejectedByInteriorRed = true;
					Candidate.InteriorRedRejectReason = FString::Printf(
						TEXT("red stroke %d has %.1fpx contiguous inside cap polygon (threshold %.1f)"),
						RedStrokeId, ContiguousInsideLength, CapInteriorRedMaxLineLengthPx);
					return;
				}
				bPrevInside = bInside;
			}
		}
	}

	static constexpr double InteriorGreenMinInsideLengthPx = 10.0;

	struct FInteriorGreenStats
	{
		int32 StrokeId = -1;
		int32 InsidePoints = 0;
		int32 TotalPoints = 0;
		double InsideRatio = 0.0;
		double InsideLength = 0.0;
		double StrokeLength = 0.0;
	};

	static double StrokeArcLength(const FColoredStroke& Stroke)
	{
		if (Stroke.bHasMetrics)
		{
			return Stroke.Arc;
		}

		double Arc = 0.0;
		const FStroke& Points = Stroke.Points;
		for (int32 k = 1; k < Points.Num(); ++k)
		{
			Arc += (Points[k] - Points[k - 1]).Size();
		}
		return Arc;
	}

	static FInteriorGreenStats MeasureInteriorGreen(const FStroke& CapPolygon, const TArray<FColoredStroke>& Strokes, int32 StrokeId)
	{
		FInteriorGreenStats Stats;
		Stats.StrokeId = StrokeId;
		if (!Strokes.IsValidIndex(StrokeId))
		{
			return Stats;
		}

		const FColoredStroke& Stroke = Strokes[StrokeId];
		const FStroke& Points = Stroke.Points;
		Stats.TotalPoints = Points.Num();
		Stats.StrokeLength = StrokeArcLength(Stroke);
		if (Points.Num() == 0)
		{
			return Stats;
		}

		bool bPrevInside = PointInPolygon(CapPolygon, Points[0]);
		if (bPrevInside)
		{
			++Stats.InsidePoints;
		}

		for (int32 k = 1; k < Points.Num(); ++k)
		{
			const bool bInside = PointInPolygon(CapPolygon, Points[k]);
			if (bInside)
			{
				++Stats.InsidePoints;
			}

			if (bPrevInside && bInside)
			{
				Stats.InsideLength += (Points[k] - Points[k - 1]).Size();
			}
			bPrevInside = bInside;
		}

		Stats.InsideRatio = (Stats.TotalPoints > 0)
			? double(Stats.InsidePoints) / double(Stats.TotalPoints)
			: 0.0;
		return Stats;
	}

	static bool PassesInteriorGreenThresholds(const FInteriorGreenStats& Stats)
	{
		return Stats.InsideLength >= InteriorGreenMinInsideLengthPx;
	}

	struct FGreenPixelClass
	{
		bool bInside = false;
		bool bOutside = false;
		bool bBoundary = false;
	};

	struct FGreenActionStats
	{
		int32 LocalGreenCount = 0;
		double InsideLength = 0.0;
		double OutsideLength = 0.0;
		double BoundaryLength = 0.0;
		FString Action = TEXT("skip");
		FString DecisionReason = TEXT("no_local_green");
		bool bHasBestInterior = false;
		FInteriorGreenStats BestInterior;
	};

	static void BuildLoopPixelMasks(const FStroke& Loop, int32 Width, int32 Height, TArray<uint8>& OutBoundary, TArray<uint8>& OutInterior)
	{
		const int32 N = Width * Height;
		OutBoundary.Init(0, N);
		OutInterior.Init(0, N);
		if (Width <= 0 || Height <= 0 || Loop.Num() < 3)
		{
			return;
		}

		TArray<FIntPoint> LineBuf;
		auto MarkLine = [&](const FVector2D& A, const FVector2D& B)
		{
			SkelGraph::LinePoints(
				FIntPoint(FMath::RoundToInt(A.X), FMath::RoundToInt(A.Y)),
				FIntPoint(FMath::RoundToInt(B.X), FMath::RoundToInt(B.Y)), LineBuf);
			for (const FIntPoint& P : LineBuf)
			{
				if (P.X >= 0 && P.X < Width && P.Y >= 0 && P.Y < Height)
				{
					OutBoundary[P.Y * Width + P.X] = 255;
				}
			}
		};

		for (int32 k = 0; k + 1 < Loop.Num(); ++k)
		{
			MarkLine(Loop[k], Loop[k + 1]);
		}
		if ((Loop[0] - Loop.Last()).SizeSquared() > 1e-6)
		{
			MarkLine(Loop.Last(), Loop[0]);
		}

		TArray<uint8> Reachable;
		Reachable.Init(0, N);
		TArray<int32> Stack;
		Stack.Reserve(1024);
		auto TryPush = [&](int32 X, int32 Y)
		{
			if (X < 0 || X >= Width || Y < 0 || Y >= Height)
			{
				return;
			}
			const int32 Idx = Y * Width + X;
			if (OutBoundary[Idx] == 0 && Reachable[Idx] == 0)
			{
				Reachable[Idx] = 1;
				Stack.Push(Idx);
			}
		};
		for (int32 X = 0; X < Width; ++X)
		{
			TryPush(X, 0);
			TryPush(X, Height - 1);
		}
		for (int32 Y = 0; Y < Height; ++Y)
		{
			TryPush(0, Y);
			TryPush(Width - 1, Y);
		}
		while (Stack.Num() > 0)
		{
			const int32 P = Stack.Pop(EAllowShrinking::No);
			const int32 X = P % Width;
			const int32 Y = P / Width;
			TryPush(X + 1, Y);
			TryPush(X - 1, Y);
			TryPush(X, Y + 1);
			TryPush(X, Y - 1);
		}

		for (int32 Idx = 0; Idx < N; ++Idx)
		{
			if (OutBoundary[Idx] == 0 && Reachable[Idx] == 0)
			{
				OutInterior[Idx] = 255;
			}
		}
	}

	static FGreenPixelClass ClassifyGreenPixelAgainstLoop(int32 X, int32 Y, int32 Width, int32 Height, const TArray<uint8>& Boundary, const TArray<uint8>& Interior)
	{
		FGreenPixelClass C;
		if (X < 0 || X >= Width || Y < 0 || Y >= Height)
		{
			C.bOutside = true;
			return C;
		}
		const int32 Idx = Y * Width + X;
		C.bBoundary = Boundary.IsValidIndex(Idx) && Boundary[Idx] != 0;
		if (C.bBoundary)
		{
			C.bInside = true;
			C.bOutside = true;
			return C;
		}
		C.bInside = Interior.IsValidIndex(Idx) && Interior[Idx] != 0;
		C.bOutside = !C.bInside;
		return C;
	}

	static void AddGreenPixelStepLength(
		const FIntPoint& Prev, const FGreenPixelClass& PrevClass,
		const FIntPoint& Cur, const FGreenPixelClass& CurClass,
		double& InsideLength, double& OutsideLength, double& BoundaryLength)
	{
		const double Dx = double(Cur.X - Prev.X);
		const double Dy = double(Cur.Y - Prev.Y);
		const double Len = FMath::Sqrt(Dx * Dx + Dy * Dy);
		if (Len <= 0.0)
		{
			return;
		}

		const bool bBoundary = PrevClass.bBoundary || CurClass.bBoundary;
		if (bBoundary)
		{
			InsideLength += Len;
			OutsideLength += Len;
			BoundaryLength += Len;
			return;
		}

		if (PrevClass.bInside && CurClass.bInside)
		{
			InsideLength += Len;
		}
		else if (PrevClass.bOutside && CurClass.bOutside)
		{
			OutsideLength += Len;
		}
		else
		{
			// A one-pixel crossing without a rasterized boundary hit is ambiguous.
			InsideLength += Len;
			OutsideLength += Len;
		}
	}

	static FGreenActionStats MeasureLocalGreenAction(
		const FStroke& CapPolygon,
		const TArray<FColoredStroke>& Strokes,
		const TArray<int32>& LocalGreen,
		int32 Width,
		int32 Height)
	{
		FGreenActionStats Out;
		Out.LocalGreenCount = LocalGreen.Num();
		if (LocalGreen.Num() == 0)
		{
			Out.Action = TEXT("skip");
			Out.DecisionReason = TEXT("no_local_green");
			return Out;
		}

		TArray<uint8> Boundary;
		TArray<uint8> Interior;
		BuildLoopPixelMasks(CapPolygon, Width, Height, Boundary, Interior);

		TArray<FIntPoint> LineBuf;
		for (int32 g : LocalGreen)
		{
			if (!Strokes.IsValidIndex(g))
			{
				continue;
			}

			const FInteriorGreenStats InteriorStats = MeasureInteriorGreen(CapPolygon, Strokes, g);
			if (!Out.bHasBestInterior || InteriorStats.InsideLength > Out.BestInterior.InsideLength)
			{
				Out.bHasBestInterior = true;
				Out.BestInterior = InteriorStats;
			}

			const FStroke& Points = Strokes[g].Points;
			for (int32 k = 0; k + 1 < Points.Num(); ++k)
			{
				SkelGraph::LinePoints(
					FIntPoint(FMath::RoundToInt(Points[k].X), FMath::RoundToInt(Points[k].Y)),
					FIntPoint(FMath::RoundToInt(Points[k + 1].X), FMath::RoundToInt(Points[k + 1].Y)), LineBuf);
				if (LineBuf.Num() == 0)
				{
					continue;
				}

				FIntPoint Prev = LineBuf[0];
				FGreenPixelClass PrevClass = ClassifyGreenPixelAgainstLoop(Prev.X, Prev.Y, Width, Height, Boundary, Interior);
				for (int32 p = 1; p < LineBuf.Num(); ++p)
				{
					const FIntPoint Cur = LineBuf[p];
					if (Cur == Prev)
					{
						continue;
					}
					const FGreenPixelClass CurClass = ClassifyGreenPixelAgainstLoop(Cur.X, Cur.Y, Width, Height, Boundary, Interior);
					AddGreenPixelStepLength(Prev, PrevClass, Cur, CurClass, Out.InsideLength, Out.OutsideLength, Out.BoundaryLength);
					Prev = Cur;
					PrevClass = CurClass;
				}
			}
		}

		if (Out.InsideLength > Out.OutsideLength)
		{
			Out.Action = TEXT("excavate");
			Out.DecisionReason = TEXT("inside_greater_than_outside");
		}
		else if (Out.OutsideLength > Out.InsideLength)
		{
			Out.Action = TEXT("attach");
			Out.DecisionReason = TEXT("outside_greater_than_inside");
		}
		else
		{
			Out.Action = TEXT("skip");
			Out.DecisionReason = TEXT("inside_outside_equal");
		}
		return Out;
	}
	static double DistancePointToSegmentSquared2D(const FVector2D& P, const FVector2D& A, const FVector2D& B)
	{
		const FVector2D AB = B - A;
		const double LenSq = AB.SizeSquared();
		if (LenSq <= 1e-12)
		{
			return (P - A).SizeSquared();
		}

		const double T = FMath::Clamp(FVector2D::DotProduct(P - A, AB) / LenSq, 0.0, 1.0);
		return (P - (A + AB * T)).SizeSquared();
	}

	static double DistancePointToPolylineSquared(const FVector2D& P, const FStroke& Poly)
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
		if (Poly.Num() > 2 && (Poly[0] - Poly.Last()).SizeSquared() > 1e-6)
		{
			Best = FMath::Min(Best, DistancePointToSegmentSquared2D(P, Poly.Last(), Poly[0]));
		}
		return Best;
	}

	static constexpr double GreenTraceEndpointTolPx = 10.0;
	static constexpr double GreenTraceMaxChainDeviationDeg = 45.0;

	struct FGreenSideTraceCandidate
	{
		bool bValid = false;
		int32 SeedStrokeId = -1;
		TArray<int32> ChainStrokeIds;
		FVector2D ChainStart = FVector2D::ZeroVector;
		FVector2D ChainEnd = FVector2D::ZeroVector;
		FVector2D SeedDirection = FVector2D::ZeroVector;
		double SideLength = 0.0;
		double PathLength = 0.0;
		double TotalGap = 0.0;
		FString StopReason;
	};

	struct FGreenTraceEnd
	{
		FVector2D Position = FVector2D::ZeroVector;
		FVector2D OutwardDirection = FVector2D::ZeroVector;
	};

	static FString TraceGreenChainEnd(
		const TArray<FColoredStroke>& Strokes,
		const TArray<int32>& AllGreen,
		TSet<int32>& VisitedGreenIds,
		FGreenTraceEnd& ChainEnd,
		bool bPrepend,
		TArray<int32>& ChainStrokeIds,
		double& TotalGap)
	{
		const double EndpointTolSq = GreenTraceEndpointTolPx * GreenTraceEndpointTolPx;
		const double MinDirectionDot = FMath::Cos(FMath::DegreesToRadians(GreenTraceMaxChainDeviationDeg));

		for (;;)
		{
			if (ChainEnd.OutwardDirection.IsNearlyZero())
			{
				return TEXT("zero_chain_direction");
			}

			int32 BestStrokeId = -1;
			FVector2D BestNearEndpoint = FVector2D::ZeroVector;
			FVector2D BestFarEndpoint = FVector2D::ZeroVector;
			double BestDirectionDot = -1.0;
			double BestGapSq = TNumericLimits<double>::Max();
			bool bFoundNearbyEndpoint = false;
			bool bFoundUnvisitedNearbyEndpoint = false;

			for (int32 GreenStrokeId : AllGreen)
			{
				if (!Strokes.IsValidIndex(GreenStrokeId) || Strokes[GreenStrokeId].Points.Num() < 2)
				{
					continue;
				}

				const FStroke& CandidatePoints = Strokes[GreenStrokeId].Points;
				for (int32 EndpointIndex = 0; EndpointIndex < 2; ++EndpointIndex)
				{
					const FVector2D NearEndpoint = EndpointIndex == 0 ? CandidatePoints[0] : CandidatePoints.Last();
					const FVector2D FarEndpoint = EndpointIndex == 0 ? CandidatePoints.Last() : CandidatePoints[0];
					const double GapSq = FVector2D::DistSquared(ChainEnd.Position, NearEndpoint);
					if (GapSq > EndpointTolSq)
					{
						continue;
					}
					bFoundNearbyEndpoint = true;
					if (VisitedGreenIds.Contains(GreenStrokeId))
					{
						continue;
					}
					bFoundUnvisitedNearbyEndpoint = true;

					const FVector2D CandidateDirection = (FarEndpoint - NearEndpoint).GetSafeNormal();
					if (CandidateDirection.IsNearlyZero())
					{
						continue;
					}

					const double DirectionDot = FVector2D::DotProduct(
						ChainEnd.OutwardDirection, CandidateDirection);
					if (DirectionDot <= MinDirectionDot)
					{
						continue;
					}

					if (DirectionDot > BestDirectionDot ||
						(DirectionDot == BestDirectionDot && GapSq < BestGapSq) ||
						(DirectionDot == BestDirectionDot && GapSq == BestGapSq &&
							(BestStrokeId < 0 || GreenStrokeId < BestStrokeId)))
					{
						BestStrokeId = GreenStrokeId;
						BestNearEndpoint = NearEndpoint;
						BestFarEndpoint = FarEndpoint;
						BestDirectionDot = DirectionDot;
						BestGapSq = GapSq;
					}
				}
			}

			if (BestStrokeId < 0)
			{
				if (!bFoundNearbyEndpoint)
				{
					return TEXT("no_nearby_green");
				}
				if (!bFoundUnvisitedNearbyEndpoint)
				{
					return TEXT("all_nearby_green_visited");
				}
				return TEXT("no_chain_direction_match");
			}

			VisitedGreenIds.Add(BestStrokeId);
			if (bPrepend)
			{
				ChainStrokeIds.Insert(BestStrokeId, 0);
			}
			else
			{
				ChainStrokeIds.Add(BestStrokeId);
			}
			TotalGap += FMath::Sqrt(BestGapSq);
			ChainEnd.Position = BestFarEndpoint;
			ChainEnd.OutwardDirection = (BestFarEndpoint - BestNearEndpoint).GetSafeNormal();
		}
	}

	static FGreenSideTraceCandidate TraceGreenSideFromSeed(
		const TArray<FColoredStroke>& Strokes,
		const TArray<int32>& AllGreen,
		int32 SeedStrokeId,
		const FStroke& CapPolygon)
	{
		FGreenSideTraceCandidate Out;
		Out.SeedStrokeId = SeedStrokeId;
		if (!Strokes.IsValidIndex(SeedStrokeId) || Strokes[SeedStrokeId].Points.Num() < 2)
		{
			Out.StopReason = TEXT("invalid_seed");
			return Out;
		}

		const FStroke& SeedPoints = Strokes[SeedStrokeId].Points;
		Out.SeedDirection = (SeedPoints.Last() - SeedPoints[0]).GetSafeNormal();
		if (Out.SeedDirection.IsNearlyZero())
		{
			Out.StopReason = TEXT("zero_seed_direction");
			return Out;
		}

		TSet<int32> VisitedGreenIds;
		VisitedGreenIds.Add(SeedStrokeId);
		Out.ChainStrokeIds.Add(SeedStrokeId);

		FGreenTraceEnd FirstEnd;
		FirstEnd.Position = SeedPoints[0];
		FirstEnd.OutwardDirection = (SeedPoints[0] - SeedPoints.Last()).GetSafeNormal();
		FGreenTraceEnd LastEnd;
		LastEnd.Position = SeedPoints.Last();
		LastEnd.OutwardDirection = (SeedPoints.Last() - SeedPoints[0]).GetSafeNormal();

		const FString FirstStopReason = TraceGreenChainEnd(
			Strokes, AllGreen, VisitedGreenIds, FirstEnd, true,
			Out.ChainStrokeIds, Out.TotalGap);
		const FString LastStopReason = TraceGreenChainEnd(
			Strokes, AllGreen, VisitedGreenIds, LastEnd, false,
			Out.ChainStrokeIds, Out.TotalGap);

		const double FirstDistanceToCapSq =
			DistancePointToPolylineSquared(FirstEnd.Position, CapPolygon);
		const double LastDistanceToCapSq =
			DistancePointToPolylineSquared(LastEnd.Position, CapPolygon);
		if (FirstDistanceToCapSq <= LastDistanceToCapSq)
		{
			Out.ChainStart = FirstEnd.Position;
			Out.ChainEnd = LastEnd.Position;
		}
		else
		{
			Out.ChainStart = LastEnd.Position;
			Out.ChainEnd = FirstEnd.Position;
			Algo::Reverse(Out.ChainStrokeIds);
		}

		Out.PathLength = Out.TotalGap;
		for (int32 GreenStrokeId : Out.ChainStrokeIds)
		{
			if (Strokes.IsValidIndex(GreenStrokeId))
			{
				Out.PathLength += StrokeArcLength(Strokes[GreenStrokeId]);
			}
		}

		Out.SideLength = FVector2D::Distance(Out.ChainStart, Out.ChainEnd);
		Out.bValid = Out.SideLength > 0.0;
		Out.StopReason = FString::Printf(
			TEXT("bidirectional:first=%s,last=%s"), *FirstStopReason, *LastStopReason);
		return Out;
	}

	static FString MakeGreenChainKey(const TArray<int32>& ChainStrokeIds)
	{
		TArray<int32> SortedIds = ChainStrokeIds;
		SortedIds.Sort();

		FString Key;
		for (int32 StrokeId : SortedIds)
		{
			Key += FString::Printf(TEXT("%d,"), StrokeId);
		}
		return Key;
	}

	// Local green strokes only seed discovery. Each seed is traced from both ends,
	// duplicate chains are removed, and the longest full path is selected. The
	// selected path is oriented only after tracing, from its cap-near endpoint to
	// its cap-far endpoint.
	static void ApplyGreenSideForCap(
		const TArray<FColoredStroke>& Strokes,
		const TArray<int32>& AllGreen,
		const TArray<int32>& LocalGreenSeeds,
		FCapExtrusionResult& Out)
	{
		Out.SideCandidateVectors.Reset();
		Out.SideCandidateStarts.Reset();
		Out.SideCandidateEnds.Reset();
		Out.SideStrokeId = -1;
		Out.SideVector = FVector2D::ZeroVector;
		Out.SideLength = 0.0;
		Out.SideChainPathLength = 0.0;
		Out.SideChainStart = FVector2D::ZeroVector;
		Out.SideChainEnd = FVector2D::ZeroVector;
		Out.SideSeedDirection = FVector2D::ZeroVector;
		Out.SideChainStrokeIds.Reset();
		Out.SideTraceTotalGap = 0.0;
		Out.SideTraceStopReason = TEXT("no_local_green_seed");

		FGreenSideTraceCandidate Best;
		TSet<FString> SeenChainKeys;
		for (int32 SeedStrokeId : LocalGreenSeeds)
		{
			const FGreenSideTraceCandidate Candidate =
				TraceGreenSideFromSeed(Strokes, AllGreen, SeedStrokeId, Out.CapPolygon);
			if (!Candidate.bValid)
			{
				continue;
			}

			const FString ChainKey = MakeGreenChainKey(Candidate.ChainStrokeIds);
			if (SeenChainKeys.Contains(ChainKey))
			{
				continue;
			}
			SeenChainKeys.Add(ChainKey);

			const bool bLongerPath = !Best.bValid || Candidate.PathLength > Best.PathLength + 1e-6;
			const bool bSamePathButLongerChord =
				Best.bValid && FMath::IsNearlyEqual(Candidate.PathLength, Best.PathLength, 1e-6) &&
				Candidate.SideLength > Best.SideLength + 1e-6;
			if (bLongerPath || bSamePathButLongerChord)
			{
				Best = Candidate;
			}
		}

		if (Best.bValid)
		{
			const FVector2D ChainVector = Best.ChainEnd - Best.ChainStart;
			Out.SideStrokeId = Best.SeedStrokeId;
			Out.SideLength = ChainVector.Size();
			Out.SideChainPathLength = Best.PathLength;
			Out.SideChainStart = Best.ChainStart;
			Out.SideChainEnd = Best.ChainEnd;
			Out.SideSeedDirection = Best.SeedDirection;
			Out.SideChainStrokeIds = Best.ChainStrokeIds;
			Out.SideTraceTotalGap = Best.TotalGap;
			Out.SideTraceStopReason = Best.StopReason;
			Out.SideVector = ChainVector;
			Out.SideCandidateStarts.Add(Best.ChainStart);
			Out.SideCandidateEnds.Add(Best.ChainEnd);
			Out.SideCandidateVectors.Add(Out.SideVector);
		}
		Out.CapPolygonTranslated.Reset();
		Out.CapPolygonTranslated.Reserve(Out.CapPolygon.Num());
		for (const FVector2D& P : Out.CapPolygon) { Out.CapPolygonTranslated.Add(P + Out.SideVector); }
	}

	static void PrefilterLoopCandidateByLocalGreen(
		FLoopCandidate& Candidate,
		const TArray<FColoredStroke>& Strokes,
		const TArray<int32>& AllGreen,
		double GreenSelectToleranceSquared,
		int32 Width,
		int32 Height)
	{
		Candidate.bHasValidLocalGreenAction = false;
		Candidate.GreenPrefilterReason.Reset();
		Candidate.LocalGreenStrokeIds.Reset();
		Candidate.GreenBoundaryLength = 0.0;

		bool bHasCandidateRedPoints = false;
		for (int32 RedStrokeId : Candidate.RealRedStrokeIds)
		{
			if (Strokes.IsValidIndex(RedStrokeId) && Strokes[RedStrokeId].Points.Num() > 0)
			{
				bHasCandidateRedPoints = true;
				break;
			}
		}
		if (!bHasCandidateRedPoints)
		{
			Candidate.GreenPrefilterReason = TEXT("candidate has no real red points for local green discovery");
			return;
		}

		CollectLocalGreenStrokeIdsForRedStrokes(
			Strokes,
			Candidate.RealRedStrokeIds,
			AllGreen,
			GreenSelectToleranceSquared,
			Candidate.LocalGreenStrokeIds);

		if (Candidate.LocalGreenStrokeIds.Num() == 0)
		{
			Candidate.GreenPrefilterReason = TEXT("candidate has no local green stroke within the selection tolerance");
			return;
		}

		FCapExtrusionResult EvaluatedResult = Candidate.Result;
		ApplyGreenSideForCap(Strokes, AllGreen, Candidate.LocalGreenStrokeIds, EvaluatedResult);
		if (EvaluatedResult.SideChainStrokeIds.Num() == 0 ||
			EvaluatedResult.SideVector.SizeSquared() <= 1e-8)
		{
			Candidate.Result = MoveTemp(EvaluatedResult);
			Candidate.GreenPrefilterReason = TEXT("local green exists but no valid non-zero green chain was traced");
			return;
		}

		const FGreenActionStats ActionStats = MeasureLocalGreenAction(
			EvaluatedResult.CapPolygon,
			Strokes,
			Candidate.LocalGreenStrokeIds,
			Width,
			Height);
		EvaluatedResult.Action = ActionStats.Action;
		EvaluatedResult.ActionDecisionReason = ActionStats.DecisionReason;
		EvaluatedResult.LocalGreenStrokeCount = ActionStats.LocalGreenCount;
		EvaluatedResult.GreenInsideTotalLength = ActionStats.InsideLength;
		EvaluatedResult.GreenOutsideTotalLength = ActionStats.OutsideLength;
		EvaluatedResult.bHasInteriorGreen = EvaluatedResult.Action == TEXT("excavate");
		if (ActionStats.bHasBestInterior)
		{
			EvaluatedResult.InteriorGreenStrokeId = ActionStats.BestInterior.StrokeId;
			EvaluatedResult.InteriorGreenInsidePoints = ActionStats.BestInterior.InsidePoints;
			EvaluatedResult.InteriorGreenTotalPoints = ActionStats.BestInterior.TotalPoints;
			EvaluatedResult.InteriorGreenInsideRatio = ActionStats.BestInterior.InsideRatio;
			EvaluatedResult.InteriorGreenInsideLength = ActionStats.BestInterior.InsideLength;
			EvaluatedResult.InteriorGreenStrokeLength = ActionStats.BestInterior.StrokeLength;
		}

		Candidate.Result = MoveTemp(EvaluatedResult);
		Candidate.GreenBoundaryLength = ActionStats.BoundaryLength;
		Candidate.bHasValidLocalGreenAction =
			Candidate.Result.Action == TEXT("attach") ||
			Candidate.Result.Action == TEXT("excavate");
		Candidate.GreenPrefilterReason = Candidate.bHasValidLocalGreenAction
			? TEXT("valid_local_green_action")
			: FString::Printf(
				TEXT("local green action is not attach/excavate: %s (%s)"),
				*Candidate.Result.Action,
				*Candidate.Result.ActionDecisionReason);
	}

	int32 RecoverCapExtrusionsPerComponent(const TArray<FColoredStroke>& Strokes, float ConnectorTol, float BlackSelectTol, int32 Width, int32 Height, const FString& PressDir, const FString& ActionPressDir, TArray<FCapExtrusionResult>& OutResults)
	{
		using namespace CapOps;
		OutResults.Reset();

		TArray<int32> SourceRedIdx, SourceBlackIdx;
		for (int32 i = 0; i < Strokes.Num(); ++i)
		{
			switch (Strokes[i].Color)
			{
			case EStrokeColor::Red:   SourceRedIdx.Add(i); break;
			case EStrokeColor::Black: SourceBlackIdx.Add(i); break;
			default: break;
			}
		}

		TArray<FColoredStroke> TraceStrokes;
		int32 FirstSyntheticStrokeId = Strokes.Num();
		BuildPlanarEndpointConnectorStrokes(
			Strokes, SourceRedIdx, SourceBlackIdx, ConnectorTol,
			TraceStrokes, FirstSyntheticStrokeId);

		TArray<int32> RedIdx, BlackIdx, GreenIdx;
		for (int32 i = 0; i < TraceStrokes.Num(); ++i)
		{
			switch (TraceStrokes[i].Color)
			{
			case EStrokeColor::Red:   RedIdx.Add(i); break;
			case EStrokeColor::Black: BlackIdx.Add(i); break;
			case EStrokeColor::Green: GreenIdx.Add(i); break;
			default: break;
			}
		}

		{
			TArray<int32> AllRedBlackIdx = RedIdx;
			AllRedBlackIdx.Append(BlackIdx);
			if (AllRedBlackIdx.Num() > 0)
			{
				FGraph AllRedBlackGraph;
				BuildGraph(TraceStrokes, AllRedBlackIdx, CapLoopGraphNodeSnapTol, FirstSyntheticStrokeId, AllRedBlackGraph);
				TArray<uint8> AllEdges;
				AllEdges.Init(1, AllRedBlackGraph.StrokeId.Num());
				SaveGraphPng(TraceStrokes, AllRedBlackGraph, AllEdges, Width, Height, PressDir / TEXT("09_all_red_black_graph.png"));
				SaveGraphJson(TraceStrokes, AllRedBlackGraph, AllEdges, PressDir / TEXT("09_all_red_black_graph.json"));
			}
		}

		if (RedIdx.Num() == 0)
		{
			return 0;
		}

		const double SelTol2 = double(BlackSelectTol) * double(BlackSelectTol);

		TArray<FLoopCandidate> Candidates;
		CollectRedFirstLoopCandidates(
			TraceStrokes,
			RedIdx,
			BlackIdx,
			GreenIdx,
			CapLoopGraphNodeSnapTol,
			FirstSyntheticStrokeId,
			SelTol2,
			Width,
			Height,
			PressDir,
			Candidates);
		for (FLoopCandidate& Candidate : Candidates)
		{
			PrefilterLoopCandidateByLocalGreen(
				Candidate,
				TraceStrokes,
				GreenIdx,
				SelTol2,
				Width,
				Height);
		}

		// Interior red line check: reject any candidate whose cap polygon
		// interior contains a red stroke segment longer than 20px.
		for (FLoopCandidate& Candidate : Candidates)
		{
			if (!Candidate.bHasValidLocalGreenAction)
			{
				continue; // already rejected by green prefilter
			}
			CheckCapInteriorRedLines(Candidate, TraceStrokes, RedIdx);
		}

		TArray<FFromLZCandidateFaceRequest> FaceRequests;
		FaceRequests.Reserve(Candidates.Num());
		for (const FLoopCandidate& Candidate : Candidates)
		{
			FFromLZCandidateFaceRequest Request;
			Request.CandidateSource = Candidate.Source;
			Request.Action = Candidate.Result.Action;
			Request.CapPolygon = Candidate.Result.CapPolygon;
			Request.CapPolygonTranslated = Candidate.Result.CapPolygonTranslated;
			Request.SideVectors = Candidate.Result.SideCandidateVectors;
			if (Request.SideVectors.Num() == 0 && Candidate.Result.SideVector.SizeSquared() > 1e-8)
			{
				Request.SideVectors.Add(Candidate.Result.SideVector);
			}
			FaceRequests.Add(MoveTemp(Request));
		}

		TArray<FFromLZCandidateFaceEvaluation> FaceEvaluations;
		FString FaceEvaluationError;
		const bool bFaceEvaluationBatchReady = FFromLZFaceReconstructor::EvaluateCandidateFaces(
			PressDir, Width, Height, FaceRequests, FaceEvaluations, FaceEvaluationError);
		for (int32 CandidateIndex = 0; CandidateIndex < Candidates.Num(); ++CandidateIndex)
		{
			FLoopCandidate& Candidate = Candidates[CandidateIndex];
			Candidate.bHasValidFaceEvaluation = false;
			if (!bFaceEvaluationBatchReady)
			{
				Candidate.FaceEvaluationReason = FString::Printf(
					TEXT("candidate face evaluation unavailable: %s"), *FaceEvaluationError);
				Candidate.Result.FaceEvaluationRejectReason = Candidate.FaceEvaluationReason;
				continue;
			}
			if (!FaceEvaluations.IsValidIndex(CandidateIndex))
			{
				Candidate.FaceEvaluationReason = TEXT("candidate face evaluation result is missing");
				Candidate.Result.FaceEvaluationRejectReason = Candidate.FaceEvaluationReason;
				continue;
			}

			const FFromLZCandidateFaceEvaluation& Evaluation = FaceEvaluations[CandidateIndex];
			Candidate.bHasValidFaceEvaluation = Evaluation.bValid;
			Candidate.FaceEvaluationReason = Evaluation.RejectReason;
			Candidate.Result.bFaceEvaluationValid = Evaluation.bValid;
			Candidate.Result.FaceEvaluationSourcePolygon = Evaluation.SourcePolygonKey;
			Candidate.Result.FaceEvaluationRejectReason = Evaluation.RejectReason;
			Candidate.Result.FaceEvaluationCapMaskPixels = Evaluation.CapMaskPixels;
			Candidate.Result.PreselectedFaceId = Evaluation.SelectedFaceId;
			Candidate.Result.PreselectedFaceOverlapPixels = Evaluation.SelectedFaceOverlapPixels;
			Candidate.Result.PreselectedFaceOverlapRatio = Evaluation.SelectedFaceOverlapRatio;
			Candidate.Result.PreselectedFaceNormalSideAngleDegrees = Evaluation.SelectedFaceNormalSideAngleDegrees;
			Candidate.Result.PreselectedFaceDistanceToCamera = Evaluation.SelectedFaceDistanceToCamera;
		}

		TArray<int32> SelectedCandidateIndices;
		FLoopSelectionDiagnostics SelectionDiagnostics;
		SelectLoopCandidates(Candidates, TraceStrokes, SelectedCandidateIndices, SelectionDiagnostics);
		SaveLoopCandidatesJson(Candidates, SelectionDiagnostics, PressDir / TEXT("09_loop_candidates.json"));

		if (SelectedCandidateIndices.Num() == 0)
		{
			return 0;
		}

		// Pre-warm the image-wrapper module on this thread before parallel writes.
		FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));

		// Each selected red-driven loop writes into its own Component_%% folder.
		OutResults.SetNum(SelectedCandidateIndices.Num());
		FromLZProcessing::LimitedParallelFor(SelectedCandidateIndices.Num(), [&](int32 i)
		{
			const FLoopCandidate Candidate = Candidates[SelectedCandidateIndices[i]];
			const FString CompDir = PressDir / FString::Printf(TEXT("Component_%02d"), i + 1);
			IFileManager::Get().MakeDirectory(*CompDir, /*Tree*/ true);

			FCapExtrusionResult R = Candidate.Result;

			{
				FGraph SelectedGraph;
				BuildGraph(TraceStrokes, R.CapStrokeIds, CapLoopGraphNodeSnapTol, FirstSyntheticStrokeId, SelectedGraph);
				TArray<uint8> All;
				All.Init(1, SelectedGraph.StrokeId.Num());
				SaveGraphPng(TraceStrokes, SelectedGraph, All, Width, Height, CompDir / TEXT("09a_caploop_candidate.png"));
				SaveGraphJson(TraceStrokes, SelectedGraph, All, CompDir / TEXT("09a_caploop_candidate_graph.json"));
				SaveGraphPng(TraceStrokes, SelectedGraph, All, Width, Height, CompDir / TEXT("09b_caploop_pruned.png"));
				SaveGraphJson(TraceStrokes, SelectedGraph, All, CompDir / TEXT("09b_caploop_pruned_graph.json"));
			}

			SaveCapExtrusionPng(TraceStrokes, R, Width, Height, CompDir / TEXT("09_cap_extrusion.png"));
			SaveCapExtrusionJson(R, CompDir / TEXT("09_cap_extrusion.json"));

			const FString ActionCompDir = ActionPressDir / FString::Printf(TEXT("Component_%02d"), i + 1);
			IFileManager::Get().MakeDirectory(*ActionCompDir, /*Tree*/ true);
			const FString ActionJson = FString::Printf(
				TEXT("{\n")
				TEXT("  \"action\": \"%s\",\n")
				TEXT("  \"has_interior_green\": %s,\n")
				TEXT("  \"local_green_count\": %d,\n")
				TEXT("  \"inside_green_total\": %.3f,\n")
				TEXT("  \"outside_green_total\": %.3f,\n")
				TEXT("  \"boundary_green_total\": %.3f,\n")
				TEXT("  \"decision_reason\": \"%s\",\n")
				TEXT("  \"interior_green_stroke_id\": %d,\n")
				TEXT("  \"interior_green_inside_length\": %.3f,\n")
				TEXT("  \"interior_green_min_inside_length\": %.3f\n")
				TEXT("}\n"),
				*R.Action,
				R.bHasInteriorGreen ? TEXT("true") : TEXT("false"),
				R.LocalGreenStrokeCount,
				R.GreenInsideTotalLength,
				R.GreenOutsideTotalLength,
				Candidate.GreenBoundaryLength,
				*JsonEscaped(R.ActionDecisionReason),
				R.InteriorGreenStrokeId,
				R.InteriorGreenInsideLength,
				InteriorGreenMinInsideLengthPx);
			FFileHelper::SaveStringToFile(ActionJson, *(ActionCompDir / TEXT("Action.json")));

			OutResults[i] = R;
		});

		return SelectedCandidateIndices.Num();
	}

	bool SaveCapExtrusionPng(const TArray<FColoredStroke>& Strokes, const FCapExtrusionResult& Res, int32 Width, int32 Height, const FString& Path, int32 Thickness)
	{
		const int32 N = Width * Height;
		TArray<uint8> RGBA;
		RGBA.Init(255, N * 4);
		const int32 Rad = FMath::Max(0, Thickness / 2);

		auto PlotCol = [&](int32 x, int32 y, uint8 r, uint8 g, uint8 b)
		{
			for (int32 oy = -Rad; oy <= Rad; ++oy)
			{
				for (int32 ox = -Rad; ox <= Rad; ++ox)
				{
					const int32 xx = x + ox;
					const int32 yy = y + oy;
					if (xx < 0 || xx >= Width || yy < 0 || yy >= Height)
					{
						continue;
					}
					const int32 Off = (yy * Width + xx) * 4;
					RGBA[Off + 0] = r; RGBA[Off + 1] = g; RGBA[Off + 2] = b; RGBA[Off + 3] = 255;
				}
			}
		};
		TArray<FIntPoint> LineBuf;
		auto DrawPoly = [&](const FStroke& Poly, uint8 r, uint8 g, uint8 b)
		{
			for (int32 k = 0; k + 1 < Poly.Num(); ++k)
			{
				SkelGraph::LinePoints(
					FIntPoint(FMath::RoundToInt(Poly[k].X), FMath::RoundToInt(Poly[k].Y)),
					FIntPoint(FMath::RoundToInt(Poly[k + 1].X), FMath::RoundToInt(Poly[k + 1].Y)), LineBuf);
				for (const FIntPoint& P : LineBuf)
				{
					PlotCol(P.X, P.Y, r, g, b);
				}
			}
		};

		// Selected source green chain. Its start-to-end vector determines both the
		// extrusion direction and side length.
		for (int32 StrokeId : Res.SideChainStrokeIds)
		{
			if (Strokes.IsValidIndex(StrokeId))
			{
				DrawPoly(Strokes[StrokeId].Points, 0, 200, 0);
			}
		}

		// Side connectors (green) between corresponding cap nodes.
		for (const FVector2D& Nd : Res.CapNodes)
		{
			const FVector2D Nd2 = Nd + Res.SideVector;
			SkelGraph::LinePoints(
				FIntPoint(FMath::RoundToInt(Nd.X), FMath::RoundToInt(Nd.Y)),
				FIntPoint(FMath::RoundToInt(Nd2.X), FMath::RoundToInt(Nd2.Y)), LineBuf);
			for (const FIntPoint& P : LineBuf)
			{
				PlotCol(P.X, P.Y, 34, 139, 34);
			}
		}

		DrawPoly(Res.CapPolygonTranslated, 255, 140, 0); // translated cap: orange
		DrawPoly(Res.CapPolygon, 220, 20, 60);           // original cap: red

		IImageWrapperModule& IWM = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
		TSharedPtr<IImageWrapper> IW = IWM.CreateImageWrapper(EImageFormat::PNG);
		if (!IW.IsValid())
		{
			return false;
		}
		IW->SetRaw(RGBA.GetData(), RGBA.Num(), Width, Height, ERGBFormat::RGBA, 8);
		const TArray64<uint8>& C = IW->GetCompressed();
		return FFileHelper::SaveArrayToFile(TArrayView<const uint8>(C.GetData(), static_cast<int32>(C.Num())), *Path);
	}

	bool SaveCapExtrusionJson(const FCapExtrusionResult& Res, const FString& Path)
	{
		FString Json;
		Json += TEXT("{\n");
		Json += FString::Printf(TEXT("  \"found\": %s,\n"), Res.bFound ? TEXT("true") : TEXT("false"));
		Json += FString::Printf(TEXT("  \"used_black\": %s,\n"), Res.bUsedBlack ? TEXT("true") : TEXT("false"));
		Json += FString::Printf(TEXT("  \"side_stroke_id\": %d,\n"), Res.SideStrokeId);
		Json += FString::Printf(TEXT("  \"side_vector\": [%.3f, %.3f],\n"), Res.SideVector.X, Res.SideVector.Y);
		Json += FString::Printf(TEXT("  \"side_length\": %.3f,\n"), Res.SideLength);
		Json += FString::Printf(TEXT("  \"side_chain_path_length\": %.3f,\n"), Res.SideChainPathLength);
		Json += FString::Printf(TEXT("  \"side_chain_start\": [%.3f, %.3f],\n"), Res.SideChainStart.X, Res.SideChainStart.Y);
		Json += FString::Printf(TEXT("  \"side_chain_end\": [%.3f, %.3f],\n"), Res.SideChainEnd.X, Res.SideChainEnd.Y);
		Json += FString::Printf(TEXT("  \"side_seed_direction\": [%.6f, %.6f],\n"), Res.SideSeedDirection.X, Res.SideSeedDirection.Y);
		Json += FString::Printf(TEXT("  \"side_trace_total_gap\": %.3f,\n"), Res.SideTraceTotalGap);
		Json += FString::Printf(TEXT("  \"side_trace_stop_reason\": \"%s\",\n"), *JsonEscaped(Res.SideTraceStopReason));
		Json += FString::Printf(TEXT("  \"green_trace_endpoint_tol\": %.3f,\n"), GreenTraceEndpointTolPx);
		Json += FString::Printf(TEXT("  \"green_trace_max_chain_deviation_degrees\": %.3f,\n"), GreenTraceMaxChainDeviationDeg);
		Json += TEXT("  \"side_chain_stroke_ids\": [");
		for (int32 i = 0; i < Res.SideChainStrokeIds.Num(); ++i)
		{
			Json += FString::Printf(TEXT("%s%d"), i == 0 ? TEXT("") : TEXT(", "), Res.SideChainStrokeIds[i]);
		}
		Json += TEXT("],\n");
		Json += FString::Printf(TEXT("  \"action\": \"%s\",\n"), *JsonEscaped(Res.Action));
		Json += FString::Printf(TEXT("  \"decision_reason\": \"%s\",\n"), *JsonEscaped(Res.ActionDecisionReason));
		Json += FString::Printf(TEXT("  \"local_green_count\": %d,\n"), Res.LocalGreenStrokeCount);
		Json += FString::Printf(TEXT("  \"inside_green_total\": %.3f,\n"), Res.GreenInsideTotalLength);
		Json += FString::Printf(TEXT("  \"outside_green_total\": %.3f,\n"), Res.GreenOutsideTotalLength);
		Json += FString::Printf(TEXT("  \"has_interior_green\": %s,\n"), Res.bHasInteriorGreen ? TEXT("true") : TEXT("false"));
		Json += FString::Printf(TEXT("  \"interior_green_stroke_id\": %d,\n"), Res.InteriorGreenStrokeId);
		Json += FString::Printf(TEXT("  \"interior_green_inside_points\": %d,\n"), Res.InteriorGreenInsidePoints);
		Json += FString::Printf(TEXT("  \"interior_green_total_points\": %d,\n"), Res.InteriorGreenTotalPoints);
		Json += FString::Printf(TEXT("  \"interior_green_inside_ratio\": %.6f,\n"), Res.InteriorGreenInsideRatio);
		Json += FString::Printf(TEXT("  \"interior_green_inside_length\": %.3f,\n"), Res.InteriorGreenInsideLength);
		Json += FString::Printf(TEXT("  \"interior_green_stroke_length\": %.3f,\n"), Res.InteriorGreenStrokeLength);
		Json += FString::Printf(TEXT("  \"interior_green_min_inside_length\": %.3f,\n"), InteriorGreenMinInsideLengthPx);
		Json += FString::Printf(TEXT("  \"candidate_source\": \"%s\",\n"), *Res.CandidateSource);
		Json += FString::Printf(TEXT("  \"candidate_anchor_stroke_id\": %d,\n"), Res.CandidateAnchorStrokeId);
		Json += FString::Printf(TEXT("  \"face_evaluation_valid\": %s,\n"), Res.bFaceEvaluationValid ? TEXT("true") : TEXT("false"));
		Json += FString::Printf(TEXT("  \"face_evaluation_source_polygon\": \"%s\",\n"), *JsonEscaped(Res.FaceEvaluationSourcePolygon));
		Json += FString::Printf(TEXT("  \"face_evaluation_reject_reason\": \"%s\",\n"), *JsonEscaped(Res.FaceEvaluationRejectReason));
		Json += FString::Printf(TEXT("  \"face_evaluation_cap_mask_pixels\": %d,\n"), Res.FaceEvaluationCapMaskPixels);
		Json += FString::Printf(TEXT("  \"preselected_face_id\": %d,\n"), Res.PreselectedFaceId);
		Json += FString::Printf(TEXT("  \"preselected_face_overlap_pixels\": %d,\n"), Res.PreselectedFaceOverlapPixels);
		Json += FString::Printf(TEXT("  \"preselected_face_overlap_ratio\": %.6f,\n"), Res.PreselectedFaceOverlapRatio);
		Json += FString::Printf(TEXT("  \"preselected_face_normal_side_angle_degrees\": %.6f,\n"), Res.PreselectedFaceNormalSideAngleDegrees);
		Json += FString::Printf(TEXT("  \"preselected_face_distance_to_camera\": %.6f,\n"), Res.PreselectedFaceDistanceToCamera);

		// Selected green side stroke (chord vector + endpoint segment).
		Json += TEXT("  \"side_vectors\": [");
		for (int32 i = 0; i < Res.SideCandidateVectors.Num(); ++i)
		{
			Json += FString::Printf(TEXT("%s[%.3f, %.3f]"), (i == 0 ? TEXT("") : TEXT(", ")),
				Res.SideCandidateVectors[i].X, Res.SideCandidateVectors[i].Y);
		}
		Json += TEXT("],\n");

		Json += TEXT("  \"side_segments\": [");
		for (int32 i = 0; i < Res.SideCandidateStarts.Num(); ++i)
		{
			Json += FString::Printf(TEXT("%s[[%.2f, %.2f], [%.2f, %.2f]]"), (i == 0 ? TEXT("") : TEXT(", ")),
				Res.SideCandidateStarts[i].X, Res.SideCandidateStarts[i].Y,
				Res.SideCandidateEnds[i].X, Res.SideCandidateEnds[i].Y);
		}
		Json += TEXT("],\n");

		Json += TEXT("  \"cap_stroke_ids\": [");
		for (int32 i = 0; i < Res.CapStrokeIds.Num(); ++i)
		{
			Json += FString::Printf(TEXT("%s%d"), (i == 0 ? TEXT("") : TEXT(", ")), Res.CapStrokeIds[i]);
		}
		Json += TEXT("],\n");

		Json += TEXT("  \"ordered_boundary_runs\": [\n");
		for (int32 RunIndex = 0; RunIndex < Res.OrderedBoundaryRuns.Num(); ++RunIndex)
		{
			const FCapBoundaryRun& Run = Res.OrderedBoundaryRuns[RunIndex];
			const TCHAR* Type = Run.bSynthetic
				? TEXT("connector")
				: (Run.Color == EStrokeColor::Black ? TEXT("black") : TEXT("red"));
			Json += TEXT("    {\n");
			Json += FString::Printf(TEXT("      \"stroke_id\": %d,\n"), Run.StrokeId);
			Json += FString::Printf(TEXT("      \"type\": \"%s\",\n"), Type);
			Json += FString::Printf(TEXT("      \"color\": \"%s\",\n"), StrokeColorToString(Run.Color));
			Json += FString::Printf(TEXT("      \"synthetic\": %s,\n"), Run.bSynthetic ? TEXT("true") : TEXT("false"));
			Json += FString::Printf(TEXT("      \"reversed\": %s,\n"), Run.bReversed ? TEXT("true") : TEXT("false"));
			Json += FString::Printf(TEXT("      \"start_node_id\": %d,\n"), Run.StartNodeId);
			Json += FString::Printf(TEXT("      \"end_node_id\": %d,\n"), Run.EndNodeId);
			Json += FString::Printf(
				TEXT("      \"start_node_position\": [%.6f, %.6f],\n"),
				Run.StartNodePosition.X,
				Run.StartNodePosition.Y);
			Json += FString::Printf(
				TEXT("      \"end_node_position\": [%.6f, %.6f],\n"),
				Run.EndNodePosition.X,
				Run.EndNodePosition.Y);
			Json += FString::Printf(TEXT("      \"arc_length_pixels\": %.9g,\n"), Run.ArcLengthPixels);
			Json += FString::Printf(TEXT("      \"chord_length_pixels\": %.9g,\n"), Run.ChordLengthPixels);
			Json += FString::Printf(TEXT("      \"straightness\": %.9g,\n"), Run.Straightness);
			Json += TEXT("      \"points\": [");
			for (int32 PointIndex = 0; PointIndex < Run.Points.Num(); ++PointIndex)
			{
				const FVector2D& Point = Run.Points[PointIndex];
				Json += FString::Printf(
					TEXT("%s[%.6f, %.6f]"),
					PointIndex == 0 ? TEXT("") : TEXT(", "),
					Point.X,
					Point.Y);
			}
			Json += TEXT("]\n");
			Json += FString::Printf(
				TEXT("    }%s\n"),
				RunIndex + 1 < Res.OrderedBoundaryRuns.Num() ? TEXT(",") : TEXT(""));
		}
		Json += TEXT("  ],\n");

		auto WritePoly = [&](const TCHAR* Key, const FStroke& Poly, bool bTrailingComma)
		{
			Json += FString::Printf(TEXT("  \"%s\": ["), Key);
			for (int32 k = 0; k < Poly.Num(); ++k)
			{
				Json += FString::Printf(TEXT("%s[%.2f, %.2f]"), (k == 0 ? TEXT("") : TEXT(", ")), Poly[k].X, Poly[k].Y);
			}
			Json += bTrailingComma ? TEXT("],\n") : TEXT("]\n");
		};
		WritePoly(TEXT("cap_polygon"), Res.CapPolygon, true);
		WritePoly(TEXT("cap_polygon_translated"), Res.CapPolygonTranslated, false);
		Json += TEXT("}\n");

		return FFileHelper::SaveStringToFile(Json, *Path);
	}

	static void StrokeColorRGB(EStrokeColor Color, uint8& R, uint8& G, uint8& B)
	{
		switch (Color)
		{
		case EStrokeColor::Red:   R = 220; G = 20;  B = 60;  break;
		case EStrokeColor::Green: R = 34;  G = 139; B = 34;  break;
		case EStrokeColor::Blue:  R = 30;  G = 144; B = 255; break;
		case EStrokeColor::Black: R = 0;   G = 0;   B = 0;   break;
		default:                  R = 160; G = 160; B = 160; break; // None -> gray
		}
	}

	bool SaveColoredStrokesPng(const TArray<FColoredStroke>& Strokes, int32 Width, int32 Height, const FString& Path, int32 Thickness)
	{
		const int32 N = Width * Height;
		TArray<uint8> RGBA;
		RGBA.Init(255, N * 4);
		const int32 Rad = FMath::Max(0, Thickness / 2);

		TArray<FIntPoint> LineBuf;
		for (const FColoredStroke& CS : Strokes)
		{
			uint8 CR, CG, CB;
			StrokeColorRGB(CS.Color, CR, CG, CB);
			for (int32 k = 0; k + 1 < CS.Points.Num(); ++k)
			{
				const FIntPoint P0(FMath::RoundToInt(CS.Points[k].X), FMath::RoundToInt(CS.Points[k].Y));
				const FIntPoint P1(FMath::RoundToInt(CS.Points[k + 1].X), FMath::RoundToInt(CS.Points[k + 1].Y));
				SkelGraph::LinePoints(P0, P1, LineBuf);
				for (const FIntPoint& P : LineBuf)
				{
					for (int32 oy = -Rad; oy <= Rad; ++oy)
					{
						for (int32 ox = -Rad; ox <= Rad; ++ox)
						{
							const int32 xx = P.X + ox;
							const int32 yy = P.Y + oy;
							if (xx < 0 || xx >= Width || yy < 0 || yy >= Height)
							{
								continue;
							}
							const int32 Off = (yy * Width + xx) * 4;
							RGBA[Off + 0] = CR;
							RGBA[Off + 1] = CG;
							RGBA[Off + 2] = CB;
							RGBA[Off + 3] = 255;
						}
					}
				}
			}
		}

		IImageWrapperModule& IWM = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
		TSharedPtr<IImageWrapper> IW = IWM.CreateImageWrapper(EImageFormat::PNG);
		if (!IW.IsValid())
		{
			return false;
		}
		IW->SetRaw(RGBA.GetData(), RGBA.Num(), Width, Height, ERGBFormat::RGBA, 8);
		const TArray64<uint8>& C = IW->GetCompressed();
		return FFileHelper::SaveArrayToFile(TArrayView<const uint8>(C.GetData(), static_cast<int32>(C.Num())), *Path);
	}

	bool SaveColoredStrokesJson(const TArray<FColoredStroke>& Strokes, int32 Width, int32 Height, const FString& Path, float EndpointTol)
	{
		const int32 M = Strokes.Num();

		// Endpoint adjacency: two strokes are neighbors when any of their endpoints
		// lie within EndpointTol pixels of each other.
		const double Tol2 = double(EndpointTol) * double(EndpointTol);
		auto Endpoints = [](const FColoredStroke& S, FVector2D& A, FVector2D& B)
		{
			A = S.Points.Num() > 0 ? S.Points[0] : FVector2D::ZeroVector;
			B = S.Points.Num() > 0 ? S.Points.Last() : FVector2D::ZeroVector;
		};

		TArray<TArray<int32>> Neighbors;
		Neighbors.SetNum(M);
		for (int32 i = 0; i < M; ++i)
		{
			FVector2D Ai, Bi;
			Endpoints(Strokes[i], Ai, Bi);
			for (int32 j = i + 1; j < M; ++j)
			{
				FVector2D Aj, Bj;
				Endpoints(Strokes[j], Aj, Bj);
				const double D[4] = {
					FVector2D::DistSquared(Ai, Aj), FVector2D::DistSquared(Ai, Bj),
					FVector2D::DistSquared(Bi, Aj), FVector2D::DistSquared(Bi, Bj)
				};
				bool bAdj = false;
				for (double d : D)
				{
					if (d <= Tol2) { bAdj = true; break; }
				}
				if (bAdj)
				{
					Neighbors[i].Add(j);
					Neighbors[j].Add(i);
				}
			}
		}

		FString Json;
		Json.Reserve(M * 256 + 256);
		Json += TEXT("{\n");
		Json += FString::Printf(TEXT("  \"width\": %d,\n  \"height\": %d,\n  \"endpoint_tol\": %.2f,\n  \"stroke_count\": %d,\n"),
			Width, Height, EndpointTol, M);
		Json += TEXT("  \"strokes\": [\n");
		for (int32 i = 0; i < M; ++i)
		{
			const FColoredStroke& S = Strokes[i];
			FVector2D A, B;
			Endpoints(S, A, B);
			Json += TEXT("    {\n");
			Json += FString::Printf(TEXT("      \"id\": %d,\n"), i);
			Json += FString::Printf(TEXT("      \"color\": \"%s\",\n"), StrokeColorToString(S.Color));
			Json += FString::Printf(TEXT("      \"num_points\": %d,\n"), S.Points.Num());
			Json += FString::Printf(TEXT("      \"connection_point_count\": %d,\n"), S.ConnectionPointCount);
			Json += FString::Printf(TEXT("      \"endpoints\": [[%.2f, %.2f], [%.2f, %.2f]],\n"), A.X, A.Y, B.X, B.Y);

			if (S.bHasMetrics)
			{
				const TCHAR* Kind = (S.Straightness >= 0.9) ? TEXT("line") : TEXT("curve");
				Json += FString::Printf(TEXT("      \"kind\": \"%s\",\n"), Kind);
				Json += FString::Printf(TEXT("      \"arc\": %.3f,\n"), S.Arc);
				Json += FString::Printf(TEXT("      \"chord\": %.3f,\n"), S.Chord);
				Json += FString::Printf(TEXT("      \"straightness\": %.4f,\n"), S.Straightness);
				Json += FString::Printf(TEXT("      \"direction\": [%.4f, %.4f],\n"), S.Direction.X, S.Direction.Y);
				Json += FString::Printf(TEXT("      \"p90_pca_line_error\": %.3f,\n"), S.P90PcaError);
				Json += FString::Printf(TEXT("      \"pca_rms_error\": %.3f,\n"), S.PcaRmsError);
				Json += FString::Printf(TEXT("      \"p90_chord_deviation\": %.3f,\n"), S.P90ChordDev);
				Json += FString::Printf(TEXT("      \"chord_deviation_ratio\": %.4f,\n"), S.ChordDevRatio);
			}

			Json += TEXT("      \"neighbors\": [");
			for (int32 n = 0; n < Neighbors[i].Num(); ++n)
			{
				Json += FString::Printf(TEXT("%s%d"), (n == 0 ? TEXT("") : TEXT(", ")), Neighbors[i][n]);
			}
			Json += TEXT("],\n");

			Json += TEXT("      \"points\": [");
			for (int32 k = 0; k < S.Points.Num(); ++k)
			{
				Json += FString::Printf(TEXT("%s[%.2f, %.2f]"), (k == 0 ? TEXT("") : TEXT(", ")), S.Points[k].X, S.Points[k].Y);
			}
			Json += TEXT("]\n");
			Json += (i + 1 < M) ? TEXT("    },\n") : TEXT("    }\n");
		}
		Json += TEXT("  ]\n}\n");

		return FFileHelper::SaveStringToFile(Json, *Path);
	}

	bool SaveMaskPng(const TArray<uint8>& Mask, int32 Width, int32 Height, const FString& Path, bool bInvertForDisplay)
	{
		const int32 N = Width * Height;
		TArray<uint8> RGBA;
		RGBA.SetNumUninitialized(N * 4);
		for (int32 i = 0; i < N; ++i)
		{
			uint8 v = Mask[i];
			if (bInvertForDisplay)
			{
				// foreground -> black, background -> white
				v = Mask[i] > 0 ? 0 : 255;
			}
			const int32 Off = i * 4;
			RGBA[Off + 0] = v;
			RGBA[Off + 1] = v;
			RGBA[Off + 2] = v;
			RGBA[Off + 3] = 255;
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
			*Path
		);
	}
}
