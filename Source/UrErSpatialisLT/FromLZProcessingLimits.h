#pragma once

#include "CoreMinimal.h"
#include "Async/ParallelFor.h"

namespace FromLZProcessing
{
	int32 GetCompositeMaxWorkers();
	int32 GetParallelForMaxThreads();

	template <typename BodyType>
	void LimitedParallelFor(int32 Num, BodyType&& Body)
	{
		if (Num <= 0)
		{
			return;
		}

		const int32 WorkerCount = FMath::Min(Num, GetParallelForMaxThreads());
		if (WorkerCount <= 1)
		{
			for (int32 Index = 0; Index < Num; ++Index)
			{
				Body(Index);
			}
			return;
		}

		ParallelFor(WorkerCount, [&](int32 WorkerIndex)
		{
			const int32 Begin = (Num * WorkerIndex) / WorkerCount;
			const int32 End = (Num * (WorkerIndex + 1)) / WorkerCount;
			for (int32 Index = Begin; Index < End; ++Index)
			{
				Body(Index);
			}
		});
	}
}
