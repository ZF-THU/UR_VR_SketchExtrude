#include "FromLZProcessingLimits.h"

#include "HAL/IConsoleManager.h"

namespace FromLZProcessing
{
namespace
{
	static int32 GCompositeMaxWorkers = 1;
	static FAutoConsoleVariableRef CVarCompositeMaxWorkers(
		TEXT("r.FromLZ.CompositeMaxWorkers"),
		GCompositeMaxWorkers,
		TEXT("Maximum number of FromLZ composite workers allowed to run concurrently. Default 1 keeps the editor/game responsive; valid range is 1..4."),
		ECVF_Default);

	static int32 GParallelForMaxThreads = 8;
	static FAutoConsoleVariableRef CVarParallelForMaxThreads(
		TEXT("r.FromLZ.ParallelForMaxThreads"),
		GParallelForMaxThreads,
		TEXT("Maximum worker chunks used by FromLZ heavy pipeline parallel loops. Default 8 is tuned for high-end laptop CPUs while leaving headroom for UE threads; valid range is 1..16."),
		ECVF_Default);
}

int32 GetCompositeMaxWorkers()
{
	return FMath::Clamp(GCompositeMaxWorkers, 1, 4);
}

int32 GetParallelForMaxThreads()
{
	return FMath::Clamp(GParallelForMaxThreads, 1, 16);
}
}
