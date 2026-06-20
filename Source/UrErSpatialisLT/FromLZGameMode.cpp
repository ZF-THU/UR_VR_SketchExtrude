#include "FromLZGameMode.h"

#include "FromLZCameraPawn.h"

AFromLZGameMode::AFromLZGameMode()
{
	DefaultPawnClass = AFromLZCameraPawn::StaticClass();
}
