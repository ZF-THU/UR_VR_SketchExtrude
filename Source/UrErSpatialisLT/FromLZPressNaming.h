#pragma once

#include "CoreMinimal.h"

class FFromLZPressNaming
{
public:
	static int32 GetNextPressIndex(const FString& TwoDDebugDir);
	static FString MakePressName(int32 PressIndex);
};
