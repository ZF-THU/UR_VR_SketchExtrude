#include "FromLZPressNaming.h"

#include "HAL/FileManager.h"
#include "Misc/Paths.h"

int32 FFromLZPressNaming::GetNextPressIndex(const FString& TwoDDebugDir)
{
	int32 MaxPress = 0;
	IFileManager::Get().IterateDirectory(*TwoDDebugDir, [&MaxPress](const TCHAR* InPath, bool bIsDir) -> bool
	{
		if (bIsDir)
		{
			const FString Name = FPaths::GetCleanFilename(FString(InPath));
			if (Name.StartsWith(TEXT("Press_")))
			{
				MaxPress = FMath::Max(MaxPress, FCString::Atoi(*Name.RightChop(6)));
			}
		}
		return true;
	});
	return MaxPress + 1;
}

FString FFromLZPressNaming::MakePressName(int32 PressIndex)
{
	return FString::Printf(TEXT("Press_%02d"), FMath::Max(0, PressIndex));
}
