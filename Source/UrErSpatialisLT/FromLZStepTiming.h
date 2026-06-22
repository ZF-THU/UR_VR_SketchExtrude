#pragma once

#include "CoreMinimal.h"

namespace FromLZStepTiming
{
	double NowSeconds();
	FString TimingJsonPath(const FString& PressDir);
	void ResetPress(const FString& PressDir, const FString& PressName);
	void RecordStep(
		const FString& PressDir,
		const FString& StepName,
		double StartSeconds,
		double EndSeconds,
		const FString& Category = FString(),
		const FString& Notes = FString());
}
