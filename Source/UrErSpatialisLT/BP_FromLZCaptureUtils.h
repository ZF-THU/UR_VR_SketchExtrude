#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintAsyncActionBase.h"
#include "BP_FromLZCaptureUtils.generated.h"

class UCameraComponent;

DECLARE_DYNAMIC_MULTICAST_DELEGATE_ThreeParams(
	FBPFromLZCaptureCompleted,
	const TArray<FString>&, OutputFiles,
	const FString&, CaptureDirectory,
	const FString&, MainPngPath);

DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(
	FBPFromLZCaptureFailed,
	const FString&, ErrorMessage,
	const TArray<FString>&, PartialOutputFiles);

UCLASS()
class URERSPATIALISLT_API UBP_FromLZCaptureUtils : public UBlueprintAsyncActionBase
{
	GENERATED_BODY()

public:
	UPROPERTY(BlueprintAssignable)
	FBPFromLZCaptureCompleted OnCompleted;

	UPROPERTY(BlueprintAssignable)
	FBPFromLZCaptureFailed OnFailed;

	UFUNCTION(BlueprintCallable, meta = (BlueprintInternalUseOnly = "true", WorldContext = "WorldContextObject", DisplayName = "BP FromLZ Capture From Camera", Category = "FromLZ|Capture", AdvancedDisplay = "CaptureResolutionOverride", CaptureResolutionOverride = "(X=1920,Y=1080)"))
	static UBP_FromLZCaptureUtils* CaptureFromCamera(
		UObject* WorldContextObject,
		UCameraComponent* CameraComponent,
		FIntPoint CaptureResolutionOverride);

	virtual void Activate() override;

private:
	UPROPERTY()
	TObjectPtr<UObject> WorldContextObject = nullptr;

	UPROPERTY()
	TObjectPtr<UCameraComponent> CameraComponent = nullptr;

	UPROPERTY()
	FIntPoint CaptureResolutionOverride = FIntPoint(1920, 1080);
};
