#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintAsyncActionBase.h"
#include "BP_FromLZTabResetUtils.generated.h"

DECLARE_DYNAMIC_MULTICAST_DELEGATE_SixParams(
	FBPFromLZSessionResetCompleted,
	const FString&, ArchiveDir,
	const FString&, LevelName,
	const TArray<FString>&, RecreatedFolders,
	bool, bArchiveOk,
	bool, bReloadIssued,
	int32, SessionGeneration);

DECLARE_DYNAMIC_MULTICAST_DELEGATE_FiveParams(
	FBPFromLZSessionResetFailed,
	const FString&, ErrorMessage,
	const FString&, ArchiveDir,
	const TArray<FString>&, RecreatedFolders,
	bool, bArchiveOk,
	int32, SessionGeneration);

UCLASS()
class URERSPATIALISLT_API UBP_FromLZTabResetUtils : public UBlueprintAsyncActionBase {
	GENERATED_BODY()

public:
	UPROPERTY(BlueprintAssignable)
	FBPFromLZSessionResetCompleted OnCompleted;

	UPROPERTY(BlueprintAssignable)
	FBPFromLZSessionResetFailed OnFailed;

	UFUNCTION(BlueprintCallable, meta = (BlueprintInternalUseOnly = "true", WorldContext = "WorldContextObject", DisplayName = "BP FromLZ Full Session Reset", Category = "FromLZ|Session"))
	static UBP_FromLZTabResetUtils* FullSessionReset(UObject* WorldContextObject);

	virtual void Activate() override;

private:
	UPROPERTY()
	TObjectPtr<UObject> WorldContextObject = nullptr;
};
