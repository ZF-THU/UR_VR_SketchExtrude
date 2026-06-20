#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintAsyncActionBase.h"
#include "BP_FromLZShiftUndoUtils.generated.h"

DECLARE_DYNAMIC_MULTICAST_DELEGATE_SixParams(
	FBPFromLZStep11UndoCompleted,
	const FString&, PressId,
	const FString&, UndoDiagnosticPath,
	const TArray<FString>&, OutputFiles,
	int32, DestroyedActorCount,
	int32, DestroyedBooleanComponentCount,
	int32, RestoredHiddenSourceCount);

DECLARE_DYNAMIC_MULTICAST_DELEGATE_ThreeParams(
	FBPFromLZStep11UndoFailed,
	const FString&, ErrorMessage,
	const FString&, PressId,
	const TArray<FString>&, PartialOutputFiles);

UCLASS()
class URERSPATIALISLT_API UBP_FromLZShiftUndoUtils : public UBlueprintAsyncActionBase {
	GENERATED_BODY()

public:
	UPROPERTY(BlueprintAssignable)
	FBPFromLZStep11UndoCompleted OnCompleted;

	UPROPERTY(BlueprintAssignable)
	FBPFromLZStep11UndoFailed OnFailed;

	UFUNCTION(BlueprintCallable, meta = (BlueprintInternalUseOnly = "true", WorldContext = "WorldContextObject", DisplayName = "BP FromLZ Undo Latest Step11 Press", Category = "FromLZ|Step11"))
	static UBP_FromLZShiftUndoUtils* UndoLatestStep11Press(UObject* WorldContextObject);

	virtual void Activate() override;

private:
	UPROPERTY()
	TObjectPtr<UObject> WorldContextObject = nullptr;
};
