// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Engine/GameInstance.h"
#include "UrErHybrid_GameInstance.generated.h"


// Declare a multicast dynamic delegate with no parameters
DECLARE_DYNAMIC_MULTICAST_DELEGATE(FOnGamePlayStart);

UCLASS()
class URERSPATIALISLT_API UUrErHybrid_GameInstance : public UGameInstance {
  GENERATED_BODY()
  
public:
  // Expose the multicast to Blueprints so designers can bind events to it
  UPROPERTY(BlueprintAssignable, Category = "GameInstance")
  FOnGamePlayStart OnGamePlayStart;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "GameInstance")
  bool bISInVR_CPP = false;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "GameInstance")
  FString PlayerNameEN = "Unnamed";

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "GameInstance")
  FString PlayerNameCN = TEXT("未名");

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "GameInstance")
  int32 PlayerColorCode = 0;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "GameInstance")
  TArray<FLinearColor> PlayerColorPalette = {
      FLinearColor::White, FLinearColor::White, FLinearColor::White, FLinearColor::White, FLinearColor::White,
      FLinearColor::White, FLinearColor::White, FLinearColor::White, FLinearColor::White, FLinearColor::White
  };

  // Expose the function to Blueprints
  UFUNCTION(BlueprintCallable, Category = "GameInstance")
  void ServerStartGamePlay();
  
  UFUNCTION(BlueprintCallable, Category = "GameInstance")
  void PrintPlayerColorPalette();

  UFUNCTION(BlueprintCallable, Category = "GameInstance")
  void PrintPlayerNames();

  // Handler that we will bind to the multicast
  UFUNCTION()
  void GamePlayStartTravel();
  
  // Bind/unbind lifecycle hooks
  virtual void Init() override;
  virtual void Shutdown() override;
};