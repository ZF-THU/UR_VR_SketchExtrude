// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "UrErEnvSign.generated.h"

class UWidgetComponent;
class AUrErHCharacter;
class APlayerController;

UCLASS()
class URERSPATIALISLT_API AUrErEnvSign : public AActor {
  GENERATED_BODY()

public:
  AUrErEnvSign();

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = UrErEnvSign)
  TObjectPtr<USceneComponent> Root;
  
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = UrErEnvSign)
  TObjectPtr<UStaticMeshComponent> SignMesh;
  
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = UrErEnvSign)
  TObjectPtr<UWidgetComponent> SignWidget;

protected:
  // Called when the game starts or when spawned
  virtual void BeginPlay() override;
  
  AUrErHCharacter* CH;
  APlayerController* PC;

public:
  // Called every frame
  virtual void Tick(float DeltaTime) override;

};