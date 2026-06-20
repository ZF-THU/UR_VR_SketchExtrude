// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "UrErTestActor.generated.h"

UCLASS()
class URERSPATIALISLT_API AUrErTestActor : public AActor {
  GENERATED_BODY()

public:
  // Sets default values for this actor's properties
  AUrErTestActor();
  
  UPROPERTY(EditAnywhere, BlueprintReadWrite)
  TObjectPtr<UStaticMeshComponent> TestMesh;

protected:
  // Called when the game starts or when spawned
  virtual void BeginPlay() override;

public:
  // UProperties
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = UrErTestActor)
  bool bIsOn = false;
  
  // Called every frame
  virtual void Tick(float DeltaTime) override;

  //UFunctions
  UFUNCTION(BlueprintCallable, Category = UrErTestActor)
  void SetScale(bool bIsOnNew, float ScaleDelta);
};