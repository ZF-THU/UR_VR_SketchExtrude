// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "UrErHCharacter.generated.h"

class UWidgetComponent;
class UUrErHybrid_GameInstance;

UCLASS()
class URERSPATIALISLT_API AUrErHCharacter : public ACharacter {
  GENERATED_BODY()
  
public:
  // Sets default values for this character's properties
  AUrErHCharacter();
  
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = UrErHCharacter)
  TObjectPtr<USceneComponent> LaserContainer;
  
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = UrErHCharacter)
  TObjectPtr<UStaticMeshComponent> LaserLineMesh;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = UrErHCharacter)
  TObjectPtr<UStaticMeshComponent> LaserHitMesh;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = UrErHCharacter)
  TObjectPtr<UWidgetComponent> NameTag;
  
protected:
  // Called when the game starts or when spawned
  virtual void BeginPlay() override;

public:

  // Engine Overrides
  virtual void Tick(float DeltaTime) override;
  virtual void SetupPlayerInputComponent(class UInputComponent *PlayerInputComponent) override;

  // Blueprint-Exposed Variables

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Laser")
  float LaserRange = 5000.f;
  
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Laser")
  float LaserPitchBase = -15.f;
  
  UPROPERTY(EditAnywhere, BlueprintReadWrite, CateGory = "Laser")
  bool bIsLaserActive = false; // For replicated player pawns

  UPROPERTY(EditAnywhere, BlueprintReadWrite, CateGory = "Laser")
  bool bIsLaserActiveLocally = false; // For human controlled local pawns
  
  // Non-Blueprint-Exposed Variables
  
  float LaserHitMeshScreenSize = 0.015f;
  int TickCount = 0;
  TObjectPtr<UUrErHybrid_GameInstance> GI;

  // Blueprint-Exposed Functions
  
  UFUNCTION(BlueprintCallable, Category = "Laser")
  void InitializeLaser(float LaserPitchCorrection, float LaserOffset);
  UFUNCTION(BlueprintCallable, Category = "Laser")
  void UpdateLaserVisuals();
  UFUNCTION(BlueprintCallable, Category = "Laser")
  void HideLaserVisuals();
  
  UFUNCTION(BlueprintCallable, Category = "NameTag")
  void InitializeNameTag(UStaticMeshComponent* HeadMesh, FVector OffsetVector);
  
  
  // Non-Blueprint-Exposed Functions
  

};