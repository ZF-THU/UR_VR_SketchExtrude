// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/AudioComponent.h"
#include "Sound/SoundCue.h"
#include "WaterBodyRiverComponent.h"
#include "WaterSplineComponent.h"
#include "UrErHybrid_GameInstance.h"
#include "UrErEnvEffectMan.generated.h"

class UAudioComponent;

UCLASS()
class URERSPATIALISLT_API AUrErEnvEffectMan : public AActor {
  GENERATED_BODY()

public: // Constructors
  // Sets default values for this actor's properties
  AUrErEnvEffectMan();

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Root")
  USceneComponent *SceneRoot;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EnvEffects")
  UAudioComponent *ForestAudioComponent;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EnvEffects")
  UAudioComponent *RiverAudioComponent;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EnvEffects")
  UAudioComponent *UnderWaterAudioComponent;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EnvEffects")
  USoundCue *ForestSoundCue;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EnvEffects")
  USoundCue *RiverSoundCue;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EnvEffects")
  USoundCue *UnderWaterSoundCue;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EnvEffects")
  AActor *PlayerHead;


protected:
  // Called when the game starts or when spawned
  virtual void BeginPlay() override;

public: // Overrides
  // Called every frame
  virtual void Tick(float DeltaTime) override;

public: // UPROPERTYs

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EnvSoundControl")
  UUrErHybrid_GameInstance *UrErGI;

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EnvSoundControl")
  float RiverOnlyStart =
      6000.f; // Distance to river center line at which forest audio drop volume
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EnvSoundControl")
  float RiverOnlyFull = 1500.f; // Distance to river center line at which forest
                                // audio is fully dropped
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EnvSoundControl")
  float UnderWaterOnlyStart =
      15.f; // Height above water at which river audio starts to drop and
            // under water audio starts to increase
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EnvSoundControl")
  float UnderWaterOnlyFull =
      -15.f; // Height above water at which river audio is fully dropped and
             // under water audio is fully increased
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EnvSoundControl")
  float ForestVolumeBaseMul = 1.f;
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EnvSoundControl")
  float RiverVolumeBaseMul = .9f;
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EnvSoundControl")
  float UnderWaterVolumeBaseMul = .9f;
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "EnvSoundControl")
  int TicksToCheck = 10; // How many ticks to spread out the distance checks
                        // across to avoid performance spikes
public: // UFUNCTIONs
  UFUNCTION(BlueprintCallable, Category = "CPPFunctions")
  void UpdateReferences();
  UFUNCTION(BlueprintCallable, Category = "CPPFunctions")
  void ABlueprintCallableFunctionExample();

public: // Non-UPROPERTY variables, no blueprint access
  APawn *PlayerPawn;
  AActor *PlayerStart;
  AWaterBodyRiver *RiverActor;
  UWaterBodyRiverComponent *RiverComp;
  UWaterSplineComponent *RiverSpline;
  FVector PlayerLocation, PlayerStartLocation, PlayerHeadLocation;
  int TickCounter = 0, TickId = 1;
};
