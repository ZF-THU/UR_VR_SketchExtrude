// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

// #include "BuoyancyComponent.h"
#include "Components/SplineComponent.h"
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "WaterBodyRiverComponent.h"
#include "WaterSplineComponent.h"
#include "UrErFloatingObject.generated.h"


UCLASS()
class URERSPATIALISLT_API AUrErFloatingObject : public AActor {
  GENERATED_BODY()

public: // Constructors
  AUrErFloatingObject();

  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mesh")
  UStaticMeshComponent *MeshComponent;


 /* UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Buoyancy")
  UBuoyancyComponent *BuoyancyComponent;*/

protected:
  virtual void BeginPlay() override;

public: // Overrides
  virtual void Tick(float DeltaTime) override;

public: // UPROPERTYs
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PhyicsSimPerformance")
  float PhySimRange =
      10000.f; // Range at which physics simulation will be enabled
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PhyicsSimPerformance")
  int TicksToCheck = 10; // How many ticks to spread out the physics simulation
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PhyicsSimPerformance")
  float MinPlayerDistance =
      2500.f; // Min distance from player to reset location
  UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PhyicsSimPerformance")
  float MaxDriftDistance =
      10000.f; // Max distance before reset location

public: // UFUNCTIONs
  UFUNCTION(BlueprintCallable, Category = "CPPFunctions")
  void UpdateReferences(); 
  UFUNCTION(BlueprintCallable, Category = "Test")
  void MyTestFunction(); 

public: // Non-UPROPERTY variables, no blueprint access
  APawn *PlayerPawn;
  AActor *PlayerStart;
  AWaterBodyRiver *RiverActor;
  UWaterBodyRiverComponent *RiverComp;
  UWaterSplineComponent *RiverSpline;
  FVector PlayerLocation, PlayerStartLocation, InitialLocation;
  FRotator InitialRotation;

  int TickCounter = 0,
      TickId = 0; // TickId is used to spread out the physics simulation checks
                  // across multiple frames to avoid performance spikes

public: // Non-UFUNCTION functions, no blueprint access
};
