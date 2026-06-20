// Fill out your copyright notice in the Description page of Project Settings.

#include "UrErEnvEffectMan.h"

#include "Camera/CameraComponent.h"
#include "GameFramework/PlayerStart.h"
#include "Kismet/GameplayStatics.h"
#include "WaterBodyRiverActor.h"

// Sets default values
AUrErEnvEffectMan::AUrErEnvEffectMan() {
  // Set this actor to call Tick() every frame.  You can turn this off to
  // improve performance if you don't need it.
  PrimaryActorTick.bCanEverTick = true;

  SceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("SceneRoot"));
  RootComponent = SceneRoot;
  ForestAudioComponent = CreateDefaultSubobject<UAudioComponent>(TEXT("ForestAudioComponent"));
  ForestAudioComponent->SetupAttachment(SceneRoot);
  RiverAudioComponent = CreateDefaultSubobject<UAudioComponent>(TEXT("RiverAudioComponent"));
  RiverAudioComponent->SetupAttachment(SceneRoot);
  UnderWaterAudioComponent = CreateDefaultSubobject<UAudioComponent>(TEXT("UnderWaterAudioComponent"));
  UnderWaterAudioComponent->SetupAttachment(SceneRoot);
}

void AUrErEnvEffectMan::UpdateReferences() {
  TArray<AActor *> FoundPlayerStarts;
  PlayerPawn = UGameplayStatics::GetPlayerPawn(GetWorld(), 0);
  UGameplayStatics::GetAllActorsOfClass(GetWorld(), APlayerStart::StaticClass(), FoundPlayerStarts);
  if (FoundPlayerStarts.Num() > 0) {
    PlayerStart = FoundPlayerStarts[0];
  }
  /*UE_LOG(LogTemp, Warning, TEXT("PlayerPawn: %s, PlayerStart: %s"), *GetNameSafe(PlayerPawn),
         *GetNameSafe(PlayerStart));*/
  FoundPlayerStarts.Empty();
  TArray<AActor *> FoundRiverActors;
  UGameplayStatics::GetAllActorsOfClass(GetWorld(), AWaterBodyRiver::StaticClass(), FoundRiverActors);
  if (FoundRiverActors.Num() > 0) {
    RiverActor = Cast<AWaterBodyRiver>(FoundRiverActors[0]);
    if (RiverActor) {
      RiverComp = RiverActor->FindComponentByClass<UWaterBodyRiverComponent>();
      RiverSpline = RiverComp->GetWaterSpline();
    }
  }

  if (RiverAudioComponent) {
    RiverSoundCue = Cast<USoundCue>(RiverAudioComponent->Sound);
  }
  if (ForestAudioComponent) {
    ForestSoundCue = Cast<USoundCue>(ForestAudioComponent->Sound);
  }
  if (UnderWaterAudioComponent) {
    UnderWaterSoundCue = Cast<USoundCue>(UnderWaterAudioComponent->Sound);
  }
}

void AUrErEnvEffectMan::ABlueprintCallableFunctionExample() { 
  UE_LOG(LogTemp, Warning, TEXT("This is a blueprint callable function example."));
}
    // Called when the game starts or when spawned
void AUrErEnvEffectMan::BeginPlay() {
  Super::BeginPlay();

  UrErGI = Cast<UUrErHybrid_GameInstance>(UGameplayStatics::GetGameInstance(GetWorld()));

  TArray<AActor *> FoundPlayerStarts;
  PlayerPawn = UGameplayStatics::GetPlayerPawn(GetWorld(), 0);
 
  UGameplayStatics::GetAllActorsOfClass(GetWorld(), APlayerStart::StaticClass(), FoundPlayerStarts);
  if (FoundPlayerStarts.Num() > 0) {
    PlayerStart = FoundPlayerStarts[0];
  }
 /* UE_LOG(LogTemp, Warning, TEXT("PlayerPawn: %s, PlayerStart: %s"), *GetNameSafe(PlayerPawn),
         *GetNameSafe(PlayerStart));*/
  FoundPlayerStarts.Empty();

  TArray<AActor *> FoundRiverActors;
  UGameplayStatics::GetAllActorsOfClass(GetWorld(), AWaterBodyRiver::StaticClass(), FoundRiverActors);
  if (FoundRiverActors.Num() > 0) {
    RiverActor = Cast<AWaterBodyRiver>(FoundRiverActors[0]);
    if (RiverActor) {
      RiverComp = RiverActor->FindComponentByClass<UWaterBodyRiverComponent>();
      RiverSpline = RiverComp->GetWaterSpline();
    }
  }

  if (RiverAudioComponent) {
    RiverSoundCue = Cast<USoundCue>(RiverAudioComponent->Sound);
  }
  if (ForestAudioComponent) {
    ForestSoundCue = Cast<USoundCue>(ForestAudioComponent->Sound);
  }
  if (UnderWaterAudioComponent) {
    UnderWaterSoundCue = Cast<USoundCue>(UnderWaterAudioComponent->Sound);
  }

  TickCounter = 0;
  TickId = 1;
}

// Called every frame
void AUrErEnvEffectMan::Tick(float DeltaTime) {
  Super::Tick(DeltaTime);

  if (TickCounter++ % TicksToCheck == TickId) // Only check every TicksToCheck frames, and only on the
                                              // frame corresponding to TickId
  {
    if (PlayerPawn) {
      PlayerLocation = PlayerPawn->GetActorLocation();
      PlayerHeadLocation = PlayerPawn->GetComponentByClass<UCameraComponent>()
                               ? PlayerPawn->GetComponentByClass<UCameraComponent>()->GetComponentLocation()
                               : PlayerLocation;
      if (UrErGI && (!UrErGI->bISInVR_CPP)) {
        PlayerHeadLocation.Z = PlayerHeadLocation.Z + 64.f;
      }
    } else {
      PlayerLocation = PlayerStart ? PlayerStart->GetActorLocation() : FVector::ZeroVector;
      PlayerHeadLocation = PlayerLocation;
    }

    float ClosestKey = RiverComp ? RiverComp->FindInputKeyClosestToWorldLocation(PlayerLocation) : 0.f;
    FVector ClosestPoint = RiverSpline
                               ? RiverSpline->GetLocationAtSplineInputKey(ClosestKey, ESplineCoordinateSpace::World)
                               : FVector::ZeroVector;
    float RiverWidth = RiverComp ? RiverComp->GetRiverWidthAtSplineInputKey(ClosestKey) : 1350.f;
    float DistanceToRiverCenter = FVector::Dist2D(PlayerLocation, ClosestPoint);
    float HeadDistanceToRiverCenter = PlayerHeadLocation.Z - ClosestPoint.Z;
    // Set transition from forest to river audio based on distance to
    // river center line
    if (ForestAudioComponent) {
      if (DistanceToRiverCenter < RiverOnlyStart) {
        float VolumeMultiplier =
            FMath::Lerp(0.f, 1.f, (DistanceToRiverCenter - RiverOnlyFull) / (RiverOnlyStart - RiverOnlyFull));
        ForestAudioComponent->SetVolumeMultiplier(VolumeMultiplier * ForestVolumeBaseMul);
      } else {
        ForestAudioComponent->SetVolumeMultiplier(ForestVolumeBaseMul);
      }
    }
    // Set transition from river to under water audio based on height
    // above river
    if (RiverAudioComponent) {
      RiverAudioComponent->SetWorldLocation(FVector(ClosestPoint.X, ClosestPoint.Y, (ClosestPoint.Z + 50.f)));
      if (DistanceToRiverCenter < RiverOnlyStart) { // Within range of river audio
        float Alpha =
            FMath::Clamp((DistanceToRiverCenter - RiverOnlyFull) / (RiverOnlyStart - RiverOnlyFull), 0.f, 1.f);
        float RiverVolMul = FMath::Lerp(1.f, 0.f, Alpha);
        RiverVolMul *= RiverVolumeBaseMul;
        RiverAudioComponent->SetVolumeMultiplier(RiverVolMul);
        if ((HeadDistanceToRiverCenter < UnderWaterOnlyStart) && // Only check underwater within river audio
                                                                 // range
            (UnderWaterAudioComponent)) {
          UnderWaterAudioComponent->SetWorldLocation(FVector(ClosestPoint.X, ClosestPoint.Y, (ClosestPoint.Z - 50.f)));
          float UnderWaterAlpha = FMath::Clamp(
              (HeadDistanceToRiverCenter - UnderWaterOnlyFull) / (UnderWaterOnlyStart - UnderWaterOnlyFull), 0.f, 1.f);
          float UnderWaterVolMul = FMath::Lerp(1.f, 0.f, UnderWaterAlpha);
          RiverAudioComponent->SetVolumeMultiplier((1 - UnderWaterVolMul) * RiverVolumeBaseMul);
          UnderWaterAudioComponent->SetVolumeMultiplier(UnderWaterVolMul * UnderWaterVolumeBaseMul);
        } else {
          UnderWaterAudioComponent->SetVolumeMultiplier(0);
        }
      } else {
        RiverAudioComponent->SetVolumeMultiplier(0);
      }
    }

   /* UE_LOG(LogTemp, Warning, TEXT("PlayerLocation: %s, PlayerHeadLocation: %s, ClosestPoint: %s"),
            *PlayerLocation.ToString(), *PlayerHeadLocation.ToString(), *ClosestPoint.ToString());
     UE_LOG(LogTemp, Warning, TEXT("DistanceToRiverCentre: %f, HeadDistanceToRiverCenter: %f"), DistanceToRiverCenter,
            HeadDistanceToRiverCenter);
     UE_LOG(LogTemp, Warning, TEXT("ForestVolume: %f, RiverVolume: %f, UnderWaterVolume: %f"),
            ForestAudioComponent->VolumeMultiplier * ForestSoundCue->VolumeMultiplier,
            RiverAudioComponent->VolumeMultiplier * RiverSoundCue->VolumeMultiplier,
            UnderWaterAudioComponent->VolumeMultiplier * UnderWaterSoundCue->VolumeMultiplier);*/
  }
}
