// Fill out your copyright notice in the Description page of Project Settings.

#include "UrErFloatingObject.h"

#include "BuoyancyComponent.h"
#include "GameFramework/PlayerStart.h"
#include "Kismet/GameplayStatics.h"
#include "WaterBodyRiverActor.h"

// Sets default values
AUrErFloatingObject::AUrErFloatingObject() {
  // Set this actor to call Tick() every frame.  You can turn this off to
  // improve performance if you don't need it.
  PrimaryActorTick.bCanEverTick = true;

  MeshComponent =
      CreateDefaultSubobject<UStaticMeshComponent>(TEXT("MeshComponent"));
  RootComponent = MeshComponent;
}

// Called when the game starts or when spawned
void AUrErFloatingObject::BeginPlay() {
  Super::BeginPlay();

  TArray<AActor *> FoundPlayerStarts;
  TickId = FMath::RandRange(0, TicksToCheck -
                                   1); // Randomize TickId to spread out physics
                                       // simulation checks across frames
  PlayerPawn = UGameplayStatics::GetPlayerPawn(GetWorld(), 0);
  UGameplayStatics::GetAllActorsOfClass(GetWorld(), APlayerStart::StaticClass(),
                                        FoundPlayerStarts);
  if (FoundPlayerStarts.Num() > 0) {
    PlayerStart = FoundPlayerStarts[0];
  }
 /* UE_LOG(LogTemp, Warning, TEXT("PlayerPawn: %s, PlayerStart: %s"),
         *GetNameSafe(PlayerPawn), *GetNameSafe(PlayerStart));*/
  FoundPlayerStarts.Empty();

  TArray<AActor *> FoundRiverActors;
  UGameplayStatics::GetAllActorsOfClass(GetWorld(), AWaterBodyRiver::StaticClass(),
                                        FoundRiverActors);
  if (FoundRiverActors.Num() > 0) {
    RiverActor = Cast<AWaterBodyRiver>(FoundRiverActors[0]);
    if (RiverActor) {
      RiverComp = RiverActor->FindComponentByClass<UWaterBodyRiverComponent>();
      RiverSpline = RiverComp ? RiverComp->GetWaterSpline() : nullptr;
    }
  }
  InitialLocation = GetActorLocation();
  InitialRotation = GetActorRotation();
  TickCounter = 0;
}

void AUrErFloatingObject::UpdateReferences() {
  TArray<AActor *> FoundPlayerStarts;
  PlayerPawn = UGameplayStatics::GetPlayerPawn(GetWorld(), 0);
  UGameplayStatics::GetAllActorsOfClass(GetWorld(), APlayerStart::StaticClass(),
                                        FoundPlayerStarts);
  if (FoundPlayerStarts.Num() > 0) {
    PlayerStart = FoundPlayerStarts[0];
  }
  /*UE_LOG(LogTemp, Warning, TEXT("PlayerPawn: %s, PlayerStart: %s"),
         *GetNameSafe(PlayerPawn), *GetNameSafe(PlayerStart));*/
  FoundPlayerStarts.Empty();
  TArray<AActor *> FoundRiverActors;
  UGameplayStatics::GetAllActorsOfClass(GetWorld(), AWaterBodyRiver::StaticClass(),
                                        FoundRiverActors);
  if (FoundRiverActors.Num() > 0) {
    RiverActor = Cast<AWaterBodyRiver>(FoundRiverActors[0]);
    if (RiverActor) {
      RiverComp = RiverActor->FindComponentByClass<UWaterBodyRiverComponent>();
      RiverSpline = RiverComp ? RiverComp->GetWaterSpline() : nullptr;
    }
  }
}

void AUrErFloatingObject::MyTestFunction() {
  /*UE_LOG(LogTemp, Warning, TEXT("MyTestFunction called on %s"), *GetName());*/
}

// Called every frame
void AUrErFloatingObject::Tick(float DeltaTime) {
  Super::Tick(DeltaTime);

  if (TickCounter++ % TicksToCheck ==
      TickId) // Only check every TicksToCheck frames, and only on the frame
              // corresponding to TickId
  {
    if (PlayerPawn) {
      PlayerLocation = PlayerPawn->GetActorLocation();
    } else {
      PlayerLocation =
          PlayerStart ? PlayerStart->GetActorLocation() : FVector::ZeroVector;
    }

    float ClosestKey =
        RiverComp
            ? RiverComp->FindInputKeyClosestToWorldLocation(PlayerLocation)
            : 0.f;
    FVector ClosestPoint = RiverSpline ? RiverSpline->GetLocationAtSplineInputKey(ClosestKey, ESplineCoordinateSpace::World)
                               : FVector::ZeroVector;
    float RiverWidth =
        RiverComp ? RiverComp->GetRiverWidthAtSplineInputKey(ClosestKey) : 999999.f;
    float DistanceToPlayer =
        FVector::Dist2D(GetActorLocation(), PlayerLocation);
    float DistanceDrifted =
        FVector::Dist2D(GetActorLocation(), InitialLocation);
    float DistanceToRiverCenter =
        FVector::Dist2D(GetActorLocation(), ClosestPoint);
    if (MeshComponent->IsSimulatingPhysics() && // Physics enabled
        (DistanceToPlayer >= MinPlayerDistance)) {   // And more than 20 meters from player
      if ((DistanceDrifted >= MaxDriftDistance) || // Drifted further than max
          ((DistanceToRiverCenter / RiverWidth >= 0.85f) && // Near river edge
           (FVector::VectorPlaneProject(MeshComponent->GetComponentVelocity(),
                                        FVector::UpVector)
                .Size() <= 1))) // And nearly stationary
      {
        SetActorLocation(InitialLocation);
        SetActorRotation(InitialRotation);
      }
    }
    if (DistanceToPlayer <= PhySimRange) {
      if (!MeshComponent->IsSimulatingPhysics()) {
        MeshComponent->SetSimulatePhysics(true);
        /* UE_LOG(LogTemp, Warning,
                TEXT("%s started simulating physics. Distance to player: %f"),
                *GetName(), DistanceToPlayer);*/
      }
    } else {
      if (MeshComponent->IsSimulatingPhysics()) {
        MeshComponent->SetSimulatePhysics(false);
        /*UE_LOG(LogTemp, Warning,
               TEXT("%s stopped simulating physics. Distance to player: %f"),
               *GetName(), DistanceToPlayer);*/
      }
    }
  }
}
