// Fill out your copyright notice in the Description page of Project Settings.


#include "UrErTestActor.h"

// Sets default values
AUrErTestActor::AUrErTestActor() {
  // Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
  PrimaryActorTick.bCanEverTick = true;
  
  TestMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("TestMesh"));
  TestMesh->SetupAttachment(RootComponent);

}

// Called when the game starts or when spawned
void AUrErTestActor::BeginPlay() {
  Super::BeginPlay();

}

// Called every frame
void AUrErTestActor::Tick(float DeltaTime) {
  Super::Tick(DeltaTime);

}

void AUrErTestActor::SetScale(bool bIsOnNew, float ScaleDelta) {
  bIsOn = bIsOnNew;
  if (bIsOn) {
    TestMesh->SetHiddenInGame(false);
    TestMesh->SetRelativeScale3D(TestMesh->GetRelativeScale3D() + FVector(ScaleDelta));
  } else {
    TestMesh->SetHiddenInGame(true);
  }
}