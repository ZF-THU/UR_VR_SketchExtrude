// Fill out your copyright notice in the Description page of Project Settings.


#include "UrErEnvSign.h"

#include "UrErHCharacter.h"
#include "Components/WidgetComponent.h"
#include "Kismet/GameplayStatics.h"

// Sets default values
AUrErEnvSign::AUrErEnvSign() {
  // Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
  PrimaryActorTick.bCanEverTick = true;

  Root = CreateDefaultSubobject<USceneComponent>("Root");
  RootComponent = Root;
  SignMesh = CreateDefaultSubobject<UStaticMeshComponent>(FName("SignMesh"));
  SignMesh->SetupAttachment(RootComponent);
  SignWidget = CreateDefaultSubobject<UWidgetComponent>(FName("SignWidget"));
  SignWidget->SetupAttachment(SignMesh);

}

// Called when the game starts or when spawned
void AUrErEnvSign::BeginPlay() {
  Super::BeginPlay();

  SignMesh->SetVisibility(false);
  SignWidget->SetVisibility(false);

  PC = UGameplayStatics::GetPlayerController(GetWorld(), 0);
  APawn *PN = PC->GetPawn();
  CH = Cast<AUrErHCharacter>(PN);
}

// Called every frame
void AUrErEnvSign::Tick(float DeltaTime) {
  Super::Tick(DeltaTime);

  // Control the visibility of the sign by distance from the character
  if (CH) {
    float DistToCH = GetDistanceTo(CH);
    // UE_LOG(LogTemp, Warning, TEXT("Player to EnvSign distance: %f"), DistToCH);
    if (DistToCH > 2000.f) {
      SignMesh->SetVisibility(false);
    } else {
      SignMesh->SetVisibility(true);
    }
  }

  // The following is to rotate the name tag to face the camera
  // But only rotate the yaw to avoid VR motion sickness
  if (PC && PC->IsLocalController())  {
    FVector ViewLocation;
    FRotator ViewRotation;
    PC->GetPlayerViewPoint(ViewLocation, ViewRotation);
    // FVector LookVector = ViewLocation - GetActorLocation();
    // LookVector = LookVector.GetSafeNormal();
    FRotator LookRotation = FRotator(0.f, ViewRotation.Yaw + 180.f, 0.f);
    SetActorRotation(LookRotation);
    // UE_LOG(LogTemp, Warning, TEXT("EnvSign rotation: %s"), *LookRotation.ToString());
  }
}