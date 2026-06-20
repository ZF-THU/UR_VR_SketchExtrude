// Fill out your copyright notice in the Description page of Project Settings.

#include "UrErHCharacter.h"
#include "UrErHybrid_GameInstance.h"
#include "Net/UnrealNetwork.h"

#include "Misc/MapErrors.h"
#include "NavigationSystemTypes.h"
#include "Components/WidgetComponent.h"
#include "Kismet/GameplayStatics.h"

// Sets default values
AUrErHCharacter::AUrErHCharacter() {
  // Set this character to call Tick() every frame.  You can turn this off to
  // improve performance if you don't need it.
  PrimaryActorTick.bCanEverTick = true;

  LaserContainer = CreateDefaultSubobject<USceneComponent>(TEXT("LaserContainer"));
  LaserContainer->SetupAttachment(GetRootComponent());
  LaserLineMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("LaserLineMesh"));
  LaserLineMesh->SetupAttachment(LaserContainer);
  LaserHitMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("HitMesh"));
  LaserHitMesh->SetupAttachment(LaserContainer);
  NameTag = CreateDefaultSubobject<UWidgetComponent>(TEXT("NameTag"));
  NameTag->SetupAttachment(GetRootComponent());
  NameTag->SetWidgetSpace(EWidgetSpace::World);
  NameTag->SetDrawSize(FVector2D(320.f,160.f));
  NameTag->SetRelativeScale3D(FVector(0.1f, 0.1f, 0.1f));
}

// Called when the game starts or when spawned
void AUrErHCharacter::BeginPlay() { 
  Super::BeginPlay(); 

  APlayerController* PC = UGameplayStatics::GetPlayerController(GetWorld(), 0);
  if (PC) {
    GI = Cast<UUrErHybrid_GameInstance>(PC->GetGameInstance());
    if (GI) {
      //UE_LOG(LogTemp, Warning, TEXT("GameInstance is: %s"), *GI->GetName());
    }
  }
  if (bIsLaserActiveLocally) {
      UpdateLaserVisuals();
  } 
  else {
      HideLaserVisuals();
  }
}

// Called every frame
void AUrErHCharacter::Tick(float DeltaTime) {
  Super::Tick(DeltaTime);

  // The following is to rotate the name tag to face the camera
  // But only rotate the yaw to avoid VR motion sickness
  //if (APlayerController *PC = Cast<APlayerController>(GetController())) {
  //  FVector CameraLocation;
  //  FRotator CameraRotation;
  //  PC->GetPlayerViewPoint(CameraLocation, CameraRotation);
  //  FVector LookVector = CameraLocation - NameTag->GetComponentLocation();
  //  LookVector = LookVector.GetSafeNormal();
  //  FRotator LookRotation = LookVector.Rotation();
  //  LookRotation = FRotator(0.f, LookRotation.Yaw, 0.f);
  //  NameTag->SetWorldRotation(LookRotation);
  // }
  

  if (bIsLaserActiveLocally) {
      UpdateLaserVisuals();
  } 
  else {
      HideLaserVisuals();
  }  
}

// Called to bind functionality to input
void AUrErHCharacter::SetupPlayerInputComponent(UInputComponent *PlayerInputComponent) {
  Super::SetupPlayerInputComponent(PlayerInputComponent);
}

void AUrErHCharacter::InitializeLaser(float LaserPitchCorrection, float LaserOffset) {
  // Correct laser pitch for user comfort in VR and NonVR
  LaserLineMesh->SetRelativeRotation(FRotator(LaserLineMesh->GetRelativeRotation().Pitch + LaserPitchCorrection,
                                              LaserLineMesh->GetRelativeRotation().Yaw, 0));
  LaserLineMesh->SetRelativeLocation(FVector(LaserLineMesh->GetRelativeLocation().X,
                                             LaserLineMesh->GetRelativeLocation().Y ,
                                             LaserLineMesh->GetRelativeLocation().Z - LaserOffset));
  LaserLineMesh->SetRelativeScale3D(FVector(0.015f, 0.015f, LaserRange * 0.01f));
  // Set hit mesh location accordingly
  LaserHitMesh->SetWorldLocation(
      LaserLineMesh->GetComponentLocation() + LaserLineMesh->GetUpVector() * LaserLineMesh->GetRelativeScale3D().Z *
      100.f);
}

void AUrErHCharacter::InitializeNameTag(UStaticMeshComponent *HeadMesh, FVector OffsetVector) {
  FAttachmentTransformRules AttachmentRules(
      EAttachmentRule::KeepWorld, // Keeps its current world position
      EAttachmentRule::KeepWorld, // Keeps its current world rotation
      EAttachmentRule::KeepWorld, // Crucial: Prevents parent scale from shrinking text
      false // Do not weld simulated physics bodies
      );
  NameTag->AttachToComponent(HeadMesh, AttachmentRules);
  NameTag->SetRelativeLocation(OffsetVector);
}

void AUrErHCharacter::UpdateLaserVisuals() {

  FVector Start = LaserLineMesh->GetComponentLocation();
  FVector End = Start + LaserLineMesh->GetUpVector() * LaserRange;
  FHitResult Hit;
  FCollisionQueryParams Params;
  Params.AddIgnoredActor(this);
  if ((GetWorld()->LineTraceSingleByChannel(Hit, Start, End, ECollisionChannel::ECC_Visibility, Params))) {
    // Hit
    End = Hit.Location;
    // Draw line mesh
    LaserLineMesh->SetRelativeScale3D(FVector(0.007f, 0.007f, FVector::Dist(Start, End) * 0.01f));
    LaserLineMesh->SetVisibility(true);
    // The origin cylinder mesh size is 100
    // Draw hit mesh, set the scale to make the hit mesh appear in constant screen size
    LaserHitMesh->SetWorldLocation(End);
    float DistToCam = FVector::Dist(GetActorLocation(), End);
    float HalfFOVRad = FMath::DegreesToRadians(50.f); // Half of FOV
    float FitScale = DistToCam * FMath::Tan(HalfFOVRad) * LaserHitMeshScreenSize;
    LaserHitMesh->SetWorldScale3D(FVector(FitScale) * 0.007f);
    LaserHitMesh->SetVisibility(true);
    // // Laserable? and local player?
    // if (GetController()) {
    //   if (GetController()->IsLocalController()) {
    //     if (Hit.GetActor() != LastHit) {
    //       CheckExit();
    //       // UE_LOG(LogUrErMTP, Log, TEXT("Value is: %f"), HalfFOVRad);
    //       // UE_LOG(LogUrErMTP, Warning, TEXT("New hit."));
    //       LastHit = Hit.GetActor();
    //       IUrErLaserInterface* IL = Cast<IUrErLaserInterface>(Hit.GetActor());
    //       if (IL) {
    //         IL->OnHover();
    //       }
    //     } else {
    //       IUrErLaserInterface* IL = Cast<IUrErLaserInterface>(Hit.GetActor());
    //       if (IL && bEnterPressed) {
    //         IL->OnEnter();
    //       }
    //     }
    //   }
    // }
  } else {
    // No hit, hide hit mesh
    LaserLineMesh->SetRelativeScale3D(FVector(0.007f, 0.007f, LaserRange * 0.01f));
    LaserLineMesh->SetVisibility(true);
    LaserHitMesh->SetVisibility(false);
  }
}

void AUrErHCharacter::HideLaserVisuals() {
  LaserLineMesh->SetVisibility(false);
  LaserHitMesh->SetVisibility(false);
}