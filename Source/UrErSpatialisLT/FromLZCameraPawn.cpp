#include "FromLZCameraPawn.h"

#include "Camera/CameraComponent.h"
#include "Components/InputComponent.h"
#include "Components/SceneComponent.h"
#include "GameFramework/SpringArmComponent.h"

AFromLZCameraPawn::AFromLZCameraPawn()
{
	PrimaryActorTick.bCanEverTick = false;

	SceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("SceneRoot"));
	SetRootComponent(SceneRoot);

	CameraBoom = CreateDefaultSubobject<USpringArmComponent>(TEXT("CameraBoom"));
	CameraBoom->SetupAttachment(SceneRoot);
	CameraBoom->TargetArmLength = 1200.0f;
	CameraBoom->SetRelativeRotation(FRotator(-45.0f, 0.0f, 0.0f));
	CameraBoom->bDoCollisionTest = false;
	CameraBoom->bEnableCameraLag = false;
	CameraBoom->bUsePawnControlRotation = false;

	FromLZ = CreateDefaultSubobject<UCameraComponent>(TEXT("FromLZ"));
	FromLZ->SetupAttachment(CameraBoom, USpringArmComponent::SocketName);
	FromLZ->bUsePawnControlRotation = false;

	AutoPossessPlayer = EAutoReceiveInput::Player0;
}

void AFromLZCameraPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	PlayerInputComponent->BindAxis(TEXT("MoveForward"), this, &AFromLZCameraPawn::MoveForward);
	PlayerInputComponent->BindAxis(TEXT("MoveRight"), this, &AFromLZCameraPawn::MoveRight);
	PlayerInputComponent->BindAxis(TEXT("MoveUp"), this, &AFromLZCameraPawn::MoveUp);
	PlayerInputComponent->BindAxis(TEXT("Zoom"), this, &AFromLZCameraPawn::Zoom);
	PlayerInputComponent->BindAxis(TEXT("LookPitch"), this, &AFromLZCameraPawn::LookPitch);
	PlayerInputComponent->BindAxis(TEXT("LookYaw"), this, &AFromLZCameraPawn::LookYaw);

	PlayerInputComponent->BindAction(TEXT("PitchLook"), IE_Pressed, this, &AFromLZCameraPawn::StartPitchLook);
	PlayerInputComponent->BindAction(TEXT("PitchLook"), IE_Released, this, &AFromLZCameraPawn::StopPitchLook);
}

void AFromLZCameraPawn::MoveForward(float Value)
{
	if (FMath::IsNearlyZero(Value))
	{
		return;
	}

	const FVector Offset = GetPlanarForwardDirection() * Value * MoveSpeed * GetWorld()->GetDeltaSeconds();
	AddActorWorldOffset(Offset, false);
}

void AFromLZCameraPawn::MoveRight(float Value)
{
	if (FMath::IsNearlyZero(Value))
	{
		return;
	}

	const FVector Offset = GetPlanarRightDirection() * Value * MoveSpeed * GetWorld()->GetDeltaSeconds();
	AddActorWorldOffset(Offset, false);
}

void AFromLZCameraPawn::MoveUp(float Value)
{
	if (FMath::IsNearlyZero(Value))
	{
		return;
	}

	const FVector Offset = FVector::UpVector * Value * MoveSpeed * GetWorld()->GetDeltaSeconds();
	AddActorWorldOffset(Offset, false);
}

void AFromLZCameraPawn::Zoom(float Value)
{
	if (FMath::IsNearlyZero(Value))
	{
		return;
	}

	const float NewArmLength = FMath::Clamp(CameraBoom->TargetArmLength - (Value * ZoomStep), MinZoom, MaxZoom);
	CameraBoom->TargetArmLength = NewArmLength;
}

void AFromLZCameraPawn::LookPitch(float Value)
{
	if (!bIsPitchLookActive || FMath::IsNearlyZero(Value))
	{
		return;
	}

	FRotator BoomRotation = CameraBoom->GetRelativeRotation();
	BoomRotation.Pitch = FMath::Clamp(BoomRotation.Pitch + (Value * PitchSpeed), MinPitch, MaxPitch);
	CameraBoom->SetRelativeRotation(BoomRotation);
}

void AFromLZCameraPawn::LookYaw(float Value)
{
	if (!bIsPitchLookActive || FMath::IsNearlyZero(Value))
	{
		return;
	}

	FRotator BoomRotation = CameraBoom->GetRelativeRotation();
	BoomRotation.Yaw += Value * YawSpeed;
	CameraBoom->SetRelativeRotation(BoomRotation);
}

void AFromLZCameraPawn::StartPitchLook()
{
	bIsPitchLookActive = true;
}

void AFromLZCameraPawn::StopPitchLook()
{
	bIsPitchLookActive = false;
}

FVector AFromLZCameraPawn::GetPlanarForwardDirection() const
{
	FVector Forward = CameraBoom->GetComponentRotation().Vector();
	Forward.Z = 0.0f;
	return Forward.GetSafeNormal();
}

FVector AFromLZCameraPawn::GetPlanarRightDirection() const
{
	FVector Right = FRotationMatrix(CameraBoom->GetComponentRotation()).GetScaledAxis(EAxis::Y);
	Right.Z = 0.0f;
	return Right.GetSafeNormal();
}
