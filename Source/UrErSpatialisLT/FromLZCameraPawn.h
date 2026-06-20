#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "FromLZCameraPawn.generated.h"

class UCameraComponent;
class USceneComponent;
class USpringArmComponent;

UCLASS()
class URERSPATIALISLT_API AFromLZCameraPawn : public APawn {
	GENERATED_BODY()

public:
	AFromLZCameraPawn();

protected:
	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;

	void MoveForward(float Value);
	void MoveRight(float Value);
	void MoveUp(float Value);
	void Zoom(float Value);
	void LookPitch(float Value);
	void LookYaw(float Value);

	void StartPitchLook();
	void StopPitchLook();

	FVector GetPlanarForwardDirection() const;
	FVector GetPlanarRightDirection() const;

	UPROPERTY(VisibleAnywhere, Category = "Camera")
	TObjectPtr<USceneComponent> SceneRoot;

	UPROPERTY(VisibleAnywhere, Category = "Camera")
	TObjectPtr<USpringArmComponent> CameraBoom;

	UPROPERTY(VisibleAnywhere, Category = "Camera")
	TObjectPtr<UCameraComponent> FromLZ;

	UPROPERTY(EditAnywhere, Category = "Camera|Movement")
	float MoveSpeed = 1200.0f;

	UPROPERTY(EditAnywhere, Category = "Camera|Zoom")
	float ZoomStep = 120.0f;

	UPROPERTY(EditAnywhere, Category = "Camera|Zoom")
	float MinZoom = 300.0f;

	UPROPERTY(EditAnywhere, Category = "Camera|Zoom")
	float MaxZoom = 2400.0f;

	UPROPERTY(EditAnywhere, Category = "Camera|Rotation")
	float PitchSpeed = 1.0f;

	UPROPERTY(EditAnywhere, Category = "Camera|Rotation")
	float YawSpeed = 1.0f;

	UPROPERTY(EditAnywhere, Category = "Camera|Rotation")
	float MinPitch = -89.0f;

	UPROPERTY(EditAnywhere, Category = "Camera|Rotation")
	float MaxPitch = +89.0f;

	bool bIsPitchLookActive = false;
};
