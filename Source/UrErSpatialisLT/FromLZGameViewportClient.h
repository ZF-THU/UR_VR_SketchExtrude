#pragma once

#include "CoreMinimal.h"
#include "Engine/GameViewportClient.h"
#include "FromLZGameViewportClient.generated.h"

class FCanvas;
class FViewport;

UCLASS()
class URERSPATIALISLT_API UFromLZGameViewportClient : public UGameViewportClient {
	GENERATED_BODY()

public:
	virtual void Init(struct FWorldContext& WorldContext, UGameInstance* OwningGameInstance, bool bCreateNewAudioDevice = true) override;
	virtual void Tick(float DeltaTime) override;
	virtual void Draw(FViewport* Viewport, FCanvas* SceneCanvas) override;
	virtual bool InputKey(const FInputKeyEventArgs& EventArgs) override;
	virtual void BeginDestroy() override;
};
