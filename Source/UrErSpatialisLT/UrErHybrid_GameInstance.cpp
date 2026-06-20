// Fill out your copyright notice in the Description page of Project Settings.


#include "UrErHybrid_GameInstance.h"
#include "Kismet/GameplayStatics.h"

void UUrErHybrid_GameInstance::Init() {
  Super::Init();

  // Bind our handler to the multicast on init
  // OnGamePlayStart.AddDynamic(this, &UUrErHybrid_GameInstance::GamePlayStartTravel);
}

void UUrErHybrid_GameInstance::Shutdown() {
  // Unbind to be safe when shutting down
  // OnGamePlayStart.RemoveDynamic(this, &UUrErHybrid_GameInstance::GamePlayStartTravel);

  Super::Shutdown();
}

void UUrErHybrid_GameInstance::ServerStartGamePlay() {
  
  // The following has been done in UI Blueprint 'Create Session'
  // const FName LevelName(TEXT("CismonValleyBetter"));
  // const FString Options = TEXT(
  //     "?Game=/Game/_UrErX/Blueprints/GameFramework/BP_UrErHybrid_GameMode.BP_UrErHybrid_GameMode_C?listen");
  // UGameplayStatics::OpenLevel(this, LevelName, true, Options);
  
  // Currently doing nothing but to broacast the game start event
  OnGamePlayStart.Broadcast();
}

void UUrErHybrid_GameInstance::PrintPlayerColorPalette() {
  for (int i = 0; i < PlayerColorPalette.Num(); i++) {
    UE_LOG(LogTemp, Warning, TEXT("PlayerColorPalette [%d] : %s"), i, *PlayerColorPalette[i].ToString());
  }
}


void UUrErHybrid_GameInstance::PrintPlayerNames() {
  UE_LOG(LogTemp, Warning, TEXT("About to create new session"));
  UE_LOG(LogTemp, Warning, TEXT("Player NameEN : %s | NameCN : %s"), *PlayerNameEN, *PlayerNameCN);
}

void UUrErHybrid_GameInstance::GamePlayStartTravel() {
  // const FName LevelName(TEXT("CismonValleyBetter"));
  // const FString Options = TEXT(
  //     "?Game=/Game/_UrErX/Blueprints/GameFramework/BP_UrErHybrid_GameMode.BP_UrErHybrid_GameMode_C");
  // UGameplayStatics::OpenLevel(this, LevelName, true, Options);
  // UE_LOG(LogTemp, Warning, TEXT("GamePlayStartTravel invoked from OnGamePlayStart delegate"));
}