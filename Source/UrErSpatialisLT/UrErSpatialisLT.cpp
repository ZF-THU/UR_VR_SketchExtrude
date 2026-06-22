// Fill out your copyright notice in the Description page of Project Settings.

#include "FromLZSessionReset.h"
#include "UrErSpatialisLT.h"
#include "Modules/ModuleManager.h"

class FUrErSpatialisLTModule : public FDefaultGameModuleImpl
{
public:
	virtual void StartupModule() override
	{
		FDefaultGameModuleImpl::StartupModule();
		FFromLZSessionReset::Initialize();
	}

	virtual void ShutdownModule() override
	{
		FFromLZSessionReset::Shutdown();
		FDefaultGameModuleImpl::ShutdownModule();
	}
};

IMPLEMENT_PRIMARY_GAME_MODULE(FUrErSpatialisLTModule, UrErSpatialisLT, "UrErSpatialisLT");
