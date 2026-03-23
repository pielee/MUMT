// Copyright Epic Games, Inc. All Rights Reserved.

#include "MUMT_Sim.h"
#include "Modules/ModuleManager.h"

DEFINE_LOG_CATEGORY_STATIC(LogMUMTSimModule, Log, All);

class FMUMTSimModule : public FDefaultGameModuleImpl
{
public:
    virtual void StartupModule() override
    {
        FDefaultGameModuleImpl::StartupModule();
        UE_LOG(LogMUMTSimModule, Log, TEXT("[UDP-STATE] MUMT_Sim module startup."));
    }

    virtual void ShutdownModule() override
    {
        UE_LOG(LogMUMTSimModule, Log, TEXT("[UDP-STATE] MUMT_Sim module shutdown."));
        FDefaultGameModuleImpl::ShutdownModule();
    }
};

IMPLEMENT_PRIMARY_GAME_MODULE(FMUMTSimModule, MUMT_Sim, "MUMT_Sim");
