#pragma once

#include "CoreMinimal.h"
#include "Subsystems/EngineSubsystem.h"
#include "Containers/Ticker.h"
#include "PyAircraftUdpStateSenderSubsystem.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogPyAircraftUdpStateSender, Log, All);

struct FAircraftState;

UCLASS()
class MUMT_SIM_API UPyAircraftUdpStateSenderSubsystem : public UEngineSubsystem
{
    GENERATED_BODY()

public:
    virtual bool ShouldCreateSubsystem(UObject* Outer) const override;
    virtual void Initialize(FSubsystemCollectionBase& Collection) override;
    virtual void Deinitialize() override;

    UFUNCTION(BlueprintCallable)
    bool StartSender();

    UFUNCTION(BlueprintCallable)
    void StopSender();

    UFUNCTION(BlueprintPure)
    bool IsSenderRunning() const;

    UFUNCTION(BlueprintPure)
    FString GetDestinationHost() const;

    UFUNCTION(BlueprintPure)
    int32 GetDestinationPort() const;

    UFUNCTION(BlueprintPure)
    int32 GetSentPacketCount() const;

private:
    bool TickSender(float DeltaTime);
    bool SendCurrentStatePacket();
    class UWorld* ResolveActiveGameWorld() const;

    class AActor* ResolveEgoAircraft() const;
    class AActor* ResolvePlayerAircraft() const;
    class AActor* FindActorByNameHint(const FString& NameHint) const;
    bool MatchesActorHint(const class AActor* Actor, const FString& NameHint) const;
    bool TryGetAircraftStateFromBlueprintFunction(class AActor* Actor, FAircraftState& OutAircraftState) const;
    bool TryGetAircraftStateFromJSBSimComponent(class AActor* Actor, FAircraftState& OutAircraftState) const;
    bool PopulateStateJsonFromAircraftState(
        class AActor* Actor,
        const FString& LogicalActorId,
        const FString& Role,
        const FAircraftState& AircraftState,
        const FString& StateSource,
        TSharedPtr<class FJsonObject>& OutStateObject
    ) const;

    bool FillActorStateJson(
        class AActor* Actor,
        const FString& LogicalActorId,
        const FString& Role,
        TSharedPtr<class FJsonObject>& OutStateObject
    ) const;

    FString SerializeJsonObject(const TSharedRef<class FJsonObject>& JsonObject) const;

private:
    FString DestinationHost = TEXT("127.0.0.1");
    int32 DestinationPort = 5007;
    float SendRateHz = 20.0f;
    FString EgoLogicalActorId = TEXT("F16_UAV_1");
    FString PlayerLogicalActorId = TEXT("M_F16");
    FString EgoActorNameHint = TEXT("F16_UAV_1");
    FString PlayerActorNameHint = TEXT("M_F16");
    double SendAccumulatorS = 0.0;
    int32 SentPacketCount = 0;

    class FSocket* SenderSocket = nullptr;
    TSharedPtr<class FInternetAddr> DestinationAddr;
    mutable TWeakObjectPtr<class UWorld> CachedResolvedWorld;
    mutable TWeakObjectPtr<class AActor> CachedEgoActor;
    mutable TWeakObjectPtr<class AActor> CachedPlayerActor;
    FTSTicker::FDelegateHandle TickHandle;
};
