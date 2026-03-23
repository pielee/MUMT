#include "PyAircraftUdpStateSenderSubsystem.h"

#include "Common/UdpSocketBuilder.h"
#include "Containers/Ticker.h"
#include "FDMTypes.h"
#include "Dom/JsonObject.h"
#include "JSBSimMovementComponent.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "GameFramework/Pawn.h"
#include "Kismet/GameplayStatics.h"
#include "UObject/UnrealType.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"
#include "SocketSubsystem.h"
#include "Sockets.h"

DEFINE_LOG_CATEGORY(LogPyAircraftUdpStateSender);

namespace
{
    constexpr double UnrealUnitsToMeters = 0.01;
    constexpr double DefaultGroundThresholdM = 1.5;
    constexpr double DefaultVerticalSpeedGroundThresholdMps = 3.0;
    constexpr double KnotsToMetersPerSecond = KNOT_TO_FEET_PER_SEC * FEET_TO_METER;
}

bool UPyAircraftUdpStateSenderSubsystem::ShouldCreateSubsystem(UObject* Outer) const
{
    return !IsRunningCommandlet();
}

void UPyAircraftUdpStateSenderSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
    Super::Initialize(Collection);

    UE_LOG(LogPyAircraftUdpStateSender, Log, TEXT("[UDP-STATE] subsystem initialize."));
    UE_LOG(
        LogPyAircraftUdpStateSender,
        Log,
        TEXT("[UDP-STATE][PORTS] sender start requested destination=%s:%d rate_hz=%.1f ego_hint=%s player_hint=%s (engine subsystem)"),
        *DestinationHost,
        DestinationPort,
        SendRateHz,
        *EgoActorNameHint,
        *PlayerActorNameHint
    );

    StartSender();
    TickHandle = FTSTicker::GetCoreTicker().AddTicker(
        FTickerDelegate::CreateUObject(this, &UPyAircraftUdpStateSenderSubsystem::TickSender)
    );
}

void UPyAircraftUdpStateSenderSubsystem::Deinitialize()
{
    if (TickHandle.IsValid())
    {
        FTSTicker::GetCoreTicker().RemoveTicker(TickHandle);
        TickHandle.Reset();
    }

    StopSender();
    Super::Deinitialize();
}

bool UPyAircraftUdpStateSenderSubsystem::StartSender()
{
    if (SenderSocket != nullptr)
    {
        UE_LOG(
            LogPyAircraftUdpStateSender,
            Log,
            TEXT("[UDP-STATE][PORTS] sender already running destination=%s:%d"),
            *DestinationHost,
            DestinationPort
        );
        return true;
    }

    bool bIpParsed = false;
    DestinationAddr = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
    DestinationAddr->SetIp(*DestinationHost, bIpParsed);
    DestinationAddr->SetPort(DestinationPort);

    if (!bIpParsed)
    {
        UE_LOG(
            LogPyAircraftUdpStateSender,
            Error,
            TEXT("[UDP-STATE] failed to parse destination host '%s'."),
            *DestinationHost
        );
        DestinationAddr.Reset();
        return false;
    }

    SenderSocket = FUdpSocketBuilder(TEXT("PyAircraftUdpStateSender"))
        .AsReusable()
        .WithSendBufferSize(1024 * 1024);

    if (SenderSocket == nullptr)
    {
        UE_LOG(
            LogPyAircraftUdpStateSender,
            Error,
            TEXT("[UDP-STATE] failed to create sender socket for %s:%d"),
            *DestinationHost,
            DestinationPort
        );
        DestinationAddr.Reset();
        return false;
    }

    SenderSocket->SetNonBlocking(true);
    SentPacketCount = 0;
    SendAccumulatorS = 0.0;

    UE_LOG(
        LogPyAircraftUdpStateSender,
        Log,
        TEXT("[UDP-STATE][PORTS] sender started destination=%s:%d"),
        *DestinationHost,
        DestinationPort
    );
    return true;
}

void UPyAircraftUdpStateSenderSubsystem::StopSender()
{
    if (SenderSocket == nullptr)
    {
        return;
    }

    SenderSocket->Close();
    ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(SenderSocket);
    SenderSocket = nullptr;
    DestinationAddr.Reset();

    UE_LOG(LogPyAircraftUdpStateSender, Log, TEXT("[UDP-STATE] sender stopped."));
}

bool UPyAircraftUdpStateSenderSubsystem::IsSenderRunning() const
{
    return SenderSocket != nullptr;
}

FString UPyAircraftUdpStateSenderSubsystem::GetDestinationHost() const
{
    return DestinationHost;
}

int32 UPyAircraftUdpStateSenderSubsystem::GetDestinationPort() const
{
    return DestinationPort;
}

int32 UPyAircraftUdpStateSenderSubsystem::GetSentPacketCount() const
{
    return SentPacketCount;
}

bool UPyAircraftUdpStateSenderSubsystem::TickSender(float DeltaTime)
{
    if (SenderSocket == nullptr || !DestinationAddr.IsValid())
    {
        StartSender();
        return true;
    }

    if (ResolveActiveGameWorld() == nullptr)
    {
        return true;
    }

    const double EffectiveDt = DeltaTime > 0.0f ? static_cast<double>(DeltaTime) : (1.0 / 60.0);
    SendAccumulatorS += EffectiveDt;
    const double SendIntervalS = 1.0 / FMath::Max(1.0f, SendRateHz);
    if (SendAccumulatorS < SendIntervalS)
    {
        return true;
    }

    SendAccumulatorS = 0.0;
    SendCurrentStatePacket();
    return true;
}

UWorld* UPyAircraftUdpStateSenderSubsystem::ResolveActiveGameWorld() const
{
    if (GEngine == nullptr)
    {
        return nullptr;
    }

    UWorld* ResolvedWorld = nullptr;

    for (const FWorldContext& WorldContext : GEngine->GetWorldContexts())
    {
        UWorld* CandidateWorld = WorldContext.World();
        if (CandidateWorld == nullptr)
        {
            continue;
        }

        if (
            WorldContext.WorldType == EWorldType::PIE ||
            WorldContext.WorldType == EWorldType::Game ||
            WorldContext.WorldType == EWorldType::GamePreview
        )
        {
            ResolvedWorld = CandidateWorld;
            break;
        }
    }

    if (ResolvedWorld == nullptr)
    {
        for (const FWorldContext& WorldContext : GEngine->GetWorldContexts())
        {
            UWorld* CandidateWorld = WorldContext.World();
            if (CandidateWorld != nullptr && WorldContext.WorldType != EWorldType::EditorPreview)
            {
                ResolvedWorld = CandidateWorld;
                break;
            }
        }
    }

    if (CachedResolvedWorld.Get() != ResolvedWorld)
    {
        CachedResolvedWorld = ResolvedWorld;
        CachedEgoActor.Reset();
        CachedPlayerActor.Reset();

        if (ResolvedWorld != nullptr)
        {
            UE_LOG(
                LogPyAircraftUdpStateSender,
                Log,
                TEXT("[UDP-STATE] active world resolved name=%s type=%d"),
                *ResolvedWorld->GetName(),
                static_cast<int32>(ResolvedWorld->WorldType)
            );
        }
    }

    return ResolvedWorld;
}

bool UPyAircraftUdpStateSenderSubsystem::SendCurrentStatePacket()
{
    if (SenderSocket == nullptr || !DestinationAddr.IsValid())
    {
        return false;
    }

    AActor* EgoActor = ResolveEgoAircraft();
    AActor* PlayerActor = ResolvePlayerAircraft();

    TArray<TSharedPtr<FJsonValue>> StatesArray;
    TArray<FString> SentActorNames;

    if (EgoActor != nullptr)
    {
        TSharedPtr<FJsonObject> EgoState;
        if (FillActorStateJson(EgoActor, EgoLogicalActorId, TEXT("ego"), EgoState))
        {
            StatesArray.Add(MakeShared<FJsonValueObject>(EgoState));
            SentActorNames.Add(EgoActor->GetName());
        }
    }

    if (PlayerActor != nullptr && PlayerActor != EgoActor)
    {
        TSharedPtr<FJsonObject> PlayerState;
        if (FillActorStateJson(PlayerActor, PlayerLogicalActorId, TEXT("enemy"), PlayerState))
        {
            StatesArray.Add(MakeShared<FJsonValueObject>(PlayerState));
            SentActorNames.Add(PlayerActor->GetName());
        }
    }

    if (StatesArray.Num() <= 0)
    {
        UE_LOG(LogPyAircraftUdpStateSender, Warning, TEXT("[UDP-STATE] no aircraft state available to send."));
        return false;
    }

    TSharedRef<FJsonObject> Packet = MakeShared<FJsonObject>();
    Packet->SetStringField(TEXT("source"), TEXT("PyAircraftUdpStateSenderSubsystem"));
    Packet->SetNumberField(TEXT("packet_seq"), SentPacketCount + 1);
    Packet->SetNumberField(TEXT("sent_count"), StatesArray.Num());
    Packet->SetNumberField(TEXT("timestamp_s"), FPlatformTime::Seconds());
    Packet->SetArrayField(TEXT("states"), StatesArray);

    const FString Payload = SerializeJsonObject(Packet);
    FTCHARToUTF8 Utf8Payload(*Payload);

    int32 BytesSent = 0;
    const bool bSent = SenderSocket->SendTo(
        reinterpret_cast<const uint8*>(Utf8Payload.Get()),
        Utf8Payload.Length(),
        BytesSent,
        *DestinationAddr
    );

    if (!bSent)
    {
        UE_LOG(LogPyAircraftUdpStateSender, Error, TEXT("[UDP-STATE] send failed for payload: %s"), *Payload);
        return false;
    }

    SentPacketCount += 1;
    UE_LOG(
        LogPyAircraftUdpStateSender,
        Log,
        TEXT("[UDP-STATE] sent_count=%d aircraft_sent=%d dest=%s:%d actor_names=%s"),
        SentPacketCount,
        StatesArray.Num(),
        *DestinationHost,
        DestinationPort,
        *FString::Join(SentActorNames, TEXT(", "))
    );

    return true;
}

AActor* UPyAircraftUdpStateSenderSubsystem::ResolveEgoAircraft() const
{
    if (CachedEgoActor.IsValid())
    {
        return CachedEgoActor.Get();
    }

    AActor* Resolved = FindActorByNameHint(EgoActorNameHint);
    if (Resolved == nullptr)
    {
        UE_LOG(
            LogPyAircraftUdpStateSender,
            Warning,
            TEXT("[UDP-STATE] ego aircraft hint '%s' not found."),
            *EgoActorNameHint
        );
        return nullptr;
    }

    CachedEgoActor = Resolved;
    UE_LOG(
        LogPyAircraftUdpStateSender,
        Log,
        TEXT("[UDP-STATE] resolved ego aircraft logical_actor_id=%s actor_name=%s"),
        *EgoLogicalActorId,
        *Resolved->GetName()
    );
    return Resolved;
}

AActor* UPyAircraftUdpStateSenderSubsystem::ResolvePlayerAircraft() const
{
    if (CachedPlayerActor.IsValid())
    {
        return CachedPlayerActor.Get();
    }

    UWorld* World = ResolveActiveGameWorld();
    if (World == nullptr)
    {
        return nullptr;
    }

    if (APawn* PlayerPawn = UGameplayStatics::GetPlayerPawn(World, 0))
    {
        if (MatchesActorHint(PlayerPawn, PlayerActorNameHint))
        {
            CachedPlayerActor = PlayerPawn;
            UE_LOG(
                LogPyAircraftUdpStateSender,
                Log,
                TEXT("[UDP-STATE] resolved player aircraft from player pawn logical_actor_id=%s actor_name=%s"),
                *PlayerLogicalActorId,
                *PlayerPawn->GetName()
            );
            return PlayerPawn;
        }
    }

    AActor* Resolved = FindActorByNameHint(PlayerActorNameHint);
    if (Resolved == nullptr)
    {
        UE_LOG(
            LogPyAircraftUdpStateSender,
            Warning,
            TEXT("[UDP-STATE] player aircraft hint '%s' not found."),
            *PlayerActorNameHint
        );
        return nullptr;
    }

    CachedPlayerActor = Resolved;
    UE_LOG(
        LogPyAircraftUdpStateSender,
        Log,
        TEXT("[UDP-STATE] resolved player aircraft logical_actor_id=%s actor_name=%s"),
        *PlayerLogicalActorId,
        *Resolved->GetName()
    );
    return Resolved;
}

AActor* UPyAircraftUdpStateSenderSubsystem::FindActorByNameHint(const FString& NameHint) const
{
    UWorld* World = ResolveActiveGameWorld();
    if (World == nullptr)
    {
        return nullptr;
    }

    for (TActorIterator<AActor> It(World); It; ++It)
    {
        AActor* Actor = *It;
        if (MatchesActorHint(Actor, NameHint))
        {
            return Actor;
        }
    }

    return nullptr;
}

bool UPyAircraftUdpStateSenderSubsystem::MatchesActorHint(const AActor* Actor, const FString& NameHint) const
{
    if (Actor == nullptr || NameHint.IsEmpty())
    {
        return false;
    }

    auto MatchesCandidate = [](const FString& CandidateValue, const FString& HintValue) -> bool
    {
        const FString Candidate = CandidateValue.ToLower();
        const FString Hint = HintValue.ToLower();
        if (Candidate == Hint || Candidate.StartsWith(Hint) || Candidate.Contains(Hint))
        {
            return true;
        }

        FString SimplifiedHint = Hint;
        int32 LastUnderscoreIndex = INDEX_NONE;
        if (Hint.FindLastChar(TEXT('_'), LastUnderscoreIndex))
        {
            const FString Tail = Hint.Mid(LastUnderscoreIndex + 1);
            bool bAllDigits = !Tail.IsEmpty();
            for (const TCHAR Character : Tail)
            {
                if (!FChar::IsDigit(Character))
                {
                    bAllDigits = false;
                    break;
                }
            }

            if (bAllDigits)
            {
                SimplifiedHint = Hint.Left(LastUnderscoreIndex);
                if (!SimplifiedHint.IsEmpty() &&
                    (Candidate == SimplifiedHint ||
                     Candidate.StartsWith(SimplifiedHint) ||
                     Candidate.Contains(SimplifiedHint)))
                {
                    return true;
                }
            }
        }

        return false;
    };

    if (MatchesCandidate(Actor->GetName(), NameHint))
    {
        return true;
    }

    if (MatchesCandidate(Actor->GetClass()->GetName(), NameHint))
    {
        return true;
    }

    for (const FName& Tag : Actor->Tags)
    {
        if (MatchesCandidate(Tag.ToString(), NameHint))
        {
            return true;
        }
    }

    return false;
}

bool UPyAircraftUdpStateSenderSubsystem::FillActorStateJson(
    AActor* Actor,
    const FString& LogicalActorId,
    const FString& Role,
    TSharedPtr<FJsonObject>& OutStateObject
) const
{
    if (Actor == nullptr)
    {
        return false;
    }

    FAircraftState AircraftState;
    if (TryGetAircraftStateFromBlueprintFunction(Actor, AircraftState))
    {
        return PopulateStateJsonFromAircraftState(
            Actor,
            LogicalActorId,
            Role,
            AircraftState,
            TEXT("GetAircraftState"),
            OutStateObject
        );
    }

    if (TryGetAircraftStateFromJSBSimComponent(Actor, AircraftState))
    {
        return PopulateStateJsonFromAircraftState(
            Actor,
            LogicalActorId,
            Role,
            AircraftState,
            TEXT("JSBSimMovementComponent"),
            OutStateObject
        );
    }

    const FVector LocationCm = Actor->GetActorLocation();
    const FVector VelocityCmPerSec = Actor->GetVelocity();
    const FRotator Rotation = Actor->GetActorRotation();

    const FVector LocationM = LocationCm * UnrealUnitsToMeters;
    const FVector VelocityMps = VelocityCmPerSec * UnrealUnitsToMeters;
    const double AirspeedMps = VelocityMps.Size();
    const bool bOnGround = (LocationM.Z <= DefaultGroundThresholdM)
        && (FMath::Abs(VelocityMps.Z) <= DefaultVerticalSpeedGroundThresholdMps);

    OutStateObject = MakeShared<FJsonObject>();
    OutStateObject->SetStringField(TEXT("actor_id"), LogicalActorId);
    OutStateObject->SetStringField(TEXT("aircraft_name"), Actor->GetName());
    OutStateObject->SetStringField(TEXT("role"), Role);

    TSharedPtr<FJsonObject> PositionObject = MakeShared<FJsonObject>();
    PositionObject->SetNumberField(TEXT("x"), LocationM.X);
    PositionObject->SetNumberField(TEXT("y"), LocationM.Y);
    PositionObject->SetNumberField(TEXT("z"), LocationM.Z);
    OutStateObject->SetObjectField(TEXT("position_m"), PositionObject);

    TSharedPtr<FJsonObject> VelocityObject = MakeShared<FJsonObject>();
    VelocityObject->SetNumberField(TEXT("x"), VelocityMps.X);
    VelocityObject->SetNumberField(TEXT("y"), VelocityMps.Y);
    VelocityObject->SetNumberField(TEXT("z"), VelocityMps.Z);
    OutStateObject->SetObjectField(TEXT("velocity_mps"), VelocityObject);

    OutStateObject->SetNumberField(TEXT("yaw_deg"), Rotation.Yaw);
    OutStateObject->SetNumberField(TEXT("pitch_deg"), Rotation.Pitch);
    OutStateObject->SetNumberField(TEXT("roll_deg"), Rotation.Roll);
    OutStateObject->SetNumberField(TEXT("airspeed_mps"), AirspeedMps);
    OutStateObject->SetNumberField(TEXT("altitude_world_m"), LocationM.Z);
    OutStateObject->SetNumberField(TEXT("altitude_m"), LocationM.Z);
    OutStateObject->SetBoolField(TEXT("on_ground"), bOnGround);
    OutStateObject->SetBoolField(TEXT("engine_running"), true);
    OutStateObject->SetBoolField(TEXT("parking_brake"), false);
    OutStateObject->SetNumberField(TEXT("wheel_brake"), 0.0);
    OutStateObject->SetBoolField(TEXT("gear_down"), true);
    OutStateObject->SetBoolField(TEXT("control_enabled"), true);
    OutStateObject->SetNumberField(TEXT("state_age_s"), 0.0);
    OutStateObject->SetStringField(TEXT("status_message"), TEXT("udp_live_state_transform_fallback"));
    OutStateObject->SetStringField(TEXT("state_source"), TEXT("ActorTransformFallback"));

    UE_LOG(
        LogPyAircraftUdpStateSender,
        Log,
        TEXT("[UDP-STATE] aircraft state queued role=%s logical_actor_id=%s actor_name=%s source=%s airspeed=%.2f on_ground=%d vel=(%.2f,%.2f,%.2f)"),
        *Role,
        *LogicalActorId,
        *Actor->GetName(),
        TEXT("ActorTransformFallback"),
        AirspeedMps,
        bOnGround,
        VelocityMps.X,
        VelocityMps.Y,
        VelocityMps.Z
    );

    return true;
}

bool UPyAircraftUdpStateSenderSubsystem::TryGetAircraftStateFromBlueprintFunction(
    AActor* Actor,
    FAircraftState& OutAircraftState
) const
{
    if (Actor == nullptr)
    {
        return false;
    }

    static const FName GetAircraftStateFunctionName(TEXT("GetAircraftState"));
    UFunction* Function = Actor->FindFunction(GetAircraftStateFunctionName);
    if (Function == nullptr)
    {
        return false;
    }

    TArray<uint8> ParametersStorage;
    ParametersStorage.SetNumZeroed(Function->ParmsSize);
    Actor->ProcessEvent(Function, ParametersStorage.GetData());

    for (TFieldIterator<FProperty> PropertyIt(Function); PropertyIt; ++PropertyIt)
    {
        FProperty* Property = *PropertyIt;
        if (!Property->HasAnyPropertyFlags(CPF_Parm))
        {
            continue;
        }

        FStructProperty* StructProperty = CastField<FStructProperty>(Property);
        if (StructProperty == nullptr || StructProperty->Struct != FAircraftState::StaticStruct())
        {
            continue;
        }

        if (
            !Property->HasAnyPropertyFlags(CPF_ReturnParm) &&
            !Property->HasAnyPropertyFlags(CPF_OutParm)
        )
        {
            continue;
        }

        if (const FAircraftState* ReflectedState =
                StructProperty->ContainerPtrToValuePtr<FAircraftState>(ParametersStorage.GetData()))
        {
            OutAircraftState = *ReflectedState;
            return true;
        }
    }

    return false;
}

bool UPyAircraftUdpStateSenderSubsystem::TryGetAircraftStateFromJSBSimComponent(
    AActor* Actor,
    FAircraftState& OutAircraftState
) const
{
    if (Actor == nullptr)
    {
        return false;
    }

    const UJSBSimMovementComponent* JSBSimMovementComponent =
        Actor->FindComponentByClass<UJSBSimMovementComponent>();
    if (JSBSimMovementComponent == nullptr)
    {
        return false;
    }

    OutAircraftState = JSBSimMovementComponent->AircraftState;
    return true;
}

bool UPyAircraftUdpStateSenderSubsystem::PopulateStateJsonFromAircraftState(
    AActor* Actor,
    const FString& LogicalActorId,
    const FString& Role,
    const FAircraftState& AircraftState,
    const FString& StateSource,
    TSharedPtr<FJsonObject>& OutStateObject
) const
{
    if (Actor == nullptr)
    {
        return false;
    }

    const FVector LocationCm = Actor->GetActorLocation();
    const FVector LocationM = LocationCm * UnrealUnitsToMeters;
    const FRotator Rotation = AircraftState.LocalEulerAngles.IsNearlyZero()
        ? Actor->GetActorRotation()
        : AircraftState.LocalEulerAngles;

    FVector HorizontalForward = AircraftState.UEForwardHorizontal;
    HorizontalForward.Z = 0.0;
    if (HorizontalForward.IsNearlyZero())
    {
        HorizontalForward = Actor->GetActorForwardVector();
        HorizontalForward.Z = 0.0;
    }
    HorizontalForward.Normalize();

    const double GroundSpeedMps = AircraftState.GroundSpeedKts * KnotsToMetersPerSecond;
    const double TrueAirspeedMps = AircraftState.TotalVelocityKts * KnotsToMetersPerSecond;
    const double CalibratedAirspeedMps = AircraftState.CalibratedAirSpeedKts * KnotsToMetersPerSecond;
    const double VerticalSpeedMps = AircraftState.AltitudeRateFtps * FEET_TO_METER;

    FVector VelocityMps = Actor->GetVelocity() * UnrealUnitsToMeters;
    if (VelocityMps.IsNearlyZero())
    {
        VelocityMps = FVector(
            HorizontalForward.X * GroundSpeedMps,
            HorizontalForward.Y * GroundSpeedMps,
            VerticalSpeedMps
        );
    }

    bool bOnGround = false;
    bool bGearDown = true;
    bool bEngineRunning = true;
    double ParkingBrake = 0.0;
    double WheelBrake = 0.0;

    if (const UJSBSimMovementComponent* JSBSimMovementComponent =
            Actor->FindComponentByClass<UJSBSimMovementComponent>())
    {
        bEngineRunning = JSBSimMovementComponent->bStartWithEngineRunning;
        bGearDown = JSBSimMovementComponent->bStartWithGearDown;

        for (const FGear& Gear : JSBSimMovementComponent->Gears)
        {
            bOnGround = bOnGround || Gear.HasWeightOnWheel;
            bGearDown = bGearDown || Gear.IsDown || Gear.NormalizedPosition > 0.5;
        }

        if (JSBSimMovementComponent->EngineStates.Num() > 0)
        {
            bEngineRunning = false;
            for (const FEngineState& EngineState : JSBSimMovementComponent->EngineStates)
            {
                bEngineRunning = bEngineRunning || EngineState.Running;
            }
        }

        ParkingBrake = JSBSimMovementComponent->Commands.ParkingBrake;
        WheelBrake = FMath::Max3(
            JSBSimMovementComponent->Commands.LeftBrake,
            JSBSimMovementComponent->Commands.RightBrake,
            JSBSimMovementComponent->Commands.CenterBrake
        );
    }

    if (!bOnGround)
    {
        const double AltitudeAglM = AircraftState.AltitudeAGLFt * FEET_TO_METER;
        bOnGround = (AltitudeAglM <= DefaultGroundThresholdM)
            && (FMath::Abs(VelocityMps.Z) <= DefaultVerticalSpeedGroundThresholdMps);
    }

    const double AirspeedMps = TrueAirspeedMps > 0.01
        ? TrueAirspeedMps
        : (CalibratedAirspeedMps > 0.01 ? CalibratedAirspeedMps : VelocityMps.Size());

    OutStateObject = MakeShared<FJsonObject>();
    OutStateObject->SetStringField(TEXT("actor_id"), LogicalActorId);
    OutStateObject->SetStringField(TEXT("aircraft_name"), Actor->GetName());
    OutStateObject->SetStringField(TEXT("role"), Role);
    OutStateObject->SetStringField(TEXT("state_source"), StateSource);

    TSharedPtr<FJsonObject> PositionObject = MakeShared<FJsonObject>();
    PositionObject->SetNumberField(TEXT("x"), LocationM.X);
    PositionObject->SetNumberField(TEXT("y"), LocationM.Y);
    PositionObject->SetNumberField(TEXT("z"), LocationM.Z);
    OutStateObject->SetObjectField(TEXT("position_m"), PositionObject);

    TSharedPtr<FJsonObject> VelocityObject = MakeShared<FJsonObject>();
    VelocityObject->SetNumberField(TEXT("x"), VelocityMps.X);
    VelocityObject->SetNumberField(TEXT("y"), VelocityMps.Y);
    VelocityObject->SetNumberField(TEXT("z"), VelocityMps.Z);
    OutStateObject->SetObjectField(TEXT("velocity_mps"), VelocityObject);

    OutStateObject->SetNumberField(TEXT("yaw_deg"), Rotation.Yaw);
    OutStateObject->SetNumberField(TEXT("pitch_deg"), Rotation.Pitch);
    OutStateObject->SetNumberField(TEXT("roll_deg"), Rotation.Roll);
    OutStateObject->SetNumberField(TEXT("airspeed_mps"), AirspeedMps);
    OutStateObject->SetNumberField(TEXT("ground_speed_mps"), GroundSpeedMps);
    OutStateObject->SetNumberField(TEXT("calibrated_airspeed_mps"), CalibratedAirspeedMps);
    OutStateObject->SetNumberField(TEXT("altitude_world_m"), LocationM.Z);
    OutStateObject->SetNumberField(TEXT("altitude_m"), LocationM.Z);
    OutStateObject->SetNumberField(TEXT("altitude_agl_m"), AircraftState.AltitudeAGLFt * FEET_TO_METER);
    OutStateObject->SetNumberField(TEXT("altitude_asl_m"), AircraftState.AltitudeASLFt * FEET_TO_METER);
    OutStateObject->SetNumberField(TEXT("vertical_speed_mps"), VerticalSpeedMps);
    OutStateObject->SetNumberField(TEXT("angle_of_attack_deg"), 0.0);
    OutStateObject->SetBoolField(TEXT("on_ground"), bOnGround);
    OutStateObject->SetBoolField(TEXT("engine_running"), bEngineRunning);
    OutStateObject->SetBoolField(TEXT("parking_brake"), ParkingBrake > 0.5);
    OutStateObject->SetNumberField(TEXT("wheel_brake"), WheelBrake);
    OutStateObject->SetBoolField(TEXT("gear_down"), bGearDown);
    OutStateObject->SetBoolField(TEXT("control_enabled"), true);
    OutStateObject->SetNumberField(TEXT("state_age_s"), 0.0);
    OutStateObject->SetStringField(
        TEXT("status_message"),
        FString::Printf(TEXT("udp_live_state_%s"), *StateSource)
    );

    UE_LOG(
        LogPyAircraftUdpStateSender,
        Log,
        TEXT("[UDP-STATE] aircraft state queued role=%s logical_actor_id=%s actor_name=%s source=%s airspeed=%.2f ground_speed=%.2f on_ground=%d vel=(%.2f,%.2f,%.2f)"),
        *Role,
        *LogicalActorId,
        *Actor->GetName(),
        *StateSource,
        AirspeedMps,
        GroundSpeedMps,
        bOnGround,
        VelocityMps.X,
        VelocityMps.Y,
        VelocityMps.Z
    );

    return true;
}

FString UPyAircraftUdpStateSenderSubsystem::SerializeJsonObject(
    const TSharedRef<FJsonObject>& JsonObject
) const
{
    FString Output;
    const TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&Output);
    FJsonSerializer::Serialize(JsonObject, Writer);
    return Output;
}
