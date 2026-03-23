#include "UDPControlReceiver.h"

#include "Engine/Engine.h"
#include "HAL/RunnableThread.h"
#include "Common/UdpSocketBuilder.h"
#include "Sockets.h"
#include "SocketSubsystem.h"
#include "IPAddress.h"

#include "Kismet/GameplayStatics.h"
#include "GameFramework/Pawn.h"
#include "UObject/UnrealType.h"
#include "Engine/World.h"
#include "Components/PrimitiveComponent.h"
#include "JSBSimMovementComponent.h"

AUDPControlReceiver::AUDPControlReceiver()
{
    PrimaryActorTick.bCanEverTick = true;
}

void AUDPControlReceiver::BeginPlay()
{
    Super::BeginPlay();

    if (StartUDPReceiver())
    {
        UE_LOG(
            LogTemp,
            Warning,
            TEXT("[UDP][PORTS] command receiver listen=0.0.0.0:%d actor_target_hint=%s exact_target=%s"),
            ListenPort,
            *TargetPawnName,
            *TargetPawnExactName
        );
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("[UDP] Failed to start receiver"));
    }

    if (bEnableStateSender)
    {
        if (StartUDPSender())
        {
            UE_LOG(
                LogTemp,
                Warning,
                TEXT("[UDP][PORTS] legacy state sender destination=%s:%d enabled=1"),
                *PythonIP,
                PythonStatePort
            );
        }
        else
        {
            UE_LOG(LogTemp, Error, TEXT("[UDP] Failed to start sender"));
        }
    }
    else
    {
        UE_LOG(
            LogTemp,
            Warning,
            TEXT("[UDP][PORTS] legacy state sender disabled on %s configured_destination=%s:%d"),
            *GetName(),
            *PythonIP,
            PythonStatePort
        );
    }

    CachedTargetPawn = FindTargetPawn();
    if (CachedTargetPawn)
    {
        UE_LOG(LogTemp, Warning, TEXT("[UDP] Target Pawn found at BeginPlay: %s"), *CachedTargetPawn->GetName());
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("[UDP] Target Pawn not found at BeginPlay. TargetPawnName=%s"), *TargetPawnName);
    }

    bHasPrevLocation = false;
    PrevStateSendTime = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0;
}

void AUDPControlReceiver::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    // 1) Python -> Unreal 제어 수신
    ReceiveUDPData();

    // 2) 대상 Pawn 확보
    if (!CachedTargetPawn || !IsValid(CachedTargetPawn))
    {
        CachedTargetPawn = FindTargetPawn();
        if (!CachedTargetPawn)
        {
            return;
        }
    }

    UJSBSimMovementComponent* MovementComponent = FindJSBSimMovementComponent(CachedTargetPawn);

    if (ShouldEmitDiagnostics())
    {
        LogPawnBindingState(CachedTargetPawn, MovementComponent, TEXT("pre_apply"));
        LogControlApplication(CachedTargetPawn, MovementComponent, TEXT("pre_apply"));
    }

    // 3) 수신값을 Blueprint 변수에 반영
    const bool bRollOk = SetBlueprintNumber(CachedTargetPawn, TEXT("UDP_Roll"), Roll);
    const bool bPitchOk = SetBlueprintNumber(CachedTargetPawn, TEXT("UDP_Pitch"), Pitch);
    const bool bYawOk = SetBlueprintNumber(CachedTargetPawn, TEXT("UDP_Yaw"), Yaw);
    const bool bThrottleOk = SetBlueprintNumber(CachedTargetPawn, TEXT("UDP_Throttle"), Throttle);

    if (bRollOk && bPitchOk && bYawOk && bThrottleOk)
    {
        UE_LOG(LogTemp, Verbose,
            TEXT("[UDP->BP] Applied Roll=%.3f Pitch=%.3f Yaw=%.3f Thr=%.3f"),
            Roll, Pitch, Yaw, Throttle);
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("[UDP] One or more Blueprint variables were not set on %s"),
            *CachedTargetPawn->GetName());
    }

    if (bDirectDriveJSBSimCommands && MovementComponent)
    {
        ApplyDirectJSBSimCommands(MovementComponent);
    }

    if (ShouldEmitDiagnostics())
    {
        LogControlApplication(CachedTargetPawn, MovementComponent, TEXT("post_apply"));
    }

    // 4) Unreal -> Python 상태 송신
    if (bEnableStateSender)
    {
        StateSendAccumulator += DeltaTime;
        if (StateSendAccumulator >= StateSendInterval)
        {
            StateSendAccumulator = 0.0f;
            SendStateToPython();
        }
    }
}

void AUDPControlReceiver::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    StopUDPReceiver();
    StopUDPSender();

    Super::EndPlay(EndPlayReason);
}

bool AUDPControlReceiver::StartUDPReceiver()
{
    ListenSocket = FUdpSocketBuilder(TEXT("UDP_Control_Receiver"))
        .AsNonBlocking()
        .AsReusable()
        .BoundToPort(ListenPort)
        .WithReceiveBufferSize(2 * 1024 * 1024);

    return ListenSocket != nullptr;
}

void AUDPControlReceiver::StopUDPReceiver()
{
    if (ListenSocket)
    {
        ListenSocket->Close();
        ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(ListenSocket);
        ListenSocket = nullptr;
    }
}

bool AUDPControlReceiver::StartUDPSender()
{
    SendSocket = FUdpSocketBuilder(TEXT("UDP_State_Sender"))
        .AsReusable()
        .WithSendBufferSize(2 * 1024 * 1024);

    if (!SendSocket)
    {
        return false;
    }

    FIPv4Address ParsedIP;
    if (!FIPv4Address::Parse(PythonIP, ParsedIP))
    {
        UE_LOG(LogTemp, Error, TEXT("[UDP] Invalid Python IP: %s"), *PythonIP);
        return false;
    }

    PythonAddr = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
    PythonAddr->SetIp(ParsedIP.Value);
    PythonAddr->SetPort(PythonStatePort);

    return true;
}

void AUDPControlReceiver::StopUDPSender()
{
    if (SendSocket)
    {
        SendSocket->Close();
        ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(SendSocket);
        SendSocket = nullptr;
    }

    PythonAddr.Reset();
}

void AUDPControlReceiver::ReceiveUDPData()
{
    if (!ListenSocket)
    {
        return;
    }

    uint32 PendingDataSize = 0;

    while (ListenSocket->HasPendingData(PendingDataSize))
    {
        TArray<uint8> ReceivedData;
        ReceivedData.SetNumZeroed(FMath::Min(PendingDataSize, 65507u) + 1);

        int32 BytesRead = 0;
        FInternetAddr& SenderAddr = *ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();

        if (ListenSocket->RecvFrom(ReceivedData.GetData(), ReceivedData.Num() - 1, BytesRead, SenderAddr))
        {
            ReceivedData[BytesRead] = '\0';

            FString Message = FString(UTF8_TO_TCHAR(reinterpret_cast<const char*>(ReceivedData.GetData())));
            Message = Message.Left(BytesRead);

            UE_LOG(LogTemp, Warning, TEXT("[UDP] Received raw: %s"), *Message);

            ParseCommand(Message);
        }
    }
}

void AUDPControlReceiver::ParseCommand(const FString& Message)
{
    TArray<FString> Parts;
    Message.ParseIntoArray(Parts, TEXT(","), true);

    if (Parts.Num() != 4)
    {
        UE_LOG(LogTemp, Error, TEXT("[UDP] Invalid message format: %s"), *Message);
        return;
    }

    Roll = FCString::Atof(*Parts[0]);
    Pitch = FCString::Atof(*Parts[1]);
    Yaw = FCString::Atof(*Parts[2]);
    Throttle = FCString::Atof(*Parts[3]);

    UE_LOG(LogTemp, Warning,
        TEXT("[UDP] Parsed -> Roll: %.3f Pitch: %.3f Yaw: %.3f Throttle: %.3f"),
        Roll, Pitch, Yaw, Throttle);
}

APawn* AUDPControlReceiver::FindTargetPawn()
{
    UWorld* World = GetWorld();
    if (!World)
    {
        return nullptr;
    }

    TArray<AActor*> FoundActors;
    UGameplayStatics::GetAllActorsOfClass(World, APawn::StaticClass(), FoundActors);

    APawn* ExactMatchPawn = nullptr;
    APawn* PartialMatchPawn = nullptr;
    int32 PartialMatchCount = 0;

    for (AActor* Actor : FoundActors)
    {
        if (!Actor)
        {
            continue;
        }

        APawn* Pawn = Cast<APawn>(Actor);
        if (!Pawn)
        {
            continue;
        }

        const FString ActorName = Actor->GetName();
        const bool bExactMatch = !TargetPawnExactName.IsEmpty() && ActorName.Equals(TargetPawnExactName, ESearchCase::CaseSensitive);
        const bool bPartialMatch = !TargetPawnName.IsEmpty() && ActorName.Contains(TargetPawnName);

        if (bLogControlDiagnostics)
        {
            const AController* Controller = Pawn->GetController();
            UE_LOG(LogTemp, Warning,
                TEXT("[UDP] Pawn candidate name=%s class=%s partial_match=%d exact_match=%d possessed=%d player_controlled=%d controller=%s"),
                *ActorName,
                *GetNameSafe(Pawn->GetClass()),
                bPartialMatch ? 1 : 0,
                bExactMatch ? 1 : 0,
                Controller ? 1 : 0,
                Pawn->IsPlayerControlled() ? 1 : 0,
                Controller ? *Controller->GetName() : TEXT("None"));
        }

        if (bExactMatch)
        {
            ExactMatchPawn = Pawn;
        }

        if (bPartialMatch)
        {
            ++PartialMatchCount;
            if (!PartialMatchPawn)
            {
                PartialMatchPawn = Pawn;
            }
        }
    }

    if (ExactMatchPawn)
    {
        UE_LOG(LogTemp, Warning, TEXT("[UDP] Using exact target pawn match: %s"), *ExactMatchPawn->GetName());
        return ExactMatchPawn;
    }

    if (PartialMatchCount > 1)
    {
        UE_LOG(LogTemp, Warning,
            TEXT("[UDP] Multiple pawns matched TargetPawnName=%s count=%d. Using first match: %s"),
            *TargetPawnName,
            PartialMatchCount,
            PartialMatchPawn ? *PartialMatchPawn->GetName() : TEXT("None"));
    }

    if (PartialMatchPawn)
    {
        UE_LOG(LogTemp, Warning, TEXT("[UDP] Using partial target pawn match: %s"), *PartialMatchPawn->GetName());
        return PartialMatchPawn;
    }

    return nullptr;
}

bool AUDPControlReceiver::SetBlueprintNumber(APawn* Pawn, const FName VarName, double Value)
{
    if (!Pawn)
    {
        return false;
    }

    FProperty* Prop = Pawn->GetClass()->FindPropertyByName(VarName);
    if (!Prop)
    {
        UE_LOG(LogTemp, Warning, TEXT("[UDP] Property %s not found on %s"),
            *VarName.ToString(), *Pawn->GetName());
        return false;
    }

    if (FFloatProperty* FloatProp = CastField<FFloatProperty>(Prop))
    {
        FloatProp->SetPropertyValue_InContainer(Pawn, static_cast<float>(Value));
        return true;
    }

    if (FDoubleProperty* DoubleProp = CastField<FDoubleProperty>(Prop))
    {
        DoubleProp->SetPropertyValue_InContainer(Pawn, Value);
        return true;
    }

    UE_LOG(LogTemp, Warning, TEXT("[UDP] Property %s is not float/double on %s"),
        *VarName.ToString(), *Pawn->GetName());
    return false;
}

bool AUDPControlReceiver::GetBlueprintNumber(APawn* Pawn, const FName VarName, double& OutValue) const
{
    if (!Pawn)
    {
        return false;
    }

    FProperty* Prop = Pawn->GetClass()->FindPropertyByName(VarName);
    if (!Prop)
    {
        return false;
    }

    if (const FFloatProperty* FloatProp = CastField<FFloatProperty>(Prop))
    {
        OutValue = static_cast<double>(FloatProp->GetPropertyValue_InContainer(Pawn));
        return true;
    }

    if (const FDoubleProperty* DoubleProp = CastField<FDoubleProperty>(Prop))
    {
        OutValue = DoubleProp->GetPropertyValue_InContainer(Pawn);
        return true;
    }

    return false;
}

UJSBSimMovementComponent* AUDPControlReceiver::FindJSBSimMovementComponent(APawn* Pawn) const
{
    return Pawn ? Pawn->FindComponentByClass<UJSBSimMovementComponent>() : nullptr;
}

void AUDPControlReceiver::LogPawnBindingState(APawn* Pawn, UJSBSimMovementComponent* MovementComponent, const TCHAR* Context) const
{
    if (!Pawn)
    {
        return;
    }

    const AController* Controller = Pawn->GetController();
    UPrimitiveComponent* RootPrimitive = Cast<UPrimitiveComponent>(Pawn->GetRootComponent());

    UE_LOG(LogTemp, Warning,
        TEXT("[UDP][BIND] context=%s actor=%s class=%s exact_target=%s partial_target=%s possessed=%d player_controlled=%d controller=%s root_simulating=%d root_awake=%d physics_state_created=%d jsbsim=%d"),
        Context,
        *Pawn->GetName(),
        *GetNameSafe(Pawn->GetClass()),
        *TargetPawnExactName,
        *TargetPawnName,
        Controller ? 1 : 0,
        Pawn->IsPlayerControlled() ? 1 : 0,
        Controller ? *Controller->GetName() : TEXT("None"),
        RootPrimitive ? (RootPrimitive->IsSimulatingPhysics() ? 1 : 0) : 0,
        RootPrimitive ? (RootPrimitive->IsAnyRigidBodyAwake() ? 1 : 0) : 0,
        RootPrimitive ? (RootPrimitive->HasValidPhysicsState() ? 1 : 0) : 0,
        MovementComponent ? 1 : 0);
}

void AUDPControlReceiver::LogControlApplication(APawn* Pawn, UJSBSimMovementComponent* MovementComponent, const TCHAR* Context) const
{
    if (!Pawn)
    {
        return;
    }

    double BlueprintRoll = 0.0;
    double BlueprintPitch = 0.0;
    double BlueprintYaw = 0.0;
    double BlueprintThrottle = 0.0;
    const bool bHasBlueprintRoll = GetBlueprintNumber(Pawn, TEXT("UDP_Roll"), BlueprintRoll);
    const bool bHasBlueprintPitch = GetBlueprintNumber(Pawn, TEXT("UDP_Pitch"), BlueprintPitch);
    const bool bHasBlueprintYaw = GetBlueprintNumber(Pawn, TEXT("UDP_Yaw"), BlueprintYaw);
    const bool bHasBlueprintThrottle = GetBlueprintNumber(Pawn, TEXT("UDP_Throttle"), BlueprintThrottle);

    UE_LOG(LogTemp, Warning,
        TEXT("[UDP][APPLY] context=%s actor=%s parsed_csv4=(roll=%.3f,pitch=%.3f,yaw=%.3f,throttle=%.3f) bp_vars_found=%d/%d/%d/%d bp_vars=(roll=%.3f,pitch=%.3f,yaw=%.3f,throttle=%.3f) direct_drive=%d"),
        Context,
        *Pawn->GetName(),
        Roll,
        Pitch,
        Yaw,
        Throttle,
        bHasBlueprintRoll ? 1 : 0,
        bHasBlueprintPitch ? 1 : 0,
        bHasBlueprintYaw ? 1 : 0,
        bHasBlueprintThrottle ? 1 : 0,
        BlueprintRoll,
        BlueprintPitch,
        BlueprintYaw,
        BlueprintThrottle,
        bDirectDriveJSBSimCommands ? 1 : 0);

    if (!MovementComponent)
    {
        UE_LOG(LogTemp, Warning, TEXT("[UDP][APPLY] context=%s actor=%s jsbsim_component=missing"), Context, *Pawn->GetName());
        return;
    }

    const bool bHasEngineCommand = MovementComponent->EngineCommands.Num() > 0;
    const bool bHasEngineState = MovementComponent->EngineStates.Num() > 0;
    const FEngineCommand* EngineCommand = bHasEngineCommand ? &MovementComponent->EngineCommands[0] : nullptr;
    const FEngineState* EngineState = bHasEngineState ? &MovementComponent->EngineStates[0] : nullptr;

    FString GearWeightSummary;
    for (int32 GearIndex = 0; GearIndex < MovementComponent->Gears.Num(); ++GearIndex)
    {
        const FGear& Gear = MovementComponent->Gears[GearIndex];
        if (!GearWeightSummary.IsEmpty())
        {
            GearWeightSummary += TEXT("|");
        }
        GearWeightSummary += FString::Printf(TEXT("%d:%d"), GearIndex, Gear.HasWeightOnWheel ? 1 : 0);
    }

    UE_LOG(LogTemp, Warning,
        TEXT("[UDP][JSBSIM] context=%s actor=%s commands=(ail=%.3f,ele=%.3f,rud=%.3f,gear=%.3f,park=%.3f,left=%.3f,right=%.3f,center=%.3f) engine_cmd=(count=%d throttle0=%.3f running0=%d starter0=%d) engine_state=(count=%d running0=%d thrust0=%.3f rpm0=%.3f n1_0=%.3f n2_0=%.3f) setup=(start_engine=%d start_gear=%d start_on_ground=%d) wow=%s"),
        Context,
        *Pawn->GetName(),
        MovementComponent->Commands.Aileron,
        MovementComponent->Commands.Elevator,
        MovementComponent->Commands.Rudder,
        MovementComponent->Commands.GearDown,
        MovementComponent->Commands.ParkingBrake,
        MovementComponent->Commands.LeftBrake,
        MovementComponent->Commands.RightBrake,
        MovementComponent->Commands.CenterBrake,
        MovementComponent->EngineCommands.Num(),
        EngineCommand ? EngineCommand->Throttle : -1.0,
        EngineCommand ? (EngineCommand->Running ? 1 : 0) : 0,
        EngineCommand ? (EngineCommand->Starter ? 1 : 0) : 0,
        MovementComponent->EngineStates.Num(),
        EngineState ? (EngineState->Running ? 1 : 0) : 0,
        EngineState ? EngineState->Thrust : -1.0,
        EngineState ? EngineState->EngineRPM : -1.0,
        EngineState ? EngineState->N1 : -1.0,
        EngineState ? EngineState->N2 : -1.0,
        MovementComponent->bStartWithEngineRunning ? 1 : 0,
        MovementComponent->bStartWithGearDown ? 1 : 0,
        MovementComponent->StartOnGround ? 1 : 0,
        *GearWeightSummary);
}

void AUDPControlReceiver::ApplyDirectJSBSimCommands(UJSBSimMovementComponent* MovementComponent)
{
    if (!MovementComponent)
    {
        return;
    }

    MovementComponent->Commands.Aileron = FMath::Clamp(static_cast<double>(Roll), -1.0, 1.0);
    MovementComponent->Commands.Elevator = FMath::Clamp(static_cast<double>(Pitch), -1.0, 1.0);
    MovementComponent->Commands.Rudder = FMath::Clamp(static_cast<double>(Yaw), -1.0, 1.0);

    const double ClampedThrottle = FMath::Clamp(static_cast<double>(Throttle), 0.0, 1.0);

    if (bAutoReleaseBrakesOnThrottle && ClampedThrottle >= DirectDriveThrottleThreshold)
    {
        MovementComponent->Commands.ParkingBrake = 0.0;
        MovementComponent->Commands.LeftBrake = 0.0;
        MovementComponent->Commands.RightBrake = 0.0;
        MovementComponent->Commands.CenterBrake = 0.0;
    }

    if (MovementComponent->EngineCommands.Num() == 0)
    {
        UE_LOG(LogTemp, Warning,
            TEXT("[UDP][DIRECT] actor=%s no engine commands available; throttle cannot reach JSBSim engine path"),
            *GetNameSafe(MovementComponent->GetOwner()));
        return;
    }

    for (int32 EngineIndex = 0; EngineIndex < MovementComponent->EngineCommands.Num(); ++EngineIndex)
    {
        FEngineCommand& EngineCommand = MovementComponent->EngineCommands[EngineIndex];
        EngineCommand.Throttle = ClampedThrottle;
    }

    UE_LOG(LogTemp, Warning,
        TEXT("[UDP][DIRECT] actor=%s applied_direct_jsbsim csv4=(%.3f,%.3f,%.3f,%.3f) commands=(ail=%.3f,ele=%.3f,rud=%.3f) engine_throttle0=%.3f brakes=(park=%.3f,left=%.3f,right=%.3f,center=%.3f)"),
        *GetNameSafe(MovementComponent->GetOwner()),
        Roll,
        Pitch,
        Yaw,
        Throttle,
        MovementComponent->Commands.Aileron,
        MovementComponent->Commands.Elevator,
        MovementComponent->Commands.Rudder,
        MovementComponent->EngineCommands[0].Throttle,
        MovementComponent->Commands.ParkingBrake,
        MovementComponent->Commands.LeftBrake,
        MovementComponent->Commands.RightBrake,
        MovementComponent->Commands.CenterBrake);
}

bool AUDPControlReceiver::ShouldEmitDiagnostics() const
{
    if (!bLogControlDiagnostics)
    {
        return false;
    }

    const UWorld* World = GetWorld();
    if (!World)
    {
        return true;
    }

    const double Now = World->GetTimeSeconds();
    if (LastDiagnosticsLogTime < 0.0 || ControlDiagnosticsInterval <= 0.0f || (Now - LastDiagnosticsLogTime) >= static_cast<double>(ControlDiagnosticsInterval))
    {
        LastDiagnosticsLogTime = Now;
        return true;
    }

    return false;
}

void AUDPControlReceiver::SendStateToPython()
{
    if (!SendSocket || !PythonAddr.IsValid())
    {
        return;
    }

    if (!CachedTargetPawn || !IsValid(CachedTargetPawn))
    {
        return;
    }

    const FVector Location = CachedTargetPawn->GetActorLocation();
    const FRotator Rotation = CachedTargetPawn->GetActorRotation();

    const double CurrentTime = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0;

    float SpeedCmPerSec = 0.0f;

    if (bHasPrevLocation)
    {
        const double DeltaTime = CurrentTime - PrevStateSendTime;

        if (DeltaTime > KINDA_SMALL_NUMBER)
        {
            const float DistanceCm = FVector::Dist(Location, PrevLocation);
            SpeedCmPerSec = DistanceCm / DeltaTime;
        }
    }

    PrevLocation = Location;
    PrevStateSendTime = CurrentTime;
    bHasPrevLocation = true;

    const float SpeedMps = SpeedCmPerSec / 100.0f;
    const float SpeedKph = SpeedMps * 3.6f;

    const float PitchDeg = Rotation.Pitch;
    const float RollDeg = Rotation.Roll;
    const float YawDeg = Rotation.Yaw;

    const FString JsonMessage = FString::Printf(
        TEXT("{\"aircraft_name\":\"%s\",\"speed\":%.3f,\"speed_mps\":%.3f,\"speed_kph\":%.3f,\"pitch\":%.3f,\"roll\":%.3f,\"yaw\":%.3f,\"x\":%.3f,\"y\":%.3f,\"z\":%.3f}"),
        *CachedTargetPawn->GetName(),
        SpeedCmPerSec,
        SpeedMps,
        SpeedKph,
        PitchDeg,
        RollDeg,
        YawDeg,
        Location.X,
        Location.Y,
        Location.Z
    );

    FTCHARToUTF8 Convert(*JsonMessage);
    int32 BytesSent = 0;

    const bool bSent = SendSocket->SendTo(
        reinterpret_cast<const uint8*>(Convert.Get()),
        Convert.Length(),
        BytesSent,
        *PythonAddr
    );

    if (bSent)
    {
        UE_LOG(LogTemp, Warning, TEXT("[UDP->PY] Sent state: %s"), *JsonMessage);
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("[UDP->PY] Failed to send state"));
    }
}
