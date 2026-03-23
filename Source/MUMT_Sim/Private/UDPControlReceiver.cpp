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

AUDPControlReceiver::AUDPControlReceiver()
{
    PrimaryActorTick.bCanEverTick = true;
}

void AUDPControlReceiver::BeginPlay()
{
    Super::BeginPlay();

    if (StartUDPReceiver())
    {
        UE_LOG(LogTemp, Warning, TEXT("[UDP] Receiver started on port %d"), ListenPort);
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("[UDP] Failed to start receiver"));
    }

    if (bEnableStateSender)
    {
        if (StartUDPSender())
        {
            UE_LOG(LogTemp, Warning, TEXT("[UDP] Sender started -> %s:%d"), *PythonIP, PythonStatePort);
        }
        else
        {
            UE_LOG(LogTemp, Error, TEXT("[UDP] Failed to start sender"));
        }
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("[UDP] Legacy state sender disabled; receiver-only mode active on %s"), *GetName());
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

    for (AActor* Actor : FoundActors)
    {
        if (!Actor)
        {
            continue;
        }

        if (Actor->GetName().Contains(TargetPawnName))
        {
            UE_LOG(LogTemp, Warning, TEXT("[UDP] Found target pawn: %s"), *Actor->GetName());
            return Cast<APawn>(Actor);
        }
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
