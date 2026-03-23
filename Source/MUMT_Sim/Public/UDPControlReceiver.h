#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Sockets.h"
#include "SocketSubsystem.h"
#include "IPAddress.h"
#include "UDPControlReceiver.generated.h"

class UJSBSimMovementComponent;

UCLASS()
class MUMT_SIM_API AUDPControlReceiver : public AActor
{
    GENERATED_BODY()

public:
    AUDPControlReceiver();

protected:
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaTime) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
    // ===== 수신 =====
    bool StartUDPReceiver();
    void StopUDPReceiver();
    void ReceiveUDPData();
    void ParseCommand(const FString& Message);

    // ===== 송신 =====
    bool StartUDPSender();
    void StopUDPSender();
    void SendStateToPython();

    // ===== Pawn 처리 =====
    APawn* FindTargetPawn();
    bool SetBlueprintNumber(APawn* Pawn, const FName VarName, double Value);
    bool GetBlueprintNumber(APawn* Pawn, const FName VarName, double& OutValue) const;
    UJSBSimMovementComponent* FindJSBSimMovementComponent(APawn* Pawn) const;
    void LogPawnBindingState(APawn* Pawn, UJSBSimMovementComponent* MovementComponent, const TCHAR* Context) const;
    void LogControlApplication(APawn* Pawn, UJSBSimMovementComponent* MovementComponent, const TCHAR* Context) const;
    void ApplyDirectJSBSimCommands(UJSBSimMovementComponent* MovementComponent);
    bool ShouldEmitDiagnostics() const;

private:
    // 수신용 소켓
    FSocket* ListenSocket = nullptr;

    // 송신용 소켓
    FSocket* SendSocket = nullptr;

    // Python 주소
    TSharedPtr<FInternetAddr> PythonAddr;

    // 캐시된 대상 Pawn
    APawn* CachedTargetPawn = nullptr;

    // 상태 송신 타이머
    float StateSendAccumulator = 0.0f;

    // 속도 계산용 이전 상태
    FVector PrevLocation = FVector::ZeroVector;
    bool bHasPrevLocation = false;
    double PrevStateSendTime = 0.0;
    mutable double LastDiagnosticsLogTime = -1.0;

public:
    // ===== 수신 설정 =====
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|Receiver")
    int32 ListenPort = 5005;

    // ===== 송신 설정 =====
    // Disable the legacy single-aircraft telemetry sender when a separate
    // multi-aircraft state bridge is active, so UDP 5006 is not polluted.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|Sender")
    bool bEnableStateSender = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|Sender")
    FString PythonIP = TEXT("127.0.0.1");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|Sender")
    int32 PythonStatePort = 5007;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|Sender")
    float StateSendInterval = 0.05f; // 20Hz

    // ===== 제어 대상 Pawn =====
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|Target")
    FString TargetPawnName = TEXT("F16_UAV");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|Target")
    FString TargetPawnExactName = TEXT("");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|Debug")
    bool bLogControlDiagnostics = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|Debug", meta = (ClampMin = "0.0"))
    float ControlDiagnosticsInterval = 0.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|Debug")
    bool bDirectDriveJSBSimCommands = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|Debug")
    bool bAutoReleaseBrakesOnThrottle = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UDP|Debug", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float DirectDriveThrottleThreshold = 0.05f;

    // ===== 수신된 조종값 =====
    UPROPERTY(BlueprintReadOnly, Category = "UDP|Control")
    float Roll = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "UDP|Control")
    float Pitch = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "UDP|Control")
    float Yaw = 0.0f;

    UPROPERTY(BlueprintReadOnly, Category = "UDP|Control")
    float Throttle = 0.0f;
};
