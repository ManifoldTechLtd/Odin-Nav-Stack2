#ifndef __ASTRALL_SDK_INTERFACE_H__
#define __ASTRALL_SDK_INTERFACE_H__

#include <stdint.h>
#include <string>
#include <cstring>
#include <iostream>
#include <functional>

#define ASTRALL_RES_FAILED          0x8004   // Execution / request / subscription failed
#define ASTRALL_RES_TIMEOUT         0x8005   // Timeout
#define ASTRALL_RES_RUNNING         0x8006   // Running
#define ASTRALL_RES_SUCCESSED       0x8008   // Succeeded
#define ASTRALL_RES_INVALID_PARAM   0x8010   // Invalid parameter
#define ASTRALL_RES_NOT_INIT        0x8011   // Not initialized
#define ASTRALL_RES_RC_NORELEASE    0x8020   // Remote controller seized and has not released control authority
#define ASTRALL_RES_BEEN_OBTAINED   0x8021   // Another client already holds control authority
#define ASTRALL_RES_WITHOUT_AUTH    0x8022   // No control authority

enum AstrallSystemCode : uint8_t
{
    ASTRALL_SYSTEM_CODE_INIT     = 0,   // Initialization and self-test
    ASTRALL_SYSTEM_CODE_STANDBY  = 1,   // Standby -> damping mode
    ASTRALL_SYSTEM_CODE_RUNNING  = 2,   // Normal operation
    ASTRALL_SYSTEM_CODE_WARNING  = 3,   // Warning -> requires attention
    ASTRALL_SYSTEM_CODE_ERROR    = 4,   // Error -> emergency stop
    ASTRALL_SYSTEM_CODE_SHUTDOWN = 5    // Shutdown
};

enum AstrallErrorCode : uint32_t
{
    ASTRALL_ERR_NONE                = 0,
    ASTRALL_ERR_MOTOR_VERSION       = (1U << 0),    // Motor driver version mismatch
    ASTRALL_ERR_MOTOR_COMM_LOSE     = (1U << 1),    // Motor communication lost
    ASTRALL_ERR_MOTOR_STATUS        = (1U << 2),    // Motor fault
    ASTRALL_ERR_MOTOR_OVER_TEMP     = (1U << 3),    // Motor over-temperature
    ASTRALL_ERR_STO_TRIGGERED       = (1U << 4),    // STO triggered
    ASTRALL_ERR_IMU_ERROR           = (1U << 5),    // IMU error
    ASTRALL_ERR_LOW_BATTERY_LEVEL   = (1U << 6),    // Battery level critically low
    ASTRALL_ERR_OVER_LOAD           = (1U << 7),    // Motor overload
    ASTRALL_ERR_MBUS_DRIVER_FAIL    = (1u << 8),    // Driver version error
    ASTRALL_ERR_LOSE_CAMERA         = (1u << 9),    // Camera device not found
};

enum AstrallWarnCode : uint32_t
{
    ASTRALL_WARN_NONE               = 0,
    ASTRALL_WARN_MOTOR_OVER_TEMP    = (1U << 0),    // Motor approaching over-temperature
    ASTRALL_WARN_JOYSTICK_LOCK      = (1U << 1),    // Remote controller locked
    ASTRALL_INFO_JOYSTICK_INTERVENTION = (1U << 2), // Remote controller acquired control authority
    ASTRALL_WARN_MOTOR_COMM_UNSTABLE = (1U << 3),   // Motor communication unstable
    ASTRALL_WARN_LOW_BATTERY_LEVEL  = (1U << 4),    // Low battery warning
    ASTRALL_WARN_JOYSTICK_LOSE      = (1U << 5),    // Remote controller offline
    ASTRALL_WARN_MOTOR_OVER_LOAD    = (1U << 6),    // Motor approaching overload
    ASTRALL_WARNING_CONNECT_SDK_LOSE = (1u << 7),   // Previously connected SDK client offline
};

enum AstrallSportModeCmd : uint16_t
{
    ASTRALL_SPORT_MODE_CMD_DAMPING       = 0xA101,       // Damping (soft emergency stop)
    ASTRALL_SPORT_MODE_CMD_FIXEDSTAND    = 0xA102,       // Fixed stand
    ASTRALL_SPORT_MODE_CMD_FIXEDDOWN     = 0xA103,       // Fixed lie down
    ASTRALL_SPORT_MODE_CMD_MOVE          = 0xA104,       // Walk (RL all-terrain gait)
    ASTRALL_SPORT_MODE_CMD_AUTOCHARGE    = 0xA105,       // Autonomous docking / charging
    ASTRALL_SPORT_MODE_CMD_EXITCHARGE    = 0xA106,       // Exit charging dock
    ASTRALL_SPORT_MODE_CMD_GET_RIGHT     = 0xA1FF        // Fall recovery to stand
};

enum AstrallSportStatus : uint16_t
{
    ASTRALL_SPORT_STATUS_UNKNOWN        = 0x0000,       // Unknown status
    ASTRALL_SPORT_STATUS_INIT           = 0xB100,       // Initializing
    ASTRALL_SPORT_STATUS_DAMPING        = 0xB101,       // Damping state
    ASTRALL_SPORT_STATUS_FIXEDSTAND     = 0xB102,       // Fixed stand state
    ASTRALL_SPORT_STATUS_FIXEDDOWN      = 0xB103,       // Fixed lie-down state
    ASTRALL_SPORT_STATUS_MOVE           = 0xB104,       // Walking (RL all-terrain gait)
    ASTRALL_SPORT_STATUS_UPSTAIRS       = 0xB105,       // Climbing stairs
    ASTRALL_SPORT_STATUS_CAR            = 0xB106,       // Wheeled carrier mode
    ASTRALL_SPORT_STATUS_AUTOCHAEGE     = 0xB107,       // Preparing for auto-charging
    ASTRALL_SPORT_STATUS_SEARCHCHARGING = 0xB108,       // Searching for charging dock
    ASTRALL_SPORT_STATUS_PLUGGINGCHARGING = 0xB109,     // Docking onto charger
    ASTRALL_SPORT_STATUS_ATCHARGING     = 0xB10A,       // On charging dock
    ASTRALL_SPORT_STATUS_EXITCHARGING   = 0xB10B,       // Exiting charging dock
    ASTRALL_SPORT_STATUS_CHARGING       = 0xB10C,       // Charging
    ASTRALL_SPORT_STATUS_FIXEDSTANDING  = 0xB10D,       // Transitioning to stand
    ASTRALL_SPORT_STATUS_FIXEDDOWNING   = 0xB10E,       // Transitioning to lie down
    ASTRALL_SPORT_STATUS_GET_RIGHT      = 0xB1FF        // Recovering from fall
};

enum AstrallAuth : uint16_t
{
    ASTRALL_AUTH_SDK                = 1U,           // SDK control
    ASTRALL_AUTH_JOYSTICK           = 2U            // Remote controller control
};

enum AstrallLightCmd : uint16_t
{
    ASTRALL_CMD_LIGHT_OPEN          = 1U,           // Turn light on
    ASTRALL_CMD_LIGHT_CLOSE         = 2U            // Turn light off
};

enum AstrallSubscribeTopicId : uint16_t
{
    ASTRALL_SUB_TOPIC_ID_IMU        = 0x0001U,    // IMU data
    ASTRALL_SUB_TOPIC_ID_SPORT      = 0x0002U,    // Sport state data
    ASTRALL_SUB_TOPIC_ID_CAMERA_RGB = 0x0003U,    // Camera RGB data
    ASTRALL_SUB_TOPIC_ID_LIDAR      = 0x0005U,    // LiDAR data
    ASTRALL_SUB_TOPIC_ID_JOYSTICK   = 0x0010U,    // Joystick data
};

enum AstrallSubscribeFreq : uint16_t
{
    ASTRALL_SUB_FREQ_CLOSE          = 0U,
    ASTRALL_SUB_FREQ_1HZ,
    ASTRALL_SUB_FREQ_25HZ,
    ASTRALL_SUB_FREQ_50HZ,
    ASTRALL_SUB_FREQ_125HZ,
    ASTRALL_SUB_FREQ_250HZ
};

struct AstrallConfig 
{
    // Heartbeat callback (data: NULL)
    std::function<void(void *data, uint16_t len)> heartbeatCb;
    // SDK status callback (data: AstrallSdkStatus*)
    std::function<void(void *data, uint16_t len)> sdkStatusCb;
};

struct AstrallSdkStatus {
    uint8_t link : 1;          // Connection status
    uint8_t ctrlAuthority : 1; // Control authority status
    uint8_t reserve : 6;       // Reserved

    AstrallSdkStatus()
    {
        link = 0;
        ctrlAuthority = 0;
        reserve  = 0;
    }
};

struct AstrallDeviceInfo
{
    std::string version;          // Firmware version
    std::string serialNumber;     // Serial number
    uint32_t model;               // Device model
    
    AstrallDeviceInfo()
    {
        version = "";
        serialNumber = "";
	    model = 0;
    }
};

struct AstrallSystemStatus
{
    AstrallSystemCode sysStatus;       // System status code
    AstrallErrorCode errorCode;        // Error bitmask
    AstrallWarnCode warnCode;          // Warning bitmask

    AstrallSystemStatus()
    {
        sysStatus = AstrallSystemCode::ASTRALL_SYSTEM_CODE_INIT;
        errorCode = AstrallErrorCode::ASTRALL_ERR_NONE;
        warnCode = AstrallWarnCode::ASTRALL_WARN_NONE;
    }
};

struct AstrallPowerStatus
{
    float soc;                  // State of charge (%)
    float temp;                 // Battery temperature
    float voltage;              // Battery voltage
    uint16_t cycleCount;        // Charge cycle count
    uint16_t charged;           // Charging state
    
    AstrallPowerStatus()
    {
        soc = 0.0f;
        temp = 0.0f;
        voltage = 0.0f;
        cycleCount = 0;
        charged = 0;
    }
};

struct AstrallImuData
{
    long long timestamp;        // IMU timestamp
    float accelerometer[3];     // Linear acceleration (m/s^2)
    float gyroscope[3];         // Angular velocity (rad/s)
    float quaternion[4];        // Orientation quaternion
    float pitch;                // Pitch (rad)
    float roll;                 // Roll (rad)
    float yaw;                  // Yaw (rad)
    float odomX;                // Odometry X
    float odomY;                // Odometry Y

    AstrallImuData()
    {
        timestamp = 0;
        memset(accelerometer, 0, sizeof(accelerometer));
        memset(gyroscope, 0, sizeof(gyroscope));
        memset(quaternion, 0, sizeof(quaternion));
        pitch = 0.0f;
        yaw = 0.0f;
        roll = 0.0f;
        odomX = 0.0f;
        odomY = 0.0f;
    }
};

struct AstrallSportData
{
    long long timestamp;
    float wheelSpeed[4];

    AstrallSportData()
    {
        timestamp = 0;
        memset(wheelSpeed, 0, sizeof(wheelSpeed));
    }
};

struct AstrallJoystickData
{
    long long timestamp;
    int16_t rocker[6];
    uint16_t l1 : 1;
    uint16_t l2 : 1;
    uint16_t l3 : 1;
    uint16_t r1 : 1;
    uint16_t r2 : 1;
    uint16_t r3 : 1;
    uint16_t sl : 1;
    uint16_t sr : 1;
    uint16_t h : 1;
    uint16_t sw : 2;
    uint16_t reserve : 5;
};

uint16_t AstrallSdkInit(AstrallConfig& cfg, uint32_t timeout = 60000);
void AstrallSdkDeinit();
uint16_t AstrallHeartbeat(uint32_t timeout = 20);
uint16_t AstrallGetSystemStatus(AstrallSystemStatus& status);
uint16_t AstrallGetPowerStatus(AstrallPowerStatus& power);
uint16_t AstrallGetSportStatus();
uint16_t AstrallGetDeviceInfo(AstrallDeviceInfo& info, uint32_t timeout = 20);
uint16_t AstrallGetImuData(AstrallImuData& data);
uint16_t AstrallSubscriptionData(AstrallSubscribeTopicId topicId,
                                 AstrallSubscribeFreq freq,
                                 std::function<void(void *data, uint16_t len)> cb, 
                                 uint32_t timeout = 20);
uint16_t AstrallSportModeControl(AstrallSportModeCmd mode, uint32_t timeout = 20);
uint16_t AstrallAuthControl(AstrallAuth auth, uint32_t timeout = 20);
uint16_t AstrallMove(float vx, float vy, float vyaw, uint32_t timeout = 20);
uint16_t AstrallLightControl(AstrallLightCmd cmd, uint32_t timeout = 20);
uint16_t AstrallSendMessage(char *data, uint16_t len);

#endif /* __ASTRALL_SDK_INTERFACE_H__ */

