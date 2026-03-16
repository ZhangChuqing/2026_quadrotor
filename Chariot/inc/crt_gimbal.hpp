/**
 ******************************************************************************
 * @file           : crt_gimbal.hpp
 * @brief          : header file for crt_gimbal.cpp
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

/* Includes ------------------------------------------------------------------*/
#include "GSRL.hpp"
#include "para_gimbal.hpp"
#include "drv_ws2812.hpp"
#include "dvc_vofa.hpp"

/* Exported types ------------------------------------------------------------*/

class Gimbal
{
public:
    using Vector3f  = GSRLMath::Vector3f;
    using Matrix33f = GSRLMath::Matrix33f;
    // 云台模式
    enum GimbalMode : uint8_t {
        GIMBAL_NO_FORCE = 0,
        CALIBRATION,
        MANUAL_CONTROL,
        AUTO_CONTROL
    };
    // 底盘模式
    enum ChassisMode : uint8_t {
        CHASSIS_NO_FORCE = 0,
        NO_FOLLOW,
        FOLLOW_GIMBAL,
        SPINNING
    };

private:
    // 电机
    MotorGM6020 *m_yawMotor;
    MotorDM4310 *m_pitchMotor;
    MotorM2006 *m_rammerMotor;
    MotorM3508 *m_frictionLeftMotor;
    MotorM3508 *m_frictionRightMotor;

    // IMU
    IMU *m_imu;
    Vector3f m_eulerAngle;

    // 云台控制相关量
    GimbalMode m_gimbalMode;
    fp32 m_yawTargetAngle;
    fp32 m_pitchTargetAngle;

    // 底盘控制相关量
    ChassisMode m_chassisMode;
    Vector3f m_gimbalTargetSpeed;  // 云台坐标系下的目标速度
    Vector3f m_chassisTargetSpeed; // 底盘坐标系下的目标速度

   // 发射机构相关量
    bool m_rammerState;   // false: 停止 true: 发射
    bool m_frictionState; // false: 停止 true: 启动
    bool m_singleShotState; // 单发状态
    fp32 m_singleShotTargetRevolutions; // 单发目标转数

    //bool m_frictionState  = false; // false: 停止 true: 启动
    //bool m_singleShotReq  = false; // 本周期产生一次单发请求（脉冲）
    //bool m_contFireEnable = false; // 连发使能（按住时为true）
    //bool m_feederArmed    = false; // 允许拨弹（摩擦轮开且热量允许等）

    // 单发/连发判定计时
    //uint32_t m_downHoldMs      = 0; // 左三档保持DOWN计时（ms）
    //bool m_downLatched         = false;
    //uint32_t m_contFireTimerMs = 0; // 连发节拍计时（ms）

    // 拨弹目标多圈（用于 revolutionsClosedloopControl）
    //fp32 m_feederTargetRev = 0.0f;

    // Shoot 状态机
    //enum ShootState : uint8_t {
        //stateIdle = 0,
        //stateFeeding,
        //stateUnjamming
    //};
    //ShootState m_shootState = stateIdle;

    // 卡弹/解卡计数
    //uint32_t m_jamCounter   = 0;
    //uint32_t m_unjamCounter = 0;

    // 遥控器
    Dr16RemoteControl m_remoteControl;

    // LED Strip
    WS2812 m_ws2812;

    // 标志位
    bool m_isInitComplete;

    // 下C板上发裁判系统数据
    uint8_t m_gameProgress;
    uint16_t m_leftShooterHeat;
    // uint16_t m_rightShooterHeat;
    uint16_t m_currentHP;

public:
    Gimbal(MotorGM6020 *yawMotor, MotorDM4310 *pitchMotor, MotorM2006 *rammerMotor, MotorM3508 *frictionLeftMotor, MotorM3508 *frictionRightMotor, IMU *imu);
    void init();
    void controlLoop();
    void imuLoop();
    void receiveGimbalMotorDataFromISR(const can_rx_message_t *rxMessage);
    void receiveRemoteControlDataFromISR(const uint8_t *rxData);

private:
    void modeSelect();
    void targetOrientationPlan();
    void targetSpeedPlan();
    void shootPlan();
    void pitchControl();
    void yawControl();
    void shootControl();
    void ledControl();
    void rammerStuckControl();
    void transmitGimbalMotorData();

    inline void setPitchAngle(const fp32 &targetAngle);
    inline void setYawAngle(const fp32 &targetAngle);
    inline void convertGimbalTargetSpeedToChassisTargetSpeed();
    inline fp32 rcStickDeadZoneFilter(const fp32 &rcStickValue);
    inline fp32 gravityCompensate(fp32 baseTorque, fp32 currentAngle, fp32 compensateCoeff);
};

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/* Defines -------------------------------------------------------------------*/
