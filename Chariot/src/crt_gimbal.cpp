/**
 ******************************************************************************
 * @file           : crt_gimbal.cpp
 * @brief          : 云台控制
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "crt_gimbal.hpp"
#include "para_gimbal.hpp"
#include "tsk_isr.hpp"
#include "drv_misc.h"
#include <math.h>
#include "tim.h" /* For htim1 */

/* Typedef -------------------------------------------------------------------*/
enum LedColor {
    LED_RED,
    LED_BLUE,
};
static LedColor currentLedColor = LED_RED;
static bool isLedChanged = true;

/* Define --------------------------------------------------------------------*/

/* Macro ---------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/

/******************************************************************************
 *                            Gimbal类实现
 ******************************************************************************/

Gimbal::Gimbal(MotorGM6020 *yawMotor, MotorDM4310 *pitchMotor, MotorM2006 *rammerMotor, MotorM3508 *frictionLeftMotor, MotorM3508 *frictionRightMotor, IMU *imu)
    : m_yawMotor(yawMotor), m_pitchMotor(pitchMotor),
      m_rammerMotor(rammerMotor), m_frictionLeftMotor(frictionLeftMotor), m_frictionRightMotor(frictionRightMotor),
      m_imu(imu),
      m_gimbalMode(GIMBAL_NO_FORCE),
      m_yawTargetAngle(0.0f), m_pitchTargetAngle(0.0f),
      m_rammerState(false), 
      m_frictionState(false),
      m_remoteControl(),
      m_ws2812(&htim1, TIM_CHANNEL_1),
      m_isInitComplete(false) {}

void Gimbal::init()
{
    DWT_Init();
    CAN_Init(&hcan1, can1RxCallback);
    CAN_Init(&hcan2, can2RxCallback);
    UART_Init(&huart3, dr16RxCallback, 36);
    m_imu->init();
    m_ws2812.Init();

    for(int i=0; i<WS2812_LED_NUM; i++) {
        m_ws2812.SetColor(i, 120, 0, 0);
    }
    m_ws2812.Update();

    m_isInitComplete = true;
}

void Gimbal::controlLoop()
{
    if (!m_isInitComplete) return;
    modeSelect();
    targetOrientationPlan();
    shootPlan();
    pitchControl();
    yawControl();
    shootControl();
    ledControl();
    transmitGimbalMotorData();
}

void Gimbal::imuLoop()
{
    if (!m_isInitComplete) return;
    m_eulerAngle = m_imu->solveAttitude();
}

void Gimbal::receiveGimbalMotorDataFromISR(const can_rx_message_t *rxMessage)
{
    if (m_yawMotor->decodeCanRxMessageFromISR(rxMessage)) return;
    if (m_pitchMotor->decodeCanRxMessageFromISR(rxMessage)) return;
    if (m_rammerMotor->decodeCanRxMessageFromISR(rxMessage)) return;
    if (m_frictionLeftMotor->decodeCanRxMessageFromISR(rxMessage)) return;
    if (m_frictionRightMotor->decodeCanRxMessageFromISR(rxMessage)) return;
}

void Gimbal::receiveRemoteControlDataFromISR(const uint8_t *rxData)
{
    m_remoteControl.receiveRxDataFromISR(rxData);
}

void Gimbal::modeSelect()
{
    m_remoteControl.updateEvent();
    if (!m_remoteControl.isConnected()) {
        m_gimbalMode  = GIMBAL_NO_FORCE;
        return;
    }

    switch (m_remoteControl.getRightSwitchStatus()) {
        case Dr16RemoteControl::SwitchStatus3Pos::SWITCH_DOWN:
            m_gimbalMode  = GIMBAL_NO_FORCE;
            if (m_remoteControl.getLeftSwitchEvent() == Dr16RemoteControl::SwitchEvent3Pos::SWITCH_TOGGLE_MIDDLE_UP) {
                m_gimbalMode = CALIBRATION;
            }
            break;

        case Dr16RemoteControl::SwitchStatus3Pos::SWITCH_MIDDLE:
            m_gimbalMode  = MANUAL_CONTROL;
            break;

        case Dr16RemoteControl::SwitchStatus3Pos::SWITCH_UP:
            m_gimbalMode  = AUTO_CONTROL;
            break;

        default:
            break;
    }
}

void Gimbal::targetOrientationPlan()
{
    switch (m_gimbalMode) {
        case MANUAL_CONTROL:
            setYawAngle(m_yawTargetAngle - rcStickDeadZoneFilter(m_remoteControl.getRightStickX()) * DT7_STICK_YAW_SENSITIVITY*0.6);
            // Pitch锁角：不再跟随右摇杆Y动态变化，保持当前目标角。
            break;

        case AUTO_CONTROL:
            break;

        default:
            break;
    }
}

void Gimbal::shootPlan()
{
    switch (m_gimbalMode) {
        case MANUAL_CONTROL:
            if (m_remoteControl.getLeftSwitchEvent() == Dr16RemoteControl::SwitchEvent3Pos::SWITCH_TOGGLE_MIDDLE_UP) {
                m_frictionState = !m_frictionState;
            }

            //连发
            if ((m_remoteControl.getLeftSwitchStatus() == Dr16RemoteControl::SwitchStatus3Pos::SWITCH_DOWN) && m_frictionState && (m_leftShooterHeat < 350)) {
                m_rammerState = true;
            } else {
                m_rammerState = false;
            }
            break;

        case AUTO_CONTROL:
            break;

        default:
            break;
    }
}

/*void Gimbal::shootPlan()
{
    if (m_gimbalMode != MANUAL_CONTROL) return;

    // 摩擦轮开关保持不变
    if (m_remoteControl.getLeftSwitchEvent() == Dr16RemoteControl::SwitchEvent3Pos::SWITCH_TOGGLE_MIDDLE_UP) {
        m_frictionState = !m_frictionState;
    }

    // 允许拨弹条件
    m_feederArmed = m_frictionState; //&& (m_leftShooterHeat < 350);
    //if (!m_feederArmed) {
        //m_contFireEnable  = false;
        //m_downHoldMs      = 0;
        //m_downLatched     = false;
        //m_contFireTimerMs = 0;0
        //return;
    //}

    // 默认清脉冲
    m_singleShotReq = false;
    
    // 获取当前滚轮值 (-1.0 ~ 1.0)
    static float lastScrollWheel = 0.0f;
    float currentScrollWheel = m_remoteControl.getScrollWheel();

    // 检测滚轮变化量，超过阈值判定为拨动
    // 阈值设为0.15，避免静止时的信号抖动误触
    if (fabsf(currentScrollWheel - lastScrollWheel) > 0.15f) {
        if (m_feederArmed) {
            m_singleShotReq = true;
        }
        // 更新历史值为当前值，准备下一次检测
        lastScrollWheel = currentScrollWheel; 
    }

        // 左拨杆打到下档 -> 开启连发
    if (m_remoteControl.getLeftSwitchStatus() == Dr16RemoteControl::SwitchStatus3Pos::SWITCH_DOWN) {
        if (m_feederArmed) {
            m_contFireEnable = true;
        } else {
             m_contFireEnable = false;
        }
    } else {
        m_contFireEnable = false;
    }

    // 清除旧逻辑相关的状态变量，防止干扰
    m_downHoldMs = 0;
    m_downLatched = false;
}*/

void Gimbal::pitchControl()
{
    switch (m_gimbalMode) {
        case GIMBAL_NO_FORCE:
            m_pitchMotor->openloopControl(0.0f);
            break;

        case CALIBRATION:
            break;

        case MANUAL_CONTROL:
        case AUTO_CONTROL: { // 手动控制和自动控制都使用同样的闭环控制
            // fp32 fdbData[2] = {GSRLMath::normalizeDeltaAngle(m_pitchTargetAngle - m_eulerAngle.y), -m_imu->getGyro().y};
            fp32 fdbData[2] = {m_pitchTargetAngle - m_eulerAngle.y, -m_imu->getGyro().y};
            fp32 pidOutput  = m_pitchMotor->externalClosedloopControl(0.0f, fdbData, 2);
#ifdef PITCH_GRAVITY_COMPENSATE
            fp32 totalTorque = gravityCompensate(pidOutput, m_pitchMotor->getCurrentAngle(), PITCH_GRAVITY_COMPENSATE);
            m_pitchMotor->openloopControl(totalTorque);
#else
            (void)pidOutput; // 防止未使用变量警告
#endif
            break;
        }

        default:
            break;
    }
}

void Gimbal::yawControl()
{
    switch (m_gimbalMode) {
        case GIMBAL_NO_FORCE:
            m_yawTargetAngle = m_eulerAngle.z;
            m_yawMotor->openloopControl(0.0f);
            break;

        case CALIBRATION:
            break;

        case MANUAL_CONTROL:
        case AUTO_CONTROL: { // 手动控制和自动控制都使用同样的闭环控制
            fp32 fdbData[2] = {m_yawTargetAngle - m_eulerAngle.z, m_imu->getGyro().z};
            m_yawMotor->externalClosedloopControl(0.0f, fdbData, 2);
            break;
        }

        default:
            break;
    }
}

void Gimbal::shootControl()
{
    if (m_gimbalMode == GIMBAL_NO_FORCE) {
        m_rammerState   = false;
        m_frictionState = false;
        m_frictionRightMotor->openloopControl(0.0f);
        m_frictionLeftMotor->openloopControl(0.0f);
        m_rammerMotor->openloopControl(0.0f);
        return;
    }

    if (m_frictionState) {
        m_frictionLeftMotor->angularVelocityClosedloopControl(-FRICTION_TARGET_ANGULAR_VELOCITY);
        m_frictionRightMotor->angularVelocityClosedloopControl(FRICTION_TARGET_ANGULAR_VELOCITY);
    } else { 
        m_frictionLeftMotor->angularVelocityClosedloopControl(0.0f);
        m_frictionRightMotor->angularVelocityClosedloopControl(0.0f);
    }

    if (m_rammerState) {
        m_rammerMotor->angularVelocityClosedloopControl(RAMMER_TARGET_ANGULAR_VELOCITY);
        rammerStuckControl();
    }
    else {
        m_rammerMotor->angularVelocityClosedloopControl(0.0f);      
    }
}

void Gimbal::rammerStuckControl()
{
    static uint8_t rammerStuckState          = 0; // 0: 正常 1: 疑似卡弹 2: 证实卡弹
    volatile static uint32_t rammerStuckTime = 0;
    switch (rammerStuckState) {
        case 0: // 正常
            if (abs(m_rammerMotor->getCurrentAngularVelocity()) < 1.0f) {
                rammerStuckTime  = DWT->CYCCNT; // 获取当前时间戳
                rammerStuckState = 1;
            }
            break;

        case 1: // 疑似卡弹
            if (abs(m_rammerMotor->getCurrentAngularVelocity()) > 1.0f) {
                rammerStuckState = 0; // 解除卡弹状态
            } else if (((uint32_t)(DWT->CYCCNT - rammerStuckTime)) / ((fp32)(SystemCoreClock)) > RAMMER_STUCK_TIMEOUT) {
                rammerStuckTime  = DWT->CYCCNT; // 获取当前时间戳
                rammerStuckState = 2;
            }
            break;

        case 2: // 证实卡弹
            m_rammerMotor->angularVelocityClosedloopControl(RAMMER_STUCK_REVERT_ANGULAR_VELOCITY);
            if (((uint32_t)(DWT->CYCCNT - rammerStuckTime)) / ((fp32)(SystemCoreClock)) > RAMMER_REVERT_TIME) {
                rammerStuckState = 0; // 解除卡弹状态
            }
            break;

        default:
            break;
    }
}

/*void Gimbal::shootControl()
{
    if (m_gimbalMode == GIMBAL_NO_FORCE) {
        m_frictionState  = false;
        m_singleShotReq  = false;
        m_contFireEnable = false;
        m_feederArmed    = false;

        m_shootState      = stateIdle;
        m_feederTargetRev = 0.0f;
        m_contFireTimerMs = 0;

        m_jamCounter   = 0;
        m_unjamCounter = 0;

        m_frictionRightMotor->openloopControl(0.0f);
        m_frictionLeftMotor->openloopControl(0.0f);
        m_rammerMotor->openloopControl(0.0f);
        return;
    }else{
        if (m_frictionState) {
            m_frictionLeftMotor->angularVelocityClosedloopControl(-FRICTION_TARGET_ANGULAR_VELOCITY);
            m_frictionRightMotor->angularVelocityClosedloopControl(FRICTION_TARGET_ANGULAR_VELOCITY);
        } else {
            m_frictionLeftMotor->angularVelocityClosedloopControl(0.0f);
            m_frictionRightMotor->angularVelocityClosedloopControl(0.0f);
        }

        bool singleShotTrigger = false;

        // 短按单发：来自 shootPlan() 的脉冲
        if (m_singleShotReq) {
            singleShotTrigger = true;
            m_singleShotReq   = false;
        }

        // 连发：按节拍周期触发“单发一次”
        if (m_contFireEnable && m_feederArmed) {
            m_contFireTimerMs += 1; // 假设 controlLoop = 1ms
            if (m_contFireTimerMs >= CONT_FIRE_PERIOD_MS) {
                m_contFireTimerMs = 0;
                singleShotTrigger = true;
            }
        } else {
            m_contFireTimerMs = 0;
        }

        switch (m_shootState) {

            case stateIdle: {
                m_jamCounter = 0;

                // 未允许拨弹 or 摩擦轮未开：拨弹停
                if (!m_feederArmed || !m_frictionState) {
                    m_rammerMotor->openloopControl(0.0f);
                    m_feederTargetRev = m_rammerMotor->getCurrentRevolutions();
                    break;
                }

                // 单发触发：目标圈数 + 1/8圈（方向反了就改成 -=）
                if (singleShotTrigger) {
                    m_feederTargetRev += (+FEED_STEP_REV);
                    m_shootState = stateFeeding;
                } else {
                    m_rammerMotor->openloopControl(0.0f);
                }
            } break;

            case stateFeeding: {
                m_rammerMotor->revolutionsClosedloopControl(m_feederTargetRev);

                const fp32 curRev   = m_rammerMotor->getCurrentRevolutions();
                const fp32 curSpd   = m_rammerMotor->getCurrentAngularVelocity();
                const fp32 revError = m_feederTargetRev - curRev;
                // 到位判定
                if (fabsf(revError) < FEED_REV_EPS && fabsf(curSpd) < FEED_SPEED_EPS) {
                    m_shootState = stateIdle;
                    m_jamCounter = 0;
                    break;
                }

                // 卡弹检测：独立 void 函数（内部可切换状态到 stateUnjamming）
                rammerStuckControl();

            } break;

            case stateUnjamming:
            default: {
                // 解卡动作：建议用 openloop 给反向电流/电压（不走位置环）
                m_rammerMotor->openloopControl(UNJAM_TORQUE);

                // 解卡计时与状态切回：独立 void 函数
                rammerStuckControl();
            } break;
        }
    }
}

void Gimbal::rammerStuckControl()
{
    if (m_shootState == stateFeeding) {

        const fp32 curSpd  = m_rammerMotor->getCurrentAngularVelocity();
        const bool jamCond = (fabsf(curSpd) < JAM_SPEED_TH);

        if (jamCond)
            m_jamCounter++;
        else
            m_jamCounter = 0;

        if (m_jamCounter >= JAM_HOLD_TICKS) {
            m_jamCounter   = 0;
            m_unjamCounter = 0;
            m_shootState   = stateUnjamming;
        }

    } else if (m_shootState == stateUnjamming) {

        m_unjamCounter++;
        if (m_unjamCounter >= UNJAM_TICKS) {
            m_unjamCounter = 0;
            m_shootState   = stateFeeding; // 解卡完成，回去继续这发
        }
    }
}*/

void Gimbal::ledControl()
{
    float leftStickX = m_remoteControl.getLeftStickX();
    static bool isStickReturned = true; 

    if (m_remoteControl.getRightSwitchStatus() == Dr16RemoteControl::SwitchStatus3Pos::SWITCH_MIDDLE) {
        if (leftStickX < -0.5f) {
            if (isStickReturned) {
                currentLedColor = LED_RED;
                isLedChanged = true;
                isStickReturned = false;
            }
        } else if (leftStickX > 0.5f) {
            if (isStickReturned) {
                currentLedColor = LED_BLUE;
                isLedChanged = true;
                isStickReturned = false;
            }
        } else if (abs(leftStickX) < 0.1f) {
            isStickReturned = true;
        }
    } else {
        isStickReturned = true;
    }

    static bool isLedOff = false; 

    if (m_gimbalMode == GIMBAL_NO_FORCE) {
        if (!isLedOff) {
            for(int i=0; i<WS2812_LED_NUM; i++) {
                m_ws2812.SetColor(i, 0, 0, 0);
            }
            m_ws2812.Update();
            isLedOff = true;
        }
    } else {
        if (isLedOff || isLedChanged) {
            uint8_t r = 0, g = 0, b = 0;
            switch (currentLedColor) {
                case LED_RED:   r = 255; g = 0; b = 0; break;
                case LED_BLUE:  r = 0; g = 0; b = 120; break;
                default: break;
            }

            for(int i=0; i<WS2812_LED_NUM; i++) {
                m_ws2812.SetColor(i, r, g, b);
            }
            m_ws2812.Update();
            
            isLedChanged = false;
            isLedOff = false;
        }
    }
}

void Gimbal::transmitGimbalMotorData()
{
    HAL_CAN_AddTxMessage(&hcan1, m_yawMotor->getMotorControlHeader(), (*m_yawMotor + *m_rammerMotor).getMotorControlData(), NULL);
    HAL_CAN_AddTxMessage(&hcan2, m_pitchMotor->getMotorControlHeader(), m_pitchMotor->getMotorControlData(), NULL);
    HAL_CAN_AddTxMessage(&hcan1, m_frictionLeftMotor->getMotorControlHeader(), (*m_frictionLeftMotor + *m_frictionRightMotor).getMotorControlData(), NULL);
}

inline void Gimbal::setPitchAngle(const fp32 &targetAngle)
{
    if (targetAngle > PITCH_UPPER_LIMIT)
        m_pitchTargetAngle = PITCH_UPPER_LIMIT;
    else if (targetAngle < PITCH_LOWER_LIMIT)
        m_pitchTargetAngle = PITCH_LOWER_LIMIT;
    else
        m_pitchTargetAngle = targetAngle;
}

inline void Gimbal::setYawAngle(const fp32 &targetAngle)
{
    if (targetAngle > YAW_UPPER_LIMIT)
        m_yawTargetAngle = YAW_UPPER_LIMIT;
    else if (targetAngle < YAW_LOWER_LIMIT)
        m_yawTargetAngle = YAW_LOWER_LIMIT;
    else
        m_yawTargetAngle = targetAngle;
}

inline void Gimbal::convertGimbalTargetSpeedToChassisTargetSpeed()
{
}

inline fp32 Gimbal::rcStickDeadZoneFilter(const fp32 &rcStickValue)
{
    if (rcStickValue > DT7_STICK_DEAD_ZONE)
        return (rcStickValue - DT7_STICK_DEAD_ZONE) / (1.0f - DT7_STICK_DEAD_ZONE);
    else if (rcStickValue < -DT7_STICK_DEAD_ZONE)
        return (rcStickValue + DT7_STICK_DEAD_ZONE) / (1.0f - DT7_STICK_DEAD_ZONE);
    else
        return 0.0f;
}

/**
 * @brief 重力前馈补偿计算
 * @param baseTorque 基础力矩(PID输出)
 * @param currentAngle 当前角度(rad)
 * @param compensateCoeff 补偿系数(Nm)
 * @return 叠加前馈后的总力矩
 */
inline fp32 Gimbal::gravityCompensate(fp32 baseTorque, fp32 currentAngle, fp32 compensateCoeff)
{
    return baseTorque + compensateCoeff * cos(currentAngle);
}
