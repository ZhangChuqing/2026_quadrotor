/**
 ******************************************************************************
 * @file           : tsk_test.cpp
 * @brief          : 测试任务
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
#include "tsk_isr.hpp"
#include "dvc_vofa.hpp"

/* Define --------------------------------------------------------------------*/
/******************************************************************************
 *                            电机相关
 ******************************************************************************/
/* PID -----------------------------------------*/
// Yaw
CascadePID::PIDParam yawOuterParam = {
    YAW_OUTER_KP,        // Kp
    YAW_OUTER_KI,        // Ki
    YAW_OUTER_KD,        // Kd
    YAW_OUTER_OUT_LIMIT, // outputLimit
    YAW_OUTER_IOUT_LIMIT // intergralLimit
};
CascadePID::PIDParam yawInnerParam = {
    YAW_INNER_KP,        // Kp
    YAW_INNER_KI,        // Ki
    YAW_INNER_KD,        // Kd
    YAW_INNER_OUT_LIMIT, // outputLimit
    YAW_INNER_IOUT_LIMIT // intergralLimit
};
LowPassFilter<fp32> yawInnerLPF(YAW_INNER_LOWPASS_FILTER_PARA);
CascadePID yawPID(yawOuterParam, yawInnerParam, nullptr, &yawInnerLPF);
// Pitch
CascadePID::PIDParam pitchOuterParam = {
    PITCH_OUTER_KP,        // Kp
    PITCH_OUTER_KI,        // Ki
    PITCH_OUTER_KD,        // Kd
    PITCH_OUTER_OUT_LIMIT, // outputLimit
    PITCH_OUTER_IOUT_LIMIT // intergralLimit
};
CascadePID::PIDParam pitchInnerParam = {
    PITCH_INNER_KP,        // Kp
    PITCH_INNER_KI,        // Ki
    PITCH_INNER_KD,        // Kd
    PITCH_INNER_OUT_LIMIT, // outputLimit
    PITCH_INNER_IOUT_LIMIT // intergralLimit
};
LowPassFilter<fp32> pitchInnerLPF(PITCH_INNER_LOWPASS_FILTER_PARA);
CascadePID pitchPID(pitchOuterParam, pitchInnerParam, nullptr, &pitchInnerLPF);
// Friction
SimplePID::PIDParam leftfrictionPIDParam = {
    FRICTION_KP,        // Kp
    FRICTION_KI,        // Ki
    FRICTION_KD,        // Kd
    FRICTION_OUT_LIMIT, // outputLimit
    FRICTION_IOUT_LIMIT // intergralLimit
};
SimplePID::PIDParam rightfrictionPIDParam = {
    FRICTION_KP,        // Kp
    FRICTION_KI,        // Ki
    FRICTION_KD,        // Kd
    FRICTION_OUT_LIMIT, // outputLimit
    FRICTION_IOUT_LIMIT // intergralLimit
};

SimplePID leftFrictionPID(SimplePID::PID_POSITION, leftfrictionPIDParam);
SimplePID rightFrictionPID(SimplePID::PID_POSITION, rightfrictionPIDParam);
// Rammer
// 使用双环PID：外环(位置) -> 内环(速度) -> 电流
SimplePID::PIDParam rammerParam = {
    RAMMER_KP,
    RAMMER_KI,
    RAMMER_KD,
    RAMMER_OUT_LIMIT,
    RAMMER_IOUT_LIMIT};
SimplePID rammerPID(SimplePID::PID_POSITION, rammerParam);

/* Motor ---------------------------------------------*/

MotorGM6020 yawMotor(1, &yawPID, 0);
MotorDM4310 pitchMotor(1, 3, 3.141593f, 30, 10, &pitchPID);
MotorM2006 rammerMotor(6, &rammerPID, 0, 36);
MotorM3508 leftFrictionMotor(4, &leftFrictionPID);
MotorM3508 rightFrictionMotor(2, &rightFrictionPID);

Vofa<4> vofa;

/******************************************************************************
 *                            IMU相关
 ******************************************************************************/
// AHRS算法
Mahony ahrs(AHRS_AUTO_FREQ, AHRS_DEFAULT_FILTER, MAHONY_KP, MAHONY_KI);
// IMU校准数据
BMI088::CalibrationInfo cali = {
    {GYRO_OFFSET_X, GYRO_OFFSET_Y, GYRO_OFFSET_Z},    // gyroOffset
    {ACCEL_OFFSET_X, ACCEL_OFFSET_Y, ACCEL_OFFSET_Z}, // accelOffset
    {MAG_OFFSET_X, MAG_OFFSET_Y, MAG_OFFSET_Z},       // magnetOffset
    {INSTALL_SPIN_MATRIX}                             // installSpinMatrix
};
// IMU类定义
BMI088 imu(&ahrs, {&hspi1, GPIOA, GPIO_PIN_4}, {&hspi1, GPIOB, GPIO_PIN_0}, cali);

// gimbal
Gimbal gimbal(&yawMotor, &pitchMotor, &rammerMotor, &leftFrictionMotor, &rightFrictionMotor, &imu);

/* Variables -----------------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/* User code -----------------------------------------------------------------*/

extern "C" void gimbal_task(void *argument)
{
    CAN_Init(&hcan1, can1RxCallback);       // 初始化CAN1
    UART_Init(&huart3, dr16RxCallback, 36); // 初始化DR16串口
    vofa.Init();

    // vofa.AddParameterListener目前未实现，暂时注释掉

    /*vofa.AddParameterListener("IKP", [](fp32 *newValue) {
        // 这里可以处理新的参数值，例如打印或应用到系统中
        printf("Received new value for 'IKP': %f\n", *newValue);
        rammerInnerParam.Kp = *newValue;                        // 将新的Kp值应用到rammer内环PID参数
        rammerPID.getInnerLoop().pidSetParam(rammerInnerParam); // 更新PID控制器的参数
    });
    vofa.AddParameterListener("IKI", [](fp32 *newValue) {
        printf("Received new value for 'IKI': %f\n", *newValue);
        rammerInnerParam.Ki = *newValue;                        // 将新的Ki值应用到rammer内环PID参数
        rammerPID.getInnerLoop().pidSetParam(rammerInnerParam); // 更新PID控制器的参数
    });
    vofa.AddParameterListener("IKD", [](fp32 *newValue) {
        printf("Received new value for 'IKD': %f\n", *newValue);
        rammerInnerParam.Kd = *newValue;                        // 将新的Kd值应用到rammer内环PID参数
        rammerPID.getInnerLoop().pidSetParam(rammerInnerParam); // 更新PID控制器的参数
    });
    vofa.AddParameterListener("OKP", [](fp32 *newValue) {
        printf("Received new value for 'OKP': %f\n", *newValue);
        rammerOuterParam.Kp = *newValue;                        // 将新的Kp值应用到rammer外环PID参数
        rammerPID.getOuterLoop().pidSetParam(rammerOuterParam); // 更新PID控制器的参数
    });
    vofa.AddParameterListener("OKI", [](fp32 *newValue) {
        printf("Received new value for 'OKI': %f\n", *newValue);
        rammerOuterParam.Ki = *newValue;                        // 将新的Ki值应用到rammer外环PID参数
        rammerPID.getOuterLoop().pidSetParam(rammerOuterParam); // 更新PID控制器的参数
    });
    vofa.AddParameterListener("OKD", [](fp32 *newValue) {
        printf("Received new value for 'OKD': %f\n", *newValue);
        rammerOuterParam.Kd = *newValue;                        // 将新的Kd值应用到rammer外环PID参数
        rammerPID.getOuterLoop().pidSetParam(rammerOuterParam); // 更新PID控制器的参数
    });*/

    TickType_t taskLastWakeTime = xTaskGetTickCount(); // 获取任务开始时间
    gimbal.init();

    // 拨弹电机PID极性反转
    //rammerMotor.setControllerOutputPolarity(false);

    while (1) {
        //printf("Hello, World!\n");

        // motor.openloopControl(0.0f); // motor未定义
        // transmitMotorsControlData(); // 函数未定义
        gimbal.controlLoop();
        /*vofa.writeData((fp32)rammerMotor.getCurrentTorqueCurrent());
        vofa.writeData(rammerMotor.getCurrentAngle());
        vofa.writeData(rammerPID.getOuterLoop().pidGetData().output);
        vofa.writeData(rammerPID.getInnerLoop().pidGetData().output);
        vofa.sendFrame();*/
        vTaskDelayUntil(&taskLastWakeTime, 1); // 确保任务以定周期1ms运行
    }
}
