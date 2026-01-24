/**
 ******************************************************************************
 * @file           : para_gimbal.hpp
 * @brief          : 云台参数宏定义，方便修改调参
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
#include "math_const.h"

/* Defines -------------------------------------------------------------------*/
/******************************************************************************
 *                            CAN ID
 ******************************************************************************/
#define GIMBAL_TO_CHASSIS_CAN_ID 0x501

/******************************************************************************
 *                            PID参数
 ******************************************************************************/
// 云台Yaw电机 (GM6020)
#define YAW_OUTER_KP                  13.0f    // 40.0f
#define YAW_OUTER_KI                  0.0f     // 0.0f
#define YAW_OUTER_KD                  0.0f     // 300.0f
#define YAW_OUTER_OUT_LIMIT           15.0f    // 15.0f
#define YAW_OUTER_IOUT_LIMIT          0.0f     // 0.0f
#define YAW_INNER_KP                  7000.0f  // 7000.0f
#define YAW_INNER_KI                  0.0f     // 0.0f
#define YAW_INNER_KD                  1.0f     // 250.0f
#define YAW_INNER_OUT_LIMIT           25000.0f // 25000.0f
#define YAW_INNER_IOUT_LIMIT          0.0f     // 10000.0f
#define YAW_INNER_LOWPASS_FILTER_PARA 0.9f     // 0.4f
// 云台Pitch电机
#define PITCH_OUTER_KP                  7.0f // 外环
#define PITCH_OUTER_KI                  0.0f
#define PITCH_OUTER_KD                  0.0f
#define PITCH_OUTER_OUT_LIMIT           10.0f
#define PITCH_OUTER_IOUT_LIMIT          0.0f
#define PITCH_INNER_KP                  0.55f // 内环
#define PITCH_INNER_KI                  0.0f
#define PITCH_INNER_KD                  0.05f
#define PITCH_INNER_OUT_LIMIT           10.0f
#define PITCH_INNER_IOUT_LIMIT          0.0f
#define PITCH_INNER_LOWPASS_FILTER_PARA 0.4f
// 重力补偿前馈（Nm）
#define PITCH_GRAVITY_COMPENSATE 0.0f
// 底盘跟随
#define CHASSIS_FOLLOW_KP 2.0f
// 摩擦轮
#define FRICTION_KP         300.0f
#define FRICTION_KI         10.0f
#define FRICTION_KD         0.0f
#define FRICTION_OUT_LIMIT  15000.0f
#define FRICTION_IOUT_LIMIT 2000.0f
// 拨弹轮
#define RAMMER_KP         2000.0f
#define RAMMER_KI         8.0f
#define RAMMER_KD         0.0f
#define RAMMER_OUT_LIMIT  10000.0f
#define RAMMER_IOUT_LIMIT 3000.0f

/******************************************************************************
 *                            IMU参数
 ******************************************************************************/
// Mahony算法参数
#define AHRS_AUTO_FREQ      0
#define AHRS_DEFAULT_FILTER 0
#define MAHONY_KP           1.0f
#define MAHONY_KI           0.0f
// IMU零飘补偿
#define GYRO_OFFSET_X  0.0f
#define GYRO_OFFSET_Y  0.0f
#define GYRO_OFFSET_Z  0.0f
#define ACCEL_OFFSET_X 0.0f
#define ACCEL_OFFSET_Y 0.0f
#define ACCEL_OFFSET_Z 0.0f
#define MAG_OFFSET_X   0.0f
#define MAG_OFFSET_Y   0.0f
#define MAG_OFFSET_Z   0.0f
// 安装朝向修正旋转矩阵
#define INSTALL_SPIN_MATRIX GSRLMath::Matrix33f((fp32[3][3]){{1, 0, 0}, {0, -1, 0}, {0, 0, -1}})
// #define INSTALL_SPIN_MATRIX GSRLMath::Matrix33f::MatrixType::IDENTITY

/******************************************************************************
 *                            遥控器灵敏度与死区
 ******************************************************************************/
#define DT7_STICK_DEAD_ZONE         0.05f
#define DT7_STICK_PITCH_SENSITIVITY 0.01f
#define DT7_STICK_YAW_SENSITIVITY   0.01f

/******************************************************************************
 *                            云台角度限制
 ******************************************************************************/
#define PITCH_UPPER_LIMIT 0.2f
#define PITCH_LOWER_LIMIT -0.35f
#define YAW_UPPER_LIMIT   0.87f
#define YAW_LOWER_LIMIT   -0.87f
/******************************************************************************
 *                            发射机构参数
 ******************************************************************************/
#define FRICTION_TARGET_ANGULAR_VELOCITY     760.0f
#define RAMMER_TARGET_ANGULAR_VELOCITY       2.6f * MATH_PI//这个拨弹和摩擦速度貌似都有一点快了
#define RAMMER_STUCK_TIMEOUT                 1.0f
#define RAMMER_REVERT_TIME                   2.0f
#define RAMMER_STUCK_REVERT_ANGULAR_VELOCITY 1.0f * MATH_PI

/******************************************************************************
 *                            发射机构参数计算
 ******************************************************************************/
#define RAMMER_SINGLE_SHOT_REVOLUTIONS (0.125f * 36.0f)                                                                                                         // 一圈八个小葡萄，motor类里面未对currentangle做减速比处理但是对currentangularvelocity做了，所以这里要乘减速比
#define SINGLE_SHOT_TARGET(cur)        (floor((cur) / RAMMER_SINGLE_SHOT_REVOLUTIONS + 0.5f) * RAMMER_SINGLE_SHOT_REVOLUTIONS + RAMMER_SINGLE_SHOT_REVOLUTIONS) // 计算转动所需圈数