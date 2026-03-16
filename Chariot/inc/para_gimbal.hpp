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

/******************************************************************************
 *                            PID参数
 ******************************************************************************/
// 注意：以下vofa监听代码被移动到tsk_gimbal.cpp或相关初始化函数中，因为不能在头文件全局作用域执行代码
// 另外，vofa类目前没有AddParameterListener成员函数，需要实现或确认接口
/*
vofa.AddParameterListener("rammer_p", [](fp32 *val) {
    rammer_pid.kp = *val;
});
vofa.AddParameterListener("rammer_i", [](fp32 *val) {
    rammer_pid.ki = *val;
});
vofa.AddParameterListener("rammer_d", [](fp32 *val) {
    rammer_pid.kd = *val;
});
*/
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
// 云台Pitch电机(有nuc版)
#define PITCH_OUTER_KP                  1.0f // 外环
#define PITCH_OUTER_KI                  0.0f
#define PITCH_OUTER_KD                  0.0f
#define PITCH_OUTER_OUT_LIMIT           0.01f
#define PITCH_OUTER_IOUT_LIMIT          0.0f
#define PITCH_INNER_KP                  0.55f // 内环
#define PITCH_INNER_KI                  0.0f
#define PITCH_INNER_KD                  0.00f
#define PITCH_INNER_OUT_LIMIT           3.0f
#define PITCH_INNER_IOUT_LIMIT          0.0f
#define PITCH_INNER_LOWPASS_FILTER_PARA 1.0f
//无nuc版的pitch参数（需要调整）
/*#define PITCH_OUTER_KP                  7.0f // 外环
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
#define PITCH_ZERO_ANGLE                1.0f */
// 重力补偿前馈（Nm）
#define PITCH_GRAVITY_COMPENSATE 0.0f
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
//#define INSTALL_SPIN_MATRIX GSRLMath::Matrix33f((fp32[3][3]){{0, -1, 0}, {-1, 0, 0}, {0, 0, -1}})

/******************************************************************************
 *                            遥控器灵敏度与死区
 ******************************************************************************/
#define DT7_STICK_DEAD_ZONE         0.05f
#define DT7_STICK_PITCH_SENSITIVITY 0.01f
#define DT7_STICK_YAW_SENSITIVITY   0.01f

/******************************************************************************
 *                            云台角度限制
 ******************************************************************************/
#define PITCH_UPPER_LIMIT 0.17f
#define PITCH_LOWER_LIMIT -0.43f
#define YAW_UPPER_LIMIT   0.78f
#define YAW_LOWER_LIMIT   -0.78f
/******************************************************************************
 *                            发射机构参数
 ******************************************************************************/
#define FRICTION_TARGET_ANGULAR_VELOCITY     760.0f
#define RAMMER_TARGET_ANGULAR_VELOCITY       2.6f * MATH_PI//这个拨弹和摩擦速度貌似都有一点快了
#define RAMMER_STUCK_TIMEOUT                 1.0f
#define RAMMER_REVERT_TIME                   2.0f
#define RAMMER_STUCK_REVERT_ANGULAR_VELOCITY 1.0f * MATH_PI
// 左开关短按/长按阈值：短按=单发，长按=连发
//#define FIRE_HOLD_MS 80

// 拨弹电机减速比（M2006为36:1）
//#define FEEDER_REDUCTION_RATIO 36.0f

// 单发步进（8弹位：1/8圈 * 减速比）
// 原本是0.125f，现在为了让输出轴转1/8圈，电机轴需要转 0.125 * 36 = 4.5圈
//#define FEED_STEP_REV  (0.125f * FEEDER_REDUCTION_RATIO)
// 到位误差允许值也要乘以减速比放大
//#define FEED_REV_EPS   (0.010f * FEEDER_REDUCTION_RATIO)
//#define FEED_SPEED_EPS 1.0f  // 速度阈值也适当放大，原0.6太小

// 连发节拍：连发模式下每隔多少ms触发一次“单发步进”
//#define CONT_FIRE_PERIOD_MS 90

// 卡弹判定与解卡（tick计数版本，假设控制周期=1ms）
//#define JAM_SPEED_TH   0.6f
//#define JAM_HOLD_TICKS 120

//#define UNJAM_TORQUE   (-2000.0f)
//#define UNJAM_TICKS    80
