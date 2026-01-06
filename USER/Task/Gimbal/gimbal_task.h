//
// Created by Gleam on 25-8-22.
//

#ifndef CTRBOARD_H7_ALL_GIMBAL_TASK_H
#define CTRBOARD_H7_ALL_GIMBAL_TASK_H

/**
 * @brief 云台模式
 */
typedef enum
{
    GIMBAL_RELAX = 0,        //云台断电
    GIMBAL_INIT = 1,         //云台初始化
    GIMBAL_GYRO = 2,         //云台跟随imu闭环
    GIMBAL_AUTO = 3          //云台自瞄
} gimbal_mode_e;

/**
  * @brief     云台回中状态枚举
  */
typedef enum
{
    BACK_STEP = 0,             //云台正在回中
    BACK_IS_OK = 1,            //云台回中完毕
} gimbal_back_e;

/**
  * @brief     自瞄相对角度传参
  */
typedef enum
{
    RELATIVE_ANGLE_TRANS = 0,             //云台正在回中
    RELATIVE_ANGLE_OK = 1,            //云台回中完毕
} auto_relative_angle_status_e;
#endif //CTRBOARD_H7_ALL_GIMBAL_TASK_H
