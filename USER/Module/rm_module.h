//
// Created by Gleam on 25-9-4.
//

#ifndef CTRBOARD_H7_ALL_RM_MODULE_H
#define CTRBOARD_H7_ALL_RM_MODULE_H

#include "BMI088driver.h"
#include "dj_motor.h"
#include "dm_motor_ctrl.h"
#include "drv_dwt.h"
#include "keyboard.h"

#ifdef BSP_USING_RC_DBUS
#include "rc_dbus.h"
#else
#include "rc_sbus.h"
#endif

#include "referee_system.h"
#include "drv_msg.h"


#endif //CTRBOARD_H7_ALL_RM_MODULE_H
