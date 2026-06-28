//
// Created by Gleam on 25-8-22.
//

#include "rm_config.h"
#include "rm_task.h"
#include "motor_def.h"
#include "rm_algorithm.h"
#include "rm_module.h"

//TODO: 弹频和弹速的控制应在cmd线程中决策

/* ----------------------------------------------- 线程间通讯话题相关 --------------------------------------------------- */
static struct shoot_cmd_msg shoot_cmd;
static struct shoot_fdb_msg shoot_fdb;

static publisher_t *pub_shoot;
static subscriber_t *sub_cmd;

static void shoot_pub_init(void);
static void shoot_sub_init(void);
static void shoot_pub_push(void);
static void shoot_sub_pull(void);

/* ------------------------------------------------- 电机控制相关 ----------------------------------------------------- */
/*发射模块电机使用数量*/
//#define SHT_MOTOR_NUM 3
#define SHT_MOTOR_NUM 4

/*发射模块电机编号：分别为左摩擦轮电机 右摩擦轮电机 拨弹电机*/
#define RIGHT_FRICTION 0
#define LEFT_FRICTION 1
#define MIDDLE_FRICTION 2
#define TRIGGER_MOTOR 3

/*pid环数结构体*/
static struct shoot_controller_t{
    pid_obj_t *pid_speed;
    pid_obj_t *pid_angle;
}sht_controller[SHT_MOTOR_NUM];

/*电机注册初始化数据*/
motor_config_t shoot_motor_config[SHT_MOTOR_NUM] ={
        {
                .motor_type = M3508,
                .can_name = CAN_GIMBAL,
                .rx_id = RIGHT_FRICTION_MOTOR_ID,
                .controller = &sht_controller[RIGHT_FRICTION],
        },
        {
                .motor_type = M3508,
                .can_name = CAN_GIMBAL,
                .rx_id = LEFT_FRICTION_MOTOR_ID,
                .controller = &sht_controller[LEFT_FRICTION],
        },
        {
                .motor_type = M3508,
                .can_name = CAN_GIMBAL,
                .rx_id = MIDDLE_FRICTION_MOTOR_ID,
                .controller = &sht_controller[MIDDLE_FRICTION],
        },
        {
                .motor_type = M3508,
                .can_name = CAN_CHASSIS,
                .rx_id = TRIGGER_MOTOR_ID,
                .controller = &sht_controller[TRIGGER_MOTOR],
        }
};

static dji_motor_object_t *sht_motor[SHT_MOTOR_NUM];  // 发射器电机实例
static float shoot_motor_ref[SHT_MOTOR_NUM]; // 电机控制期望值
/* ------------------------------------------------- 射击控制相关 ----------------------------------------------------- */
//转子角度标志位，防止切换设计模式时拨弹电机反转
static int total_angle_flag=SHOOT_ANGLE_CONTINUE;
/*函数声明*/
static void shoot_motor_init();
//static void servo_init();
static int16_t motor_control_right(dji_motor_measure_t measure);
static int16_t motor_control_middle(dji_motor_measure_t measure);
static int16_t motor_control_left(dji_motor_measure_t measure);
static int16_t motor_control_trigger(dji_motor_measure_t measure);

//拨弹反转期望记录
static int reverse_ref;

/* ----------------------------------------------- 射击线程入口 --------------------------------------------------- */
static float sht_dt;
static int flag;
UBaseType_t shootuxHighWaterMark;
/**
 * @brief shoot线程入口函数
 */
void ShootTask_Entry(void const * argument)
{
    static float sht_start;
    static int servo_cvt_num;
    static float sht_gap_time;
    static float sht_gap_start_time;

    shoot_motor_init();
    shoot_pub_init();
    shoot_sub_init();
/*----------------------射击状态初始化----------------------------------*/
    shoot_cmd.ctrl_mode=SHOOT_STOP;
    shoot_cmd.trigger_status=TRIGGER_OFF;
    shoot_motor_ref[TRIGGER_MOTOR]=0;
    for (;;)
    {
        sht_start = dwt_get_time_ms();
        /* 更新该线程所有的订阅者 */
        shoot_sub_pull();
        /* 倍镜舵机控制 */
        shootuxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );


        /* 电机控制启动 */
        for (uint8_t i = 0; i < SHT_MOTOR_NUM; i++)
        {
            dji_motor_enable(sht_motor[i]);
        }


        shoot_fdb.trigger_motor_current=sht_motor[TRIGGER_MOTOR]->measure.real_current;
        /*控制模式判断*/
        /*subs遥控器*/

        /*开关摩擦轮*/
        if (shoot_cmd.friction_status==SHOOT_ONE)
        {
            shoot_motor_ref[RIGHT_FRICTION] = -FRICTION_SPEED_ONE;//摩擦轮常转 实测最大转速为8800
            shoot_motor_ref[MIDDLE_FRICTION] = -FRICTION_SPEED_ONE;
            shoot_motor_ref[LEFT_FRICTION] = FRICTION_SPEED_ONE;
            if(shoot_cmd.ctrl_mode == SHOOT_COUNTINUE)
            {
                shoot_motor_ref[RIGHT_FRICTION] = -FRICTION_SPEED_CONTINUE;//摩擦轮常转 实测最大转速为8800
                shoot_motor_ref[MIDDLE_FRICTION] = -FRICTION_SPEED_CONTINUE;
                shoot_motor_ref[LEFT_FRICTION] = FRICTION_SPEED_CONTINUE;
            }
            /*从自动连发模式切换三连发及单发模式时，要继承总转子角度*/
        }
        else
        {
            shoot_motor_ref[TRIGGER_MOTOR] = 0;
            shoot_motor_ref[RIGHT_FRICTION] =0;
            shoot_motor_ref[MIDDLE_FRICTION] = 0;
            shoot_motor_ref[LEFT_FRICTION] = 0;
            total_angle_flag=SHOOT_ANGLE_CONTINUE;
        }
        switch (shoot_cmd.ctrl_mode)
        {

            case SHOOT_STOP:
                shoot_motor_ref[TRIGGER_MOTOR] = 0;
                total_angle_flag=0;
                shoot_fdb.trigger_status=SHOOT_WAITING;
                break;

            case SHOOT_ONE:

                if(shoot_cmd.trigger_status == TRIGGER_OFF)
                {
                    shoot_fdb.trigger_status=SHOOT_WAITING;
                }
                if(total_angle_flag == SHOOT_ANGLE_CONTINUE)
                {
                    shoot_motor_ref[TRIGGER_MOTOR]= sht_motor[TRIGGER_MOTOR]->measure.total_angle;
                    total_angle_flag=SHOOT_ANGLE_SINGLE;
                }

                if (shoot_cmd.trigger_status == TRIGGER_ON)
                {
                    sht_gap_time = dwt_get_time_ms() - sht_gap_start_time;
                    sht_gap_start_time = dwt_get_time_ms();
                    flag = 1;
                    if(sht_gap_time>=300)
                    {
                        flag = 2;
                        //M3508的减速比为 19:1，因此转轴旋转51.43度，要在转子的基础上乘19倍
                        if(shoot_motor_ref[TRIGGER_MOTOR] - sht_controller[TRIGGER_MOTOR].pid_angle->Measure > TRIGGER_MOTOR_120_TO_ANGLE * 19 * 0.3)
                        {
                            flag = 3;
                            shoot_motor_ref[TRIGGER_MOTOR]= shoot_motor_ref[TRIGGER_MOTOR]; //此时堵转了,不能再转到拨弹
                        }
                        else if(reverse_ref !=0 && shoot_cmd.last_mode == SHOOT_REVERSE)
                        {
                            flag = 4;
                            if(sht_controller[TRIGGER_MOTOR].pid_angle->Measure - reverse_ref > TRIGGER_MOTOR_120_TO_ANGLE * 19 * 0.4)
                            {
                                flag = 5;
                                shoot_motor_ref[TRIGGER_MOTOR]= shoot_motor_ref[TRIGGER_MOTOR]; //此时反转尚未完成，保持期望角度，避免再次转动抵消反转
                            }
                        }
                        else
                        {
                            flag = 6;
                            shoot_motor_ref[TRIGGER_MOTOR]= shoot_motor_ref[TRIGGER_MOTOR] + TRIGGER_MOTOR_120_TO_ANGLE * 19; //正常转动

                            // shooter_barrel_heat_remain = msg->robot_status.shooter_barrel_heat_limit - msg->power_heat_data.shooter_42mm_barrel_heat;
                            // if(shooter_barrel_heat_remain >= 120
                            //     || (shooter_barrel_heat_remain >= 100 && msg->robot_status.robot_level == 1))
                            // {
                            //     flag = 7;
                            //     shoot_fdb.shoot_cnt++;
                            //     shoot_motor_ref[TRIGGER_MOTOR]= shoot_motor_ref[TRIGGER_MOTOR] + TRIGGER_MOTOR_51_TO_ANGLE * 19;
                            // }

                        }

                    }
                    shoot_cmd.trigger_status=TRIGGER_OFF;//扳机归零
                    shoot_fdb.trigger_status=SHOOT_OK;
                }

                break;

            case SHOOT_THREE:
                /*从自动连发模式切换三连发及单发模式时，要继承总转子角度*/
                if(total_angle_flag == SHOOT_ANGLE_CONTINUE)
                {
                    shoot_motor_ref[TRIGGER_MOTOR]= sht_motor[TRIGGER_MOTOR]->measure.total_angle;
                    total_angle_flag = SHOOT_ANGLE_SINGLE;
                }
                if (shoot_cmd.trigger_status == TRIGGER_ON)
                {

                    shoot_motor_ref[TRIGGER_MOTOR]= shoot_motor_ref[TRIGGER_MOTOR] + TRIGGER_MOTOR_120_TO_ANGLE * 19;//M3508的减速比为 19:1，因此转轴旋转120度，要在转子的基础上乘19倍
                    shoot_cmd.trigger_status=TRIGGER_OFF;//扳机归零
                    shoot_fdb.trigger_status=SHOOT_OK;
                }

                break;

            case SHOOT_COUNTINUE:
                shoot_motor_ref[TRIGGER_MOTOR] = shoot_cmd.shoot_freq;//自动模式的时候，只用速度环控制拨弹电机
                total_angle_flag = SHOOT_ANGLE_CONTINUE;
                shoot_fdb.trigger_status= SHOOT_OK;
                break;

            case SHOOT_REVERSE:
                if(shoot_motor_ref[TRIGGER_MOTOR] - reverse_ref < TRIGGER_MOTOR_120_TO_ANGLE * 19 * 1.8)
                {
                    shoot_motor_ref[TRIGGER_MOTOR]= shoot_motor_ref[TRIGGER_MOTOR]; //没有余量进行反转
                }
                else
                {
                    shoot_motor_ref[TRIGGER_MOTOR]= shoot_motor_ref[TRIGGER_MOTOR] - TRIGGER_MOTOR_120_TO_ANGLE * 19 * 2;
                    reverse_ref = shoot_motor_ref[TRIGGER_MOTOR]; //记录反转后的期望角度
                }

                total_angle_flag = SHOOT_ANGLE_SINGLE;

                break;

            default:
                for (uint8_t i = 0; i < SHT_MOTOR_NUM; i++)
                {
                    dji_motor_relax(sht_motor[i]); // 错误情况电机全部松电
                }
                shoot_fdb.trigger_status=SHOOT_ERR;
                break;
        }
        /* 更新发布该线程的msg */
        shoot_pub_push();

/* ------------------------------ 调试监测线程调度 ------------------------------ */
        sht_dt = dwt_get_time_ms() - sht_start;
        if (sht_dt > 1)
            printf("shoot Task is being DELAY! dt = [%f]", &sht_dt);
        vTaskDelay(1);

    }
}
/* ----------------------------------------------- 线程间通讯话题相关 --------------------------------------------------- */
/**
 * @brief shoot 线程中所有发布者初始化
 */
static void shoot_pub_init()
{
    pub_shoot = pub_register("shoot_fdb", sizeof(struct shoot_fdb_msg));
}
/**
 * @brief shoot 线程中所有订阅者初始化
 */
static void shoot_sub_init()
{
    sub_cmd = sub_register("shoot_cmd", sizeof(struct shoot_cmd_msg));
}
/**
 * @brief shoot 线程中所有发布者推送更新话题
 */
static void shoot_pub_push()
{
    pub_push_msg(pub_shoot, &shoot_fdb);
}
/**
 * @brief shoot 线程中所有订阅者推送更新话题
 */
static void shoot_sub_pull()
{
    sub_get_msg(sub_cmd, &shoot_cmd);
}

/**
 * @brief 舵机初始化
 */
//static void servo_init(){
//    servo_cover_dev=(struct rt_device_pwm *) rt_device_find(PWM_COVER);
//    if(servo_cover_dev == RT_NULL)
//    {
//        LOG_E("Can't find cover servo pwm device!");
//        return;
//    }
//    rt_pwm_set(servo_cover_dev, PWM_COVER_CH, 20000000, 780000);
//    rt_pwm_enable(servo_cover_dev, PWM_COVER_CH);
//}



/* ------------------------------------------------- 电机控制相关 ----------------------------------------------------- */
/**
 * @brief shoot 线程电机初始化
 */
static void shoot_motor_init(){
    /* -------------------------------------- right_friction 右摩擦轮电机 ----------------------------------------- */
    pid_config_t right_speed_config = INIT_PID_CONFIG(RIGHT_KP_V, RIGHT_KI_V, RIGHT_KD_V,RIGHT_INTEGRAL_V,RIGHT_MAX_V,
                                                      (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    sht_controller[RIGHT_FRICTION].pid_speed = pid_register(& right_speed_config);

/* ------------------------------------------- left_friction 左摩擦轮电机------------------------------------------------- */
    pid_config_t left_speed_config = INIT_PID_CONFIG(LEFT_KP_V,  LEFT_KI_V, LEFT_KD_V , LEFT_INTEGRAL_V, LEFT_MAX_V,
                                                     (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    sht_controller[LEFT_FRICTION].pid_speed = pid_register(&left_speed_config);

    /* ------------------------------------------- middle_friction 中摩擦轮电机------------------------------------------------- */
    pid_config_t middle_speed_config = INIT_PID_CONFIG(MIDDLE_KP_V,  MIDDLE_KI_V, MIDDLE_KD_V , MIDDLE_INTEGRAL_V, MIDDLE_MAX_V,
                                                       (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    sht_controller[MIDDLE_FRICTION].pid_speed = pid_register(&middle_speed_config);
/* ------------------------------------------------  拨弹电机------------------------------------------------------------------------- */
    pid_config_t toggle_speed_config = INIT_PID_CONFIG(TRIGGER_KP_V  , TRIGGER_KI_V , TRIGGER_KD_V  , TRIGGER_INTEGRAL_V, TRIGGER_MAX_V ,
                                                       (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    pid_config_t toggle_angle_config = INIT_PID_CONFIG(TRIGGER_KP_A, TRIGGER_KI_A, TRIGGER_KD_A, TRIGGER_INTEGRAL_A , TRIGGER_MAX_A ,
                                                       (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    sht_controller[TRIGGER_MOTOR].pid_speed = pid_register(&toggle_speed_config);
    sht_controller[TRIGGER_MOTOR].pid_angle = pid_register(&toggle_angle_config);

/* ---------------------------------- shoot电机初注册---------------------------------------------------------------------------------------- */
    sht_motor[TRIGGER_MOTOR] = dji_motor_register(&shoot_motor_config[TRIGGER_MOTOR], motor_control_trigger);
    sht_motor[LEFT_FRICTION] = dji_motor_register(&shoot_motor_config[LEFT_FRICTION], motor_control_left);
    sht_motor[MIDDLE_FRICTION] = dji_motor_register(&shoot_motor_config[MIDDLE_FRICTION], motor_control_middle);
    sht_motor[RIGHT_FRICTION] = dji_motor_register(&shoot_motor_config[RIGHT_FRICTION], motor_control_right);
}



/* ------------------------------------------------- 射击控制相关 ----------------------------------------------------- */
/*右摩擦轮电机控制算法*/
static int16_t motor_control_right(dji_motor_measure_t measure)
{
    static int16_t set = 0;
    set =(int16_t) pid_calculate(sht_controller[RIGHT_FRICTION].pid_speed, measure.speed_rpm, shoot_motor_ref[RIGHT_FRICTION]);
    return set;
}

/*左摩擦轮电机控制算法*/
static int16_t motor_control_left(dji_motor_measure_t measure)
{
    static int16_t set = 0;
    set = (int16_t) pid_calculate(sht_controller[LEFT_FRICTION].pid_speed, measure.speed_rpm, shoot_motor_ref[LEFT_FRICTION]);
    return set;
}
/*中摩擦轮电机控制算法*/
static int16_t motor_control_middle(dji_motor_measure_t measure)
{
    static int16_t set = 0;
    set = (int16_t) pid_calculate(sht_controller[MIDDLE_FRICTION].pid_speed, measure.speed_rpm, shoot_motor_ref[MIDDLE_FRICTION]);
    return set;
}

/*拨弹电机控制算法*/
static int16_t motor_control_trigger(dji_motor_measure_t measure)
{
    /* PID局部指针，切换不同模式下PID控制器 */
    static pid_obj_t *pid_angle;
    static pid_obj_t *pid_speed;
    static float get_speed, get_angle;  // 闭环反馈量
    static float pid_out_angle;         // 角度环输出
    static int16_t send_data;        // 最终发送给电调的数据

    /*拨弹电机采用串级pid，一个角度环和一个速度环*/
    pid_speed = sht_controller[TRIGGER_MOTOR].pid_speed;
    pid_angle = sht_controller[TRIGGER_MOTOR].pid_angle;
    get_angle=measure.total_angle;
    get_speed=measure.speed_rpm;

    /* 切换模式需要清空控制器历史状态 */
    if(shoot_cmd.ctrl_mode != shoot_cmd.last_mode)
    {
        pid_clear(pid_angle);
        pid_clear(pid_speed);
    }

    /*pid计算输出*/
    if (shoot_cmd.ctrl_mode==SHOOT_ONE||shoot_cmd.ctrl_mode==SHOOT_THREE||shoot_cmd.ctrl_mode==SHOOT_REVERSE) //非连发模式的时候，用双环pid控制拨弹电机
    {
        pid_out_angle = (int16_t) pid_calculate(pid_angle, get_angle, shoot_motor_ref[TRIGGER_MOTOR]);  // 编码器增长方向与imu相反
        send_data = (int16_t) pid_calculate(pid_speed, get_speed, pid_out_angle);     // 电机转动正方向与imu相反
    }
        /*pid计算输出*/
    else if(shoot_cmd.ctrl_mode==SHOOT_COUNTINUE||shoot_cmd.ctrl_mode==SHOOT_STOP)//自动模式的时候，只用速度环控制拨弹电机
    {
        send_data = (int16_t) pid_calculate(pid_speed, get_speed, shoot_motor_ref[TRIGGER_MOTOR] );
    }
    return send_data;
}


