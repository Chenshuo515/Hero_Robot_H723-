/*
* Change Logs:
* Date            Author          Notes
* 2023-09-24      ChuShicheng     first version
* 2023-10-10      ChenSihan       发射模块状态机
*/

#include <stdio.h>
#include <string.h>
#include "rm_config.h"
#include "rm_task.h"
#include "rm_algorithm.h"
#include "rm_module.h"

/* ----------------------------------------------- 线程间通讯话题相关 --------------------------------------------------- */



extern struct referee_fdb_msg referee_fdb;

static publisher_t *pub_gim, *pub_chassis, *pub_shoot,*pub_ui;
static subscriber_t *sub_gim, *sub_shoot,*sub_trans,*sub_ins,*sub_referee,*sub_chassis;
static struct gimbal_cmd_msg gim_cmd;
static struct gimbal_fdb_msg gim_fdb;
static struct shoot_cmd_msg  shoot_cmd;
static struct shoot_fdb_msg  shoot_fdb;
static struct trans_fdb_msg  trans_fdb;
static struct ins_msg ins_data;

static struct chassis_cmd_msg chassis_cmd;
static struct chassis_fdb_msg chassis_fdb;

static void cmd_pub_init(void);
static void cmd_pub_push(void);
static void cmd_sub_init(void);
static void cmd_sub_pull(void);


/* ---------------------------------------------------- 底盘相关 ------------------------------------------------------ */
/*小陀螺堵塞计次*/
static int spin_cnt=0;
/*部署模式标志位*/
static int deploy_flag = 0;

/* ---------------------------------------------------- 云台相关 ------------------------------------------------------ */
/*舵机控制倍镜旋转标志位*/
static int mirror_servo_flag=0;
/*自瞄鼠标累计操作值*/
static float mouse_accumulate_x=0;
static float mouse_accumulate_y=0;
/*自瞄继承角度*/
static float gyro_yaw_inherit;
static float gyro_pitch_inherit;
/* --------------------------------------------------- 发射机构相关 ---------------------------------------------------- */
/*发射停止标志位*/
static int trigger_flag=0;
/*堵转电流反转记次*/
static int reverse_cnt;

/* --------------------------------------------------- 遥控器相关 ---------------------------------------------------- */
#ifdef BSP_USING_RC_DBUS
extern rc_dbus_obj_t dbus_data_fdb;
static rc_dbus_obj_t *rc_now, *rc_last;

#else
extern sbus_data_t sbus_data_fdb;
#endif

extern SemaphoreHandle_t sbus_cmd_mutex;

/* --------------------------------------------------- 键鼠相关 ---------------------------------------------------- */
/*按键状态标志位*/
static int key_e_status=-1;
static int key_f_status=-1;
static int key_g_status=-1;
static int key_v_status=-1;
/*键鼠开关摩擦轮标志位*/
static int rc_f_status=-1;

/*键盘加速度的斜坡*/
ramp_obj_t *km_vx_ramp = NULL;;//x轴控制斜坡
ramp_obj_t *km_vy_ramp = NULL;//y周控制斜坡
ramp_obj_t *km_vw_ramp = NULL;//y周控制斜坡

/*储存鼠标坐标数据*/
First_Order_Filter_t mouse_y_lpf,mouse_x_lpf;
float Ballistic;  //对自瞄数据进行手动鼠标弹道补偿

/* ----------------------------------------------- 其余变量与函数声明 ---------------------------------------------------- */
static int cnt_flag=0;

/* 外部变量声明 */
/*自瞄相对角传参反馈*/
extern auto_relative_angle_status_e auto_relative_angle_status;
/*用于清除环形缓冲区buffer的指针*/
extern uint8_t *r_buffer_point;

void remote_to_cmd_sbus(void);
static void remote_to_cmd_pc_DT7(void);


/* ------------------------------------------------- cmd线程入口 ------------------------------------------------------ */
static float gim_dt;

void CmdTask_Entry(void const * argument)
{
    static float gim_start;

    cmd_pub_init();
    cmd_sub_init();


    km_vx_ramp = ramp_register(0, 200); //2500000
    km_vy_ramp = ramp_register(0, 200);  // 0 -2的累加次数
    km_vw_ramp = ramp_register(0, 200);

    /* 鼠标一阶滤波器初始化*/
    First_Order_Filter_Init(&mouse_x_lpf,0.014f,0.1f);
    First_Order_Filter_Init(&mouse_y_lpf,0.014f,0.1f);


#ifdef BSP_USING_RC_DBUS
    /* 初始化拨杆为上位 */
    dbus_data_fdb.sw1 = RC_UP;
    dbus_data_fdb.sw2 = RC_UP;

#else
    /* 初始化拨杆为上位 */
    sbus_data_fdb.sw1 = RC_UP;
    sbus_data_fdb.sw2 = RC_UP;
    sbus_data_fdb.sw3 = RC_UP;
    sbus_data_fdb.sw4 = RC_UP;
#endif

    for (;;)
    {
        gim_start = dwt_get_time_ms();
        cmd_sub_pull();

//        remote_to_cmd_sbus();
        remote_to_cmd_pc_DT7();
        /* 更新发布该线程的msg */
        cmd_pub_push();
        /* 用于调试监测线程调度使用 */
        gim_dt = dwt_get_time_ms() - gim_start;
        vTaskDelay(1);
    }
}
/* ----------------------------------------------- 线程间通讯话题相关 --------------------------------------------------- */
/**
 * @brief cmd 线程中所有发布者初始化
 */
static void cmd_pub_init(void)
{
    pub_gim = pub_register("gim_cmd", sizeof(struct gimbal_cmd_msg));
    pub_chassis = pub_register("chassis_cmd", sizeof(struct chassis_cmd_msg));
    pub_shoot= pub_register("shoot_cmd", sizeof(struct shoot_cmd_msg));
//    pub_ui= pub_register("ui_cmd", sizeof(struct ui_cmd_msg));
}

/**
 * @brief cmd 线程中所有发布者推送更新话题
 */
static void cmd_pub_push(void)
{
    pub_push_msg(pub_gim, &gim_cmd);
    pub_push_msg(pub_chassis, &chassis_cmd);
    pub_push_msg(pub_shoot, &shoot_cmd);
//    pub_push_msg(pub_ui, &ui_cmd);
}

/**
 * @brief cmd 线程中所有订阅者初始化
 */
static void cmd_sub_init(void)
{
    sub_gim = sub_register("gim_fdb", sizeof(struct gimbal_fdb_msg));
    sub_shoot= sub_register("shoot_fdb", sizeof(struct shoot_fdb_msg));
    sub_trans= sub_register("trans_fdb", sizeof(struct trans_fdb_msg));
    sub_ins = sub_register("ins_msg", sizeof(struct ins_msg));
//    sub_referee= sub_register("referee_fdb",sizeof(struct referee_msg));
    sub_chassis = sub_register("chassis_fdb", sizeof(struct chassis_fdb_msg));
}


/**
 * @brief cmd 线程中所有订阅者获取更新话题
 */
static void cmd_sub_pull(void)
{
    sub_get_msg(sub_gim, &gim_fdb);
    sub_get_msg(sub_shoot, &shoot_fdb);
    sub_get_msg(sub_trans,&trans_fdb);
    sub_get_msg(sub_ins, &ins_data);
    sub_get_msg(sub_referee, &referee_fdb);
    sub_get_msg(sub_chassis, &chassis_fdb);
}


/* -------------------------------------------- 将遥控器数据转换为控制指令 ----------------------------------------------- */
#ifdef BSP_USING_RC_DBUS
static void remote_to_cmd_pc_DT7(void)
{
    /* 保存上一次数据 */
    gim_cmd.last_mode = gim_cmd.ctrl_mode;
    chassis_cmd.last_mode = chassis_cmd.ctrl_mode;
    shoot_cmd.last_mode=shoot_cmd.ctrl_mode;
    xSemaphoreTake(sbus_cmd_mutex, portMAX_DELAY);
    *rc_now = dbus_data_fdb;  // 复制到临时变量
    rc_last = (rc_now + 1);   // rc_obj[0]:当前数据NOW,[1]:上一次的数据LAST
    xSemaphoreGive(sbus_cmd_mutex);    gim_cmd.last_mode = gim_cmd.ctrl_mode;

    //目前此版本代码这里有问题，可能是因为键鼠数据缺失导致的野指针，暂时注释掉了用他的地方
    float fx=First_Order_Filter_Calculate(&mouse_x_lpf,rc_now->mouse.x);
    float fy=First_Order_Filter_Calculate(&mouse_y_lpf,rc_now->mouse.y);

    Ballistic += First_Order_Filter_Calculate(&mouse_y_lpf,rc_now->mouse.y)*0.05f;

// TODO: 目前状态机转换较为简单，有很多优化和改进空间
//遥控器的控制信息转化为标准单位，平移为(mm/s)旋转为(degree/s)
    /*底盘命令*/
    chassis_cmd.vx =  (float)rc_now->ch1 * CHASSIS_RC_MOVE_RATIO_X / RC_DBUS_MAX_VALUE * MAX_CHASSIS_VX_SPEED + km.vx * CHASSIS_PC_MOVE_RATIO_X;
    chassis_cmd.vy =  (float)rc_now->ch2 * CHASSIS_RC_MOVE_RATIO_Y / RC_DBUS_MAX_VALUE * MAX_CHASSIS_VY_SPEED + km.vy * CHASSIS_PC_MOVE_RATIO_Y;
    //chassis_cmd.vw =  (float)rc_now->ch3 * CHASSIS_RC_MOVE_RATIO_R / RC_DBUS_MAX_VALUE * MAX_CHASSIS_VR_SPEED + rc_now->mouse.x * CHASSIS_PC_MOVE_RATIO_R;

    chassis_cmd.offset_angle = gim_fdb.yaw_relative_angle;

    /*云台命令*/
    if (gim_cmd.ctrl_mode==GIMBAL_GYRO)
    {
        gim_cmd.yaw +=   (float)rc_now->ch3 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW ;//+ fx * KB_RATIO * GIMBAL_PC_MOVE_RATIO_YAW;
        gim_cmd.pitch += (float)rc_now->ch4 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_PIT;//- fy * KB_RATIO * GIMBAL_PC_MOVE_RATIO_PIT;
        gyro_yaw_inherit =gim_cmd.yaw;
        gyro_pitch_inherit =gim_cmd.pitch;
        mouse_accumulate_x=0;
        mouse_accumulate_y=0;
    }
    if (gim_cmd.ctrl_mode==GIMBAL_AUTO)
    {
        if (auto_relative_angle_status==RELATIVE_ANGLE_TRANS)
        {
            trans_fdb.yaw=0;
            trans_fdb.pitch=0;
        }
        //mouse_accumulate_x+=fx * KB_RATIO * GIMBAL_PC_MOVE_RATIO_YAW; /*鼠标x轴的自瞄补偿，测试结果建议注释掉暂不使用*/
        //mouse_accumulate_y-=fy * KB_RATIO * GIMBAL_PC_MOVE_RATIO_PIT; //建议打开
        if(trans_fdb.roll != 2)
        {
            gim_cmd.yaw = trans_fdb.yaw+gyro_yaw_inherit + mouse_accumulate_x/* + 150 * rc_now->ch3 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW*/;//上位机自瞄
            gim_cmd.pitch = trans_fdb.pitch+gyro_pitch_inherit + mouse_accumulate_y/* +100 * rc_now->ch4 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_PIT */;//上位机自瞄
        }

    }

    /*!限制云台pitch轴角度 */
    VAL_LIMIT(gim_cmd.pitch, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
    /*开环状态和遥控器归中*/
    if (gim_cmd.ctrl_mode==GIMBAL_INIT||gim_cmd.ctrl_mode==GIMBAL_RELAX)
    {
        gim_cmd.pitch=0;
        gim_cmd.yaw=0;
    }
    /*-------------------------------------------------底盘_云台状态机--------------------------------------------------------------*/


    /*TODO:手动模式和自瞄模式状态机*/
    /*!如果鼠标未按下右键*/
    if (rc_now->mouse.r==0)
    {
        if (gim_cmd.last_mode == GIMBAL_RELAX)
        {/* 判断上次状态是否为RELAX，是则先归中 */
            gim_cmd.ctrl_mode = GIMBAL_INIT;
            chassis_cmd.ctrl_mode=CHASSIS_RELAX;

        }
        else
        {
            if (gim_fdb.back_mode == BACK_IS_OK)  /*!是否归中完成*/
            {
                gim_cmd.ctrl_mode = GIMBAL_GYRO;
                chassis_cmd.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;
                memset(r_buffer_point,0,sizeof (*r_buffer_point));
            }
            else if(gim_fdb.back_mode==BACK_STEP)
            {
                chassis_cmd.ctrl_mode=CHASSIS_RELAX;
                gim_cmd.ctrl_mode=GIMBAL_INIT;
            }
        }
    }

    if (rc_now->mouse.r==1||rc_now->sw2==RC_DN) /*!如果鼠标按下右键或者遥控器选择自瞄模式*/
    {
        if (gim_cmd.last_mode == GIMBAL_RELAX)
        {/* 判断上次状态是否为RELAX，是则先归中 */
            gim_cmd.ctrl_mode = GIMBAL_INIT;
            chassis_cmd.ctrl_mode=CHASSIS_RELAX;
        }
        else
        {
            if (gim_fdb.back_mode == 1)
//            if (gim_fdb.back_mode == BACK_IS_OK)
            {/* 判断归中是否完成 */
                gim_cmd.ctrl_mode = GIMBAL_AUTO;
                chassis_cmd.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;
            }
            else if(gim_fdb.back_mode==BACK_STEP)
            {
                chassis_cmd.ctrl_mode=CHASSIS_RELAX;
                gim_cmd.ctrl_mode=GIMBAL_INIT;
            }
        }
    }

    /* -------------初始化ui按键B-----------------*/
//    if(km.b_sta == KEY_PRESS_DOWN){
//
//        ui_cmd.ui_init = 1;//B键被按下满足ui初始化条件
//
//    }else{
//
//        ui_cmd.ui_init = 0;
//
//    }


    /*TODO:小陀螺*/
    //开小陀螺
    if(km.e_sta==KEY_PRESS_ONCE)
    {
        key_e_status=1;
    }
    if ( key_e_status==1||rc_now->sw1==RC_DN)
    {
        if (gim_fdb.back_mode==BACK_IS_OK)
        {
            chassis_cmd.ctrl_mode=CHASSIS_SPIN;
        }
        else
        {
            chassis_cmd.ctrl_mode=CHASSIS_RELAX;
            gim_cmd.ctrl_mode=GIMBAL_INIT;
        }
    }
    //关小陀螺
    if(rc_now->kb.bit.Q == 1 )
    {
        key_e_status=0;
    }
    if ( key_e_status==0)
    {
        chassis_cmd.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;
    }

    if (chassis_cmd.ctrl_mode==CHASSIS_SPIN)
    {
        chassis_cmd.vw=3;// * msg_cmd->robot_status.chassis_power_limit/55;/*!小陀螺转速，随着功率限制提升加快转速*/
        if(chassis_fdb.vw_ch < chassis_cmd.vw*0.85f) //当小陀螺被堵住时，自动退出小陀螺模式
        {
            spin_cnt++;
            if(spin_cnt>2000)
            {
                chassis_cmd.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;
                spin_cnt=0;
            }
        }
        else
        {
            spin_cnt =0;
        }

    }

    /*TODO:--------------------------------------------------发射模块状态机--------------------------------------------------------------*/
    /*!-----------------------------------------开关摩擦轮--------------------------------------------*/
    /*-----------------------------------------开关摩擦轮--------------------------------------------*/
    if(km.f_sta==KEY_PRESS_ONCE)
    {
        key_f_status=1;
    }
    if(rc_now->sw1==RC_MI)
    {
        rc_f_status = 1;
    }
    if ( key_f_status==1||rc_f_status == 1)
    {
        shoot_cmd.friction_status=1;
    }
    else
    {
        shoot_cmd.friction_status = 0;
    }

    //关摩擦轮
    if(rc_now->kb.bit.G == 1)
    {
        key_f_status=0;
    }
    if(rc_now->sw1 != RC_MI)
    {
        if(rc_f_status == 1)
        {
            rc_f_status = 0;
        }
    }




    /*!------------------------------------------------------------扳机连发模式---------------------------------------------------------*/
    //自瞄模式下，开启自动扳机
    // if(gim_cmd.ctrl_mode==GIMBAL_AUTO)
    // {    //单发模式
    //     if((rc_now->mouse.l==1||rc_now->wheel >= 300)&&gim_cmd.ctrl_mode==GIMBAL_AUTO
    //          && shoot_cmd.friction_status==1)
    //     {
    //         if(trans_fdb.roll == 1)
    //         {
    //             shoot_cmd.ctrl_mode = SHOOT_ONE;
    //             shoot_cmd.trigger_status = TRIGGER_ON;
    //         }
    //         else
    //         {
    //             if(shoot_fdb.trigger_status == SHOOT_OK)
    //             {
    //                 shoot_cmd.ctrl_mode = SHOOT_ONE;
    //                 shoot_cmd.trigger_status = TRIGGER_OFF;
    //             }
    //
    //         }
    //     }//连发模式
    //     else if((rc_now->mouse.l==1||rc_now->wheel <= -300)&&gim_cmd.ctrl_mode==GIMBAL_AUTO
    //          && shoot_cmd.friction_status==1 )
    //     {
    //         if(trans_fdb.roll == 1)
    //         {
    //             shoot_cmd.ctrl_mode = SHOOT_COUNTINUE;
    //             shoot_cmd.shoot_freq = 2000;
    //         }
    //         else
    //         {
    //             shoot_cmd.ctrl_mode = SHOOT_STOP;
    //             shoot_cmd.shoot_freq = 0;
    //         }
    //     }
    // }
    // else

    //连发模式
    if((km.v_sta != KEY_RELEASE && km.v_sta != KEY_WAIT_EFFECTIVE)
       ||(rc_now->wheel <= -300)
        /*&&(referee_fdb.power_heat_data.shooter_17mm_1_barrel_heat < (referee_fdb.robot_status.shooter_barrel_heat_limit-10))*/)
    {
        if(shoot_cmd.friction_status==1)
        {
            shoot_cmd.ctrl_mode=SHOOT_COUNTINUE;
            shoot_cmd.shoot_freq=2000;
        }
    }
        //开启摩擦轮默认进入单发模式,首先判断鼠标左键键是否按下或者拨轮是否向下，标记开火标志位
    else if(km.lk_sta == KEY_PRESS_ONCE || rc_now->wheel>=400 &&(shoot_cmd.friction_status==1))
    {
        trigger_flag=1;
        shoot_cmd.ctrl_mode=SHOOT_ONE;
        shoot_cmd.trigger_status=TRIGGER_OFF;
    }
        //当V键松开时或拨轮恢复到0时，根据标志位判断状态
    else if(km.lk_sta == KEY_RELEASE || rc_now->wheel ==0 && (shoot_cmd.friction_status==1))
    {
        //当shoot线程反馈信息显示开火完成后，清零开火标志位（记得在shoot线程shoot_stop状态将反馈信息设置为SHOOT_WAITNG）
        if(shoot_fdb.trigger_status == SHOOT_OK)
        {
            trigger_flag=0;
        }
        //如果开火标志位等于0，不开火
        if(trigger_flag ==0)
        {
            shoot_cmd.ctrl_mode=SHOOT_ONE;
            shoot_cmd.trigger_status=TRIGGER_OFF;
        }
            //如果开火标志位等于1，表示进入单发模式，开火
        else if(trigger_flag == 1)
        {
            shoot_cmd.ctrl_mode=SHOOT_ONE;
            shoot_cmd.trigger_status=TRIGGER_ON;
        }
    }
    else
    {
        shoot_cmd.ctrl_mode=SHOOT_STOP;
        shoot_cmd.shoot_freq=0;
    }


    /*-------------------------------------------------------------堵弹反转检测------------------------------------------------------------*/
    if (shoot_fdb.trigger_motor_current>=16300)/*M3508电机的堵转电流是2500*/
    {
        reverse_cnt++;
        if (reverse_cnt<300)
            reverse_cnt++;
        else
        {
            reverse_cnt=0;
            shoot_cmd.ctrl_mode=SHOOT_REVERSE;
        }

    }

    // /*-----------------------------------------------------------舵机开盖关盖--------------------------------------------------------------*/
    // if(rc_now->kb.bit.R==1||rc_now->wheel<=-200)
    // {
    //     shoot_cmd.cover_open=1;
    // }
    // else
    // {
    //     shoot_cmd.cover_open=0;
    // }
    /*-----------------------------------------------------------倍镜舵机控制--------------------------------------------------------------*/
    //在未开启摩擦轮时，向上拨轮选择是否旋转倍镜
    if(rc_now->kb.bit.R==1||rc_now->wheel <= -600 && shoot_cmd.friction_status==0)
    {
        mirror_servo_flag = 1;//旋转倍镜标志位
    }
        //当拨轮恢复到0时，根据旋转倍镜标志位判断是否旋转倍镜
    else if(rc_now->wheel == 0 && mirror_servo_flag==1)
    {
        mirror_servo_flag = 0;
        if(shoot_cmd.mirror_enable==1)
            shoot_cmd.mirror_enable=0;
        else
            shoot_cmd.mirror_enable=1;
    }

    /*--------------------------------------------------手动模式下清空自瞄传过来的角度buffer------------------------------------------------------*/
    if (gim_cmd.ctrl_mode==GIMBAL_GYRO)
    {
        memset(r_buffer_point,0,sizeof (*r_buffer_point));
    }
    /*----------------------------------------------------------------使能判断---------------------------------------------------------------*/

    //关闭云台接口，便于调试
    if(rc_now->wheel >= 600 && rc_now->sw2==RC_MI)
    {
        cnt_flag++;
        if(cnt_flag >=800)
        {
            cnt_flag =0;
            if(deploy_flag == 0)
                deploy_flag = 1;
            else if(deploy_flag == 1)
                deploy_flag =0;
        }
    }
    else
    {
        cnt_flag =0;
    }
    if(deploy_flag == 1)
    {
        chassis_cmd.ctrl_mode = CHASSIS_RELAX;
    }


    //TODO:使能判断放最后，防止抽风
    if (rc_now->sw2==RC_UP)
    {
        gim_cmd.ctrl_mode = GIMBAL_RELAX;
        chassis_cmd.ctrl_mode = CHASSIS_RELAX;
        shoot_cmd.ctrl_mode=SHOOT_STOP;
        shoot_cmd.friction_status = 0; //在失能状态下，摩擦轮也不能开启，从而也进一步保证拨弹盘不能被开启
        /*放开状态下，gim不接收值*/
        gim_cmd.pitch=0;
        gim_cmd.yaw=0;
        gyro_yaw_inherit=0;
        gyro_pitch_inherit=0;
        /*案件状态标志位重置*/
        key_e_status=-1;
        key_f_status=-1;
        memset(r_buffer_point,0,sizeof (*r_buffer_point));
    }

}

#else
void remote_to_cmd_sbus(void)
{
    /* 保存上一次数据 */
    gim_cmd.last_mode = gim_cmd.ctrl_mode;
    chassis_cmd.last_mode = chassis_cmd.ctrl_mode;
    shoot_cmd.last_mode=shoot_cmd.ctrl_mode;
    xSemaphoreTake(sbus_cmd_mutex, portMAX_DELAY);
    sbus_data_t tmp_data = sbus_data_fdb;  // 复制到临时变量
    xSemaphoreGive(sbus_cmd_mutex);


//    float fx=First_Order_Filter_Calculate(&mouse_x_lpf,rc_now->mouse.x);
//    float fy=First_Order_Filter_Calculate(&mouse_y_lpf,rc_now->mouse.y);
//
//    Ballistic += First_Order_Filter_Calculate(&mouse_y_lpf,rc_now->mouse.y)*0.05f;
// TODO: 目前状态机转换较为简单，有很多优化和改进空间
//遥控器的控制信息转化为标准单位，平移为(mm/s)旋转为(degree/s)
    /*底盘命令*/
    chassis_cmd.vx =  (float)tmp_data.ch1 * CHASSIS_RC_MOVE_RATIO_X / RC_MAX_VALUE * MAX_CHASSIS_VX_SPEED; //+ km.vx * CHASSIS_PC_MOVE_RATIO_X;
    chassis_cmd.vy =  (float)tmp_data.ch3 * CHASSIS_RC_MOVE_RATIO_Y / RC_MAX_VALUE * MAX_CHASSIS_VY_SPEED; //+ km.vy * CHASSIS_PC_MOVE_RATIO_Y;
    chassis_cmd.vw =  (float)tmp_data.ch4 * CHASSIS_RC_MOVE_RATIO_R / RC_MAX_VALUE * MAX_CHASSIS_VR_SPEED; //+ rc_now->mouse.x * CHASSIS_PC_MOVE_RATIO_R;

    chassis_cmd.offset_angle = gim_fdb.yaw_relative_angle;

    /*云台命令*/
    if (gim_cmd.ctrl_mode==GIMBAL_GYRO)
    {
        gim_cmd.yaw +=   (float)tmp_data.ch4 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW; //+ fx * KB_RATIO * GIMBAL_PC_MOVE_RATIO_YAW;
        gim_cmd.pitch += (float)tmp_data.ch2 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_PIT; //- fy * KB_RATIO * GIMBAL_PC_MOVE_RATIO_PIT;
        gyro_yaw_inherit =gim_cmd.yaw;
        gyro_pitch_inherit =gim_cmd.pitch;
//        mouse_accumulate_x=0;
//        mouse_accumulate_y=0;
    }
    if (gim_cmd.ctrl_mode==GIMBAL_AUTO)
    {
        if (auto_relative_angle_status==RELATIVE_ANGLE_TRANS)
        {
            trans_fdb.yaw=0;
            trans_fdb.pitch=0;
        }
        //mouse_accumulate_x+=fx * KB_RATIO * GIMBAL_PC_MOVE_RATIO_YAW; /*鼠标x轴的自瞄补偿，测试结果建议注释掉暂不使用*/
        //mouse_accumulate_y-=fy * KB_RATIO * GIMBAL_PC_MOVE_RATIO_PIT;  y轴自瞄补偿，建议加上，目前未添加
        if(trans_fdb.roll != 2)
        {
            gim_cmd.yaw = trans_fdb.yaw+gyro_yaw_inherit /*+ mouse_accumulate_x  + 150 * rc_now->ch3 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW*/;//上位机自瞄
            gim_cmd.pitch = trans_fdb.pitch+gyro_pitch_inherit /*+ mouse_accumulate_y +100 * rc_now->ch4 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_PIT */;//上位机自瞄
        }

    }

    /*!限制云台pitch轴角度 */
    VAL_LIMIT(gim_cmd.pitch, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
    /*开环状态和遥控器归中*/
    if (gim_cmd.ctrl_mode==GIMBAL_INIT||gim_cmd.ctrl_mode==GIMBAL_RELAX)
    {
        gim_cmd.pitch=0;
        gim_cmd.yaw=0;
    }
    /*-------------------------------------------------底盘_云台状态机---------------------------------------------------------*/


    /*TODO:手动模式和自瞄模式状态机*/
    /*!如果鼠标未按下右键*/
//    if (rc_now->mouse.r==0)
//    {
        if (gim_cmd.last_mode == GIMBAL_RELAX)
        {/* 判断上次状态是否为RELAX，是则先归中 */
            gim_cmd.ctrl_mode = GIMBAL_INIT;
            chassis_cmd.ctrl_mode=CHASSIS_RELAX;

        }
        else
        {
            if (gim_fdb.back_mode == BACK_IS_OK)  /*!是否归中完成*/
            {
                gim_cmd.ctrl_mode = GIMBAL_GYRO;
                chassis_cmd.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;

                //上位机通信的环形缓存区
                memset(r_buffer_point,0,sizeof (*r_buffer_point));
            }
            else if(gim_fdb.back_mode==BACK_STEP)
            {
                chassis_cmd.ctrl_mode=CHASSIS_RELAX;
                gim_cmd.ctrl_mode=GIMBAL_INIT;
            }
        }
//    }

    if (/*rc_now->mouse.r==1||*/tmp_data.sw3==RC_DN) /*!如果鼠标按下右键或者遥控器选择自瞄模式*/
    {
        if (gim_cmd.last_mode == GIMBAL_RELAX)
        {/* 判断上次状态是否为RELAX，是则先归中 */
            gim_cmd.ctrl_mode = GIMBAL_INIT;
            chassis_cmd.ctrl_mode=CHASSIS_RELAX;
        }
        else
        {
            if (gim_fdb.back_mode == 1)
//            if (gim_fdb.back_mode == BACK_IS_OK)
            {/* 判断归中是否完成 */
                gim_cmd.ctrl_mode = GIMBAL_AUTO;
                chassis_cmd.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;
            }
            else if(gim_fdb.back_mode==BACK_STEP)
            {
                chassis_cmd.ctrl_mode=CHASSIS_RELAX;
                gim_cmd.ctrl_mode=GIMBAL_INIT;
            }
        }
    }

    /* -------------初始化ui按键B-----------------*/
//    if(km.b_sta == KEY_PRESS_DOWN){
//
//        ui_cmd.ui_init = 1;//B键被按下满足ui初始化条件
//
//    }else{
//
//        ui_cmd.ui_init = 0;
//
//    }


    /*TODO:小陀螺*/
    //开小陀螺
//    if(km.e_sta==KEY_PRESS_ONCE)
//    {
//        key_e_status=1;
//    }
    if ( /*key_e_status==1||*/tmp_data.sw2==RC_DN)
    {
        if (gim_fdb.back_mode==BACK_IS_OK)
        {
            chassis_cmd.ctrl_mode=CHASSIS_SPIN;
        }
        else
        {
            chassis_cmd.ctrl_mode=CHASSIS_RELAX;
            gim_cmd.ctrl_mode=GIMBAL_INIT;
        }
    }
    //关小陀螺
//    if(rc_now->kb.bit.Q == 1 )
//    {
//        key_e_status=0;
//    }
//    if ( key_e_status==0)
//    {
//        chassis_cmd.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;
//    }

    if (chassis_cmd.ctrl_mode==CHASSIS_SPIN)
    {
        chassis_cmd.vw=3;// * msg_cmd->robot_status.chassis_power_limit/55;/*!小陀螺转速，随着功率限制提升加快转速*/
        if(chassis_fdb.vw_ch < chassis_cmd.vw*0.85f) //当小陀螺被堵住时，自动退出小陀螺模式
        {
            spin_cnt++;
            if(spin_cnt>2000)
            {
                chassis_cmd.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;
                spin_cnt=0;
            }
        }
        else
        {
            spin_cnt =0;
        }

    }

    /*------------------------------------------------发射模块状态机-------------------------------------------------------*/
    /*-----------------------------------------开关摩擦轮--------------------------------------------*/

//    if(km.f_sta==KEY_PRESS_ONCE)
//    {
//        key_f_status=1;
//    }
    if(tmp_data.sw1==RC_DN)
    {
        rc_f_status = 1;
    }
    if ( key_f_status==1||rc_f_status == 1)
    {
        shoot_cmd.friction_status=1;
    }
    else
    {
        shoot_cmd.friction_status = 0;
    }

    //关摩擦轮
//    if(rc_now->kb.bit.G == 1)
//    {
//        key_f_status=0;
//    }
    if(tmp_data.sw1 == RC_UP)
    {
        if(rc_f_status == 1)
        {
            rc_f_status = 0;
        }
    }



    /*-----------------------------------------扳机连发模式------------------------------------------*/

        //开启摩擦轮默认进入单发模式,首先判断鼠标左键键是否按下或者拨轮是否向下，标记开火标志位
    if(/*km.lk_sta == KEY_PRESS_ONCE ||*/ tmp_data.sw4 == RC_DN &&(shoot_cmd.friction_status==1))
    {
        trigger_flag=1;
        shoot_cmd.ctrl_mode=SHOOT_ONE;
        shoot_cmd.trigger_status=TRIGGER_OFF;
    }
        //当V键松开时或拨轮恢复到0时，根据标志位判断状态
    else if(/*km.lk_sta == KEY_RELEASE ||*/ tmp_data.sw4 == RC_UP && (shoot_cmd.friction_status==1))
    {
        //当shoot线程反馈信息显示开火完成后，清零开火标志位（记得在shoot线程shoot_stop状态将反馈信息设置为SHOOT_WAITNG）
        if(shoot_fdb.trigger_status == SHOOT_OK)
        {
            trigger_flag=0;
        }
        //如果开火标志位等于0，不开火
        if(trigger_flag ==0)
        {
            shoot_cmd.ctrl_mode=SHOOT_ONE;
            shoot_cmd.trigger_status=TRIGGER_OFF;
        }
        //如果开火标志位等于1，表示进入单发模式，开火
        else if(trigger_flag == 1)
        {
            shoot_cmd.ctrl_mode=SHOOT_ONE;
            shoot_cmd.trigger_status=TRIGGER_ON;
        }
    }
    else
    {
        shoot_cmd.ctrl_mode=SHOOT_STOP;
        shoot_cmd.shoot_freq=0;
    }


    /*--------------------------------------------堵弹反转检测-------------------------------------*/
    if (shoot_fdb.trigger_motor_current>=16300)/*M3508电机的堵转电流是2500*/
    {
        reverse_cnt++;
        if (reverse_cnt<300)
            reverse_cnt++;
        else
        {
            reverse_cnt=0;
            shoot_cmd.ctrl_mode=SHOOT_REVERSE;
        }

    }


    /*------------------------------------------倍镜舵机控制-----------------------------------------*/


    /*----------------------------------------手动模式下清空自瞄传过来的角度buffer------------------------------------------*/
    if (gim_cmd.ctrl_mode==GIMBAL_GYRO)
    {
        memset(r_buffer_point,0,sizeof (*r_buffer_point));
    }


    /*------------------------------------------------使能判断----------------------------------------------------------*/

    //TODO:使能判断放最后，防止抽风
    if (tmp_data.sw3 == RC_UP)
    {
        gim_cmd.ctrl_mode = GIMBAL_RELAX;
        chassis_cmd.ctrl_mode = CHASSIS_RELAX;
        shoot_cmd.ctrl_mode=SHOOT_STOP;
        shoot_cmd.friction_status = 0; //在失能状态下，摩擦轮也不能开启，从而也进一步保证拨弹盘不能被开启
        /*放开状态下，gim不接收值*/
        gim_cmd.pitch=0;
        gim_cmd.yaw=0;
        gyro_yaw_inherit=0;
        gyro_pitch_inherit=0;
        /*案件状态标志位重置*/
        key_e_status=-1;
        key_f_status=-1;
        memset(r_buffer_point,0,sizeof (*r_buffer_point));
    }

}

#endif
