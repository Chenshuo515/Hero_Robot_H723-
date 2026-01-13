/*
 * @file    supercap_task.c
 * @brief   超级电容通信任务
 * @note
 */

#include "supercap_task.h"
#include "rm_config.h"
#include "rm_algorithm.h"
#include "rm_module.h"
#include "rm_task.h"

/* ================================ 变量定义 ================================ */

/* 发送/接收数据结构 */
static SuperCapTxDataTypeDef supercap_tx;
static SuperCapRxDataTypeDef supercap_rx;

/* 控制指令(订阅) */
static struct supercap_cmd_msg cap_cmd = {
    .charge_cmd   = 1,
    .enable_cmd   = 0,
    .power_limit  = 60,   // 默认60W
    .charge_power = 30,   // 默认30W
};

/* 反馈数据(发布) */
static struct supercap_fdb_msg cap_fdb;

/* 订阅者/发布者 */
static publisher_t  *pub_cap;
static subscriber_t *sub_cap;

/* CAN设备和消息 */

// supercap_can_msg的can控制帧
struct supercap_can_msg
{
    uint32_t id;
    uint8_t data[8];
};

static struct supercap_can_msg supercap_tx_msg= {
    .id   = CAN_ID_BUS_TO_SUPERCAP,
    .data = {0, 0, 0, 0, 0, 0, 0, 0}
};

/* 时间和状态 */
static uint32_t last_rx_time_ms = 0;
static uint32_t online_flag     = 0;
static uint32_t cnt_flag   = 0;


/* ================================ 函数实现 ================================ */

/**
 * @brief 填充发送帧并直接发送
 * @note  发送顺序：[Enable, Charge, PowerLimit, ChargePower, 0, 0, 0, 0]
 */
static void fill_and_send_tx_frame(void)
{
    /* 从控制指令更新发送数据 */
    supercap_tx.Enable      = cap_cmd.enable_cmd;
    supercap_tx.Charge      = cap_cmd.charge_cmd;
    supercap_tx.PowerLimit  = cap_cmd.power_limit;
    supercap_tx.ChargePower = cap_cmd.charge_power;

    /* 按超电协议顺序填充CAN帧 */
    supercap_tx_msg.data[0] = supercap_tx.Enable;       // 字节0: 使能
    supercap_tx_msg.data[1] = supercap_tx.Charge;       // 字节1: 充放电
    supercap_tx_msg.data[2] = supercap_tx.PowerLimit;   // 字节2: 功率限制
    supercap_tx_msg.data[3] = supercap_tx.ChargePower;  // 字节3: 充电功率
    supercap_tx_msg.data[4] = 0;
    supercap_tx_msg.data[5] = 0;
    supercap_tx_msg.data[6] = 0;
    supercap_tx_msg.data[7] = 0;

    /* 直接发送CAN帧 */
    if(fdcanx_send_data(&hfdcan1,supercap_tx_msg.id,supercap_tx_msg.data,8)==0)
        cnt_flag++;

}

/**
 * @brief 处理接收到的超电数据
 * @param data CAN帧数据指针
 * @note  接收顺序：[Ready, State, Energy, ChassisPower, VoltageBat, DebugBat, DebugCap, Reserved]
 */
static void handle_rx_payload(const uint8_t *data)
{
    /* 更新接收时间和在线标志 */
    last_rx_time_ms = dwt_get_time_ms();
    online_flag     = 1;

    /* 按超电协议顺序解析 */
    supercap_rx.SuperCapReady          = (SuperCapReadyTypeDef)data[0];   // 字节0: 就绪状态
    supercap_rx.SuperCapState          = (SuperCapStateTypeDef)data[1];   // 字节1: 工作状态
    supercap_rx.SuperCapEnergy         = data[2];                          // 字节2: 能量百分比
    supercap_rx.ChassisPower           = data[3];                          // 字节3: 底盘功率
    supercap_rx.VoltageBat             = data[4];                          // 字节4: 电池电压
    supercap_rx.DebugOut_BatPower      = data[5];                          // 字节5: 调试-电池功率
    supercap_rx.DebugOut_SuperCapPower = (int8_t)data[6];                  // 字节6: 调试-超电功率
}

/**
 * @brief CAN接收回调（供usr_callback.c调用）
 * @param dev    CAN设备
 * @param id     CAN ID
 * @param data   数据指针
 */
void supercap_can_rx_callback(uint32_t id, uint8_t *data)
{
    /* 只处理超电反馈帧 */
    if (id == CAN_ID_SUPERCAP_TO_BUS)
    {
        handle_rx_payload(data);
    }
}

/**
 * @brief 发布反馈数据
 */
static void publish_feedback(void)
{
    /* 填充反馈结构 */
    cap_fdb.energy_percent    = supercap_rx.SuperCapEnergy;
    cap_fdb.chassis_power_cap = supercap_rx.ChassisPower;
    cap_fdb.is_online         = online_flag;
    cap_fdb.is_ready          = (supercap_rx.SuperCapReady == SUPERCAP_READY_YES) ? 1 : 0;
    cap_fdb.is_charging       = (supercap_rx.SuperCapState == SUPERCAP_STATE_CHARGING) ? 1 : 0;

    /* 发布消息 */
    pub_push_msg(pub_cap, &cap_fdb);
}

/**
 * @brief 超级电容任务初始化
 */
void supercap_task_init(void)
{
    /* 创建发布者 */
    pub_cap = pub_register("cap_fdb", sizeof(struct supercap_fdb_msg));

    /* 创建订阅者 */
    sub_cap = sub_register("cap_cmd", sizeof(struct supercap_cmd_msg));


}

/**
 * @brief 超级电容任务主循环
 * @param argument 未使用
 */
/* USER CODE END Header_Supercap_Entry */
void Supercap_Entry(void const * argument)
{
    /* USER CODE BEGIN Supercap_Entry */
    /* Infinite loop */
    /* 等待系统稳定 */
    static float supercap_dt;
    static float supercap_start;

    /* 初始化 */
    supercap_task_init();
    for(;;)
    {
        supercap_start = dwt_get_time_ms();

        /* 1. 获取控制指令 */
        sub_get_msg(sub_cap, &cap_cmd);

        /* 2. 填充并发送CAN帧 */
        fill_and_send_tx_frame();

        /* 3. 在线状态判定 */
        if (dwt_get_time_ms() - last_rx_time_ms > SUPERCAP_OFFLINE_TIME)
        {
            online_flag = 0;
        }

        /* 4. 发布反馈数据 */
        publish_feedback();

        /* 5. 计算任务耗时（调试用） */
        supercap_dt = dwt_get_time_ms() - supercap_start;
        (void)supercap_dt;  // 避免未使用警告

        osDelay(SUPERCAP_TASK_PERIOD);
    }
    /* USER CODE END Supercap_Entry */
}

/* ================================ 兼容接口 ================================ */

/**
 * @brief 兼容旧接口 - 供dji_motor.c调用（可保留或删除）
 * @param data 接收数据
 * @note  如果dji_motor.c中仍调用此函数，保留此接口
 */
void supercap_rx_handler(uint8_t *data)
{
    handle_rx_payload(data);
}
