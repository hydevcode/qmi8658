/*
* Copyright (c) 2006-2025 RT-Thread Development Team
*
* SPDX-License-Identifier: Apache-2.0
*
* Change Logs:
* Date           Author       Notes
* 2025-04-24     Hydevcode    the first version
*/
#ifndef QMI8658_H
#define QMI8658_H

#include <stdio.h>
#include <string.h>
#include <rthw.h>
#include <rtthread.h>

#define QMI8658_SLAVE_ADDR_L            0x6a
#define QMI8658_SLAVE_ADDR_H            0x6b

#define QMI8658_DISABLE_ALL             (0x0)
#define QMI8658_ACC_ENABLE              (0x1)
#define QMI8658_GYR_ENABLE              (0x2)
#define QMI8658_ACCGYR_ENABLE           (QMI8658_ACC_ENABLE | QMI8658_GYR_ENABLE)

#define M_PI            (3.14159265358979323846f)
#define ONE_G           (9.807f)
#define QFABS(x)        (((x)<0.0f)?(-1.0f*(x)):(x))


#define PKG_QMI_USING_8658A
#if defined(PKG_QMI_USING_8658A)
enum Qmi8658AReg
{
    Register_WhoAmI = 0,
    Register_Revision,
    Register_Ctrl1,
    Register_Ctrl2,
    Register_Ctrl3,
    Register_Reserved,
    Register_Ctrl5,
    Register_Reserved1,
    Register_Ctrl7,
    Register_Ctrl8,
    Register_Ctrl9,
    Register_Cal1_L = 11,
    Register_Cal1_H,
    Register_Cal2_L,
    Register_Cal2_H,
    Register_Cal3_L,
    Register_Cal3_H,
    Register_Cal4_L,
    Register_Cal4_H,
    Register_FifoWmkTh = 19,
    Register_FifoCtrl = 20,
    Register_FifoCount = 21,
    Register_FifoStatus = 22,
    Register_FifoData = 23,
    Register_StatusInt = 45,
    Register_Status0,
    Register_Status1,
    Register_Timestamp_L = 48,
    Register_Timestamp_M,
    Register_Timestamp_H,
    Register_Tempearture_L = 51,
    Register_Tempearture_H,
    Register_Ax_L = 53,
    Register_Ax_H,
    Register_Ay_L,
    Register_Ay_H,
    Register_Az_L,
    Register_Az_H,
    Register_Gx_L = 59,
    Register_Gx_H,
    Register_Gy_L,
    Register_Gy_H,
    Register_Gz_L,
    Register_Gz_H,
    Register_COD_Status = 70,
    Register_dQW_L = 73,
    Register_dQW_H,
    Register_dQX_L,
    Register_dQX_H,
    Register_dQY_L,
    Register_dQY_H,
    Register_dQZ_L,
    Register_dQZ_H,
    Register_dVX_L,
    Register_dVX_H,
    Register_dVY_L,
    Register_dVY_H,
    Register_dVZ_L,
    Register_dVZ_H,

    Register_TAP_Status = 89,
    Register_Step_Cnt_L = 90,
    Register_Step_Cnt_M = 91,
    Register_Step_Cnt_H = 92,

    Register_Reset = 96
};

enum Ctrl9Command
{
    Ctrl9_Cmd_Ack                   = 0X00,
    Ctrl9_Cmd_RstFifo               = 0X04,
    Ctrl9_Cmd_ReqFifo               = 0X05,//Get FIFO data from Device
    Ctrl9_Cmd_WoM_Setting           = 0x08,// 设置并启用运动唤醒
    Ctrl9_Cmd_AccelHostDeltaOffset  = 0x09,//更改加速度计偏移
    Ctrl9_Cmd_GyroHostDeltaOffset   = 0x0A,//更改陀螺仪偏移
    Ctrl9_Cmd_CfgTap                = 0x0C,//配置TAP检测
    Ctrl9_Cmd_CfgPedometer          = 0x0D,//配置计步器
    Ctrl9_Cmd_Motion                = 0x0E,//配置任何运动/无运动/显着运动检测
    Ctrl9_Cmd_RstPedometer          = 0x0F,//重置计步器计数（步数）
    Ctrl9_Cmd_CopyUsid              = 0x10,//将 USID 和 FW 版本复制到 UI 寄存器
    Ctrl9_Cmd_SetRpu                = 0x11,//配置 IO 上拉
    Ctrl9_Cmd_AHBClockGating        = 0x12,//内部 AHB 时钟门控开关
    Ctrl9_Cmd_OnDemandCalivration   = 0xA2,//陀螺仪按需校准
    Ctrl9_Cmd_ApplyGyroGains        = 0xAA//恢复保存的陀螺仪增益
};
#else
enum Qmi8658Register
{
    Register_WhoAmI = 0,
    Register_Revision,
    Register_Ctrl1,
    Register_Ctrl2,
    Register_Ctrl3,
    Register_Ctrl4,
    Register_Ctrl5,
    Register_Ctrl6,
    Register_Ctrl7,
    Register_Ctrl8,
    Register_Ctrl9,
    Register_Cal1_L = 11,
    Register_Cal1_H,
    Register_Cal2_L,
    Register_Cal2_H,
    Register_Cal3_L,
    Register_Cal3_H,
    Register_Cal4_L,
    Register_Cal4_H,
    Register_FifoWmkTh = 19,
    Register_FifoCtrl = 20,
    Register_FifoCount = 21,
    Register_FifoStatus = 22,
    Register_FifoData = 23,
    Register_StatusI2CM = 44,
    Register_StatusInt = 45,
    Register_Status0,
    Register_Status1,
    Register_Timestamp_L = 48,
    Register_Timestamp_M,
    Register_Timestamp_H,
    Register_Tempearture_L = 51,
    Register_Tempearture_H,
    Register_Ax_L = 53,
    Register_Ax_H,
    Register_Ay_L,
    Register_Ay_H,
    Register_Az_L,
    Register_Az_H,
    Register_Gx_L = 59,
    Register_Gx_H,
    Register_Gy_L,
    Register_Gy_H,
    Register_Gz_L,
    Register_Gz_H,
    Register_Mx_L = 65,
    Register_Mx_H,
    Register_My_L,
    Register_My_H,
    Register_Mz_L,
    Register_Mz_H,
    Register_firmware_id = 73,
    Register_uuid = 81,

    Register_Pedo_L = 90,
    Register_Pedo_M = 91,
    Register_Pedo_H = 92,

    Register_Reset = 96
};

enum qmi8658_Ois_Register
{
    OIS_Reg_Ctrl1 = 0x02,
    OIS_Reg_Ctrl2,
    OIS_Reg_Ctrl3,
    OIS_Reg_Ctrl5   = 0x06,
    OIS_Reg_Ctrl7   = 0x08,
    OIS_Reg_StatusInt = 0x2D,
    OIS_Reg_Status0   = 0x2E,
    OIS_Reg_Ax_L  = 0x33,
    OIS_Reg_Ax_H,
    OIS_Reg_Ay_L,
    OIS_Reg_Ay_H,
    OIS_Reg_Az_L,
    OIS_Reg_Az_H,

    OIS_Reg_Gx_L  = 0x3B,
    OIS_Reg_Gx_H,
    OIS_Reg_Gy_L,
    OIS_Reg_Gy_H,
    OIS_Reg_Gz_L,
    OIS_Reg_Gz_H,
};

enum qmi8658_Ctrl9Command
{
    Ctrl9_Cmd_NOP                   = 0X00,
    Ctrl9_Cmd_GyroBias              = 0X01,
    Ctrl9_Cmd_Rqst_Sdi_Mod          = 0X03,
    Ctrl9_Cmd_Rst_Fifo              = 0X04,
    Ctrl9_Cmd_Req_Fifo              = 0X05,
    Ctrl9_Cmd_I2CM_Write            = 0X06,
    Ctrl9_Cmd_WoM_Setting           = 0x08,
    Ctrl9_Cmd_AccelHostDeltaOffset  = 0x09,
    Ctrl9_Cmd_GyroHostDeltaOffset   = 0x0A,
    Ctrl9_Cmd_EnableExtReset        = 0x0B,
    Ctrl9_Cmd_EnableTap             = 0x0C,
    Ctrl9_Cmd_EnablePedometer       = 0x0D,
    Ctrl9_Cmd_Motion                = 0x0E,
    Ctrl9_Cmd_CopyUsid              = 0x10,
    Ctrl9_Cmd_SetRpu                = 0x11,
    Ctrl9_Cmd_On_Demand_Cali        = 0xA2,
    Ctrl9_Cmd_Dbg_WoM_Data_Enable   = 0xF8
};
#endif

enum qmi8658_LpfConfig
{
    Qmi8658Lpf_Disable,
    Qmi8658Lpf_Enable
};

enum qmi8658_HpfConfig
{
    Qmi8658Hpf_Disable,
    Qmi8658Hpf_Enable
};

enum qmi8658_StConfig
{
    Qmi8658St_Disable,
    Qmi8658St_Enable
};

enum qmi8658_LpfMode
{
    A_LSP_MODE_0 = 0x00<<1,
    A_LSP_MODE_1 = 0x01<<1,
    A_LSP_MODE_2 = 0x02<<1,
    A_LSP_MODE_3 = 0x03<<1,

    G_LSP_MODE_0 = 0x00<<5,
    G_LSP_MODE_1 = 0x01<<5,
    G_LSP_MODE_2 = 0x02<<5,
    G_LSP_MODE_3 = 0x03<<5
};

enum qmi_AccRange
{
    Qmi8658AccRange_2g = 0x00 << 4,
    Qmi8658AccRange_4g = 0x01 << 4,
    Qmi8658AccRange_8g = 0x02 << 4,
    Qmi8658AccRange_16g = 0x03 << 4
};

enum qmi_AccOdr
{
    Qmi8658AccOdr_8000Hz = 0x00,
    Qmi8658AccOdr_4000Hz = 0x01,
    Qmi8658AccOdr_2000Hz = 0x02,
    Qmi8658AccOdr_1000Hz = 0x03,
    Qmi8658AccOdr_500Hz = 0x04,
    Qmi8658AccOdr_250Hz = 0x05,
    Qmi8658AccOdr_125Hz = 0x06,
    Qmi8658AccOdr_62_5Hz = 0x07,
    Qmi8658AccOdr_31_25Hz = 0x08,
    Qmi8658AccOdr_LowPower_128Hz = 0x0c,
    Qmi8658AccOdr_LowPower_21Hz = 0x0d,
    Qmi8658AccOdr_LowPower_11Hz = 0x0e,
    Qmi8658AccOdr_LowPower_3Hz = 0x0f
};

enum qmi_GyrRange
{
    Qmi8658GyrRange_16dps = 0 << 4,
    Qmi8658GyrRange_32dps = 1 << 4,
    Qmi8658GyrRange_64dps = 2 << 4,
    Qmi8658GyrRange_128dps = 3 << 4,
    Qmi8658GyrRange_256dps = 4 << 4,
    Qmi8658GyrRange_512dps = 5 << 4,
    Qmi8658GyrRange_1024dps = 6 << 4,
    Qmi8658GyrRange_2048dps = 7 << 4
};

enum qmi_GyrOdr
{
    Qmi8658GyrOdr_8000Hz = 0x00,
    Qmi8658GyrOdr_4000Hz = 0x01,
    Qmi8658GyrOdr_2000Hz = 0x02,
    Qmi8658GyrOdr_1000Hz = 0x03,
    Qmi8658GyrOdr_500Hz = 0x04,
    Qmi8658GyrOdr_250Hz = 0x05,
    Qmi8658GyrOdr_125Hz = 0x06,
    Qmi8658GyrOdr_62_5Hz    = 0x07,
    Qmi8658GyrOdr_31_25Hz   = 0x08
};

enum qmi8658_AccUnit
{
    Qmi8658AccUnit_g,
    Qmi8658AccUnit_ms2
};

enum qmi8658_GyrUnit
{
    Qmi8658GyrUnit_dps,
    Qmi8658GyrUnit_rads
};

enum qmi8658_FifoMode
{
    qmi8658_Fifo_Bypass = 0,
    qmi8658_Fifo_Fifo = 1,
    qmi8658_Fifo_Stream = 2,
    qmi8658_Fifo_StreamToFifo = 3
};


enum qmi8658_FifoWmkLevel
{
    qmi8658_Fifo_WmkEmpty =         (0 << 4),
    qmi8658_Fifo_WmkOneQuarter =    (1 << 4),
    qmi8658_Fifo_WmkHalf =          (2 << 4),
    qmi8658_Fifo_WmkThreeQuarters = (3 << 4)
};

enum qmi8658_FifoSize
{
    qmi8658_Fifo_16 = (0 << 2),
    qmi8658_Fifo_32 = (1 << 2),
    qmi8658_Fifo_64 = (2 << 2),
    qmi8658_Fifo_128 = (3 << 2)
};

enum qmi8658_Interrupt
{
    qmi8658_Int_none,
    qmi8658_Int1,
    qmi8658_Int2,

    qmi8658_Int_total
};

enum qmi8658_InterruptState
{
    Qmi8658State_high = (1 << 7),
    Qmi8658State_low  = (0 << 7)
};

enum qmi8658_set_cmd
{
    QMI8658_DEVICE_IDENTIFIER,
    QMI8658_REVISION_ID,
    QMI8658_CTRL1,
    QMI8658_ACCE_CONFIG,
    QMI8658_GYRO_CONFIG,
    QMI8658_LPF_CONFIG,

    QMI_Register_Ctrl3,
    QMI_Register_Ctrl7,
};
typedef enum qmi8658_set_cmd qmi8658_set_cmd_t;

typedef struct
{
    unsigned char           enSensors;
    enum qmi_AccRange   accRange;
    enum qmi_AccOdr     accOdr;
    enum qmi_GyrRange   gyrRange;
    enum qmi_GyrOdr     gyrOdr;
    rt_uint8_t          ctrl8_value;
} qmi8658_config;

typedef struct
{
    rt_uint8_t addr;
    rt_uint16_t ssvt_a;
    rt_uint16_t ssvt_g;
    rt_uint32_t timestamp;
    rt_uint32_t step;
    rt_uint16_t imu[6];
} qmi8658_state;

struct qmi8658_device
{
    rt_device_t     bus;
    qmi8658_state   *state_t;
    qmi8658_config  *cfg_t;
};
typedef struct qmi8658_device *qmi8658_device_t;

rt_err_t qmi8658_set_param(qmi8658_device_t dev, qmi8658_set_cmd_t cmd, uint8_t value);
rt_err_t qmi8658_get_param(qmi8658_device_t dev, qmi8658_set_cmd_t cmd, rt_uint8_t *value);

qmi8658_device_t qmi8658_init(const char *i2c_bus_name,rt_uint8_t dev_addr);
void qmi8658_deinit(qmi8658_device_t dev);

void qmi8658_sample(void);

#endif