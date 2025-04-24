/*
* Copyright (c) 2006-2025 RT-Thread Development Team
*
* SPDX-License-Identifier: Apache-2.0
*
* Change Logs:
* Date           Author       Notes
* 2025-04-24     Hydevcode    the first version
*/
#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>
#include <finsh.h>

#include <string.h>
#include "qmi8658.h"

#ifdef PKG_USING_QMI8658

#define DBG_ENABLE
#define DBG_SECTION_NAME "qmi8658"
#define DBG_LEVEL DBG_LOG
#define DBG_COLOR
#include <rtdbg.h>


static rt_err_t write_reg(qmi8658_device_t dev, rt_uint8_t reg, rt_uint8_t data)
{
    rt_uint8_t buf[2];

    buf[0] = reg;
    buf[1] = data;

    if (rt_i2c_master_send((struct rt_i2c_bus_device *)dev->bus, dev->state_t->addr, 0, buf, 2) == 2)
        return RT_EOK;
    else
        return -RT_ERROR;
}

static rt_err_t read_regs(qmi8658_device_t dev, rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *buf)
{
    struct rt_i2c_msg msgs[2];

    msgs[0].addr = dev->state_t->addr;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf = &reg;
    msgs[0].len = 1;

    msgs[1].addr = dev->state_t->addr;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf = buf;
    msgs[1].len = len;

    if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, msgs, 2) == 2)
    {
        return RT_EOK;
    }
    else
    {
        LOG_E("Reading command error");
        return -RT_ERROR;
    }
}

void qmi8658_config_acc(qmi8658_device_t dev,enum qmi_AccRange range, enum qmi_AccOdr odr, enum qmi8658_LpfConfig lpfEnable, enum qmi8658_StConfig stEnable)
{
    unsigned char ctl_dada;

    switch(range)
    {
        case Qmi8658AccRange_2g:
        dev->state_t->ssvt_a = (1<<14);
            break;
        case Qmi8658AccRange_4g:
        dev->state_t->ssvt_a = (1<<13);
            break;
        case Qmi8658AccRange_8g:
        dev->state_t->ssvt_a = (1<<12);
            break;
        case Qmi8658AccRange_16g:
        dev->state_t->ssvt_a = (1<<11);
            break;
        default:
            range = Qmi8658AccRange_8g;
            dev->state_t->ssvt_a = (1<<12);
    }
    if(stEnable == Qmi8658St_Enable)
        ctl_dada = (unsigned char)range|(unsigned char)odr|0x80;
    else
        ctl_dada = (unsigned char)range|(unsigned char)odr;

    write_reg(dev,Register_Ctrl2, ctl_dada);
    read_regs(dev,Register_Ctrl5, 1,&ctl_dada);
    ctl_dada &= 0xf0;
    if(lpfEnable == Qmi8658Lpf_Enable)
    {
        ctl_dada |= A_LSP_MODE_3;
        ctl_dada |= 0x01;
    }
    else
    {
        ctl_dada &= ~0x01;
    }
    write_reg(dev,Register_Ctrl5,ctl_dada);

}
void qmi8658_read_sensor_data(qmi8658_device_t dev)
{
    unsigned char   buf_reg[12];
    short           raw_acc_xyz[3];
    short           raw_gyro_xyz[3];

    read_regs(dev,Register_Ax_L,12,buf_reg);
    raw_acc_xyz[0] = (short)((unsigned short)(buf_reg[1]<<8) |( buf_reg[0]));
    raw_acc_xyz[1] = (short)((unsigned short)(buf_reg[3]<<8) |( buf_reg[2]));
    raw_acc_xyz[2] = (short)((unsigned short)(buf_reg[5]<<8) |( buf_reg[4]));

    raw_gyro_xyz[0] = (short)((unsigned short)(buf_reg[7]<<8) |( buf_reg[6]));
    raw_gyro_xyz[1] = (short)((unsigned short)(buf_reg[9]<<8) |( buf_reg[8]));
    raw_gyro_xyz[2] = (short)((unsigned short)(buf_reg[11]<<8) |( buf_reg[10]));

    dev->state_t->imu[0] = raw_acc_xyz[0];
    dev->state_t->imu[1] = raw_acc_xyz[1];
    dev->state_t->imu[2] = raw_acc_xyz[2];
    dev->state_t->imu[3] = raw_gyro_xyz[0];
    dev->state_t->imu[4] = raw_gyro_xyz[1];
    dev->state_t->imu[5] = raw_gyro_xyz[2];
}

void qmi8658_read_xyz(qmi8658_device_t dev)
{
    unsigned char   status;
    unsigned char data_ready = 0;

    read_regs(dev,Register_Status0,1, &status);
    if(status&0x03)
    {
        data_ready = 1;
    }

    if(data_ready)
    {
        qmi8658_read_sensor_data(dev);

    }
    else
    {
        rt_kprintf("data ready fail!\n");
    }
}
void qmi8658_config_gyro(qmi8658_device_t dev,enum qmi_GyrRange range, enum qmi_GyrOdr odr, enum qmi8658_LpfConfig lpfEnable, enum qmi8658_StConfig stEnable)
{
    unsigned char ctl_dada;
    switch (range)
    {
        case Qmi8658GyrRange_16dps:
            dev->state_t->ssvt_g = 2048;
            break;
        case Qmi8658GyrRange_32dps:
        dev->state_t->ssvt_g = 1024;
            break;
        case Qmi8658GyrRange_64dps:
        dev->state_t->ssvt_g = 512;
            break;
        case Qmi8658GyrRange_128dps:
        dev->state_t->ssvt_g = 256;
            break;
        case Qmi8658GyrRange_256dps:
        dev->state_t->ssvt_g = 128;
            break;
        case Qmi8658GyrRange_512dps:
        dev->state_t->ssvt_g = 64;
            break;
        case Qmi8658GyrRange_1024dps:
        dev->state_t->ssvt_g = 32;
            break;
        case Qmi8658GyrRange_2048dps:
        dev->state_t->ssvt_g = 16;
            break;
        default:
            range = Qmi8658GyrRange_512dps;
            dev->state_t->ssvt_g = 64;
            break;
    }

    if(stEnable == Qmi8658St_Enable)
        ctl_dada = (unsigned char)range|(unsigned char)odr|0x80;
    else
        ctl_dada = (unsigned char)range | (unsigned char)odr;

    write_reg(dev,Register_Ctrl3,ctl_dada);

    read_regs(dev,Register_Ctrl5,1,&ctl_dada);
    ctl_dada &= 0x0f;
    if(lpfEnable == Qmi8658Lpf_Enable)
    {
        ctl_dada |= G_LSP_MODE_3;
        ctl_dada |= 0x10;
    }
    else
    {
        ctl_dada &= ~0x10;
    }
    write_reg(dev,Register_Ctrl5, ctl_dada);

}

static rt_err_t qmi8658_sensor_init(qmi8658_device_t dev)
{
    rt_err_t result = -RT_ERROR;

    RT_ASSERT(dev);

    rt_uint8_t dev_identifier;

    result=qmi8658_get_param(dev,QMI8658_DEVICE_IDENTIFIER,&dev_identifier);
    if(dev_identifier!=0x5)
    {
        goto __exit;
    }

    qmi8658_set_param(dev,QMI_Register_Ctrl7,QMI8658_DISABLE_ALL);

    write_reg(dev,Register_Ctrl1,0x40);

#if defined(PKG_QMI_MODE_LOW_POWER)
        dev->cfg_t->enSensors = QMI8658_ACC_ENABLE;
        dev->cfg_t->accRange = Qmi8658AccRange_8g;
        dev->cfg_t->accOdr = Qmi8658AccOdr_LowPower_21Hz;
        dev->cfg_t->gyrRange = Qmi8658GyrRange_1024dps;
        dev->cfg_t->gyrOdr = Qmi8658GyrOdr_250Hz;
#else
        dev->cfg_t->enSensors = QMI8658_ACCGYR_ENABLE;
        dev->cfg_t->accRange = Qmi8658AccRange_8g;
        dev->cfg_t->accOdr = Qmi8658AccOdr_250Hz;
        dev->cfg_t->gyrRange = Qmi8658GyrRange_1024dps;
        dev->cfg_t->gyrOdr = Qmi8658GyrOdr_250Hz;
#endif
    if(dev->cfg_t->enSensors & QMI8658_ACC_ENABLE)
    {
        qmi8658_config_acc(dev,dev->cfg_t->accRange, dev->cfg_t->accOdr, Qmi8658Lpf_Disable, Qmi8658St_Disable);
    }
    if(dev->cfg_t->enSensors & QMI8658_GYR_ENABLE)
    {
        qmi8658_config_gyro(dev,dev->cfg_t->gyrRange, dev->cfg_t->gyrOdr, Qmi8658Lpf_Disable, Qmi8658St_Disable);
    }

    qmi8658_set_param(dev,QMI_Register_Ctrl7,dev->cfg_t->enSensors);

__exit:
    if (result != RT_EOK)
    {
        LOG_E("This sensor initializes failure");
    }

    return result;
}

 qmi8658_device_t qmi8658_init(const char *i2c_bus_name,rt_uint8_t dev_addr)
{
    qmi8658_device_t dev = RT_NULL;

    RT_ASSERT(i2c_bus_name);

    dev = rt_calloc(1, sizeof(struct qmi8658_device));
    if (dev == RT_NULL)
    {
        LOG_E("Can't allocate memory for qmi8658 device on '%s' ", i2c_bus_name);
        rt_free(dev);
        return RT_NULL;
    }

    dev->bus = rt_device_find(i2c_bus_name);
    if (dev->bus == RT_NULL)
    {
        LOG_E("Can't find qmi8658 device on '%s'", i2c_bus_name);
        rt_free(dev);
        return RT_NULL;
    }

    dev->state_t = rt_calloc(1, sizeof(qmi8658_state));
    if (dev->state_t == RT_NULL)
    {
        LOG_E("Can't allocate memory for state_t device on '%s'", i2c_bus_name);
        rt_free(dev);
        return RT_NULL;
    }

    dev->state_t->addr = dev_addr;
    if (dev->state_t->addr == RT_NULL)
    {
        LOG_E("Can't find dev_addr for qmi8658 device on '%s'", i2c_bus_name);
        rt_free(dev);
        return RT_NULL;
    }

    dev->cfg_t = rt_calloc(1, sizeof(qmi8658_config));
    if (dev->cfg_t == RT_NULL)
    {
        LOG_E("Can't allocate memory for state_t device on '%s'", i2c_bus_name);
        rt_free(dev);
        return RT_NULL;
    }

    /* init qmi8658 sensor */
    if (qmi8658_sensor_init(dev) != RT_EOK)
    {
        LOG_E("Can't init qmi8658 device on '%s'", i2c_bus_name);
        rt_free(dev);
        return RT_NULL;
    }

    return dev;
}

/**
 * This function releases memory and deletes mutex lock
 *
 * @param dev the pointer of device driver structure
 */
void qmi8658_deinit(qmi8658_device_t dev)
{
    RT_ASSERT(dev);

    rt_free(dev);
}

/**
 * This function sets parameter of qmi8658 sensor
 *
 * @param dev the pointer of device driver structure
 * @param cmd the parameter cmd of device
 * @param value for setting value in cmd register
 *
 * @return the setting parameter status,RT_EOK reprensents setting successfully.
 */
rt_err_t qmi8658_set_param(qmi8658_device_t dev, qmi8658_set_cmd_t cmd, uint8_t value)
{
    rt_err_t result = -RT_ERROR;
    RT_ASSERT(dev);

    switch (cmd)
    {
        case QMI_Register_Ctrl7:/* Enable Sensors and Configure Data Reads. */
        {
            rt_uint8_t args;
            if(value==0)
            {
                write_reg(dev,Register_Ctrl7,value);
                dev->cfg_t->enSensors = value&0x03;
            }else{
                read_regs(dev,Register_Ctrl7,1,&args);
                args |= value;
                write_reg(dev,Register_Ctrl7,args);
                dev->cfg_t->enSensors = args&0x03;
            }
            break;
        }

        default:
        {
            LOG_E("This cmd'%2x' can not be set or supported", cmd);

            return -RT_ERROR;
        }
    }

    return result;
}

/**
 * This function gets parameter of qmi8658 sensor
 *
 * @param dev the pointer of device driver structure
 * @param cmd the parameter cmd of device
 * @param value to get value in cmd register
 *
 * @return the getting parameter status,RT_EOK reprensents getting successfully.
 */
rt_err_t qmi8658_get_param(qmi8658_device_t dev, qmi8658_set_cmd_t cmd, rt_uint8_t *value)
{
    rt_err_t result = -RT_ERROR;

    RT_ASSERT(dev);

    switch (cmd)
    {
        case QMI8658_DEVICE_IDENTIFIER:
        {
            result = read_regs(dev, Register_WhoAmI, 1, value);
            break;
        }
        case QMI8658_REVISION_ID:
        {
            result = read_regs(dev, Register_Revision, 1, value);
            break;
        }
        case QMI8658_CTRL1:
        {
            result = read_regs(dev, Register_Ctrl1, 1, value);
            break;
        }
        case QMI8658_ACCE_CONFIG:
        {
            result = read_regs(dev, Register_Ctrl2, 1, value);
            break;
        }
        case QMI8658_GYRO_CONFIG:
        {
            result = read_regs(dev, Register_Ctrl3, 1, value);
            break;
        }
        case QMI8658_LPF_CONFIG:
        {
            result = read_regs(dev, Register_Ctrl5, 1, value);
            break;
        }
        default:
        {
            LOG_E("This cmd'%2x' can not be get or supported", cmd);

            return -RT_ERROR;
        }
    }

    return result;
}

void qmi8658_sample(void)
{
    static qmi8658_device_t dev = RT_NULL;
    dev = qmi8658_init("i2c1",QMI8658_SLAVE_ADDR_L);

    float acc[3];
    float gyro[3];
    while(1)
    {
        qmi8658_read_xyz(dev);
        rt_kprintf("ACC: X:%hd    Y:%hd    Z:%hd\r\n", dev->state_t->imu[0], dev->state_t->imu[1], dev->state_t->imu[2]);
        rt_kprintf("GRY: X:%hd    Y:%hd    Z:%hd\r\n",dev->state_t->imu[3], dev->state_t->imu[4], dev->state_t->imu[5]);
        rt_kprintf("\n\n");
        rt_thread_mdelay(1000);
    }
    qmi8658_deinit(dev);
 }
 MSH_CMD_EXPORT(qmi8658_sample, qmi8658 sensor function);

 #endif /* PKG_USING_QMI8658 */