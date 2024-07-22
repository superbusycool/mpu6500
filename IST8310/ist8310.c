//
// Created by 刘嘉俊 on 24-7-19.
//

#include "ist8310.h"
#include "mpu6500.h"
#include "mpu6500_reg.h"
#include "ist8310_reg.h"

//通过MPU6500主机编写IST8310寄存器
static void ist_reg_write_by_mpu(uint8_t addr, uint8_t data)
{
    /* 先关闭从机1 */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
    HAL_Delay(2);
    /* 从从机1 addr地址 开始传输数据 */
    mpu_write_byte(MPU6500_I2C_SLV1_REG, addr);
    HAL_Delay(2);
    /* 从机1输出数据data*/
    mpu_write_byte(MPU6500_I2C_SLV1_DO, data);
    HAL_Delay(2);
    /* 打开从机1发送1个字节 */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
    /* 等待确保数据发出 */
    HAL_Delay(10);
}

//通过MPU6500主机读取IST8310寄存器
static uint8_t ist_reg_read_by_mpu(uint8_t addr)
{
    uint8_t retval;
    /* 从从机4 addr地址 开始接收数据 */
    mpu_write_byte(MPU6500_I2C_SLV4_REG, addr);
    HAL_Delay(10);
    /* 先开启从机4 */
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x80);
    HAL_Delay(10);
    /* 将从机4读取的数据存入retval*/
    retval = mpu_read_byte(MPU6500_I2C_SLV4_DI);
    /* 读取完关闭从机4 */
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
    HAL_Delay(10);
    /* 返回retval*/
    return retval;
}
//初始化从机0经行I2C寄存器读取
static void mpu_master_i2c_auto_read_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
    /*
	   * 配置IST8310设备地址
     * 使用从机1开启自动传输单测量模式
	   */
    mpu_write_byte(MPU6500_I2C_SLV1_ADDR, device_address);
    HAL_Delay(2);
    mpu_write_byte(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
    HAL_Delay(2);
    mpu_write_byte(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
    HAL_Delay(2);

    /* 使用从机0自动读取数据 */
    mpu_write_byte(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
    HAL_Delay(2);
    mpu_write_byte(MPU6500_I2C_SLV0_REG, reg_base_addr);
    HAL_Delay(2);

    /* 每8个mpu6500内部采样周期，经行一次I2C数据读取 */
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x03);
    HAL_Delay(2);
    /* 使能从机1，从机0延时接收*/
    mpu_write_byte(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
    HAL_Delay(2);
    /* 使能从机1自动发送 */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
    /* 等待6个时钟周期 (最小等待时间为内部平均设置的16倍) */
    HAL_Delay(6);
    /* 使能从机0进行数据读取 */
    mpu_write_byte(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
    HAL_Delay(2);
}

//ist8310初始化
uint8_t ist8310_init()
{
    /*使能I2C主模式 复位I2C从模式将串口置位SPI模式 */
    mpu_write_byte(MPU6500_USER_CTRL, 0x30);
    HAL_Delay(10);
    /* MPU6500主时钟400khz */
    mpu_write_byte(MPU6500_I2C_MST_CTRL, 0x0d);
    HAL_Delay(10);
    /* 从机1写 */
    mpu_write_byte(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);
    HAL_Delay(10);
    /* 从机4读 */
    mpu_write_byte(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
    HAL_Delay(10);
    /* IST8310_R_CONFB 0x01 = 复位 */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);
    HAL_Delay(10);

    if (IST8310_DEVICE_ID_A != ist_reg_read_by_mpu(IST8310_WHO_AM_I))
        return 1;

    /* 软件复位 */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);
    HAL_Delay(10);

    /* 将IST8310配置为就绪状态 */
    ist_reg_write_by_mpu(IST8310_R_CONFA, 0x00);
    if (ist_reg_read_by_mpu(IST8310_R_CONFA) != 0x00)
        return 2;
    HAL_Delay(10);

    /* 正常状态 无初始化 */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x00);
    if (ist_reg_read_by_mpu(IST8310_R_CONFB) != 0x00)
        return 3;
    HAL_Delay(10);

    /* 设置成低噪音状态, x,y,z 轴 16 次采集求 1 次平均值 */
    ist_reg_write_by_mpu(IST8310_AVGCNTL, 0x24); //100100
    if (ist_reg_read_by_mpu(IST8310_AVGCNTL) != 0x24)
        return 4;
    HAL_Delay(10);

    /* 设置/复位脉冲周期,正常模式 */
    ist_reg_write_by_mpu(IST8310_PDCNTL, 0xc0);
    if (ist_reg_read_by_mpu(IST8310_PDCNTL) != 0xc0)
        return 5;
    HAL_Delay(10);

    /* 关闭从机1和从机4 */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
    HAL_Delay(10);
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
    HAL_Delay(10);

    /* 配置打开从机0 */
    mpu_master_i2c_auto_read_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
    HAL_Delay(100);
    return 0;
}

//获取IST8310的数据
void ist8310_get_data(uint8_t* buff)
{
    mpu_read_bytes(MPU6500_EXT_SENS_DATA_00, buff, 6);
}
