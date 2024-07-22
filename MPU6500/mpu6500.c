//
// Created by 刘嘉俊 on 24-7-19.
//
#include "main.h"
#include "mpu6500.h"
#include "spi.h"
#include "mpu6500_reg.h"
#include "string.h"
#include "ist8310.h"
#include "process.h"

static uint8_t        tx, rx;				//定义读写变量
static uint8_t        tx_buff[14] = { 0xff };			//MPU6500数据变量（加速度，温度，角度）
uint8_t               mpu_buff[14]; // 保存IMU原始数据
mpu_data_t            mpu_data;			//定义MPU数据句柄
imu_t                 imu={0};			//IMU数据储存
uint8_t id;													//定义ID
uint8_t               ist_buff[6];  //保存IST8310原始数据

//快速平方根倒数，计算 1/Sqrt(x)
//X:数字还需要计算
//调用imu ahrs update()函数
float inv_sqrt(float x)
{
    float halfx = 0.5f * x;
    float y     = x;
    long  i     = *(long*)&y;

    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));

    return y;
}

//MPU6500单次写命令
uint8_t mpu_write_byte(uint8_t const reg, uint8_t const data)
{
    MPU_NSS_LOW;					//开始通讯
    tx = reg & 0x7F;			//使第一位为0（写模式）
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);		//写入命令地址
    tx = data;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);		//写入数据
    MPU_NSS_HIGH;					//结束通讯
    return 0;
}

//MPU6500单次读取单字节数据
uint8_t mpu_read_byte(uint8_t const reg)
{
    MPU_NSS_LOW;
    tx = reg | 0x80;		//使地址第一位为1（读模式）
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);	//写入需要读取的地址
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);  //为读取的数据提供存储空间
    MPU_NSS_HIGH;
    return rx;
}

//MPU6500单次读取多字节数据
uint8_t mpu_read_bytes(uint8_t const regAddr, uint8_t* pData, uint8_t len)
{
    MPU_NSS_LOW;
    tx         = regAddr | 0x80;
    tx_buff[0] = tx;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    HAL_SPI_TransmitReceive(&MPU_HSPI, tx_buff, pData, len, 55);
    MPU_NSS_HIGH;
    return 0;
}

//设置imu 6500陀螺仪测量范围
uint8_t mpu_set_gyro_fsr(uint8_t fsr)
{
    return mpu_write_byte(MPU6500_GYRO_CONFIG, fsr << 3);
}
//设置imu 6050/6500加速测量范围
uint8_t mpu_set_accel_fsr(uint8_t fsr)
{
    return mpu_write_byte(MPU6500_ACCEL_CONFIG, fsr << 3);
}
//获取IMU数据
void mpu_get_data()
{
    mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

    mpu_data.ax   = mpu_buff[0] << 8 | mpu_buff[1];
    mpu_data.ay   = mpu_buff[2] << 8 | mpu_buff[3];
    mpu_data.az   = mpu_buff[4] << 8 | mpu_buff[5];
    mpu_data.temp = mpu_buff[6] << 8 | mpu_buff[7];

//    mpu_data.ax = mpu_data.ax / 4096;
//    mpu_data.ay = mpu_data.ay / 4096;
//    mpu_data.az = mpu_data.az / 4096;//转换为单位为g!!!!不能怎么转换,姿态角解算会报错


    mpu_data.gx = ((mpu_buff[8]  << 8 | mpu_buff[9])  - mpu_data.gx_offset);
    mpu_data.gy = ((mpu_buff[10] << 8 | mpu_buff[11]) - mpu_data.gy_offset);
    mpu_data.gz = ((mpu_buff[12] << 8 | mpu_buff[13]) - mpu_data.gz_offset);


    ist8310_get_data(ist_buff);
    memcpy(&mpu_data.mx, ist_buff, 6);

    memcpy(&imu.ax, &mpu_data.ax, 6 * sizeof(int16_t));

    imu.temp = 21 + mpu_data.temp / 333.87f;
    /* 2000dps -> rad/s */
    imu.wx   = mpu_data.gx / 16.384f / 57.3f;
    imu.wy   = mpu_data.gy / 16.384f / 57.3f;
    imu.wz   = mpu_data.gz / 16.384f / 57.3f;

    IMUupdate1(imu.wx,imu.wy,imu.wz,mpu_data.ax,mpu_data.ay,mpu_data.az,mpu_data.mx,mpu_data.my,mpu_data.mz);
}
//获取MPU6500的偏移量数据
void mpu_offset_call(void)
{
    int i;
    for (i=0; i<300;i++)
    {
        mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

        mpu_data.ax_offset += mpu_buff[0] << 8 | mpu_buff[1];
        mpu_data.ay_offset += mpu_buff[2] << 8 | mpu_buff[3];
        mpu_data.az_offset += mpu_buff[4] << 8 | mpu_buff[5];

        mpu_data.gx_offset += mpu_buff[8]  << 8 | mpu_buff[9];
        mpu_data.gy_offset += mpu_buff[10] << 8 | mpu_buff[11];
        mpu_data.gz_offset += mpu_buff[12] << 8 | mpu_buff[13];

        HAL_Delay(5);
    }
    mpu_data.ax_offset=mpu_data.ax_offset / 300;
    mpu_data.ay_offset=mpu_data.ay_offset / 300;
    mpu_data.az_offset=mpu_data.az_offset / 300;
    mpu_data.gx_offset=mpu_data.gx_offset / 300;
    mpu_data.gy_offset=mpu_data.gx_offset / 300;
    mpu_data.gz_offset=mpu_data.gz_offset / 300;
}

//初始化mpu6500和ist8310
uint8_t mpu_device_init(void)
{
    HAL_Delay(100);

    id                               = mpu_read_byte(MPU6500_WHO_AM_I);
    uint8_t i                        = 0;
    uint8_t MPU6500_Init_Data[10][2] = {{ MPU6500_PWR_MGMT_1, 0x80 },     /* 重置设备*/
                                        { MPU6500_PWR_MGMT_1, 0x03 },     /* 陀螺仪时钟源设置 */
                                        { MPU6500_PWR_MGMT_2, 0x00 },     /* 启动 Acc & Gyro */
                                        { MPU6500_CONFIG, 0x04 },         /* 低通滤波 频率41Hz */
                                        { MPU6500_GYRO_CONFIG, 0x18 },    /* +-2000dps */
                                        { MPU6500_ACCEL_CONFIG, 0x10 },   /* +-8G */
                                        { MPU6500_ACCEL_CONFIG_2, 0x02 }, /* 使能低通滤波器  设置 Acc 低通滤波 */
                                        { MPU6500_USER_CTRL, 0x20 },};    /* 使能 AUX */
    for (i = 0; i < 10; i++)
    {
        mpu_write_byte(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
        HAL_Delay(1);
    }

    mpu_set_gyro_fsr(3);//陀螺仪+-2000dps
    mpu_set_accel_fsr(2);//加速度+-8g

    ist8310_init();

    mpu_offset_call();
    return 0;
}
//SPI通讯起始符号
void MPU6500_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_6,GPIO_PIN_RESET);
}

//SPI通讯结束信号
void MPU6500_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_6,GPIO_PIN_SET);
}

