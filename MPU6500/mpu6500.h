//
// Created by 刘嘉俊 on 24-7-19.
//

#ifndef A_6500_MPU6500_H
#define A_6500_MPU6500_H

#include "mpu6500.h"
//#include "mytype.h"
#include "main.h"

#define  MPU_HSPI hspi5			//SPI句柄定义
#define  MPU_NSS_LOW MPU6500_GYRO_NS_L()    //SPI通讯起始符号
#define  MPU_NSS_HIGH MPU6500_GYRO_NS_H()	//SPI通讯结束信号
#define Kp 1.55f                                              /*
                                                              * 比例增益
                                                              * 融合加速度计和磁强计
																															*/
#define Ki 0.005f                                             /*
                                                              * 积分增益
                                                              * 陀螺仪偏差收敛
																															*/

extern SPI_HandleTypeDef hspi5;			//重新声明

//
void MPU6500_GYRO_NS_H(void);
void MPU6500_GYRO_NS_L(void);



//MPU6500初始数据结构体
typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t mx;
    int16_t my;
    int16_t mz;

    int16_t temp;

    int16_t gx;
    int16_t gy;
    int16_t gz;

    int16_t ax_offset;
    int16_t ay_offset;
    int16_t az_offset;

    int16_t gx_offset;
    int16_t gy_offset;
    int16_t gz_offset;
} mpu_data_t;

//惯性测量单元数据结构体
typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t mx;
    int16_t my;
    int16_t mz;

    float temp;

    float wx; /*!< omiga, +- 2000dps => +-32768  so gx/16.384/57.3 =	rad/s */
    float wy;
    float wz;

    float vx;
    float vy;
    float vz;

    float rol;
    float pit;
    float yaw;

    float rol0;
    float pit0;
    float yaw0;
} imu_t;



static volatile float gx, gy, gz, ax, ay, az, mx, my, mz; 		//储存IMU更新值
extern mpu_data_t mpu_data;
extern imu_t      imu;



// 功能函数
//快速平方根倒数，计算 1/Sqrt(x)
//X:数字还需要计算
//调用imu ahrs update()函数
float inv_sqrt(float x);
//MPU6500单次写命令
uint8_t mpu_write_byte(uint8_t const reg, uint8_t const data);
//MPU6500单次读取单字节数据
uint8_t mpu_read_byte(uint8_t const reg);
//MPU6500单次读取多字节数据
uint8_t mpu_read_bytes(uint8_t const regAddr, uint8_t* pData, uint8_t len);
//设置imu 6500陀螺仪测量范围
uint8_t mpu_set_gyro_fsr(uint8_t fsr);
//设置imu 6050/6500加速测量范围
uint8_t mpu_set_accel_fsr(uint8_t fsr);

//主要使用函数
uint8_t   mpu_device_init(void);
void mpu_get_data(void);
void mpu_offset_call(void);


#endif //A_6500_MPU6500_H
