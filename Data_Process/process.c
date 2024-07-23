//
// Created by SuperChen on 2024/7/20.
//

#include "process.h"
#include "main.h"
#include <math.h>
#include "mpu6500.h"
#include "kalman.h"


volatile float        q0 = 1.0f;		//四元数系数
volatile float        q1 = 0.0f;
volatile float        q2 = 0.0f;
volatile float        q3 = 0.0f;
volatile uint32_t     last_update, now_update;               /* 采样周期, 单位 ms */
volatile float        exInt, eyInt, ezInt;                   /* 错误提示 */

//float Kp=2.0;
//float Ki=0.25;



extern imu_t  imu;

//加速度单位g，陀螺仪rad/s
void IMUupdate1(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{


    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    float hx, hy, hz, bx, bz;
    float wx, wy, wz;
    float  halfT;


    now_update  = HAL_GetTick(); //ms
    halfT       = ((float)(now_update - last_update) / 2000.0f);
    last_update = now_update;


    // 测量正常化,把加计的三维向量转成单位向量。
    norm = sqrt(ax*ax + ay*ay + az*az);
    ax = ax / norm;                   //单位化
    ay = ay / norm;
    az = az / norm;

    norm = sqrt(mx*mx + my*my + mz*mz);
    mx = mx / norm;
    my = my / norm;
    mz = mz / norm;
    // 这里计算得到的是地磁计在理论地磁坐标系下的机体上三个轴的分量
    hx = 2*mx*(0.5 - q2*q2 - q3*q3) + 2*my*(q1*q2 - q0*q3) + 2*mz*(q1*q3 + q0*q2);
    hy = 2*mx*(q1*q2 + q0*q3) + 2*my*(0.5 - q1*q1 - q3*q3) + 2*mz*(q2*q3 - q0*q1);
    hz = 2*mx*(q1*q3 - q0*q2) + 2*my*(q2*q3 + q0*q1) + 2*mz*(0.5 - q1*q1 - q2*q2);


    //bx计算的是当前航向角和磁北的夹角，也就是北天东坐标下的航向角
    //当罗盘水平旋转的时候，航向角在0-360之间变化
    bx = sqrt((hx*hx) + (hy*hy));
    bz = hz;

    //地磁计在n系下磁向量转换到b系下，反向使用DCM得到
    wx = 2*bx*(0.5 - q2*q2 - q3*q3) + 2*bz*(q1*q3 - q0*q2);
    wy = 2*bx*(q1*q2 - q0*q3) + 2*bz*(q0*q1 + q2*q3);
    wz = 2*bx*(q0*q2 + q1*q3) + 2*bz*(0.5 - q1*q1 - q2*q2);

    // 估计方向的重力,世界坐标系重力分向量是通过方向旋转矩阵的最后一列的三个元素乘上加速度就可以算出机体坐标系中的重力向量。
    vx = 2*(q1*q3 - q0*q2);//由下向上方向的加速度在加速度计X分量
    vy = 2*(q0*q1 + q2*q3);//由下向上方向的加速度在加速度计X分量
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;//由下向上方向的加速度在加速度计Z分量


//这个叉积向量仍旧是位于机体坐标系上的，而陀螺积分误差也是在机体坐标系，而且叉积的大小与陀螺积分误差成正比，正好拿来纠正陀螺。
//（你可以自己拿东西想象一下）由于陀螺是对机体直接积分，所以对陀螺的纠正量会直接体现在对机体坐标系的纠正。

//                ex = (ay*vz - az*vy);
//        ey = (az*vx - ax*vz);
//        ez = (ax*vy - ay*vx);
    ex = (ay*vz - az*vy) + (my*wz - mz*wy);
    ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
    ez = (ax*vy - ay*vx) + (mx*wy - my*wx);




    // 积分误差比例积分增益,计算陀螺仪测量的重力向量与估计方向的重力向量之间的误差。
//        exInt = exInt + ex*Ki;
//        eyInt = eyInt + ey*Ki;
//        ezInt = ezInt + ez*Ki;
    /* PI */
    if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
    {
        exInt = exInt + ex*Ki*halfT;
        eyInt = eyInt + ey*Ki*halfT;
        ezInt = ezInt + ez*Ki*halfT;

        // 调整后的陀螺仪测量,使用叉积误差来进行比例-积分（PI）修正陀螺仪的零偏。将修正量乘以比例增益Kp，并加上之前计算的积分误差exInt、eyInt和ezInt。
        gx = gx + Kp*ex + exInt;
        gy = gy + Kp*ey + eyInt;
        gz = gz + Kp*ez + ezInt;
    }
    // 整合四元数率和正常化,根据陀螺仪的测量值和比例-积分修正值，对四元数进行更新。根据微分方程的离散化形式，将四元数的每个分量加上相应的微分项乘以采样周期的一半（halfT）。
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

    // 正常化四元数
    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;


    imu.pit  = asin(2 * q2 * q3 + 2 * q0* q1)* 57.3; // pitch ,转换为度数
    imu.rol = atan2(-2 * q1 * q3 + 2 * q0 * q2, q0*q0-q1*q1-q2*q2+q3*q3)* 57.3; // rollv
    imu.yaw = atan2(2*(q1*q2 - q0*q3),q0*q0-q1*q1+q2*q2-q3*q3) * 57.3;

    imu.pit0 =  KalmanFilter(imu.pit,10.0,0.05);
    imu.rol0 =  KalmanFilter(imu.rol,10.0,0.05);
    imu.yaw0 =  KalmanFilter(imu.yaw,10.0,0.05);//最简单的一阶卡尔曼滤波,Q,R值没调的很好

}
