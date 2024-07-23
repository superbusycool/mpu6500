//
// Created by SuperChen on 2024/7/20.
//

#include "process.h"
#include "main.h"
#include <math.h>
#include "mpu6500.h"
#include "kalman.h"


volatile float        q0 = 1.0f;		//��Ԫ��ϵ��
volatile float        q1 = 0.0f;
volatile float        q2 = 0.0f;
volatile float        q3 = 0.0f;
volatile uint32_t     last_update, now_update;               /* ��������, ��λ ms */
volatile float        exInt, eyInt, ezInt;                   /* ������ʾ */

//float Kp=2.0;
//float Ki=0.25;



extern imu_t  imu;

//���ٶȵ�λg��������rad/s
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


    // ����������,�ѼӼƵ���ά����ת�ɵ�λ������
    norm = sqrt(ax*ax + ay*ay + az*az);
    ax = ax / norm;                   //��λ��
    ay = ay / norm;
    az = az / norm;

    norm = sqrt(mx*mx + my*my + mz*mz);
    mx = mx / norm;
    my = my / norm;
    mz = mz / norm;
    // �������õ����ǵشż������۵ش�����ϵ�µĻ�����������ķ���
    hx = 2*mx*(0.5 - q2*q2 - q3*q3) + 2*my*(q1*q2 - q0*q3) + 2*mz*(q1*q3 + q0*q2);
    hy = 2*mx*(q1*q2 + q0*q3) + 2*my*(0.5 - q1*q1 - q3*q3) + 2*mz*(q2*q3 - q0*q1);
    hz = 2*mx*(q1*q3 - q0*q2) + 2*my*(q2*q3 + q0*q1) + 2*mz*(0.5 - q1*q1 - q2*q2);


    //bx������ǵ�ǰ����Ǻʹű��ļнǣ�Ҳ���Ǳ��춫�����µĺ����
    //������ˮƽ��ת��ʱ�򣬺������0-360֮��仯
    bx = sqrt((hx*hx) + (hy*hy));
    bz = hz;

    //�شż���nϵ�´�����ת����bϵ�£�����ʹ��DCM�õ�
    wx = 2*bx*(0.5 - q2*q2 - q3*q3) + 2*bz*(q1*q3 - q0*q2);
    wy = 2*bx*(q1*q2 - q0*q3) + 2*bz*(q0*q1 + q2*q3);
    wz = 2*bx*(q0*q2 + q1*q3) + 2*bz*(0.5 - q1*q1 - q2*q2);

    // ���Ʒ��������,��������ϵ������������ͨ��������ת��������һ�е�����Ԫ�س��ϼ��ٶȾͿ��������������ϵ�е�����������
    vx = 2*(q1*q3 - q0*q2);//�������Ϸ���ļ��ٶ��ڼ��ٶȼ�X����
    vy = 2*(q0*q1 + q2*q3);//�������Ϸ���ļ��ٶ��ڼ��ٶȼ�X����
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;//�������Ϸ���ļ��ٶ��ڼ��ٶȼ�Z����


//�����������Ծ���λ�ڻ�������ϵ�ϵģ������ݻ������Ҳ���ڻ�������ϵ�����Ҳ���Ĵ�С�����ݻ����������ȣ����������������ݡ�
//��������Լ��ö�������һ�£����������ǶԻ���ֱ�ӻ��֣����Զ����ݵľ�������ֱ�������ڶԻ�������ϵ�ľ�����

//                ex = (ay*vz - az*vy);
//        ey = (az*vx - ax*vz);
//        ez = (ax*vy - ay*vx);
    ex = (ay*vz - az*vy) + (my*wz - mz*wy);
    ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
    ez = (ax*vy - ay*vx) + (mx*wy - my*wx);




    // ������������������,���������ǲ�����������������Ʒ������������֮�����
//        exInt = exInt + ex*Ki;
//        eyInt = eyInt + ey*Ki;
//        ezInt = ezInt + ez*Ki;
    /* PI */
    if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
    {
        exInt = exInt + ex*Ki*halfT;
        eyInt = eyInt + ey*Ki*halfT;
        ezInt = ezInt + ez*Ki*halfT;

        // ������������ǲ���,ʹ�ò����������б���-���֣�PI�����������ǵ���ƫ�������������Ա�������Kp��������֮ǰ����Ļ������exInt��eyInt��ezInt��
        gx = gx + Kp*ex + exInt;
        gy = gy + Kp*ey + eyInt;
        gz = gz + Kp*ez + ezInt;
    }
    // ������Ԫ���ʺ�������,���������ǵĲ���ֵ�ͱ���-��������ֵ������Ԫ�����и��¡�����΢�ַ��̵���ɢ����ʽ������Ԫ����ÿ������������Ӧ��΢������Բ������ڵ�һ�루halfT����
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

    // ��������Ԫ��
    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;


    imu.pit  = asin(2 * q2 * q3 + 2 * q0* q1)* 57.3; // pitch ,ת��Ϊ����
    imu.rol = atan2(-2 * q1 * q3 + 2 * q0 * q2, q0*q0-q1*q1-q2*q2+q3*q3)* 57.3; // rollv
    imu.yaw = atan2(2*(q1*q2 - q0*q3),q0*q0-q1*q1+q2*q2-q3*q3) * 57.3;

    imu.pit0 =  KalmanFilter(imu.pit,10.0,0.05);
    imu.rol0 =  KalmanFilter(imu.rol,10.0,0.05);
    imu.yaw0 =  KalmanFilter(imu.yaw,10.0,0.05);//��򵥵�һ�׿������˲�,Q,Rֵû���ĺܺ�

}
