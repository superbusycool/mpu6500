#include "kalman.h"

double  KalmanFilter(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R) //��һ��������1~20���ڶ�������0.01����
{
    double R = MeasureNoise_R;
    double Q = ProcessNiose_Q;
    static double x_last;
    double x_mid = x_last;
    double x_now;
    static double p_last;
    double p_mid ;
    double p_now;
    double kg;
    x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
    p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1
    kg=p_mid/(p_mid+R);
    x_now=x_mid+kg*(ResrcData-x_mid);
    p_now=(1-kg)*p_mid;
    p_last = p_now;
    x_last = x_now;
    return x_now;
}


//void KF_Struct_Init(KF_Struct* KFS)
//{
//    KFS->x_last	=0;
//    KFS->P_now	=0;
//    KFS->P_last	=0.02;
//    KFS->K		=0;
//    KFS->Q_cov	=8.0;//���̼�������Э����,�����ɵ�
//    KFS->R_cov	=0.25;//��������Э��������������������йأ������ɵ�
//    KFS->out	=0;
//}
//
//
///*
//* @brief    �������˲���
//* @param    KFS:�������˲����ṹ��ָ��
//* @param    z:����������������
//* @return   ��ǰʱ�̵����Ź���ֵ
//*/
//float KMFilter_Simple(KF_Struct* KFS,float z)
//{
//    KFS->P_now = KFS->P_last + KFS->Q_cov;
//    KFS->K = KFS->P_now / (KFS->P_now + KFS->R_cov );
//    KFS->out = KFS->out + KFS->K * (z - KFS->out);
//    KFS->P_last = (1.0f - KFS->K)* KFS->P_now;
//
//    return KFS->out;
//}



////*
////-------------------------------------------------------
////Kalman�˲���8MHz�Ĵ���ʱ��Լ1.8ms��
////-------------------------------------------------------
//static float angle, angle_dot; 		//�ⲿ��Ҫ���õı���
////-------------------------------------------------------
//static const float Q_angle=0.001, Q_gyro=0.003, R_angle=0.5, dt=0.01;
////ע�⣺dt��ȡֵΪkalman�˲�������ʱ��;
//static float P[2][2] = {
//        { 1, 0 },
//        { 0, 1 }
//};
//
//static float Pdot[4] ={0,0,0,0};
//
//static const char C_0 = 1;
//
//static float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
////-------------------------------------------------------
//void Kalman_Filter(float angle_m,float gyro_m)			//gyro_m:gyro_measure
//{
//    angle+=(gyro_m-q_bias) * dt;
//
//    Pdot[0]=Q_angle - P[0][1] - P[1][0];
//    Pdot[1]=- P[1][1];
//    Pdot[2]=- P[1][1];
//    Pdot[3]=Q_gyro;
//
//    P[0][0] += Pdot[0] * dt;
//    P[0][1] += Pdot[1] * dt;
//    P[1][0] += Pdot[2] * dt;
//    P[1][1] += Pdot[3] * dt;
//
//
//    angle_err = angle_m - angle;
//
//
//    PCt_0 = C_0 * P[0][0];
//    PCt_1 = C_0 * P[1][0];
//
//    E = R_angle + C_0 * PCt_0;
//
//    K_0 = PCt_0 / E;
//    K_1 = PCt_1 / E;
//
//    t_0 = PCt_0;
//    t_1 = C_0 * P[0][1];
//
//    P[0][0] -= K_0 * t_0;
//    P[0][1] -= K_0 * t_1;
//    P[1][0] -= K_1 * t_0;
//    P[1][1] -= K_1 * t_1;
//
//
//    angle	+= K_0 * angle_err;
//    q_bias	+= K_1 * angle_err;
//    angle_dot = gyro_m - q_bias;
//}


//float angle=0.0;
//float measure=0.0;
//float Q_bias=0.5;//������������Э����
//float Q_angle=0.5;
//float Q_gyro=0.5;
//float R_angle=0.2;        //��������Э����
//float p[2][2]={0.02,0.02,0.02,0.02};
//float K0=0.0,K1=0.0;
//float dt = 0.02;
//
//void Kalman_Filter(float newAngle,float newGyro){
//
//
//    angle = angle - Q_bias*dt+newGyro*dt;
//
//    p[0][0]=p[0][0]+Q_angle-(p[0][1]-p[1][0])*dt;
//    p[0][1]=p[0][1]-p[1][1]*dt;
//    p[1][0]=p[1][0]-p[1][1]*dt;
//    p[1][0]=p[1][0]+Q_gyro;
//
//    measure=newAngle;
//
//    K0=(p[0][0]/(p[0][0]+R_angle));
//    K1=(p[1][0]/(p[0][0]+R_angle));
//
//    angle=angle+K0*(newAngle-angle);
//    Q_bias=Q_bias+K1*(newAngle-angle);
//
//    p[0][0]=p[0][0]-K0*p[0][0];
//    p[0][1]=p[0][1]-K0*p[0][1];
//    p[1][0]=p[1][0]-K1*p[0][0];
//    p[1][0]=p[1][0]-K1*p[0][1];
//
//
//}