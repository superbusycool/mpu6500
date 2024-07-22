//
// Created by Lenovo on 2024/7/17.
//

#ifndef MPU6500_IST8310_TRY1_KALMAN_H
#define MPU6500_IST8310_TRY1_KALMAN_H


double  KalmanFilter(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);

//typedef struct Kalman_Filter{
//    float x_last;
//    float P_now;
//    float P_last;
//    float K;
//    float Q_cov;
//    float R_cov;
//    float out;
//}KF_Struct;
//
//void KF_Struct_Init(KF_Struct* KFS);
//float KMFilter_Simple(KF_Struct* KFS,float z);

//void Kalman_Filter(float newAngle,float newGyro);


#endif //MPU6500_IST8310_TRY1_KALMAN_H
