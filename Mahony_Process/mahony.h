//
// Created by SuperChen on 2024/7/22.
//

#ifndef A_6500_MAHONY_H
#define A_6500_MAHONY_H


static float invSqrt(float number);
static void NonlinearAHRSinit(float ax, float ay, float az, float mx, float my, float mz);
static void NonlinearAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float twoKp, float twoKi, float dt);
void IMUThread(void);


#endif //A_6500_MAHONY_H
