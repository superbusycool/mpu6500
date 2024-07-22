//
// Created by ���ο� on 24-7-19.
//

#ifndef A_6500_IST8310_H
#define A_6500_IST8310_H
//#include "mytype.h"
#include <stdio.h>

//ͨ��MPU6500������дIST8310�Ĵ���
static void ist_reg_write_by_mpu(uint8_t addr, uint8_t data);
//ͨ��MPU6500������ȡIST8310�Ĵ���
static uint8_t ist_reg_read_by_mpu(uint8_t addr);
//��ʼ���ӻ�0����I2C�Ĵ�����ȡ
static void mpu_master_i2c_auto_read_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num);
//ist8310��ʼ��
uint8_t ist8310_init();
//��ȡIST8310������
void ist8310_get_data(uint8_t* buff);


#endif //A_6500_IST8310_H
