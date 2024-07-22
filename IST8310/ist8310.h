//
// Created by 刘嘉俊 on 24-7-19.
//

#ifndef A_6500_IST8310_H
#define A_6500_IST8310_H
//#include "mytype.h"
#include <stdio.h>

//通过MPU6500主机编写IST8310寄存器
static void ist_reg_write_by_mpu(uint8_t addr, uint8_t data);
//通过MPU6500主机读取IST8310寄存器
static uint8_t ist_reg_read_by_mpu(uint8_t addr);
//初始化从机0经行I2C寄存器读取
static void mpu_master_i2c_auto_read_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num);
//ist8310初始化
uint8_t ist8310_init();
//获取IST8310的数据
void ist8310_get_data(uint8_t* buff);


#endif //A_6500_IST8310_H
