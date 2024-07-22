//
// Created by ���ο� on 24-7-19.
//

#include "ist8310.h"
#include "mpu6500.h"
#include "mpu6500_reg.h"
#include "ist8310_reg.h"

//ͨ��MPU6500������дIST8310�Ĵ���
static void ist_reg_write_by_mpu(uint8_t addr, uint8_t data)
{
    /* �ȹرմӻ�1 */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
    HAL_Delay(2);
    /* �Ӵӻ�1 addr��ַ ��ʼ�������� */
    mpu_write_byte(MPU6500_I2C_SLV1_REG, addr);
    HAL_Delay(2);
    /* �ӻ�1�������data*/
    mpu_write_byte(MPU6500_I2C_SLV1_DO, data);
    HAL_Delay(2);
    /* �򿪴ӻ�1����1���ֽ� */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
    /* �ȴ�ȷ�����ݷ��� */
    HAL_Delay(10);
}

//ͨ��MPU6500������ȡIST8310�Ĵ���
static uint8_t ist_reg_read_by_mpu(uint8_t addr)
{
    uint8_t retval;
    /* �Ӵӻ�4 addr��ַ ��ʼ�������� */
    mpu_write_byte(MPU6500_I2C_SLV4_REG, addr);
    HAL_Delay(10);
    /* �ȿ����ӻ�4 */
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x80);
    HAL_Delay(10);
    /* ���ӻ�4��ȡ�����ݴ���retval*/
    retval = mpu_read_byte(MPU6500_I2C_SLV4_DI);
    /* ��ȡ��رմӻ�4 */
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
    HAL_Delay(10);
    /* ����retval*/
    return retval;
}
//��ʼ���ӻ�0����I2C�Ĵ�����ȡ
static void mpu_master_i2c_auto_read_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
    /*
	   * ����IST8310�豸��ַ
     * ʹ�ôӻ�1�����Զ����䵥����ģʽ
	   */
    mpu_write_byte(MPU6500_I2C_SLV1_ADDR, device_address);
    HAL_Delay(2);
    mpu_write_byte(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
    HAL_Delay(2);
    mpu_write_byte(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
    HAL_Delay(2);

    /* ʹ�ôӻ�0�Զ���ȡ���� */
    mpu_write_byte(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
    HAL_Delay(2);
    mpu_write_byte(MPU6500_I2C_SLV0_REG, reg_base_addr);
    HAL_Delay(2);

    /* ÿ8��mpu6500�ڲ��������ڣ�����һ��I2C���ݶ�ȡ */
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x03);
    HAL_Delay(2);
    /* ʹ�ܴӻ�1���ӻ�0��ʱ����*/
    mpu_write_byte(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
    HAL_Delay(2);
    /* ʹ�ܴӻ�1�Զ����� */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
    /* �ȴ�6��ʱ������ (��С�ȴ�ʱ��Ϊ�ڲ�ƽ�����õ�16��) */
    HAL_Delay(6);
    /* ʹ�ܴӻ�0�������ݶ�ȡ */
    mpu_write_byte(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
    HAL_Delay(2);
}

//ist8310��ʼ��
uint8_t ist8310_init()
{
    /*ʹ��I2C��ģʽ ��λI2C��ģʽ��������λSPIģʽ */
    mpu_write_byte(MPU6500_USER_CTRL, 0x30);
    HAL_Delay(10);
    /* MPU6500��ʱ��400khz */
    mpu_write_byte(MPU6500_I2C_MST_CTRL, 0x0d);
    HAL_Delay(10);
    /* �ӻ�1д */
    mpu_write_byte(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);
    HAL_Delay(10);
    /* �ӻ�4�� */
    mpu_write_byte(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
    HAL_Delay(10);
    /* IST8310_R_CONFB 0x01 = ��λ */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);
    HAL_Delay(10);

    if (IST8310_DEVICE_ID_A != ist_reg_read_by_mpu(IST8310_WHO_AM_I))
        return 1;

    /* �����λ */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);
    HAL_Delay(10);

    /* ��IST8310����Ϊ����״̬ */
    ist_reg_write_by_mpu(IST8310_R_CONFA, 0x00);
    if (ist_reg_read_by_mpu(IST8310_R_CONFA) != 0x00)
        return 2;
    HAL_Delay(10);

    /* ����״̬ �޳�ʼ�� */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x00);
    if (ist_reg_read_by_mpu(IST8310_R_CONFB) != 0x00)
        return 3;
    HAL_Delay(10);

    /* ���óɵ�����״̬, x,y,z �� 16 �βɼ��� 1 ��ƽ��ֵ */
    ist_reg_write_by_mpu(IST8310_AVGCNTL, 0x24); //100100
    if (ist_reg_read_by_mpu(IST8310_AVGCNTL) != 0x24)
        return 4;
    HAL_Delay(10);

    /* ����/��λ��������,����ģʽ */
    ist_reg_write_by_mpu(IST8310_PDCNTL, 0xc0);
    if (ist_reg_read_by_mpu(IST8310_PDCNTL) != 0xc0)
        return 5;
    HAL_Delay(10);

    /* �رմӻ�1�ʹӻ�4 */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
    HAL_Delay(10);
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
    HAL_Delay(10);

    /* ���ô򿪴ӻ�0 */
    mpu_master_i2c_auto_read_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
    HAL_Delay(100);
    return 0;
}

//��ȡIST8310������
void ist8310_get_data(uint8_t* buff)
{
    mpu_read_bytes(MPU6500_EXT_SENS_DATA_00, buff, 6);
}
