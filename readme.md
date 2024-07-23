## About task4
### �����Ķ�ȡrobomasterA�������е�mpu6500ģ���ist8310ģ��

<img src="Image/����ԭ��ͼ.png" width = "600" height = "600">

����Ҫ����A�Ϳ������ԭ��ͼ����CubeMX������,��ͼ��֪����SPI5,PF6�ŵ�ƽ�ߵ�,����Ƭѡ,�͵�ƽΪѡ��Ƭѡ,�ߵ�ƽΪ�ر�Ƭѡ.

��ԭ��ͼ��֪Mpu6500����ΪI2c�������Ӵ��豸ist8310,������Ҫͨ���Ĵ����ֲ�,ͨ��������д��������mpu6500�ļĴ�������
```c
uint8_t mpu_write_byte(uint8_t const reg, uint8_t const data)
{
    DEVIICE_SPI5_ENABLE();
    tx = reg & 0x7F;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    tx = data;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    DEVIICE_SPI5_DISABLE();
    return 0;
}


uint8_t mpu_read_byte(uint8_t const reg)
{
    DEVIICE_SPI5_ENABLE();
    tx = reg | 0x80;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    DEVIICE_SPI5_DISABLE();
    return rx;
}
```
ͨ��������������Ķ�д��������mpu6500�ĳ�ʼ��,Ȼ��������ݵĶ�ȡ

��Ȼ����֮�⻹��Ҫͨ����ؼĴ������ü��ٶȼ����̺������ǵ�����

�ٶ�ȡ�����ݺ�Ҫ���иߵ�λ��������λƴ�����16λ��ԭʼ����,֮��Ҫ���ݼ��ٶȺ������ǵ����̶��������ԭʼ���ݵ�λ����ת���Է���֮�����̬�ǽ���

<img src="Image/�Ĵ�����ͼ.png" width = "600" height = "400">

��mpu6500�ٷ��ĵ����ǿ�֪mpu6500������ΪI2c�����������4���ӻ�,����������Ҫ������ͨ����дmpu6500�Ĵ���ʹ���Master����ģʽ,����slave0->slve4����ist8310������ԭʼ���ݵĶ�ȡ

��ȡ�Ĺ���Ҳ����mpu6500��ist8310�������Ͷ�ist8310��д����Ϊ����,����ist8310�ĳ�ʼ��,�ٽ���ֵ�Ķ�ȡ


<img src="Image/i2c�洢���ݼĴ���.png" width = "600" height = "400">

���ն�ȡ�������ݻ���ڼĴ���EXT_SENS_DATA_XX��

<font color=green>**����** :��ʼmpu6500��Ϊi2c������ȡist830���̺ܳ���,slave0-4�ַ���ist8310���в���,һ���Ǵӻ�0��ist8310����д����,�������������ִ��ڴӻ�4��Ӧ�Ĵ�����,�����Ƿ����������ͼ,slaveӦ����mpu6500��Ϊ���豸��mcuͨ���õ�,����ist8310ͨ���õ������Aux_cl��Aux_Da��������������,�����ֶ�ȡ����Aux_cl��Aux_Da������������ŵĸ���,���ھ���Ĳ������̹ٷ��ĵ�Ҳ��δ˵��</font>


### ���������о�����Ԫ����̬�ǽ���
Ҫ������ǰ���е�λ��ת��,��Ԫ�����㿴�˽���̳�,˵ʵ��ȷʵ�Ƚ������,��������������ҵ���,���ݳ���ħ����һ��,�ȽϺõ�һ������Ԫ������͸��¶���һ�������ڲ����,���������ԳƷɿش���,����һ��ѽ�������׼,��ʱ���Ƚϸ�,��������û�п����ܺõ�**Mahony�㷨**�󽮹ٷ�������㲻׼��������Ϊ���ݴ����Ƶ�ֵ������Ӧ��Ԫ������
```c
//���ٶȵ�λg��������rad/s
void IMUupdate1(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{


    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    float hx, hy, hz, bx, bz;
    float wx, wy, wz;
    float  halfT;


    now_update  = HAL_GetTick(); //����ʱ��ms
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
```
���Խ������,��Ӧ�ٶȽϿ�

<img src="Image/������.png" width = "600" height = "400">

### �Ƚ�ͷ�۵��ǿ������˲�,����һ���˲��㶮�����ʽ����

<img src="Image/5����ʽ.jpg" width = "600" height = "400">

- <font color=white>x ���Ź���;p Э����;K ����������;F ״̬ת�ƾ���;
Q ���̼�������;</font>

<img src="Image/����k.jpg" width = "600" height = "400">

- <font color = "brown">������ͼ�еĹ��ڿ����������ʽ�ӿ���kԽ���ʾԽ���ι۲���,kԽС��ʾԽ�������Ź�����.��QԽС,RԽСʱ,kԽС,Խ�������۲�ֵ;QԽ��kԽ��Խ���ι۲�ֵ</font>

- <font color=green>RֵΪ��������,R̫�󿨶����˲���Ӧ�����,��Ϊ���µĲ��������ζȻ���.RԽС,ϵͳ�����ٶ�Խ��,����С���׳�����.
- ����ʱ�ɽ�Q��С�����,��R�Ӵ���С����;�ȹ̶�һ��ֵ��ȥ��������һ��ֵ,�������ٶȺͲ������</font>
 
#### ��Ȼ�����˿������˲�,����Ч��������(������һ���õ�,�Ӳ������������ݲ���ʱ,�����˲��Ĳ���������,����ҲҪ��һ��)

<img src="Image/Ч��.png" width = "1500" height = "400">


<font color="#dc143c">

- ���(�˹�)</font>

- �ڵ�(û��)

## <font color=pink>����:
- ���ڿ������˲��һ���ֻ�����������һ���˲�,���ڻ���С�����ｲ�ĸ��ݲ�ͬģ�ͽ��н�ģ,���ض�ģ�ͽ����ض��˲�,���и߽׿������˲���û����.

- ���ܻ���ֵû�е���,�������õĿ������˲�Ч����û����ô��
