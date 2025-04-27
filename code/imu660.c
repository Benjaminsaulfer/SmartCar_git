#include "imu660.h"
#define N 20

INT16_XYZ Acc_original,Gyro_original,Acc_offset,Gyro_offset; //ԭʼ����
FLOAT_XYZ Acc_filt,Gyro_filt;

//-------------------------------------------------------------------------------------------------------------------
// ��  �� void ICM963RA_AccRead(FLOAT_XYZ *Acc_filt)
// ��  �� ��ȡ���ٶȵ�ԭʼ���ݣ�����һ�׿������˲�
// ��  �� *ICM963RA_ACC_RAW ԭʼ���ݵ�ָ��
// ����ֵ  void
// ��  ע void
//-------------------------------------------------------------------------------------------------------------------
void imu660ra_AccRead(FLOAT_XYZ *Acc_filt,INT16_XYZ *imu660ra_ACC_RAW)
{
    int16_t accData[3];
    uint8_t i;
   accData[0] = imu660ra_ACC_RAW->X;
   accData[1] = imu660ra_ACC_RAW->Y;
   accData[2] = imu660ra_ACC_RAW->Z;

    for(i=0;i<3;i++){                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
       static struct _1_ekf_filter ekf[3] = {{0.05,0,0,0,0.005,0.28},{0.05,0,0,0,0.005,0.28},{0.05,0,0,0,0.005,0.28}};
       kalman_1(&ekf[i],accData[i]);
       accData[i] = (int16_t)ekf[i].out;
    }
   Acc_filt->X  = (float)accData[0];
    Acc_filt->Y  = (float)accData[1];
    Acc_filt->Z  = (float)accData[2];
}
//-------------------------------------------------------------------------------------------------------------------
// ��  ��  ICM963RA_GyroRead(FLOAT_XYZ *Gyr_filt,INT16_XYZ *ICM963RA_GYRO_RAW)
// ��  ��  ��ȡ�����ǵ�ԭʼ���ݣ�����һ�׵�ͨ�˲�
// ��  ��  *gyroData ԭʼ���ݵ�ָ��
// ����ֵ   void
// ��  ע   void
//-------------------------------------------------------------------------------------------------------------------
void imu660ra_GyroRead(FLOAT_XYZ *Gyr_filt,INT16_XYZ *imu660ra_GYRO_RAW)
{
    int16_t gyroData[3];
    uint8_t i;
    gyroData[0] =  imu660ra_GYRO_RAW->X;
    gyroData[1] =  imu660ra_GYRO_RAW->Y;
    gyroData[2] =  imu660ra_GYRO_RAW->Z;

    for(i=0;i<3;i++){
       const float factor = 0.15f;  //�˲�����
       static float tBuff[3];
      gyroData[i] = tBuff[i] = (float)tBuff[i] *(1 - factor) + gyroData[i] * factor;
    }
      Gyr_filt->X  = (float)gyroData[0];
      Gyr_filt->Y  = (float)gyroData[1];
      Gyr_filt->Z  = (float)gyroData[2];
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ�����ǵ�ԭʼ����(���ᴫ����imu660ra)  ���ٶȼƺͽ��ٶȼƵ�ֵ
// ����˵��     1.acc:�洢���ٶȼ����ݵ�����    2.gyro:�洢���ٶȼ����ݵ�����
// ���ز���     void
// ʹ��ʾ��     Get_imu660ra_RAWData(INT16_XYZ *acc,INT16_XYZ *gyro)
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void Get_imu660ra_RAWData(INT16_XYZ *acc,INT16_XYZ *gyro)
{
    imu660ra_get_acc ();
    imu660ra_get_gyro ();

    acc->X = imu660ra_acc_x - Acc_offset.X;
    acc->Y = imu660ra_acc_y - Acc_offset.Y;
    acc->Z = imu660ra_acc_z - Acc_offset.Z;

    gyro->X = imu660ra_gyro_x - Gyro_offset.X;
    gyro->Y = imu660ra_gyro_y - Gyro_offset.Y; //(-)
    gyro->Z = imu660ra_gyro_z - Gyro_offset.Z; //(-)
}
//-------------------------------------------------------------------------------------------------------------------
// �������     1.��ȡԭʼ����  2.��ƫУ׼  3.�˲�
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     imu660ra_accgyroRead();
// ��ע��Ϣ     ��λת��:see:https://www.elecfans.com/d/1906295.html
//-------------------------------------------------------------------------------------------------------------------
void imu660ra_accgyroRead(void)
{
    Get_imu660ra_RAWData(&Acc_original,&Gyro_original);      //��ȡ������ԭʼ����
/*
    if(GET_FLAG(GYRO_OFFSET)){                                       //�����ǽ�����ƫУ׼
        if(imu660ra_OffSet(&imu660ra_GYRO_RAW,&GYRO_OFFSET_RAW,0)){  //�ж��������Ƿ�У׼���
             SENSER_FLAG_RESET(GYRO_OFFSET);                         //�ر�������У׼
             SENSER_FLAG_SET(ACC_OFFSET);                            //�������ٶ�У׼
        }
    }
    if(GET_FLAG(ACC_OFFSET)){                                        //���ٶȼƽ�����ƫУ׼
        if(imu660ra_OffSet(&imu660ra_ACC_RAW,&ACC_OFFSET_RAW,8196)){ //�жϼ��ٶȼ��Ƿ�У׼���
             SENSER_FLAG_RESET(ACC_OFFSET);                          //�رռ��ٶ�У׼
        }
    }
*/
    imu660ra_AccRead(&Acc_filt,&Acc_original);                   //���ٶ��˲�
    imu660ra_GyroRead(&Gyro_filt,&Gyro_original);                 //���ٶ��˲�

    Aver_FilterXYZ_ACC(&Acc_filt,15);                                //�˴��Լ��ٶȼƽ��л��������˲����� �˲�ϵ����һ��Ч�������  ���ǱȽϳ�����
    Aver_FilterXYZ_GYRO(&Gyro_filt,15);                               //�˴��Խ��ٶȼƽ��л��������˲�����

//  printf("%f,%f,%f\n",Acc_filt.X,Acc_filt.Y,Acc_filt.Z);           //�˲�֮����ٶ�����
//  printf("%f,%f,%f\n",Gyr_filt.X,Gyr_filt.Y,Gyr_filt.Z);           //�˲�֮����ٶ�����

    //���ٶ�ADֵ ת���� ��/ƽ����
    //(ԭʼ����/8192*g)��Ϊ����Ϊ+-4gת���ɵ�λm/s^2�ļ��㷽��
    Acc_filt.X = (float)imu660ra_acc_transition(Acc_filt.X) ;
    Acc_filt.Y =(float)imu660ra_acc_transition(Acc_filt.Y) ;
    Acc_filt.Z = (float)imu660ra_acc_transition(Acc_filt.Z) ;

    //������ADֵ ת���� ����/��
    //(ԭʼ����/65.6/57.3)��Ϊ����Ϊ+-500dpsת���ɵ�λrad/s�ļ��㷽��
    Gyro_filt.X = (float)imu660ra_gyro_transition (Gyro_filt.X );
    Gyro_filt.Y = (float) imu660ra_gyro_transition (Gyro_filt.Y );
    Gyro_filt.Z = (float) imu660ra_gyro_transition (Gyro_filt.Z );

  //  printf("%f,%f,%f\n",Acc_filt.X,Acc_filt.Y,Acc_filt.Z);           //��λ����֮����ٶ�����
  //  printf("%f,%f,%f\n",Gyr_filt.X,Gyr_filt.Y,Gyr_filt.Z);           //��λ����֮����ٶ�����
}



FLOAT_ANGLE Angle;
Integral_flag Gyro_Integral_flag;

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡŷ����
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     IMU_ReadAngle();
// ��ע��Ϣ     ��
//-------------------------------------------------------------------------------------------------------------------
void IMU_ReadAngle(void)
{
    accgyroRead_Updata();
    //imuMahonyAHRSupdate(&Gyro_filt,&Acc_filt,&Angle);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��������ȥ��Ư��������˲�
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     accgyroRead_Updata();
// ��ע��Ϣ     ��
//-------------------------------------------------------------------------------------------------------------------
void accgyroRead_Updata(void)
{
    imu660ra_accgyroRead();
}
//-------------------------------------------------------------------------------------------------------------------
// �������     ���ټ���1/Sqrt(x)
// ����˵��     float x:Ҫ�����ֵ
// ���ز���     float y:����Ľ��
// ʹ��ʾ��     invSqrt(float x);
// ��ע��Ϣ     (��ǰ����ʹ��)����ͨ��Sqrt()����Ҫ���ı�  See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
//                                                https://www.imangodoc.com/98221.html
//-------------------------------------------------------------------------------------------------------------------
float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ���þ���Mahony�㷨����ŷ����
// ����˵��     Gyr_filt:�˲�֮����ٶ�ԭʼ����   Acc_filt:�˲�֮����ٶ�ԭʼ����  Euler_Angle:���������ŷ����
// ���ز���     void
// ʹ��ʾ��     imuMahonyAHRSupdate(FLOAT_XYZ *Gyr_filt,FLOAT_XYZ *Acc_filt,FLOAT_ANGLE *Euler_Angle);
//
// ��ע��Ϣ     �����Ԫ����ŷ���Ƕ��ڴ˺��������   ����kp=ki=0   ������ȫ����������  ���ں�Kp��Ki���Դﵽ���벻����Ч��
//-------------------------------------------------------------------------------------------------------------------
#define Kp 0.000f                         // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.000f                         // integral gain governs rate of convergence of gyroscope biases
#define halfT 0.000045f                      // half the sample period
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;     // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error

void imuMahonyAHRSupdate(FLOAT_XYZ *Gyro_filt,FLOAT_XYZ *Acc_filt,FLOAT_ANGLE *Euler_Angle)
{
    float ax = Acc_filt->X,ay = Acc_filt->Y,az = Acc_filt->Z;
    float gx = Gyro_filt->X,gy = Gyro_filt->Y,gz = Gyro_filt->Z;
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;

   /*���ٶ�ֵΪ0,������ŷ���ǽ���*/
    if(ax*ay*az==0)
    return;
    /*��һ��:�����ٶ�ԭʼ���ݽ��й�һ������*/
    norm = invSqrt(ax*ax + ay*ay + az*az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    /*�ڶ���:��Ԫ�����Ƴ�������������(����)  [0,0,1] ��˷������Ҿ���ó����½��*/
    vx = 2*(q1q3 - q0q2);                //����(2,0)��
    vy = 2*(q0q1 + q2q3);                //����(2,1)��
    vz = 1 - 2*q1q1 - 2*q2q2;            //����(2,2)��  ����q0q0 - q1q1 - q2q2 + q3q3

    /*������:����������õ�ֵ �����������(ʵ�ʵ��������������۵������������)(ʵ���������������۵�������������������������)*/
    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);

    /*���Ĳ�:���������������(����)*/
    exInt = exInt + ex * Ki;
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;

    /*���岽:�����PI�󲹳���������  �������۵��������ٶȺ�ʵ�ʲ�������������ٶȵ���������Ԫ��
    //�õ����������������������������������������PI����������֮��ĵĽ��ٶ�ֵgx gy gz*/
    gx = gx + Kp*ex + exInt;
    gy = gy + Kp*ey + eyInt;
    //�����gz����û�й۲��߽��н��������Ư�ƣ����ֳ����ľ��ǻ����������Լ�(���ԼӶ�����������������)
    gz = gz + Kp*ez + ezInt;

    /*������:��Ԫ�ص�΢�ַ���  ���ٶȺ;ɵ���Ԫ�صĵ����õ��µ���Ԫ��*/
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

    /*���߲�:��һ����Ԫ������ȡŷ����*/
    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;

    //��Ԫ��ת����ŷ����(Z->Y->X)
    /**ƫ���ǻ���**/
    if(((Gyro_filt->Z *RadtoDeg > 1.0f) || (Gyro_filt->Z *RadtoDeg < -1.0f))&&Gyro_Integral_flag.Integral_flag_Z == 1)
        Euler_Angle->yaw += Gyro_filt->Z *RadtoDeg*halfT*2;
    else if(Gyro_Integral_flag.Integral_flag_Z == 0)
        Euler_Angle->yaw = 0.0f;
    /**�����ǻ���**/
    if(((Gyro_filt->X *RadtoDeg > 1.0f) || (Gyro_filt->X *RadtoDeg < -1.0f))&&Gyro_Integral_flag.Integral_flag_Y == 1){
        Euler_Angle->pit += Gyro_filt->X *RadtoDeg*halfT*2;}
    else if(Gyro_Integral_flag.Integral_flag_Y == 0) Euler_Angle->pit = 0.0f;

    Euler_Angle->yaw += Gyro_filt->Z *RadtoDeg*halfT*2;
   Euler_Angle->yaw = atan2(2.f * (q1q2 +  q0q3), q0q0 + q1q1 - q2q2 - q3q3)* 57.3f ;/**ƫ����**///�޹۲�������Ư�ƣ����ԼӴ����ƾ���
    Euler_Angle->pit = -atan2(2.f * q2q3 + 2.f * q0q1, q0q0 - q1q1 - q2q2 + q3q3)* RadtoDeg ;/**�����**/
    Euler_Angle->rol = -asin(2.f * (q1q3 - q0q2))* RadtoDeg;
//    printf("%f,%f,%f\n",Euler_Angle->pit,Euler_Angle->rol,Euler_Angle->yaw);
}









//-------------------------------------------------------------------------------------------------------------------
// �������     һ�׿������˲��㷨
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     kalman_1(struct _1_ekf_filter *ekf,float input);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void kalman_1(struct _1_ekf_filter *ekf,float input)
{
    ekf->Now_P = ekf->LastP + ekf->Q;
    ekf->Kg = ekf->Now_P / (ekf->Now_P + ekf->R);
    ekf->out = ekf->out + ekf->Kg * (input - ekf->out);
    ekf->LastP = (1-ekf->Kg) * ekf->Now_P ;
}
//-------------------------------------------------------------------------------------------------------------------
// �������     ���������˲�(��������)
// ����˵��     1.*acc Ҫ�˲����ݵ�ַ
//           2.*Acc_filt �˲������ݵ�ַ
// ���ز���     void
// ʹ��ʾ��     Aver_FilterXYZ(FLOAT_XYZ *Acc_filt,uint8_t n);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void Aver_FilterXYZ_ACC(FLOAT_XYZ *Acc_filt,uint8_t n)
{
    static int32_t bufax[N],bufay[N],bufaz[N];
    static uint8_t cnt =0,flag = 1;
    int32_t temp1=0,temp2=0,temp3=0,i;
    bufax[cnt] = Acc_filt->X;
    bufay[cnt] = Acc_filt->Y;
    bufaz[cnt] = Acc_filt->Z;
    cnt++;                                               //�����λ�ñ����ڸ�ֵ���󣬷���bufax[0]���ᱻ��ֵ
    if(cnt<n && flag)
        return;                                          //�������������
    else
    flag = 0;
    for(i=0;i<n;i++)
    {
        temp1 += bufax[i];
        temp2 += bufay[i];
        temp3 += bufaz[i];
    }
     if(cnt>=n)  cnt = 0;
     Acc_filt->X  = (float)(temp1/n);
     Acc_filt->Y  = (float)(temp2/n);
     Acc_filt->Z  = (float)(temp3/n);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ���������˲�(��������)
// ����˵��     1.*acc Ҫ�˲����ݵ�ַ
//           2.*Acc_filt �˲������ݵ�ַ
// ���ز���     void
// ʹ��ʾ��     Aver_FilterXYZ(FLOAT_XYZ *Acc_filt,uint8_t n);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void Aver_FilterXYZ_GYRO(FLOAT_XYZ *Gyr_filt,uint8_t n)
{
    static int32_t bufgx[N],bufgy[N],bufgz[N];
    static uint8_t cnt_g =0,G_flag = 1;
    int32_t temp1=0,temp2=0,temp3=0,i;
    bufgx[cnt_g] = Gyr_filt->X;
    bufgy[cnt_g] = Gyr_filt->Y;
    bufgz[cnt_g] = Gyr_filt->Z;
    cnt_g++;                                             //�����λ�ñ����ڸ�ֵ���󣬷���bufax[0]���ᱻ��ֵ
    if(cnt_g<n && G_flag)
        return;                                          //�������������
    else
        G_flag = 0;
    for(i=0;i<n;i++)
    {
        temp1 += bufgx[i];
        temp2 += bufgy[i];
        temp3 += bufgz[i];
    }
     if(cnt_g>=n)  cnt_g = 0;
     Gyr_filt->X  = (float)(temp1/n);
     Gyr_filt->Y  = (float)(temp2/n);
     Gyr_filt->Z  = (float)(temp3/n);
}


void get_gyro_angle()
{
    float gyro_angle = 0;
  
    imu660ra_accgyroRead();
    IMU_ReadAngle();
    gyro_angle += Gyro_filt.Z * 0.005;

}