#include "imu660.h"
#define N 20

INT16_XYZ Acc_original,Gyro_original,Acc_offset,Gyro_offset; //原始数据
FLOAT_XYZ Acc_filt,Gyro_filt;

//-------------------------------------------------------------------------------------------------------------------
// 函  数 void ICM963RA_AccRead(FLOAT_XYZ *Acc_filt)
// 功  能 读取加速度的原始数据，并做一阶卡尔曼滤波
// 参  数 *ICM963RA_ACC_RAW 原始数据的指针
// 返回值  void
// 备  注 void
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
// 函  数  ICM963RA_GyroRead(FLOAT_XYZ *Gyr_filt,INT16_XYZ *ICM963RA_GYRO_RAW)
// 功  能  读取陀螺仪的原始数据，并做一阶低通滤波
// 参  数  *gyroData 原始数据的指针
// 返回值   void
// 备  注   void
//-------------------------------------------------------------------------------------------------------------------
void imu660ra_GyroRead(FLOAT_XYZ *Gyr_filt,INT16_XYZ *imu660ra_GYRO_RAW)
{
    int16_t gyroData[3];
    uint8_t i;
    gyroData[0] =  imu660ra_GYRO_RAW->X;
    gyroData[1] =  imu660ra_GYRO_RAW->Y;
    gyroData[2] =  imu660ra_GYRO_RAW->Z;

    for(i=0;i<3;i++){
       const float factor = 0.15f;  //滤波参数
       static float tBuff[3];
      gyroData[i] = tBuff[i] = (float)tBuff[i] *(1 - factor) + gyroData[i] * factor;
    }
      Gyr_filt->X  = (float)gyroData[0];
      Gyr_filt->Y  = (float)gyroData[1];
      Gyr_filt->Z  = (float)gyroData[2];
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取陀螺仪的原始数据(六轴传感器imu660ra)  加速度计和角速度计的值
// 参数说明     1.acc:存储加速度计数据的数组    2.gyro:存储角速度计数据的数组
// 返回参数     void
// 使用示例     Get_imu660ra_RAWData(INT16_XYZ *acc,INT16_XYZ *gyro)
// 备注信息
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
// 函数简介     1.获取原始数据  2.零偏校准  3.滤波
// 参数说明     void
// 返回参数     void
// 使用示例     imu660ra_accgyroRead();
// 备注信息     单位转换:see:https://www.elecfans.com/d/1906295.html
//-------------------------------------------------------------------------------------------------------------------
void imu660ra_accgyroRead(void)
{
    Get_imu660ra_RAWData(&Acc_original,&Gyro_original);      //获取陀螺仪原始数据
/*
    if(GET_FLAG(GYRO_OFFSET)){                                       //陀螺仪进行零偏校准
        if(imu660ra_OffSet(&imu660ra_GYRO_RAW,&GYRO_OFFSET_RAW,0)){  //判断是陀螺仪否校准完成
             SENSER_FLAG_RESET(GYRO_OFFSET);                         //关闭陀螺仪校准
             SENSER_FLAG_SET(ACC_OFFSET);                            //启动加速度校准
        }
    }
    if(GET_FLAG(ACC_OFFSET)){                                        //加速度计进行零偏校准
        if(imu660ra_OffSet(&imu660ra_ACC_RAW,&ACC_OFFSET_RAW,8196)){ //判断加速度计是否校准完成
             SENSER_FLAG_RESET(ACC_OFFSET);                          //关闭加速度校准
        }
    }
*/
    imu660ra_AccRead(&Acc_filt,&Acc_original);                   //加速度滤波
    imu660ra_GyroRead(&Gyro_filt,&Gyro_original);                 //角速度滤波

    Aver_FilterXYZ_ACC(&Acc_filt,15);                                //此处对加速度计进行滑动窗口滤波处理 滤波系数大一点效果会更好  但是比较吃算力
    Aver_FilterXYZ_GYRO(&Gyro_filt,15);                               //此处对角速度计进行滑动窗口滤波处理

//  printf("%f,%f,%f\n",Acc_filt.X,Acc_filt.Y,Acc_filt.Z);           //滤波之后加速度数据
//  printf("%f,%f,%f\n",Gyr_filt.X,Gyr_filt.Y,Gyr_filt.Z);           //滤波之后角速度数据

    //加速度AD值 转换成 米/平方秒
    //(原始数据/8192*g)即为量程为+-4g转换成单位m/s^2的计算方法
    Acc_filt.X = (float)imu660ra_acc_transition(Acc_filt.X) ;
    Acc_filt.Y =(float)imu660ra_acc_transition(Acc_filt.Y) ;
    Acc_filt.Z = (float)imu660ra_acc_transition(Acc_filt.Z) ;

    //陀螺仪AD值 转换成 弧度/秒
    //(原始数据/65.6/57.3)即为量程为+-500dps转换成单位rad/s的计算方法
    Gyro_filt.X = (float)imu660ra_gyro_transition (Gyro_filt.X );
    Gyro_filt.Y = (float) imu660ra_gyro_transition (Gyro_filt.Y );
    Gyro_filt.Z = (float) imu660ra_gyro_transition (Gyro_filt.Z );

  //  printf("%f,%f,%f\n",Acc_filt.X,Acc_filt.Y,Acc_filt.Z);           //单位换算之后加速度数据
  //  printf("%f,%f,%f\n",Gyr_filt.X,Gyr_filt.Y,Gyr_filt.Z);           //单位换算之后角速度数据
}



FLOAT_ANGLE Angle;
Integral_flag Gyro_Integral_flag;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取欧拉角
// 参数说明     void
// 返回参数     void
// 使用示例     IMU_ReadAngle();
// 备注信息     无
//-------------------------------------------------------------------------------------------------------------------
void IMU_ReadAngle(void)
{
    accgyroRead_Updata();
    //imuMahonyAHRSupdate(&Gyro_filt,&Acc_filt,&Angle);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     对陀螺仪去零漂后的数据滤波
// 参数说明     void
// 返回参数     void
// 使用示例     accgyroRead_Updata();
// 备注信息     无
//-------------------------------------------------------------------------------------------------------------------
void accgyroRead_Updata(void)
{
    imu660ra_accgyroRead();
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     快速计算1/Sqrt(x)
// 参数说明     float x:要计算的值
// 返回参数     float y:计算的结果
// 使用示例     invSqrt(float x);
// 备注信息     (当前函数使用)比普通的Sqrt()函数要快四倍  See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
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
// 函数简介     采用经典Mahony算法解算欧拉角
// 参数说明     Gyr_filt:滤波之后角速度原始数据   Acc_filt:滤波之后加速度原始数据  Euler_Angle:解算出来的欧拉角
// 返回参数     void
// 使用示例     imuMahonyAHRSupdate(FLOAT_XYZ *Gyr_filt,FLOAT_XYZ *Acc_filt,FLOAT_ANGLE *Euler_Angle);
//
// 备注信息     求解四元数和欧拉角都在此函数中完成   若让kp=ki=0   就是完全相信陀螺仪  调节好Kp和Ki可以达到意想不到的效果
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

   /*加速度值为0,不进行欧拉角解算*/
    if(ax*ay*az==0)
    return;
    /*第一步:将加速度原始数据进行归一化操作*/
    norm = invSqrt(ax*ax + ay*ay + az*az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    /*第二步:四元数估计出来的重力向量(理论)  [0,0,1] 左乘方向余弦矩阵得出以下结果*/
    vx = 2*(q1q3 - q0q2);                //矩阵(2,0)项
    vy = 2*(q0q1 + q2q3);                //矩阵(2,1)项
    vz = 1 - 2*q1q1 - 2*q2q2;            //矩阵(2,2)项  还是q0q0 - q1q1 - q2q2 + q3q3

    /*第三步:向量叉乘所得的值 做向量的外积(实际的重力向量和理论的重力向量相乘)(实际重力向量与理论的重力向量差积求出向量间的误差)*/
    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);

    /*第四步:用上面求出误差进行(积分)*/
    exInt = exInt + ex * Ki;
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;

    /*第五步:将误差PI后补偿到陀螺仪  利用理论的重力加速度和实际测出来的重力加速度的误差补偿到四元数
    //得到重力向量与估算的重力向量差积求出向量间的误差，用PI控制器补偿之后的的角速度值gx gy gz*/
    gx = gx + Kp*ex + exInt;
    gy = gy + Kp*ey + eyInt;
    //这里的gz由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或自减(可以加多磁力计来解决该问题)
    gz = gz + Kp*ez + ezInt;

    /*第六步:四元素的微分方程  角速度和旧的四元素的迭代得到新的四元素*/
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

    /*第七步:归一化四元数，获取欧拉角*/
    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;

    //四元数转换成欧拉角(Z->Y->X)
    /**偏航角积分**/
    if(((Gyro_filt->Z *RadtoDeg > 1.0f) || (Gyro_filt->Z *RadtoDeg < -1.0f))&&Gyro_Integral_flag.Integral_flag_Z == 1)
        Euler_Angle->yaw += Gyro_filt->Z *RadtoDeg*halfT*2;
    else if(Gyro_Integral_flag.Integral_flag_Z == 0)
        Euler_Angle->yaw = 0.0f;
    /**俯仰角积分**/
    if(((Gyro_filt->X *RadtoDeg > 1.0f) || (Gyro_filt->X *RadtoDeg < -1.0f))&&Gyro_Integral_flag.Integral_flag_Y == 1){
        Euler_Angle->pit += Gyro_filt->X *RadtoDeg*halfT*2;}
    else if(Gyro_Integral_flag.Integral_flag_Y == 0) Euler_Angle->pit = 0.0f;

    Euler_Angle->yaw += Gyro_filt->Z *RadtoDeg*halfT*2;
   Euler_Angle->yaw = atan2(2.f * (q1q2 +  q0q3), q0q0 + q1q1 - q2q2 - q3q3)* 57.3f ;/**偏航角**///无观测器，会漂移，可以加磁力计纠正
    Euler_Angle->pit = -atan2(2.f * q2q3 + 2.f * q0q1, q0q0 - q1q1 - q2q2 + q3q3)* RadtoDeg ;/**横滚角**/
    Euler_Angle->rol = -asin(2.f * (q1q3 - q0q2))* RadtoDeg;
//    printf("%f,%f,%f\n",Euler_Angle->pit,Euler_Angle->rol,Euler_Angle->yaw);
}









//-------------------------------------------------------------------------------------------------------------------
// 函数简介     一阶卡尔曼滤波算法
// 参数说明     void
// 返回参数     void
// 使用示例     kalman_1(struct _1_ekf_filter *ekf,float input);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void kalman_1(struct _1_ekf_filter *ekf,float input)
{
    ekf->Now_P = ekf->LastP + ekf->Q;
    ekf->Kg = ekf->Now_P / (ekf->Now_P + ekf->R);
    ekf->out = ekf->out + ekf->Kg * (input - ekf->out);
    ekf->LastP = (1-ekf->Kg) * ekf->Now_P ;
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     滑动窗口滤波(三组数据)
// 参数说明     1.*acc 要滤波数据地址
//           2.*Acc_filt 滤波后数据地址
// 返回参数     void
// 使用示例     Aver_FilterXYZ(FLOAT_XYZ *Acc_filt,uint8_t n);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Aver_FilterXYZ_ACC(FLOAT_XYZ *Acc_filt,uint8_t n)
{
    static int32_t bufax[N],bufay[N],bufaz[N];
    static uint8_t cnt =0,flag = 1;
    int32_t temp1=0,temp2=0,temp3=0,i;
    bufax[cnt] = Acc_filt->X;
    bufay[cnt] = Acc_filt->Y;
    bufaz[cnt] = Acc_filt->Z;
    cnt++;                                               //这个的位置必须在赋值语句后，否则bufax[0]不会被赋值
    if(cnt<n && flag)
        return;                                          //数组填不满不计算
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
// 函数简介     滑动窗口滤波(三组数据)
// 参数说明     1.*acc 要滤波数据地址
//           2.*Acc_filt 滤波后数据地址
// 返回参数     void
// 使用示例     Aver_FilterXYZ(FLOAT_XYZ *Acc_filt,uint8_t n);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Aver_FilterXYZ_GYRO(FLOAT_XYZ *Gyr_filt,uint8_t n)
{
    static int32_t bufgx[N],bufgy[N],bufgz[N];
    static uint8_t cnt_g =0,G_flag = 1;
    int32_t temp1=0,temp2=0,temp3=0,i;
    bufgx[cnt_g] = Gyr_filt->X;
    bufgy[cnt_g] = Gyr_filt->Y;
    bufgz[cnt_g] = Gyr_filt->Z;
    cnt_g++;                                             //这个的位置必须在赋值语句后，否则bufax[0]不会被赋值
    if(cnt_g<n && G_flag)
        return;                                          //数组填不满不计算
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