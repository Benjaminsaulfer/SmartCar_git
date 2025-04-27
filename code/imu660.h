#ifndef _IMU660_H
#define _IMU660_H

#include "zf_common_headfile.h"
#define g           9.80665f                                              //m/s^2
#define RadtoDeg    57.324841f                                            //���ȵ��Ƕ� (���� * 180/3.1415)
typedef struct
{
   int16_t X;
   int16_t Y;
   int16_t Z;
}INT16_XYZ;

typedef struct
{
    float X;
    float Y;   
    float Z;
    float gkd;
}FLOAT_XYZ;

typedef struct
{
    float rol;
    float pit;
    float yaw;
}FLOAT_ANGLE;

struct _1_ekf_filter
{
    float LastP;
    float Now_P;
    float out;
    float Kg;
    float Q;
    float R;
};

typedef struct
{
    uint8 Integral_flag_X;
    uint8 Integral_flag_Y;
    uint8 Integral_flag_Z;
}Integral_flag;


extern FLOAT_ANGLE Angle;
extern FLOAT_XYZ Acc_filt,Gyro_filt;
extern INT16_XYZ Acc_original,Gyro_original,Acc_offset,Gyro_offset; 


void imu660ra_accgyroRead(void);
void imuMahonyAHRSupdate(FLOAT_XYZ *Gyr_filt,FLOAT_XYZ *Acc_filt,FLOAT_ANGLE *Euler_Angle);
void get_gyro_angle(void);
/////////////////////////////////////////////filter
void Aver_FilterXYZ_ACC(FLOAT_XYZ *Acc_filt,uint8_t n);  
void Aver_FilterXYZ_GYRO(FLOAT_XYZ *Gyro_filt,uint8_t n); 
void kalman_1(struct _1_ekf_filter *ekf,float input);    
void IMU_ReadAngle(void);
void accgyroRead_Updata(void);
float invSqrt(float x);

#endif
