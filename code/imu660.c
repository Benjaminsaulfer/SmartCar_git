#include "imu660.h"

float yaw_angle = 0;

float get_yaw_angle()
{
    static float tBuff;
    const float dt = 0.01f;//积分时间
    const float factor = 0.15f;  //滤波参数
    imu660ra_get_gyro ();//获取原始角加速度

    tBuff = tBuff *(1 - factor) + imu660ra_gyro_z * factor;//一阶低通滤波

    //陀螺仪AD值 转换成 弧度/秒
    //(原始数据/65.6/57.3)即为量程为+-500dps转换成单位rad/s的计算方法
    yaw_angle += imu660ra_gyro_transition (tBuff) * dt + 0.0006;//0.0005手动抵消零点漂移
    return yaw_angle;
}