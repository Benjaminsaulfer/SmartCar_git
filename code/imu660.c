#include "imu660.h"

float yaw_angle = 0;

float get_yaw_angle()
{
    static float tBuff;
    const float dt = 0.01f;//����ʱ��
    const float factor = 0.15f;  //�˲�����
    imu660ra_get_gyro ();//��ȡԭʼ�Ǽ��ٶ�

    tBuff = tBuff *(1 - factor) + imu660ra_gyro_z * factor;//һ�׵�ͨ�˲�

    //������ADֵ ת���� ����/��
    //(ԭʼ����/65.6/57.3)��Ϊ����Ϊ+-500dpsת���ɵ�λrad/s�ļ��㷽��
    yaw_angle += imu660ra_gyro_transition (tBuff) * dt + 0.0006;//0.0005�ֶ��������Ư��
    return yaw_angle;
}