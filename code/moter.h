#pragma once

#define Max_encoder 1000 //����������ٶ�,��ռ�ձ�������ߣ����ɵõ�����������������ٶ�
#define  Encoder_speed(x) (float)(x)*Max_encoder/100  /*�����������ת��/100Ŀ���ǰѱ�����100�ȷ�
                                        ���Ժ������ٶȻ���ʱ��ֱ������xΪ,50��Ϊ�԰ٷ�֮50���ٶ�ǰ��  */
#define Encoder_TO_PWM(encoder) 10*encoder    //10000/Max_encoder*encoder������ֵתPWM

#define MoterR TCPWM_CH25_P09_1
#define MoterL TCPWM_CH24_P09_0

//PID�ṹ��
typedef struct {
  float Kp;
  float Ki;
  float Kd;
  float target;         //Ŀ���ٶȣ�����Ҫ���ٶȣ�
  float LastError;      //��һ�����
  float PrevError;     //����һ�����
  float integral;  // �����������Ա
  float OUT;
  float current;        // ��ǰֵ��������
  float error;          // ��ǰ��������
}PID;

void   moter_init();//�����ʼ��

////////////////////////Ӧ����PID/////////////////////////
void   PID_init(PID* pid,float Kp,float Ki,float Kd,float target);
void   Cascade_FeedBack(PID * SteeringPID,PID * SpeedPID_L,PID * SpeedPID_R,float Error);   //����PID (ת�򻷣��ٶȻ�)  

/////////////////////////������/////////////////////////////
void Encoder_loop_Test(PID * SpeedPID_L,PID * SpeedPID_R);   //�ٶȱջ�����
void Encoder_Test();        //���Ա��������ֵ������ʵ��ֵ