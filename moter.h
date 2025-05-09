#pragma once

#define Max_encoder 500 //����������ٶ�,��ռ�ձ�������ߣ����ɵõ�����������������ٶ�
#define  Encoder_speed(x) (float)(x)*Max_encoder/100  /*�����������ת��/100Ŀ���ǰѱ�����100�ȷ�
                                        ���Ժ������ٶȻ���ʱ��ֱ������xΪ,50��Ϊ�԰ٷ�֮50���ٶ�ǰ��  */
#define MoterL TCPWM_CH25_P09_1
#define MoterR TCPWM_CH24_P09_0

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
/////////////////////////�����������//////////////////////////
void exponentialRamp(int targetPwm, int durationMs, int stepIntervalMs);
void sCurveRamp(pwm_channel_enum moter, int targetPwm, int durationMs, int stepIntervalMs);

/////////////////////////�˲���////////////////////////////
float sliding_window_filter(int window_size, float input, float *history, int *index);//���������˲�

///////////////////////�ײ�PID����////////////////////////
void   PID_init(PID* pid,float Kp,float Ki,float Kd,float target);
float  PID_location(PID *pid, float current_value);           //λ��ʽPID

////////////////////////Ӧ����PID/////////////////////////
void   Steering_FeedBack(PID * pid,float Erro);             //ת��
void   Speed_FeedBack(PID * pid,uint8_t moter);             //�ٶȻ�
void   Cascade_FeedBack(PID * SteeringPID,PID * SpeedPID_L,PID * SpeedPID_R,float Error);   //����PID (ת�򻷣��ٶȻ�)  

/////////////////////////������/////////////////////////////
void Encoder_loop_Test(PID * SpeedPID_L,PID * SpeedPID_R);   //�ٶȱջ�����
void Encoder_Get_Max(int16* Encoder_L,int16* Encoder_R);     //��ȡ���������ֵ
void Encoder_Test();                                           //���Ա��������ֵ������ʵ��ֵ