#include "zf_common_headfile.h"
#include "moter.h"

extern uint8 remenber_point;
extern int16 EncoderL;
extern int16 EncoderR;
extern uint16_t motor_base;
extern int16 Max_encoderL;//����������ٶ�
extern int16 Max_encoderR;

//�����ʼ��
void  moter_init(){
    gpio_init(P10_2,GPO,0,GPO_PUSH_PULL);
    gpio_init(P10_3,GPO,0,GPO_PUSH_PULL);
    pwm_init(MoterL, 1000, 0);//Init_PWM 
    pwm_init(MoterR, 1000, 0);//Init_PWM
}

/*PID_init()��ʼ������
PID* pid  ����PID�ṹ��
uint8 Kp 
uint8 Ki
uint8 Kd
float target ����ֵ
*/
void PID_init(PID* pid,float Kp,float Ki,float Kd,float target){
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->LastError = 0;
    pid->PrevError = 0;
    pid->target = target;
    pid->OUT=0;
}


/*////////����ʽPID///////////
PID*             ����PID�Ľṹ��
current_value    �������Ĳ⵽��ʵ���ٶ�
*/
float  PID_Increse(PID* pid,float current_value){
    float error = pid->target - current_value;
    pid->PrevError += error;
    //�����޷�
    if(pid->PrevError>= 4000)pid->PrevError = 4000;//�����޷�
    float derivative = error - pid->LastError;
    pid->LastError = error;
    pid->OUT =  pid->Kp * error + pid->Ki * pid->PrevError + pid->Kd * derivative;

    return pid->OUT;
}

/*
PID*             ����PID�Ľṹ��
current_value    ��ǰ��λ��(���ߵ�ǰ���)
*/
float PID_location(PID *pid, float current_value) {
    float error = pid->target - current_value;  // ��ǰ���
    static float integral = 0;//������
    integral += error;         // �������ۼ�

    // ��� = Kp * ��ǰ��� + Ki *  ������ + Kd * (������ - ��һ�����)
    pid->OUT = pid->Kp * error + pid->Ki * integral + pid->Kd * (error - pid->LastError);

    // ������һ�ε����
    pid->LastError = error;

    return pid->OUT;
}

//����
void   Steering_FeedBack(PID * pid,float Error){
  static int input = 0;
  static int Rinput = 0;
  static int Linput = 0;
  input = (int)PID_location(pid,Error);
  Rinput = motor_base+input;
  Linput = motor_base-input;
  //Protect��λ����
  if(Rinput>=10000)Rinput = 10000;
  else if(Rinput<=0)Rinput=0;
  if(Linput>=10000)Linput = 10000;
  else if(Linput<=0)Linput=0;
  //��PID���ֵ����PWM
  pwm_set_duty(MoterR, Rinput);
  pwm_set_duty(MoterL, Linput);
}

//�ٶȻ�
void   Speed_FeedBack(PID * pid,Moter_WHO moter){
  //���ڲ���
  if(moter == Left){//���������
    pwm_set_duty(MoterL,  (uint32)(PID_Increse(pid,EncoderL*20)));
    ips200_Printf(0,280,(ips200_font_size_enum)0,"set:%.0f ",pid->target);//���õ�ֵ
    ips200_Printf(0,272,(ips200_font_size_enum)0,"pid:%.0f ",pid->OUT);//pid���ֵ
  }
  else{//������ҵ��
    pwm_set_duty(MoterR,  (uint32)(PID_Increse(pid,EncoderR*20)));
    ips200_Printf(120,280,(ips200_font_size_enum)0,"set:%.0f ",pid->target);//���õ�ֵ
    ips200_Printf(120,272,(ips200_font_size_enum)0,"pid:%.0f ",pid->OUT);//pid���ֵ
  }
}

/*����PID �� ת��->�ٶȻ�
PID * SteeringPID ת��pid����
PID * SpeedPID_L ���ٶȻ�pid����
PID * SpeedPID_R ���ٶȻ�pid����
*/
void   Cascade_FeedBack(PID * SteeringPID,PID * SpeedPID_L,PID * SpeedPID_R,float Error){
   float basic_Speed = motor_base;

  //ת�������pid�����¼��SteeringPID->OUT
  PID_location(SteeringPID,Error);//ת�򻷽������
  ips200_Printf(130,260,(ips200_font_size_enum)0,"trun:%.0f ",SteeringPID->OUT);//��ʾL�ߵ��pwmֵ
  //�ٶȻ���������ת�򻷵Ĳ���,�޸ĵ��ٶȻ���Ŀ��ֵ(����ֵ)
  SpeedPID_L->target = basic_Speed - SteeringPID->OUT;//�����ٶ� + SteeringPID->OUT
  SpeedPID_R->target = basic_Speed + SteeringPID->OUT;//�����ٶ� - SteeringPID->OUT
  
  //�ٶȻ����м��㣬pid�����¼��SpeedPID_L->OUT �� SpeedPID_R->OUT
  PID_Increse(SpeedPID_L,EncoderL*20);
  PID_Increse(SpeedPID_R,EncoderR*20);
  
  //Protectռ�ձȱ���
  if(SpeedPID_L->OUT >=10000)SpeedPID_L->OUT  = 10000;
  else if(SpeedPID_L->OUT <=0)SpeedPID_L->OUT = 0;
  if(SpeedPID_R->OUT >=10000)SpeedPID_R->OUT  = 10000;
  else if(SpeedPID_R->OUT <=0)SpeedPID_R->OUT = 0;
  
  //���ٶȻ�pidOUT�ֱ�����ڵ����
  pwm_set_duty(MoterL, (uint16_t)SpeedPID_L->OUT);
  pwm_set_duty(MoterR, (uint16_t)SpeedPID_R->OUT);

  /////////////////���ڲ���/////////////////

  ips200_Printf(52,280,(ips200_font_size_enum)0,"%.0f ",SpeedPID_L->target);//��ʾL�ߵ��pwmֵ
  ips200_Printf(172,280,(ips200_font_size_enum)0,"%.0f ",SpeedPID_R->target);//��ʾL�ߵ��pwmֵ
  /////////////////���ڲ���/////////////////
}

//Encoder_L �� Encoder_R �ֱ��������ձ����������ֵ(type int16),�ϵ��������������ٶ�����
void Encoder_Get_Max(int16* Encoder_L,int16* Encoder_R){
    static uint8_t onece_flag = 1;
    if(onece_flag){
      pwm_set_duty(MoterR, 10000);
      pwm_set_duty(MoterL, 10000);
      onece_flag = 0;
    }
    if(EncoderL>*Encoder_L)
      *Encoder_L = EncoderL;
    if(EncoderR>*Encoder_R)
      *Encoder_R = EncoderR;
}

void Encoder_Test(){//�����ڲ���
   gpio_set_level(P06_1,1);
   //ips200_Printf(0,166,(ips200_font_size_enum)0,"   SpeedL:%.2f ",speedL);
   //ips200_Printf(120,166,(ips200_font_size_enum)0," SpeedR:%.2f ",speedR);
   ips200_Printf(0,240,(ips200_font_size_enum)0,"Max_L:%d",Max_encoderL);//��ʾ����������ֵ
   ips200_Printf(0,248,(ips200_font_size_enum)0,"Max_R:%d",Max_encoderR);//��ʾ�ұ��������ֵ
   ips200_Printf(58,288,(ips200_font_size_enum)0,"%d ",EncoderL);//��ʾ���������ֵ
   ips200_Printf(178,288,(ips200_font_size_enum)0,"%d ",EncoderR);//��ʾ�ұ���������
   Encoder_Get_Max(&Max_encoderL,&Max_encoderR);
}