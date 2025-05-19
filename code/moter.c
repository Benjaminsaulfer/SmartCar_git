#include "zf_common_headfile.h"
#include "moter.h"

extern int16 EncoderL;
extern int16 EncoderR;
extern uint8_t SPEED;

//�����ʼ��
void  moter_init(){
    gpio_init(P10_2,GPO,0,GPO_PUSH_PULL);
    gpio_init(P10_3,GPO,0,GPO_PUSH_PULL);
    pwm_init(MoterL, 17000, 0);//Init_PWM 
    pwm_init(MoterR, 17000, 0);//Init_PWM
}

/**
 * @brief ���������������
 * @param targetPwm PWM�����÷�Χ0-10000
 * @param durationMs ����ʱ��
 * @param stepIntervalMs �������
 */
void sCurveRamp(int targetPwm, int durationMs, int stepIntervalMs) {
    int startPwm = 0;
    int steps = durationMs / stepIntervalMs;
    
    for (int i = 0; i < steps; i++) {
        float progress = (float)i / steps;
        float sCurveProgress = progress * progress / (progress * progress + (1 - progress) * (1 - progress));
        int pwmValue = startPwm + (int)((targetPwm - startPwm) * sCurveProgress);
        
        printf("%d,%d,%d\n",Encoder_TO_PWM(SPEED),Encoder_TO_PWM(EncoderR),Encoder_TO_PWM(EncoderL));
        pwm_set_duty(MoterR, pwmValue);
        pwm_set_duty(MoterL, pwmValue);
        system_delay_ms(stepIntervalMs);
    }
    
    pwm_set_duty(MoterL, targetPwm);
    pwm_set_duty(MoterR, targetPwm);
}

/*////////λ��ʽPID///////////
PID*             ����PID�Ľṹ��
current_value    �������Ĳ⵽��ʵ���ٶ�
*/
// λ��ʽPID����
static float PID_location(PID* pid, float current_value) {
  // ���㵱ǰ���
  pid->current = current_value;
  pid->error = pid->target - current_value;
  
  // �������ۼӲ��޷�
  pid->integral += pid->error;
  if(gpio_get_level(P06_1)){//��������
      if(pid->integral>= 8000)pid->integral = 8000;//�����޷�
  }else{//������û�д�
      pid->integral = 0;//����������
  }
  // ����΢������仯�ʣ�
  float derivative = pid->error - pid->LastError;
  pid->OUT = pid->Kp * pid->error + pid->Ki * pid->integral + pid->Kd * derivative;
  pid->LastError = pid->error;

  return pid->OUT;
}

//����
static void   Steering_FeedBack(PID * pid,float Error){
  static int input = 0;
  static int Rinput = 0;
  static int Linput = 0;
  input = (int)PID_location(pid,Error);
  Rinput = Encoder_TO_PWM(SPEED)+input;
  Linput = Encoder_TO_PWM(SPEED)-input;
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
static void  Speed_FeedBack(PID * pid,uint8_t moter){
  //���ڲ���
  if(moter){//���������
    //float filtered = lowPassFilter(PID_location(pid,EncoderL*20),0.2);
    //pwm_set_duty(MoterL,  (uint32)filtered);
    uint32_t temp =  Encoder_TO_PWM((uint32_t)PID_location(pid,EncoderL)) + 320;
    if(temp>=10000)temp = 10000;
    else if(temp<=0)temp =0;
    pwm_set_duty(MoterL,  temp);
    ips200_Printf(0,280,(ips200_font_size_enum)0,"set:%.0f ",pid->target);//���õ�ֵ
    ips200_Printf(0,272,(ips200_font_size_enum)0,"now:%d ",EncoderL);//pid���ֵ
  }
  else{//������ҵ��
    //float filtered = movingAverageFilter(PID_location(pid,EncoderR*20));
    //pwm_set_duty(MoterR,  (uint32)filtered);
    uint32_t temp =  Encoder_TO_PWM((uint32_t)PID_location(pid,EncoderR)) + 450;
    if(temp>=10000)temp = 10000;
    else if(temp<=0)temp =0;
    pwm_set_duty(MoterR,  temp);
    ips200_Printf(120,280,(ips200_font_size_enum)0,"set:%.0f ",pid->target);//���õ�ֵ
    ips200_Printf(120,272,(ips200_font_size_enum)0,"now:%d ",EncoderR);//pid���ֵ
  }
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


/*����PID �� ת��->�ٶȻ�
PID * SteeringPID ת��pid����
PID * SpeedPID_L ���ٶȻ�pid����
PID * SpeedPID_R ���ٶȻ�pid����s
*/
void   Cascade_FeedBack(PID * SteeringPID,PID * SpeedPID_L,PID * SpeedPID_R,float Error){
  //ת�������pid�����¼��SteeringPID->OUT
  PID_location(SteeringPID,Error);//ת�򻷽������(PWMֵ)

  //�ٶȻ���������ת�򻷵Ĳ���,�����ٶȻ���Ŀ��ֵ
  SpeedPID_L->target = SPEED*10 - SteeringPID->OUT*0.1;//�����ٶ� - SteeringPID->OUT(������ֵ)
  SpeedPID_R->target = SPEED*10 + SteeringPID->OUT*0.1;//�����ٶ� + SteeringPID->OUT(������ֵ)
  
  //�ٶȻ����м��㣬ȷ���������ӵ�ȷ��ת�򻷵������ת(������ֵ)
  PID_location(SpeedPID_L,EncoderL);
  PID_location(SpeedPID_R,EncoderR);
  
  //Protectռ�ձȱ����Ѿ�ƫ��
  uint32_t MoterL_duty = Encoder_TO_PWM((uint32_t)PID_location(SpeedPID_L,EncoderL)) + 320;//������ֵ->PWM
  uint32_t MoterR_duty = Encoder_TO_PWM((uint32_t)PID_location(SpeedPID_R,EncoderR)) + 450;//������ֵ->PWM
  if(MoterL_duty >=10000)MoterL_duty  = 10000;
  else if(MoterL_duty <=0)MoterL_duty = 0;
  if(MoterR_duty >=10000)MoterR_duty  = 10000;
  else if(MoterR_duty <=0)MoterR_duty = 0;
  
  //���ٶȻ�pidOUT�ֱ�����ڵ����
  pwm_set_duty(MoterL, MoterL_duty);
  pwm_set_duty(MoterR, MoterR_duty);

  /////////////////���ڲ���/////////////////
  //printf("%.2f,%.2f,%.2f\n",SpeedPID_L->target,SpeedPID_R->OUT,SpeedPID_L->OUT);
  //ips200_Printf(52,280,(ips200_font_size_enum)0,"%d  ",MoterL_duty);//��ʾL�ߵ��������ֵ
  //ips200_Printf(172,280,(ips200_font_size_enum)0,"%d  ",MoterR_duty);//��ʾR�ߵ��������ֵ
  /////////////////���ڲ���/////////////////
}

//////////////////////////////////���Ժ���////////////////////////////////////////////
/*��������pid�Ľṹ�����*/
void Encoder_loop_Test(PID * SpeedPID_L,PID * SpeedPID_R){
    //�������ջ����
    Speed_FeedBack(SpeedPID_L,1);//�����ٶȻ�
    Speed_FeedBack(SpeedPID_R,0);//�ҵ���ٶȻ�
    printf("%.2f,%.2f,%.2f\n",SpeedPID_L->target,SpeedPID_R->OUT,SpeedPID_L->OUT);
}

//Encoder_L �� Encoder_R �ֱ��������ձ����������ֵ(type int16),�ϵ��������������ٶ�����
void Encoder_Test(){//�����ڲ���
    static uint8_t moter_en = 1;
    static uint16_t time = 0;
    static int16 Max_encoderL = 0;//����������ٶ�
    static int16 Max_encoderR = 0;
    if(moter_en){//�������
      gpio_set_level(P06_1,1);
      sCurveRamp(10000, 4000, 20);//��������
      moter_en = 0;
    } 
   //ips200_Printf(0,166,(ips200_font_size_enum)0,"   SpeedL:%.2f ",speedL);
   //ips200_Printf(120,166,(ips200_font_size_enum)0," SpeedR:%.2f ",speedR);
   ips200_Printf(0,240,(ips200_font_size_enum)0,"Max_L:%d",Max_encoderL);//��ʾ����������ֵ
   ips200_Printf(0,248,(ips200_font_size_enum)0,"Max_R:%d",Max_encoderR);//��ʾ�ұ��������ֵ
   ips200_Printf(58,288,(ips200_font_size_enum)0,"%d ",EncoderL);//��ʾ���������ֵ
   ips200_Printf(178,288,(ips200_font_size_enum)0,"%d ",EncoderR);//��ʾ�ұ���������
   
    if(EncoderL>Max_encoderL)
      Max_encoderL = EncoderL;
    if(EncoderR>Max_encoderR)
      Max_encoderR = EncoderR;
    time++;
    if(time>=1000){
      gpio_set_level(P06_1,0);
      pwm_set_duty(MoterL, 0);
      pwm_set_duty(MoterR, 0);
    }
}