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
    pwm_init(MoterL, 17000, 0);//Init_PWM 
    pwm_init(MoterR, 17000, 0);//Init_PWM
}
//////////////////////////////////�����������////////////////////////////////////
/**
 * @brief ָ�����߷�ʽ����������ṩ��ƽ������������
 * 
 * @param moter PWMͨ��ö��ֵ��ָ��Ҫ���Ƶĵ��
 * @param targetPwm Ŀ��PWMֵ����Χ0-10000
 * @param durationMs �������̳���ʱ�䣨���룩
 * @param stepIntervalMs ÿ��PWM���µ�ʱ���������룩
 */
void exponentialRamp(int targetPwm, int durationMs, int stepIntervalMs) {
    int startPwm = 0;
    int steps = durationMs / stepIntervalMs;
    float base = pow((float)(targetPwm + 1) / (startPwm + 1), 1.0 / steps);
    
    for (uint16 i = 0; i < steps; i++) {
        uint16 pwmValue = (int)((startPwm + 1) * pow(base, i)) - 1;
        printf("%d,%d,%d\n",motor_base,EncoderR*20,EncoderL*20);
        pwm_set_duty(MoterL, pwmValue);
        pwm_set_duty(MoterR, pwmValue);
        system_delay_ms(stepIntervalMs);
    }
    
    pwm_set_duty(MoterL, targetPwm);
    pwm_set_duty(MoterR, targetPwm);
}
/**
 * @brief S�����߷�ʽ���������������Ժ�ָ������
 * 
 * @param moter PWMͨ��ö��ֵ��ָ��Ҫ���Ƶĵ��
 * @param targetPwm Ŀ��PWMֵ����Χ0-10000
 * @param durationMs �������̳���ʱ�䣨���룩
 * @param stepIntervalMs ÿ��PWM���µ�ʱ���������룩
 */
void sCurveRamp(pwm_channel_enum moter, int targetPwm, int durationMs, int stepIntervalMs) {
    int startPwm = 0;
    int steps = durationMs / stepIntervalMs;
    
    for (int i = 0; i < steps; i++) {
        float progress = (float)i / steps;
        // S�����߹�ʽ: f(x) = x^2 / (x^2 + (1-x)^2)
        float sCurveProgress = progress * progress / (progress * progress + (1 - progress) * (1 - progress));
        int pwmValue = startPwm + (int)((targetPwm - startPwm) * sCurveProgress);
        
        printf("%d,%d,%d\n",motor_base,EncoderR*20,EncoderL*20);
        pwm_set_duty(MoterR, pwmValue);
        pwm_set_duty(MoterL, pwmValue);
        system_delay_ms(stepIntervalMs);
    }
    
    pwm_set_duty(MoterL, targetPwm);
    pwm_set_duty(MoterR, targetPwm);
}

//////////////////////////////////�˲���///////////////////////////////////
// ALPHA�˲�ϵ����0~1����ԽС�˲�Ч��Խǿ������ӦԽ��
static float filteredValue = 0;
float lowPassFilter(float newSample,float ALPHA) {
    filteredValue = ALPHA * newSample + (1 - ALPHA) * filteredValue;
    return filteredValue;
}

static float filterBuffer[5];
static int bufferIndex = 0;
static float sum = 0;
float movingAverageFilter(float newSample) {
    // ��ȥ����Ĳ���ֵ
    sum -= filterBuffer[bufferIndex];
    // ����²���ֵ
    filterBuffer[bufferIndex] = newSample;
    sum += newSample;
    // ��������
    bufferIndex = (bufferIndex + 1) % 5;
    // ����ƽ��ֵ
    return sum / 5;
}

#define FILTER_LENGTH 5  // �˲������ȣ�����Ϊ������
static float filterBuffer[FILTER_LENGTH];
float medianFilter(float newSample) {
    // �滻����Ĳ���ֵ
    static int bufferIndex = 0;
    filterBuffer[bufferIndex] = newSample;
    bufferIndex = (bufferIndex + 1) % FILTER_LENGTH;
    
    // ���Ƶ���ʱ���鲢����
    float temp[FILTER_LENGTH];
    for (int i = 0; i < FILTER_LENGTH; i++) {
        temp[i] = filterBuffer[i];
    }
    // ð������
    for (int i = 0; i < FILTER_LENGTH - 1; i++) {
        for (int j = 0; j < FILTER_LENGTH - i - 1; j++) {
            if (temp[j] > temp[j + 1]) {
                float t = temp[j];
                temp[j] = temp[j + 1];
                temp[j + 1] = t;
            }
        }
    }
    // �����м�ֵ
    return temp[FILTER_LENGTH / 2];
}

//////////////////////////////////�˲���///////////////////////////////////

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


/*////////λ��ʽPID///////////
PID*             ����PID�Ľṹ��
current_value    �������Ĳ⵽��ʵ���ٶ�
*/
// λ��ʽPID����
float PID_location(PID* pid, float current_value) {
  // ���㵱ǰ���
  pid->current = current_value;
  pid->error = pid->target - current_value;
  
  // �������ۼӲ��޷�
  pid->integral += pid->error;
  if(gpio_get_level(P06_1)){
      if(pid->integral>= 10000)pid->integral = 10000;//�����޷�
  }else{
      if(pid->integral>= 10)pid->integral = 10;//�����޷���������
  }
  // ����΢������仯�ʣ�
  float derivative = pid->error - pid->LastError;
  pid->OUT = pid->Kp * pid->error + pid->Ki * pid->integral + pid->Kd * derivative;
  pid->LastError = pid->error;
  
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
void   Speed_FeedBack(PID * pid,uint8_t moter){
  //���ڲ���
  if(moter == 1){//���������
    //float filtered = lowPassFilter(PID_location(pid,EncoderL*20),0.2);
    //pwm_set_duty(MoterL,  (uint32)filtered);
    pwm_set_duty(MoterL,  (uint32)(PID_location(pid,EncoderL*20+400)));
    ips200_Printf(0,280,(ips200_font_size_enum)0,"set:%.0f ",pid->target);//���õ�ֵ
    ips200_Printf(0,272,(ips200_font_size_enum)0,"pid:%.0f ",pid->OUT);//pid���ֵ
  }
  else{//������ҵ��
    //float filtered = movingAverageFilter(PID_location(pid,EncoderR*20));
    //pwm_set_duty(MoterR,  (uint32)filtered);
    pwm_set_duty(MoterR,  (uint32)(PID_location(pid,EncoderR*20+400)));
    ips200_Printf(120,280,(ips200_font_size_enum)0,"set:%.0f ",pid->target);//���õ�ֵ
    ips200_Printf(120,272,(ips200_font_size_enum)0,"pid:%.0f ",pid->OUT);//pid���ֵ
  }
}

/*����PID �� ת��->�ٶȻ�
PID * SteeringPID ת��pid����
PID * SpeedPID_L ���ٶȻ�pid����
PID * SpeedPID_R ���ٶȻ�pid����s
*/
void   Cascade_FeedBack(PID * SteeringPID,PID * SpeedPID_L,PID * SpeedPID_R,float Error){
   float basic_Speed = motor_base;
  
  //ת�������pid�����¼��SteeringPID->OUT
  PID_location(SteeringPID,Error);//ת�򻷽������

  //�ٶȻ���������ת�򻷵Ĳ���,�޸ĵ��ٶȻ���Ŀ��ֵ(����ֵ)
  SpeedPID_L->target = basic_Speed - SteeringPID->OUT;//�����ٶ� + SteeringPID->OUT
  SpeedPID_R->target = basic_Speed + SteeringPID->OUT;//�����ٶ� - SteeringPID->OUT
  
  //�ٶȻ����м��㣬pid�����¼��SpeedPID_L->OUT �� SpeedPID_R->OUT
  PID_location(SpeedPID_L,EncoderL*20+400);
  PID_location(SpeedPID_R,EncoderR*20+400);
  
  //Protectռ�ձȱ���
  if(SpeedPID_L->OUT >=10000)SpeedPID_L->OUT  = 10000;
  else if(SpeedPID_L->OUT <=0)SpeedPID_L->OUT = 0;
  if(SpeedPID_R->OUT >=10000)SpeedPID_R->OUT  = 10000;
  else if(SpeedPID_R->OUT <=0)SpeedPID_R->OUT = 0;
  
  //���ٶȻ�pidOUT�ֱ�����ڵ����
  pwm_set_duty(MoterL, (uint16_t)SpeedPID_L->OUT);
  pwm_set_duty(MoterR, (uint16_t)SpeedPID_R->OUT);

  /////////////////���ڲ���/////////////////
  //ips200_Printf(52,280,(ips200_font_size_enum)0,"%.0f ",SpeedPID_L->target);//��ʾL�ߵ��pwmֵ
  //ips200_Printf(172,280,(ips200_font_size_enum)0,"%.0f ",SpeedPID_R->target);//��ʾL�ߵ��pwmֵ
  /////////////////���ڲ���/////////////////
}

//////////////////////////////////���Ժ���////////////////////////////////////////////
void Encoder_loop_Test(PID * SpeedPID_L,PID * SpeedPID_R){
    Speed_FeedBack(SpeedPID_R,0);//�ҵ���ٶȻ�
    Speed_FeedBack(SpeedPID_L,1);//�����ٶȻ�
    //printf("%d,%d,%d\n",motor_base,EncoderL*20,EncoderR*20);
    printf("%d,%.2f,%.2f\n",motor_base,SpeedPID_R->OUT,SpeedPID_L->OUT);
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
   sCurveRamp(MoterR,10000, 4000, 20);
   //ips200_Printf(0,166,(ips200_font_size_enum)0,"   SpeedL:%.2f ",speedL);
   //ips200_Printf(120,166,(ips200_font_size_enum)0," SpeedR:%.2f ",speedR);
   ips200_Printf(0,240,(ips200_font_size_enum)0,"Max_L:%d",Max_encoderL);//��ʾ����������ֵ
   ips200_Printf(0,248,(ips200_font_size_enum)0,"Max_R:%d",Max_encoderR);//��ʾ�ұ��������ֵ
   ips200_Printf(58,288,(ips200_font_size_enum)0,"%d ",EncoderL);//��ʾ���������ֵ
   ips200_Printf(178,288,(ips200_font_size_enum)0,"%d ",EncoderR);//��ʾ�ұ���������
   Encoder_Get_Max(&Max_encoderL,&Max_encoderR);
}
//////////////////////////////////���Ժ���////////////////////////////////////////