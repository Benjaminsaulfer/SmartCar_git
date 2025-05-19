#include "zf_common_headfile.h"
#include "moter.h"

extern int16 EncoderL;
extern int16 EncoderR;
extern uint8_t SPEED;

//电机初始化
void  moter_init(){
    gpio_init(P10_2,GPO,0,GPO_PUSH_PULL);
    gpio_init(P10_3,GPO,0,GPO_PUSH_PULL);
    pwm_init(MoterL, 17000, 0);//Init_PWM 
    pwm_init(MoterR, 17000, 0);//Init_PWM
}

/**
 * @brief 电机曲线启动函数
 * @param targetPwm PWM的设置范围0-10000
 * @param durationMs 启动时间
 * @param stepIntervalMs 启动间隔
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

/*////////位置式PID///////////
PID*             传入PID的结构体
current_value    编码器的测到的实际速度
*/
// 位置式PID计算
static float PID_location(PID* pid, float current_value) {
  // 计算当前误差
  pid->current = current_value;
  pid->error = pid->target - current_value;
  
  // 积分项累加并限幅
  pid->integral += pid->error;
  if(gpio_get_level(P06_1)){//如果电机打开
      if(pid->integral>= 8000)pid->integral = 8000;//积分限幅
  }else{//如果电机没有打开
      pid->integral = 0;//不允许弹射起步
  }
  // 计算微分项（误差变化率）
  float derivative = pid->error - pid->LastError;
  pid->OUT = pid->Kp * pid->error + pid->Ki * pid->integral + pid->Kd * derivative;
  pid->LastError = pid->error;

  return pid->OUT;
}

//方向环
static void   Steering_FeedBack(PID * pid,float Error){
  static int input = 0;
  static int Rinput = 0;
  static int Linput = 0;
  input = (int)PID_location(pid,Error);
  Rinput = Encoder_TO_PWM(SPEED)+input;
  Linput = Encoder_TO_PWM(SPEED)-input;
  //Protect限位保护
  if(Rinput>=10000)Rinput = 10000;
  else if(Rinput<=0)Rinput=0;
  if(Linput>=10000)Linput = 10000;
  else if(Linput<=0)Linput=0;
  //把PID输出值传入PWM
  pwm_set_duty(MoterR, Rinput);
  pwm_set_duty(MoterL, Linput);
}

//速度环
static void  Speed_FeedBack(PID * pid,uint8_t moter){
  //用于测试
  if(moter){//如果是左电机
    //float filtered = lowPassFilter(PID_location(pid,EncoderL*20),0.2);
    //pwm_set_duty(MoterL,  (uint32)filtered);
    uint32_t temp =  Encoder_TO_PWM((uint32_t)PID_location(pid,EncoderL)) + 320;
    if(temp>=10000)temp = 10000;
    else if(temp<=0)temp =0;
    pwm_set_duty(MoterL,  temp);
    ips200_Printf(0,280,(ips200_font_size_enum)0,"set:%.0f ",pid->target);//设置的值
    ips200_Printf(0,272,(ips200_font_size_enum)0,"now:%d ",EncoderL);//pid输出值
  }
  else{//如果是右电机
    //float filtered = movingAverageFilter(PID_location(pid,EncoderR*20));
    //pwm_set_duty(MoterR,  (uint32)filtered);
    uint32_t temp =  Encoder_TO_PWM((uint32_t)PID_location(pid,EncoderR)) + 450;
    if(temp>=10000)temp = 10000;
    else if(temp<=0)temp =0;
    pwm_set_duty(MoterR,  temp);
    ips200_Printf(120,280,(ips200_font_size_enum)0,"set:%.0f ",pid->target);//设置的值
    ips200_Printf(120,272,(ips200_font_size_enum)0,"now:%d ",EncoderR);//pid输出值
  }
}

/*PID_init()初始化函数
PID* pid  传入PID结构体
uint8 Kp 
uint8 Ki
uint8 Kd
float target 期望值
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


/*串级PID ， 转向环->速度环
PID * SteeringPID 转向环pid对象
PID * SpeedPID_L 左速度环pid对象
PID * SpeedPID_R 右速度环pid对象s
*/
void   Cascade_FeedBack(PID * SteeringPID,PID * SpeedPID_L,PID * SpeedPID_R,float Error){
  //转向环输出，pid输出记录在SteeringPID->OUT
  PID_location(SteeringPID,Error);//转向环进行输出(PWM值)

  //速度环接收来自转向环的参数,重置速度环的目标值
  SpeedPID_L->target = SPEED*10 - SteeringPID->OUT*0.1;//基础速度 - SteeringPID->OUT(编码器值)
  SpeedPID_R->target = SPEED*10 + SteeringPID->OUT*0.1;//基础速度 + SteeringPID->OUT(编码器值)
  
  //速度环进行计算，确保两边轮子的确以转向环的误差旋转(编码器值)
  PID_location(SpeedPID_L,EncoderL);
  PID_location(SpeedPID_R,EncoderR);
  
  //Protect占空比保护已经偏移
  uint32_t MoterL_duty = Encoder_TO_PWM((uint32_t)PID_location(SpeedPID_L,EncoderL)) + 320;//编码器值->PWM
  uint32_t MoterR_duty = Encoder_TO_PWM((uint32_t)PID_location(SpeedPID_R,EncoderR)) + 450;//编码器值->PWM
  if(MoterL_duty >=10000)MoterL_duty  = 10000;
  else if(MoterL_duty <=0)MoterL_duty = 0;
  if(MoterR_duty >=10000)MoterR_duty  = 10000;
  else if(MoterR_duty <=0)MoterR_duty = 0;
  
  //将速度环pidOUT分别输出在电机上
  pwm_set_duty(MoterL, MoterL_duty);
  pwm_set_duty(MoterR, MoterR_duty);

  /////////////////用于测试/////////////////
  //printf("%.2f,%.2f,%.2f\n",SpeedPID_L->target,SpeedPID_R->OUT,SpeedPID_L->OUT);
  //ips200_Printf(52,280,(ips200_font_size_enum)0,"%d  ",MoterL_duty);//显示L边电机编码器值
  //ips200_Printf(172,280,(ips200_font_size_enum)0,"%d  ",MoterR_duty);//显示R边电机编码器值
  /////////////////用于测试/////////////////
}

//////////////////////////////////测试函数////////////////////////////////////////////
/*传入两个pid的结构体对象*/
void Encoder_loop_Test(PID * SpeedPID_L,PID * SpeedPID_R){
    //编码器闭环电机
    Speed_FeedBack(SpeedPID_L,1);//左电机速度环
    Speed_FeedBack(SpeedPID_R,0);//右电机速度环
    printf("%.2f,%.2f,%.2f\n",SpeedPID_L->target,SpeedPID_R->OUT,SpeedPID_L->OUT);
}

//Encoder_L 和 Encoder_R 分别用来接收编码器的最大值(type int16),上电后电机马上以最大速度运行
void Encoder_Test(){//适用于测试
    static uint8_t moter_en = 1;
    static uint16_t time = 0;
    static int16 Max_encoderL = 0;//编码器最大速度
    static int16 Max_encoderR = 0;
    if(moter_en){//启动电机
      gpio_set_level(P06_1,1);
      sCurveRamp(10000, 4000, 20);//曲线启动
      moter_en = 0;
    } 
   //ips200_Printf(0,166,(ips200_font_size_enum)0,"   SpeedL:%.2f ",speedL);
   //ips200_Printf(120,166,(ips200_font_size_enum)0," SpeedR:%.2f ",speedR);
   ips200_Printf(0,240,(ips200_font_size_enum)0,"Max_L:%d",Max_encoderL);//显示左编码器最大值
   ips200_Printf(0,248,(ips200_font_size_enum)0,"Max_R:%d",Max_encoderR);//显示右编码器最大值
   ips200_Printf(58,288,(ips200_font_size_enum)0,"%d ",EncoderL);//显示左编码器的值
   ips200_Printf(178,288,(ips200_font_size_enum)0,"%d ",EncoderR);//显示右编码器数字
   
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