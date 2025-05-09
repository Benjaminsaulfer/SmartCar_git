#include "zf_common_headfile.h"
#include "moter.h"

extern uint8 remenber_point;
extern int16 EncoderL;
extern int16 EncoderR;
extern uint16_t motor_base;
extern int16 Max_encoderL;//编码器最大速度
extern int16 Max_encoderR;

//电机初始化
void  moter_init(){
    gpio_init(P10_2,GPO,0,GPO_PUSH_PULL);
    gpio_init(P10_3,GPO,0,GPO_PUSH_PULL);
    pwm_init(MoterL, 17000, 0);//Init_PWM 
    pwm_init(MoterR, 17000, 0);//Init_PWM
}
//////////////////////////////////电机启动函数////////////////////////////////////
/**
 * @brief 指数曲线方式启动电机，提供更平滑的启动特性
 * 
 * @param moter PWM通道枚举值，指定要控制的电机
 * @param targetPwm 目标PWM值，范围0-10000
 * @param durationMs 启动过程持续时间（毫秒）
 * @param stepIntervalMs 每次PWM更新的时间间隔（毫秒）
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
 * @brief S型曲线方式启动电机，结合线性和指数特性
 * 
 * @param moter PWM通道枚举值，指定要控制的电机
 * @param targetPwm 目标PWM值，范围0-10000
 * @param durationMs 启动过程持续时间（毫秒）
 * @param stepIntervalMs 每次PWM更新的时间间隔（毫秒）
 */
void sCurveRamp(pwm_channel_enum moter, int targetPwm, int durationMs, int stepIntervalMs) {
    int startPwm = 0;
    int steps = durationMs / stepIntervalMs;
    
    for (int i = 0; i < steps; i++) {
        float progress = (float)i / steps;
        // S型曲线公式: f(x) = x^2 / (x^2 + (1-x)^2)
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

//////////////////////////////////滤波器///////////////////////////////////
// ALPHA滤波系数（0~1），越小滤波效果越强，但响应越慢
static float filteredValue = 0;
float lowPassFilter(float newSample,float ALPHA) {
    filteredValue = ALPHA * newSample + (1 - ALPHA) * filteredValue;
    return filteredValue;
}

static float filterBuffer[5];
static int bufferIndex = 0;
static float sum = 0;
float movingAverageFilter(float newSample) {
    // 减去最早的采样值
    sum -= filterBuffer[bufferIndex];
    // 添加新采样值
    filterBuffer[bufferIndex] = newSample;
    sum += newSample;
    // 更新索引
    bufferIndex = (bufferIndex + 1) % 5;
    // 返回平均值
    return sum / 5;
}

#define FILTER_LENGTH 5  // 滤波器长度（必须为奇数）
static float filterBuffer[FILTER_LENGTH];
float medianFilter(float newSample) {
    // 替换最早的采样值
    static int bufferIndex = 0;
    filterBuffer[bufferIndex] = newSample;
    bufferIndex = (bufferIndex + 1) % FILTER_LENGTH;
    
    // 复制到临时数组并排序
    float temp[FILTER_LENGTH];
    for (int i = 0; i < FILTER_LENGTH; i++) {
        temp[i] = filterBuffer[i];
    }
    // 冒泡排序
    for (int i = 0; i < FILTER_LENGTH - 1; i++) {
        for (int j = 0; j < FILTER_LENGTH - i - 1; j++) {
            if (temp[j] > temp[j + 1]) {
                float t = temp[j];
                temp[j] = temp[j + 1];
                temp[j + 1] = t;
            }
        }
    }
    // 返回中间值
    return temp[FILTER_LENGTH / 2];
}

//////////////////////////////////滤波器///////////////////////////////////

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


/*////////位置式PID///////////
PID*             传入PID的结构体
current_value    编码器的测到的实际速度
*/
// 位置式PID计算
float PID_location(PID* pid, float current_value) {
  // 计算当前误差
  pid->current = current_value;
  pid->error = pid->target - current_value;
  
  // 积分项累加并限幅
  pid->integral += pid->error;
  if(gpio_get_level(P06_1)){
      if(pid->integral>= 10000)pid->integral = 10000;//积分限幅
  }else{
      if(pid->integral>= 10)pid->integral = 10;//积分限幅用于启动
  }
  // 计算微分项（误差变化率）
  float derivative = pid->error - pid->LastError;
  pid->OUT = pid->Kp * pid->error + pid->Ki * pid->integral + pid->Kd * derivative;
  pid->LastError = pid->error;
  
  return pid->OUT;
}

//方向环
void   Steering_FeedBack(PID * pid,float Error){
  static int input = 0;
  static int Rinput = 0;
  static int Linput = 0;
  input = (int)PID_location(pid,Error);
  Rinput = motor_base+input;
  Linput = motor_base-input;
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
void   Speed_FeedBack(PID * pid,uint8_t moter){
  //用于测试
  if(moter == 1){//如果是左电机
    //float filtered = lowPassFilter(PID_location(pid,EncoderL*20),0.2);
    //pwm_set_duty(MoterL,  (uint32)filtered);
    pwm_set_duty(MoterL,  (uint32)(PID_location(pid,EncoderL*20+400)));
    ips200_Printf(0,280,(ips200_font_size_enum)0,"set:%.0f ",pid->target);//设置的值
    ips200_Printf(0,272,(ips200_font_size_enum)0,"pid:%.0f ",pid->OUT);//pid输出值
  }
  else{//如果是右电机
    //float filtered = movingAverageFilter(PID_location(pid,EncoderR*20));
    //pwm_set_duty(MoterR,  (uint32)filtered);
    pwm_set_duty(MoterR,  (uint32)(PID_location(pid,EncoderR*20+400)));
    ips200_Printf(120,280,(ips200_font_size_enum)0,"set:%.0f ",pid->target);//设置的值
    ips200_Printf(120,272,(ips200_font_size_enum)0,"pid:%.0f ",pid->OUT);//pid输出值
  }
}

/*串级PID ， 转向环->速度环
PID * SteeringPID 转向环pid对象
PID * SpeedPID_L 左速度环pid对象
PID * SpeedPID_R 右速度环pid对象s
*/
void   Cascade_FeedBack(PID * SteeringPID,PID * SpeedPID_L,PID * SpeedPID_R,float Error){
   float basic_Speed = motor_base;
  
  //转向环输出，pid输出记录在SteeringPID->OUT
  PID_location(SteeringPID,Error);//转向环进行输出

  //速度环接收来自转向环的参数,修改掉速度环的目标值(期望值)
  SpeedPID_L->target = basic_Speed - SteeringPID->OUT;//基础速度 + SteeringPID->OUT
  SpeedPID_R->target = basic_Speed + SteeringPID->OUT;//基础速度 - SteeringPID->OUT
  
  //速度环进行计算，pid输出记录在SpeedPID_L->OUT 和 SpeedPID_R->OUT
  PID_location(SpeedPID_L,EncoderL*20+400);
  PID_location(SpeedPID_R,EncoderR*20+400);
  
  //Protect占空比保护
  if(SpeedPID_L->OUT >=10000)SpeedPID_L->OUT  = 10000;
  else if(SpeedPID_L->OUT <=0)SpeedPID_L->OUT = 0;
  if(SpeedPID_R->OUT >=10000)SpeedPID_R->OUT  = 10000;
  else if(SpeedPID_R->OUT <=0)SpeedPID_R->OUT = 0;
  
  //将速度环pidOUT分别输出在电机上
  pwm_set_duty(MoterL, (uint16_t)SpeedPID_L->OUT);
  pwm_set_duty(MoterR, (uint16_t)SpeedPID_R->OUT);

  /////////////////用于测试/////////////////
  //ips200_Printf(52,280,(ips200_font_size_enum)0,"%.0f ",SpeedPID_L->target);//显示L边电机pwm值
  //ips200_Printf(172,280,(ips200_font_size_enum)0,"%.0f ",SpeedPID_R->target);//显示L边电机pwm值
  /////////////////用于测试/////////////////
}

//////////////////////////////////测试函数////////////////////////////////////////////
void Encoder_loop_Test(PID * SpeedPID_L,PID * SpeedPID_R){
    Speed_FeedBack(SpeedPID_R,0);//右电机速度环
    Speed_FeedBack(SpeedPID_L,1);//左电机速度环
    //printf("%d,%d,%d\n",motor_base,EncoderL*20,EncoderR*20);
    printf("%d,%.2f,%.2f\n",motor_base,SpeedPID_R->OUT,SpeedPID_L->OUT);
}

//Encoder_L 和 Encoder_R 分别用来接收编码器的最大值(type int16),上电后电机马上以最大速度运行
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

void Encoder_Test(){//适用于测试
   gpio_set_level(P06_1,1);
   sCurveRamp(MoterR,10000, 4000, 20);
   //ips200_Printf(0,166,(ips200_font_size_enum)0,"   SpeedL:%.2f ",speedL);
   //ips200_Printf(120,166,(ips200_font_size_enum)0," SpeedR:%.2f ",speedR);
   ips200_Printf(0,240,(ips200_font_size_enum)0,"Max_L:%d",Max_encoderL);//显示左编码器最大值
   ips200_Printf(0,248,(ips200_font_size_enum)0,"Max_R:%d",Max_encoderR);//显示右编码器最大值
   ips200_Printf(58,288,(ips200_font_size_enum)0,"%d ",EncoderL);//显示左编码器的值
   ips200_Printf(178,288,(ips200_font_size_enum)0,"%d ",EncoderR);//显示右编码器数字
   Encoder_Get_Max(&Max_encoderL,&Max_encoderR);
}
//////////////////////////////////测试函数////////////////////////////////////////