#pragma once

#define Max_encoder 500 //编码器最大速度,把占空比拉到最高，即可得到编码器编码器最大速度
#define  Encoder_speed(x) (float)(x)*Max_encoder/100  /*编码器的最大转速/100目的是把编码器100等分
                                        ，以后设置速度环的时候直接设置x为,50则为以百分之50的速度前进  */
#define MoterL TCPWM_CH25_P09_1
#define MoterR TCPWM_CH24_P09_0

//PID结构体
typedef struct {
  float Kp;
  float Ki;
  float Kd;
  float target;         //目标速度（你想要的速度）
  float LastError;      //上一次误差
  float PrevError;     //上上一次误差
  float integral;  // 新增积分项成员
  float OUT;
  float current;        // 当前值（新增）
  float error;          // 当前误差（新增）
}PID;

void   moter_init();//电机初始化
/////////////////////////电机启动函数//////////////////////////
void exponentialRamp(int targetPwm, int durationMs, int stepIntervalMs);
void sCurveRamp(pwm_channel_enum moter, int targetPwm, int durationMs, int stepIntervalMs);

/////////////////////////滤波器////////////////////////////
float sliding_window_filter(int window_size, float input, float *history, int *index);//滑动窗口滤波

///////////////////////底层PID函数////////////////////////
void   PID_init(PID* pid,float Kp,float Ki,float Kd,float target);
float  PID_location(PID *pid, float current_value);           //位置式PID

////////////////////////应用型PID/////////////////////////
void   Steering_FeedBack(PID * pid,float Erro);             //转向环
void   Speed_FeedBack(PID * pid,uint8_t moter);             //速度环
void   Cascade_FeedBack(PID * SteeringPID,PID * SpeedPID_L,PID * SpeedPID_R,float Error);   //串级PID (转向环，速度环)  

/////////////////////////测试用/////////////////////////////
void Encoder_loop_Test(PID * SpeedPID_L,PID * SpeedPID_R);   //速度闭环测试
void Encoder_Get_Max(int16* Encoder_L,int16* Encoder_R);     //获取编码器最大值
void Encoder_Test();                                           //测试编码器最大值，还是实际值