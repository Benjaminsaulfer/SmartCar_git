#pragma once

#define Max_encoder 1000 //编码器最大速度,把占空比拉到最高，即可得到编码器编码器最大速度
#define  Encoder_speed(x) (float)(x)*Max_encoder/100  /*编码器的最大转速/100目的是把编码器100等分
                                        ，以后设置速度环的时候直接设置x为,50则为以百分之50的速度前进  */
#define Encoder_TO_PWM(encoder) 10*encoder    //10000/Max_encoder*encoder编码器值转PWM

#define MoterR TCPWM_CH25_P09_1
#define MoterL TCPWM_CH24_P09_0

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

////////////////////////应用型PID/////////////////////////
void   PID_init(PID* pid,float Kp,float Ki,float Kd,float target);
void   Cascade_FeedBack(PID * SteeringPID,PID * SpeedPID_L,PID * SpeedPID_R,float Error);   //串级PID (转向环，速度环)  

/////////////////////////测试用/////////////////////////////
void Encoder_loop_Test(PID * SpeedPID_L,PID * SpeedPID_R);   //速度闭环测试
void Encoder_Test();        //测试编码器最大值，还是实际值