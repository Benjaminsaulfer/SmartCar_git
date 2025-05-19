/*********************************************************************************************************************
* CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main_cm7_0
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "moter.h"
#include "trace.h"
#include "imu660.h"

/////////////////串口///////////
#define UART_INDEX              (DEBUG_UART_INDEX   )                           // 默认 UART_0
#define UART_BAUDRATE           (DEBUG_UART_BAUDRATE)                           // 默认 115200
#define UART_TX_PIN             (DEBUG_UART_TX_PIN  )                           // 默认 UART0_TX_P00_1
#define UART_RX_PIN             (DEBUG_UART_RX_PIN  )                           // 默认 UART0_RX_P00_0

uint8 uart_get_data[64];                                                        // 串口接收数据缓冲区
uint8 fifo_get_data[64];                                                        // fifo 输出读出缓冲区

uint8  get_data = 0;                                                            // 接收数据变量
uint32 fifo_data_count = 0;                                                     // fifo 数据个数

fifo_struct uart_data_fifo;
/////////////////串口///////////
#pragma location = 0x28001000  
__no_init float m7_1_data[5];
#define M1_speed m7_1_data[0]//第二个核心速度
#define Battery_V m7_1_data[1] //电池电压
 
uint32_t M0_speed;
uint16 ArrowPos = 128;//arrow position
int8 threshold_add = 0;//曝光度 
uint8_t threshold_gate = 130;
uint8_t SPEED = 4;//电机初始速度
int8 PID_ = 0;//第几个PID
uint32_t CPU_Speed=0;
////////标志位////////
uint8_t turnL_flag = 0;
uint8_t turnR_flag = 0;

////////编码器////////
int16 EncoderL;
int16 EncoderR;

///////创建PID对象/////
PID PID_Steering;
PID PID_Steering_turn;
PID PID_Speed_R;//右边电机
PID PID_Speed_L;//左边电机

Rectangle_Struct REC;//矩形结构体

void key_even(){
    //如果按键按下且电机关闭的时候则箭头移动
    if(gpio_get_level(P20_2) == 0 && gpio_get_level(P06_1) == 0)
    {
        while (gpio_get_level(P20_2) == 0)
        ips200_Printf(60,ArrowPos,(ips200_font_size_enum)0,"   ");
        ArrowPos+=8;
        if(ArrowPos>=192)ArrowPos=128;
    }
    if(gpio_get_level(P06_1) == 0){//如果电机关闭
      switch(ArrowPos){
         case 128://PID选择
           if(gpio_get_level(P20_0) == 0) {
             while(gpio_get_level(P20_0)==0); 
             PID_++;
           }
           else if(gpio_get_level(P20_3) == 0){
             while(gpio_get_level(P20_3)==0);
             PID_--;
           }
           //限位保护
            if(PID_>=4)PID_ = 3;
            else if (PID_<= 0)PID_ = 0;
            break;
         case 136://Kp
           if(gpio_get_level(P20_0) == 0){
             if(PID_==2||PID_==3){
                if(PID_==2)PID_Speed_L.Kp+=0.1;
                else PID_Speed_R.Kp+=0.1;
             }
             else{
                if(PID_==0)PID_Steering.Kp+=5;
                else PID_Steering_turn.Kp+=5;   
             }
           }
           else if(gpio_get_level(P20_3) == 0){
             if(PID_==2||PID_==3){
                if(PID_==2)PID_Speed_L.Kp-=0.1;
                else PID_Speed_R.Kp-=0.1;   
             }
             else {
               if(PID_==0)PID_Steering.Kp-=5;
                else PID_Steering_turn.Kp-=5;
             }
           }
            break;
         case 144://Ki
           if(gpio_get_level(P20_0) == 0){
              if(PID_==0)PID_Steering.Ki+=0.1;
              else PID_Steering_turn.Ki+=0.1;
           }
           else if(gpio_get_level(P20_3) == 0){
              if(PID_==0)PID_Steering.Ki-=0.1;
              else PID_Steering_turn.Ki-=0.1;
           }
            break;
         case 152://Kd
           if(gpio_get_level(P20_0) == 0){
              if(PID_==0)PID_Steering.Kd+=5;
              else PID_Steering_turn.Kd+=5;
           }
           else if(gpio_get_level(P20_3) == 0){
              if(PID_==0)PID_Steering.Kd-=5;
              else PID_Steering_turn.Kd-=5;
           }
            break;
         case 160://曝光度
            if(gpio_get_level(P20_0) == 0 && threshold_add<125)threshold_add++;
            else if(gpio_get_level(P20_3) == 0 && threshold_add>-20)threshold_add--;
            break;
         case 168://电机速度
            if(gpio_get_level(P20_0) == 0)SPEED+=1;
            else if(gpio_get_level(P20_3) == 0)SPEED-=1;
            break;
         case 176://电机使能
            if(gpio_get_level(P20_0) == 0)gpio_set_level(P06_1,0);
            else if(gpio_get_level(P20_3) == 0){
              gpio_set_level(P06_1,1);
              PID_init(&PID_Speed_L,2,0.1,0.1,Encoder_speed(SPEED));//左电机闭环
              PID_init(&PID_Speed_R,2,0.1,0.1,Encoder_speed(SPEED));//右电机闭环 
              ips200_Printf(46,176,(ips200_font_size_enum)0,"1");// 电机使能
              //system_delay_ms(1000);
            }
            break;
         case 184://电机使能
            if(gpio_get_level(P20_0) == 0)threshold_gate++;
            else if(gpio_get_level(P20_3) == 0)threshold_gate--;
            break;
      }
    }else{//如果电机打开
      if(gpio_get_level(P20_0) == 0)gpio_set_level(P06_1,0);
      else if(gpio_get_level(P20_3) == 0)gpio_set_level(P06_1,1);  
    }
}

void Scrren_Init(){
    ips200_init(IPS200_TYPE_PARALLEL8);//屏幕初始化
    ips200_pen_color(RGB565_BLUE);
    ips200_show_chinese(190,0, 16, dianya[0], 2, RGB565_RED);//显示font文件里面的
    ips200_Printf(8,128,(ips200_font_size_enum)0,"pid:");//电机初始速度
    ips200_Printf(0,136,(ips200_font_size_enum)0,"Kp:");//kp
    ips200_Printf(0,144,(ips200_font_size_enum)0,"Ki:");//ki
    ips200_Printf(0,152,(ips200_font_size_enum)0,"Kd:");//kd
    ips200_Printf(0,160,(ips200_font_size_enum)0,"light:");//摄像头曝光度
    ips200_Printf(0,168,(ips200_font_size_enum)0,"M_V:");//电机初始速度
    ips200_Printf(0,176,(ips200_font_size_enum)0,"motor:");// 电机使能
    ips200_Printf(0,184,(ips200_font_size_enum)0,"gate:");// 电机使能
    ips200_Printf(0,216,(ips200_font_size_enum)0,"pid:");//显示PID转向环
    ips200_Printf(0,280,(ips200_font_size_enum)0,"now:");//显示L边电机pwm值
    ips200_Printf(122,280,(ips200_font_size_enum)0,"now:");//显示L边电机pwm值
    ips200_Printf(0,288,(ips200_font_size_enum)0,"EncoderL:");//左边编码器值
    ips200_Printf(120,288,(ips200_font_size_enum)0,"EncoderR:");//右边编码器值
    ips200_Printf(0,300,(ips200_font_size_enum)0,"M0:");       //第一个核心的速度
    ips200_Printf(80,300,(ips200_font_size_enum)0,"M1:");//第二个核心运行速度显示运行速度
    ips200_draw_rectangle(120,160,220,240,RGB565_GRAY);
    ips200_pen_color(RGB565_RED);
}

static void bubbleSort(uint32_t arr[], uint8_t n) {
    uint8_t i, j;
    uint32_t temp;
    for (i = 0; i < n - 1; i++) {
        for (j = 0; j < n - i - 1; j++) {
            if (arr[j] > arr[j + 1]) {
                // 交换 arr[j] 和 arr[j+1]
                temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}

int main(void)
{     
    static uint8_t threshold=100;
    uint8_t thread = 0;
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_init();                       // 调试串口信息初始化
   
    //外设初始化
    mt9v03x_init();//摄像头初始化
    imu660ra_init();//姿态传感器初始化
    moter_init();//电机初始化
    //编码器初始化
    encoder_quad_init(TC_CH07_ENCODER, TC_CH07_ENCODER_CH1_P07_6, TC_CH07_ENCODER_CH2_P07_7);
    encoder_quad_init(TC_CH20_ENCODER, TC_CH20_ENCODER_CH1_P08_1, TC_CH20_ENCODER_CH2_P08_2);
    
    //PID初始化
    PID_init(&PID_Steering,105,0,200,0);//直线PID
    PID_init(&PID_Steering_turn,450,0,2,0);//弯道PID
    PID_init(&PID_Speed_L,2,0.1,0.1,Encoder_speed(SPEED));//左电机闭环,你可以设置的目标值,最大Encoder_speed 100
    PID_init(&PID_Speed_R,2,0.1,0.1,Encoder_speed(SPEED));//右电机闭环,你可以设置的目标值,最大为Encoder_speed 100
    //定时器中断
    pit_ms_init(PIT_CH0, 10);//初始化 PIT0 为周期中断 10ms
    pit_ms_init(PIT_CH1, 10);//初始化 PIT0 为周期中断 5ms
    //屏幕初始化
    Scrren_Init();
    adc_init(ADC0_CH00_P06_0, ADC_12BIT); //检测电池电压
    
    while(true)
    {
        timer_clear(TC_TIME2_CH0) ;
        timer_start(TC_TIME2_CH0) ;
        key_even();
        thread++;
        ///////////////////////////////////////////测速区间/////////////////////////////////////////////////
        
        ////////////////////循迹方案////////////////////
        if(thread%2){
          threshold = otsu_threshold(mt9v03x_image[0]) + threshold_add;//大津法自适应算法5ms
          if(threshold<=threshold_gate)threshold = threshold_gate;//限制曝光
        }
        
        binarizeImage(2,threshold);
        
        WhitePoint_amount(&REC);//白点算法
        
        if(gpio_get_level(P06_1) == 0){//如果电机没有打开允许屏幕运行
            //Screen_Add(threshold);//显示屏添加化图形
            Show_Binaray_map();//显示实际二值化图像
            ips200_Printf(60,ArrowPos,(ips200_font_size_enum)0,"<<");
            ips200_Printf(188,18,(ips200_font_size_enum)1,"%.2fV",Battery_V);//显示电池电压  
            ips200_Printf(36,128,(ips200_font_size_enum)0,"%d",PID_);//切换PID
            if(PID_==0){
              ips200_Printf(20,136,(ips200_font_size_enum)0,"%.1f ",PID_Steering.Kp);//kp
              ips200_Printf(20,144,(ips200_font_size_enum)0,"%.1f ",PID_Steering.Ki);//ki
              ips200_Printf(20,152,(ips200_font_size_enum)0,"%.1f ",PID_Steering.Kd);//kd
            }else if(PID_==1){
              ips200_Printf(20,136,(ips200_font_size_enum)0,"%.1f ",PID_Steering_turn.Kp);//kp
              ips200_Printf(20,144,(ips200_font_size_enum)0,"%.1f ",PID_Steering_turn.Ki);//ki
              ips200_Printf(20,152,(ips200_font_size_enum)0,"%.1f ",PID_Steering_turn.Kd);//kd
            }else if(PID_==2){
              ips200_Printf(20,136,(ips200_font_size_enum)0,"%.1f ",PID_Speed_L.Kp);//kp
              ips200_Printf(20,144,(ips200_font_size_enum)0,"%.1f ",PID_Speed_L.Ki);//ki
              ips200_Printf(20,152,(ips200_font_size_enum)0,"%.1f ",PID_Speed_L.Kd);//kd
            }
            else if(PID_==3){
              ips200_Printf(20,136,(ips200_font_size_enum)0,"%.1f ",PID_Speed_R.Kp);//kp
              ips200_Printf(20,144,(ips200_font_size_enum)0,"%.1f ",PID_Speed_R.Ki);//ki
              ips200_Printf(20,152,(ips200_font_size_enum)0,"%.1f ",PID_Speed_R.Kd);//kd
            }
            ips200_Printf(36,160,(ips200_font_size_enum)0,"%d ",threshold);//摄像头曝光度
            ips200_Printf(28,168,(ips200_font_size_enum)0,"%d ",SPEED);//电机初始速度
            ips200_Printf(46,176,(ips200_font_size_enum)0,"%d",gpio_get_level(P06_1));// 电机使能
            ips200_Printf(30,184,(ips200_font_size_enum)0,"%d",threshold_gate);//曝光限制
            ips200_Printf(30,216,(ips200_font_size_enum)0,"%.0f ",PID_Steering.OUT);//显示PID转向环
            //ips200_Printf(30,216,(ips200_font_size_enum)0,"%.0f ",PID_Steering_turn.OUT);//显示PID转向环
            ips200_Printf(20,300,(ips200_font_size_enum)0,"%d ",(uint32_t)M0_speed);//显示第一个核心运行速度
            ips200_Printf(100,300,(ips200_font_size_enum)0,"%d ",(uint32_t)M1_speed);//显示第二个核心运行速度
            ips200_Printf(126,168,(ips200_font_size_enum)0,"M0Speed:%d ",CPU_Speed/10);
            ips200_Printf(126,176,(ips200_font_size_enum)0,"point:%d ",REC.L+REC.R+REC.LL+REC.RR);
            ips200_Printf(126,184,(ips200_font_size_enum)0,"LL:%d ",REC.LL);
            ips200_Printf(126,192,(ips200_font_size_enum)0,"RR:%d ",REC.RR);
            ips200_Printf(126,200,(ips200_font_size_enum)0,"GL:%d ",REC.GL);
            ips200_Printf(126,208,(ips200_font_size_enum)0,"GR:%d ",REC.GR);
        }
        else{//记录CPU速度
            static uint32_t Speed_array[20];
            static uint8_t i = 0;
            static uint8_t once_flag = 1;
            if(i < 20){//记录数据
              Speed_array[i] = M0_speed;
              i++;
            }
            else if(once_flag){
              bubbleSort(Speed_array,20);
                for(int k = 5;k<15;k++)
                  CPU_Speed+=Speed_array[k];
                once_flag = 0;
            }
        }
        
        if(REC.LL>=20&&REC.RR>=20){
          
        }
        else if(REC.LL>=25){//识别到左转
          turnL_flag = 1;//触发左转 
          gpio_set_level(P19_0,0);//亮灯
        }
        else if(REC.RR>=25){
          turnR_flag = 1;//触发右转 
          gpio_set_level(P19_0,0);//亮灯
        }
        
        if(turnL_flag){
          static uint8_t once = 1;
          if(once){
            once=0;
            yaw_angle = 0;//重新赋值角度
          }
          if(yaw_angle<=-70){
            once = 1;
            turnL_flag = 0;//跳出
            gpio_set_level(P19_0,1);
          }
          pwm_set_duty(MoterL, 0);
          pwm_set_duty(MoterR, 1000);//转弯力度
        }
        else if(turnR_flag){
          static uint8_t once = 1;
          if(once){
            once=0;
            yaw_angle = 0;//重新赋值角度
          }
          if(yaw_angle>=70){
            once = 1;
            turnR_flag = 0;//跳出
            gpio_set_level(P19_0,1);
          }
          pwm_set_duty(MoterL, 1000);
          pwm_set_duty(MoterR, 0);//转弯力度
        }
        else
          Cascade_FeedBack(&PID_Steering,&PID_Speed_L,&PID_Speed_R,Sum_of_Dif(REC.L,REC.R));

        ///////测试函数///////
        //Encoder_loop_Test(&PID_Speed_L,&PID_Speed_R);//用于测试速度闭环，调整参数,会逼近PID->OUT应该逼近Moter_Base
        //Encoder_Test();//用于编码器测试,取消注释之后，电机马上以最大速度前进，并且记录下编码器的最大速度,并打印出来
        /////////////////////////////////////////测速区间/////////////////////////////////////////////////
        SCB_CleanInvalidateDCache_by_Addr(&m7_1_data, sizeof(m7_1_data));//更新RAM区数据
        M0_speed = timer_get(TC_TIME2_CH0);//获取第一个核心的运行时间
        timer_stop(TC_TIME2_CH0);
    }
}
