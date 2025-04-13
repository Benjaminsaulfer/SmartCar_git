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

/*/////////////////串口///////////
#define UART_INDEX              (DEBUG_UART_INDEX   )                           // 默认 UART_0
#define UART_BAUDRATE           (DEBUG_UART_BAUDRATE)                           // 默认 115200
#define UART_TX_PIN             (DEBUG_UART_TX_PIN  )                           // 默认 UART0_TX_P00_1
#define UART_RX_PIN             (DEBUG_UART_RX_PIN  )                           // 默认 UART0_RX_P00_0

uint8 uart_get_data[64];                                                        // 串口接收数据缓冲区
uint8 fifo_get_data[64];                                                        // fifo 输出读出缓冲区

uint8  get_data = 0;                                                            // 接收数据变量
uint32 fifo_data_count = 0;                                                     // fifo 数据个数

fifo_struct uart_data_fifo;
*//////////////////串口///////////

uint32_t speed;
uint16 ArrowPos = 128;//arrow position
uint8 threshold = 130;//曝光度 
uint16 motor_base = 3000;//电机初始速度

////////编码器////////
int16 EncoderL = 0;//编码器速度
int16 EncoderR = 0;
//float speedL =0;//轮子线速度
//float speedR =0;
int16 Max_encoderL = 0;//编码器最大速度
int16 Max_encoderR = 0;

///////创建PID对象/////
PID PID_Steering;
PID PID_Steering_turn;
PID PID_Speed_R;//右边电机
PID PID_Speed_L;//左边电机

////////标志位////////
uint8 motor_flag = 0;//电机标志位 

#pragma location = 0x28001000  
__no_init float m7_1_data[50];

void key_even(){
    //如果按键按下且电机关闭的时候则箭头移动
    if(gpio_get_level(P20_2) == 0 && motor_flag == 0)
    {
        while (gpio_get_level(P20_2) == 0)
        ips200_Printf(80,ArrowPos,(ips200_font_size_enum)0,"   ");
        ArrowPos+=8;
        if(ArrowPos>=176)ArrowPos=128;
    }
    if(motor_flag == 0){//如果电机关闭
      switch(ArrowPos){
         case 128://Kp
            if(gpio_get_level(P20_0) == 0)PID_Steering.Kp+=0.1;
            if(gpio_get_level(P20_3) == 0)PID_Steering.Kp-=0.1;
            break;
         case 136://Ki
            if(gpio_get_level(P20_0) == 0)PID_Steering.Ki+=0.1;
            if(gpio_get_level(P20_3) == 0)PID_Steering.Ki-=0.1;
            break;
         case 144://Kd
            if(gpio_get_level(P20_0) == 0)PID_Steering.Kd+=0.1;
            if(gpio_get_level(P20_3) == 0)PID_Steering.Kd-=0.1;
            break;
         case 152://曝光度
            if(gpio_get_level(P20_0) == 0 && threshold<200)threshold++;
            if(gpio_get_level(P20_3) == 0 && threshold>0)threshold--;
            break;
         case 160://电机速度
            if(gpio_get_level(P20_0) == 0)motor_base+=100;
            if(gpio_get_level(P20_3) == 0)motor_base-=100;
            break;
         case 168://电机使能
            if(gpio_get_level(P20_0) == 0)motor_flag = 0;
            if(gpio_get_level(P20_3) == 0){
              motor_flag = 1;
              ips200_Printf(0,168,(ips200_font_size_enum)0,"motor:  %d",motor_flag);// 电机使能
            }
            break;
      }
    }else{//如果电机打开
      if(gpio_get_level(P20_0) == 0)motor_flag = 0;
         if(gpio_get_level(P20_3) == 0){
          motor_flag = 1;
          ips200_Printf(0,168,(ips200_font_size_enum)0,"motor:  %d",motor_flag);// 电机使能
      }
    }
}



void cross_Memory_Init(){//交叉内存参数初始化
  m7_1_data[0] = speed;
  m7_1_data[1] = motor_flag;
  m7_1_data[2] = PID_Steering.Kp;
  m7_1_data[3] = PID_Steering.Ki;
  m7_1_data[4] = PID_Steering.Kd;
  m7_1_data[5] = threshold;//增加曝光度
  m7_1_data[6] = motor_base;//电机初始速度
}

void cross_Memory(){//交叉内存
  m7_1_data[0] = speed;
  motor_flag = (uint32_t)m7_1_data[1];
  if(m7_1_data[1]==0){//如果电机没有打开的时候
    PID_Steering.Kp = m7_1_data[2];
    PID_Steering.Ki = m7_1_data[3];
    PID_Steering.Kd = m7_1_data[4];
    threshold = (uint32_t)m7_1_data[5];//增加曝光度
    motor_base =  (uint32_t)m7_1_data[6];//电机初始速度
    m7_1_data[7] = PID_Steering.OUT;
    m7_1_data[8] = PID_Speed_L.OUT; 
    m7_1_data[9] = PID_Speed_R.OUT;
    
    for (uint8_t y = 0; y < 120; y++) {
        for (uint8_t x = 0; x < 188; x++) {
            //m7_1_data_BinMap[x][y] = Binary_map[x][y];
        }
    }
  }
}

int main(void)
{     
    static uint8_t threshold_add = 0;//增加曝光度
    
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_init();                       // 调试串口信息初始化
    
    //GPIO初始化
    gpio_init(P20_0,GPI,1,GPI_PULL_UP);//按键
    gpio_init(P20_3,GPI,1,GPI_PULL_UP);//按键
    gpio_init(P20_2,GPI,1,GPI_PULL_UP);//按键
    gpio_init(P19_0,GPO,1,GPO_PUSH_PULL);//板子上的灯P19_0
    gpio_init(P19_4,GPO,0,GPO_PUSH_PULL);//蜂鸣器P19_4
    
    //定时器初始化
    timer_init(TC_TIME2_CH0, TIMER_US);//us
    
    
    //外设初始化
    mt9v03x_init();//摄像头初始化
    //imu660ra_init();//姿态传感器初始化
    moter_init();//电机初始化
    ips200_init(IPS200_TYPE_PARALLEL8);//屏幕初始化
    
    //编码器初始化
    encoder_quad_init(TC_CH07_ENCODER, TC_CH07_ENCODER_CH1_P07_6, TC_CH07_ENCODER_CH2_P07_7);
    encoder_quad_init(TC_CH20_ENCODER, TC_CH20_ENCODER_CH1_P08_1, TC_CH20_ENCODER_CH2_P08_2);
   
    //中断初始化
    pit_ms_init(PIT_CH0, 10);//初始化 PIT0 为周期中断 10ms
    
    //PID初始化
    PID_init(&PID_Steering,5,0,2,94);//直线PID
    PID_init(&PID_Speed_L,12,0.5,2,Encoder_speed(5));//左电机闭环,你可以设置的目标值,最大Encoder_speed 100
    PID_init(&PID_Speed_R,12,0.5,2,Encoder_speed(5));//右电机闭环,你可以设置的目标值,最大为Encoder_speed 100
    
    //交叉内存参数初始化
    cross_Memory_Init();
    
    adc_init(ADC0_CH00_P06_0, ADC_12BIT); //检测电池电压
    //屏幕初始化
    ips200_Printf(190,0,(ips200_font_size_enum)0,"Battery");
    ips200_Printf(0,300,(ips200_font_size_enum)0,"speed:");
    ips200_Printf(0,152,(ips200_font_size_enum)0,"threshold:");//摄像头曝光度
    ips200_Printf(0,160,(ips200_font_size_enum)0,"Moter_V:");//电机初始速度
    ips200_Printf(0,168,(ips200_font_size_enum)0,"motor:");// 电机使能
    ips200_Printf(0,216,(ips200_font_size_enum)0,"pid:");//显示PID转向环
    ips200_Printf(0,288,(ips200_font_size_enum)0,"EncoderL:");//左边编码器值
    ips200_Printf(120,288,(ips200_font_size_enum)0,"EncoderR:");//右边编码器值
    while(true)
    {
        timer_clear(TC_TIME2_CH0) ;
        timer_start(TC_TIME2_CH0) ;
        //////////////////////////////////////////////////////////////////////////////////////////////////////
        //Encoder_Test();//用于编码器测试,取消注释之后，电机马上以最大速度前进，并且记录下编码器的最大速度
        
        //循迹方案
        threshold = otsu_threshold(mt9v03x_image[0]) + threshold_add;//大津法自适应算法5ms
        //binarizeImage_ZIP(threshold);//压缩后的二值化图像
        binarizeImage(threshold);//二值化图像
        //Trace_middleLine();//普通扫线
        //Trace_Eight_fields_new();//优化八邻域
        
        if(motor_flag == 0){//如果电机没有打开允许屏幕运行
            Screen_Add(threshold);//显示屏添加
            ips200_Printf(80,ArrowPos,(ips200_font_size_enum)0,"<<");
            ips200_Printf(190,8,(ips200_font_size_enum)1,"%.2fV ",(float)adc_convert(ADC0_CH00_P06_0)/4096 * 3.3*4.1);//显示电池电压
            
            ips200_Printf(50,168,(ips200_font_size_enum)0,"%d",motor_flag);// 电机使能
            ips200_Printf(0,128,(ips200_font_size_enum)0,"Kp:%.1f ",PID_Steering.Kp);//kp
            ips200_Printf(0,136,(ips200_font_size_enum)0,"Ki:%.1f ",PID_Steering.Ki);//ki
            ips200_Printf(0,144,(ips200_font_size_enum)0,"Kd:%.1f ",PID_Steering.Kd);//kd
            ips200_Printf(60,152,(ips200_font_size_enum)0,"%d ",threshold);//摄像头曝光度
            ips200_Printf(50,160,(ips200_font_size_enum)0,"%d ",motor_base);//电机初始速度
            ips200_Printf(24,216,(ips200_font_size_enum)0,"%.1f ",PID_Steering.OUT);//显示PID转向环
            ips200_Printf(0,272,(ips200_font_size_enum)0,"pidL:%d ",PID_Speed_L.OUT);//显示L边电机pwm值
            ips200_Printf(120,272,(ips200_font_size_enum)0,"pidR:%d ",PID_Speed_R.OUT);//显示R边电机pwm值  
        }
        //Steering_FeedBack(&PID_Steering);//转向环
        //Speed_FeedBack(&PID_Speed_R,Right);//右电机速度环
        //Speed_FeedBack(&PID_Speed_L,Left);//左电机速度环
        //Cascade_FeedBack(&PID_Steering,&PID_Speed_L,&PID_Speed_R);//串级PID
        
        //////////////////////////////////////////////////////////////////////////////////////////////////////
        ips200_Printf(40,300,(ips200_font_size_enum)0,"%d ",speed);//显示运行速度
        //ips200_Printf(80,300,(ips200_font_size_enum)0,"M2:%d ",speed);//第二个核心运行速度显示运行速度
        speed = timer_get(TC_TIME2_CH0);
        timer_stop(TC_TIME2_CH0);
        //cross_Memory();//交叉内存
        //SCB_CleanInvalidateDCache_by_Addr(&m7_1_data, sizeof(m7_1_data));//更新RAM区数据
    }
}

