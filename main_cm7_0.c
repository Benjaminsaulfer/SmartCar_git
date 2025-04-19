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

#define M0_speed m7_1_data[0]
#define M1_speed m7_1_data[1]
#define motor_flag m7_1_data[2]//电机标志位 
#define Battery_V m7_1_data[3] //电池电压
#define EncoderL m7_1_data[4]  //编码器
#define EncoderR m7_1_data[5]  //编码器

uint16 ArrowPos = 128;//arrow position
uint8 threshold_add = 30;//曝光度 
uint16 motor_base = 2500;//电机初始速度

////////编码器////////
int16 Max_encoderL = 0;//编码器最大速度
int16 Max_encoderR = 0;

///////创建PID对象/////
PID PID_Steering;
PID PID_Steering_turn;
PID PID_Speed_R;//右边电机
PID PID_Speed_L;//左边电机

#pragma location = 0x28001000  
__no_init float m7_1_data[20];

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
            if(gpio_get_level(P20_0) == 0)PID_Steering.Kp+=1;
            if(gpio_get_level(P20_3) == 0)PID_Steering.Kp-=1;
            break;
         case 136://Ki
            if(gpio_get_level(P20_0) == 0)PID_Steering.Ki+=1;
            if(gpio_get_level(P20_3) == 0)PID_Steering.Ki-=1;
            break;
         case 144://Kd
            if(gpio_get_level(P20_0) == 0)PID_Steering.Kd+=1;
            if(gpio_get_level(P20_3) == 0)PID_Steering.Kd-=1;
            break;
         case 152://曝光度
            if(gpio_get_level(P20_0) == 0 && threshold_add<200)threshold_add++;
            if(gpio_get_level(P20_3) == 0 && threshold_add>0)threshold_add--;
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
          ips200_Printf(0,168,(ips200_font_size_enum)0,"motor:  %d ",(uint32_t)motor_flag);// 电机使能
      }
    }
}

int main(void)
{     
    static uint8_t threshold=0;
    uint8_t thread = 0;
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_init();                       // 调试串口信息初始化
   
    //外设初始化
    mt9v03x_init();//摄像头初始化
    //imu660ra_init();//姿态传感器初始化
    moter_init();//电机初始化
    
    //PID初始化
    //PID_init(&PID_Steering,5,0,2,94);//直线PID
    PID_init(&PID_Steering,1500,0,200,0);//直线PID
    PID_init(&PID_Speed_L,12,0.5,2,Encoder_speed(5));//左电机闭环,你可以设置的目标值,最大Encoder_speed 100
    PID_init(&PID_Speed_R,12,0.5,2,Encoder_speed(5));//右电机闭环,你可以设置的目标值,最大为Encoder_speed 100

    //屏幕初始化
    ips200_init(IPS200_TYPE_PARALLEL8);//屏幕初始化
    ips200_Printf(190,0,(ips200_font_size_enum)0,"Battery");
    ips200_Printf(0,300,(ips200_font_size_enum)0,"M1:");       //第一个核心的速度
    ips200_Printf(80,300,(ips200_font_size_enum)0,"M2:");//第二个核心运行速度显示运行速度
    ips200_Printf(0,152,(ips200_font_size_enum)0,"threshold:");//摄像头曝光度
    ips200_Printf(0,160,(ips200_font_size_enum)0,"Moter_V:");//电机初始速度
    ips200_Printf(0,168,(ips200_font_size_enum)0,"motor:");// 电机使能
    ips200_Printf(0,216,(ips200_font_size_enum)0,"pid:");//显示PID转向环
    ips200_Printf(0,288,(ips200_font_size_enum)0,"EncoderL:");//左边编码器值
    ips200_Printf(120,288,(ips200_font_size_enum)0,"EncoderR:");//右边编码器值
    adc_init(ADC0_CH00_P06_0, ADC_12BIT); //检测电池电压
    while(true)
    {
        timer_clear(TC_TIME2_CH0) ;
        timer_start(TC_TIME2_CH0) ;
        key_even();
        thread++;
        //Encoder_Test();//用于编码器测试,取消注释之后，电机马上以最大速度前进，并且记录下编码器的最大速度,并打印出来
        ///////////////////////////////////////////测速区间/////////////////////////////////////////////////
        
        ////////////////////循迹方案////////////////////
        if(thread%2)
            threshold = otsu_threshold(mt9v03x_image[0]) + threshold_add;//大津法自适应算法5ms
        binarizeImage(2,threshold);//(压缩率,曝光度)二值化图像
        //Trace_middleLine();//普通扫线
        //Trace_Eight_fields_new();//优化八邻域
        ////////////////////循迹方案///////////////////

        //白点数量
        uint8_t Lnum = White_amount(rectangleL);
        uint8_t Rnum = White_amount(rectangleR);
        uint8_t LLnum = White_amount(rectangleLL);
        uint8_t RRnum = White_amount(rectangleRR);
        
        ips200_Printf(180,180,(ips200_font_size_enum)0,"%.2f ",Sum_of_Dif(Lnum,Rnum,LLnum,RRnum));
        ips200_Printf(160,200,(ips200_font_size_enum)0,"%d ",LLnum);
        ips200_Printf(200,200,(ips200_font_size_enum)0,"%d ",RRnum);
        if(motor_flag == 0){//如果电机没有打开允许屏幕运行
            //Screen_Add(threshold);//显示屏添加化图形
            Show_Binaray_map();//显示实际二值化图像
            ips200_Printf(80,ArrowPos,(ips200_font_size_enum)0,"<<");
            ips200_Printf(188,8,(ips200_font_size_enum)1,"%.2fV",Battery_V);//显示电池电压
            ips200_Printf(50,168,(ips200_font_size_enum)0,"%d",(uint32_t)motor_flag);// 电机使能
            ips200_Printf(0,128,(ips200_font_size_enum)0,"Kp:%.1f ",PID_Steering.Kp);//kp
            ips200_Printf(0,136,(ips200_font_size_enum)0,"Ki:%.1f ",PID_Steering.Ki);//ki
            ips200_Printf(0,144,(ips200_font_size_enum)0,"Kd:%.1f ",PID_Steering.Kd);//kd
            ips200_Printf(60,152,(ips200_font_size_enum)0,"%d ",threshold);//摄像头曝光度
            ips200_Printf(50,160,(ips200_font_size_enum)0,"%d ",motor_base);//电机初始速度
            ips200_Printf(24,216,(ips200_font_size_enum)0,"%.1f ",PID_Steering.OUT);//显示PID转向环
            if(PID_Steering.OUT>0)
              ips200_Printf(100,200,(ips200_font_size_enum)1,"<<<");//显示转向方向
            else
              ips200_Printf(100,200,(ips200_font_size_enum)1,">>>");//显示转向方向
        }
        
        Steering_FeedBack(&PID_Steering,Sum_of_Dif(Lnum,Rnum,LLnum,RRnum));//转向环
        //Speed_FeedBack(&PID_Speed_R,Right);//右电机速度环
        //Speed_FeedBack(&PID_Speed_L,Left);//左电机速度环
        //Cascade_FeedBack(&PID_Steering,&PID_Speed_L,&PID_Speed_R);//串级PID

        if (motor_flag == 1)
        {
            if (Lnum== 0 && Rnum== 0 && LLnum== 0 && RRnum== 0)
            {
              motor_flag = 0;
            }
        
        }
        
        /////////////////////////////////////////测速区间/////////////////////////////////////////////////
        ips200_Printf(20,300,(ips200_font_size_enum)0,"%d ",(uint32_t)M0_speed);//显示第一个核心运行速度
        ips200_Printf(100,300,(ips200_font_size_enum)0,"%d ",(uint32_t)M1_speed);//显示第二个核心运行速度
        ips200_Printf(24,216,(ips200_font_size_enum)0,"%.1f ",PID_Steering.OUT);//显示PID转向环
        SCB_CleanInvalidateDCache_by_Addr(&m7_1_data, sizeof(m7_1_data));//更新RAM区数据
        M0_speed = timer_get(TC_TIME2_CH0);
        timer_stop(TC_TIME2_CH0);
    }
}

