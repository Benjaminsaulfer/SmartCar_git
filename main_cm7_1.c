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
* 文件名称          main_cm7_1
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

#pragma location = 0x28001000
float m7_1_data[50] = {0};//CPU速度
//地址变动50*4=200byte,0x28001000 + 200(C8)

uint16 ArrowPos = 128;//arrow position

void key_even(){
    //如果按键按下且电机关闭的时候则箭头移动
    if(gpio_get_level(P20_2) == 0 && m7_1_data[1] == 0)
    {
        while (gpio_get_level(P20_2) == 0)
        ips200_Printf(80,ArrowPos,(ips200_font_size_enum)0,"   ");
        ArrowPos+=8;
        if(ArrowPos>=176)ArrowPos=128;
    }
    if(m7_1_data[1] == 0){//如果电机关闭
      switch(ArrowPos){
         case 128://Kp
            if(gpio_get_level(P20_0) == 0)m7_1_data[2]+=0.1;
            if(gpio_get_level(P20_3) == 0)m7_1_data[2]-=0.1;
            break;
         case 136://Ki
            if(gpio_get_level(P20_0) == 0)m7_1_data[3]+=0.1;
            if(gpio_get_level(P20_3) == 0)m7_1_data[3]-=0.1;
            break;
         case 144://Kd
            if(gpio_get_level(P20_0) == 0)m7_1_data[4]+=0.1;
            if(gpio_get_level(P20_3) == 0)m7_1_data[4]-=0.1;
            break;
         case 152://曝光度
            if(gpio_get_level(P20_0) == 0 && m7_1_data[5]<200)m7_1_data[5]++;
            if(gpio_get_level(P20_3) == 0 && m7_1_data[5]>0)m7_1_data[5]--;
            break;
         case 160://电机速度
            if(gpio_get_level(P20_0) == 0)m7_1_data[6]+=100;
            if(gpio_get_level(P20_3) == 0)m7_1_data[6]-=100;
            break;
         case 168://电机使能
            if(gpio_get_level(P20_0) == 0)m7_1_data[1] = 0;
            if(gpio_get_level(P20_3) == 0){
              m7_1_data[1] = 1;
              ips200_Printf(0,168,(ips200_font_size_enum)0,"motor:  %d",(uint32_t)m7_1_data[1]);// 电机使能
            }
            break;
      }
    }else{//如果电机打开
      if(gpio_get_level(P20_0) == 0)m7_1_data[1] = 0;
         if(gpio_get_level(P20_3) == 0){
          m7_1_data[1] = 1;
          ips200_Printf(0,168,(ips200_font_size_enum)0,"motor:  %d",(uint32_t)m7_1_data[1]);// 电机使能
      }
    }
}

void Baterry_ChecK(){
  if(m7_1_data[1]>= 6 &&   m7_1_data[1] <= 7.2 ){//对于2S电池而言(6~7.2警告)
    gpio_toggle_level(P19_4);
  }
  else if(m7_1_data[1]>= 9 &&   m7_1_data[1] <= 10.8 ){//对于3S电池而言9~10.8警告)
    gpio_toggle_level(P19_4);
  }
}

int main(void)
{
    static uint32_t speed;
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_info_init();                  // 调试串口信息初始化
    
    //ips200_init(IPS200_TYPE_PARALLEL8);//屏幕初始化
    timer_init(TC_TIME2_CH1, TIMER_US);//us
    adc_init(ADC0_CH00_P06_0, ADC_12BIT); //检测电池电压
    /*
    ips200_Printf(190,0,(ips200_font_size_enum)0,"Battery");
    ips200_Printf(0,300,(ips200_font_size_enum)0,"speed:");
    ips200_Printf(0,152,(ips200_font_size_enum)0,"threshold:");//摄像头曝光度
    ips200_Printf(0,160,(ips200_font_size_enum)0,"Moter_V:");//电机初始速度
    ips200_Printf(0,168,(ips200_font_size_enum)0,"motor:");// 电机使能
    ips200_Printf(0,216,(ips200_font_size_enum)0,"pid:");//显示PID转向环
    ips200_Printf(0,288,(ips200_font_size_enum)0,"EncoderL:");//左边编码器值
    ips200_Printf(120,288,(ips200_font_size_enum)0,"EncoderR:");//右边编码器值
  */
    while(true)
    {
        timer_clear(TC_TIME2_CH1);
        timer_start(TC_TIME2_CH1);
        ////////////////////////////////////////////////////////////////////////////////////
        
        SCB_CleanInvalidateDCache_by_Addr(&m7_1_data, sizeof(m7_1_data));//更新RAM数据
        //key_even();
        Baterry_ChecK();
        /*
 
        if(m7_1_data[1] == 0){
            ips200_Printf(80,ArrowPos,(ips200_font_size_enum)0,"<<");
            ips200_Printf(190,8,(ips200_font_size_enum)1,"%.2fV ",(float)adc_convert(ADC0_CH00_P06_0)/4096 * 3.3*4.1);//显示电池电压
            
            ips200_Printf(50,168,(ips200_font_size_enum)0,"%d",(uint32_t)m7_1_data[1]);// 电机使能
            ips200_Printf(0,128,(ips200_font_size_enum)0,"Kp:%.1f ",m7_1_data[2]);//kp
            ips200_Printf(0,136,(ips200_font_size_enum)0,"Ki:%.1f ",m7_1_data[3]);//ki
            ips200_Printf(0,144,(ips200_font_size_enum)0,"Kd:%.1f ",m7_1_data[4]);//kd
            ips200_Printf(60,152,(ips200_font_size_enum)0,"%d ",(uint32_t)m7_1_data[5]);//摄像头曝光度
            ips200_Printf(50,160,(ips200_font_size_enum)0,"%d ",(uint32_t)m7_1_data[6]);//电机初始速度
            ips200_Printf(24,216,(ips200_font_size_enum)0,"%.1f ",m7_1_data[7]);//显示PID转向环
            ips200_Printf(0,272,(ips200_font_size_enum)0,"pidL:%d ",(uint32_t)m7_1_data[8]);//显示L边电机pwm值
            ips200_Printf(120,272,(ips200_font_size_enum)0,"pidR:%d ",(uint32_t)m7_1_data[9]);//显示R边电机pwm值  
        }

        ////////////////////////////////////////////////////////////////////////////////////
        ips200_Printf(40,300,(ips200_font_size_enum)0,"%d ",(uint32_t)m7_1_data[0]);//显示运行速度
        ips200_Printf(80,300,(ips200_font_size_enum)0,"M2:%d ",speed);//第二个核心运行速度显示运行速度
*/
        speed = timer_get(TC_TIME2_CH1);
        timer_stop(TC_TIME2_CH1);
    }
}

// **************************** 代码区域 ****************************
