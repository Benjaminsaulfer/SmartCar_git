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
#include "trace.h"

#pragma location = 0x28001000
float m7_1_data[20] = {0};
//地址变动50*4=200byte,0x28001000 + 200(C8)

#define M0_speed m7_1_data[0]
#define M1_speed m7_1_data[1]
#define Battery_V m7_1_data[3]

void Baterry_ChecK(){
  if(Battery_V>= 6  &&  Battery_V <= 7.3 ){//对于2S电池而言(6~732警告)
    for(uint8_t i = 0;i<10;i++){
      gpio_toggle_level(P19_4);
      system_delay_ms(100);
    }
    gpio_set_level(P19_4,0);
  }
    
  else if(Battery_V >= 9  &&  Battery_V <= 10.8 ){
    for(uint8_t i = 0;i<10;i++){
      gpio_toggle_level(P19_4);
      system_delay_ms(100);
    }
    gpio_set_level(P19_4,0);
  }//对于3S电池而言9~10.8警告)
  
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_info_init();                  // 调试串口信息初始化
    
    //GPIO初始化
    gpio_init(P20_0,GPI,1,GPI_PULL_UP);//按键
    gpio_init(P20_3,GPI,1,GPI_PULL_UP);//按键
    gpio_init(P20_2,GPI,1,GPI_PULL_UP);//按键
    gpio_init(P19_0,GPO,1,GPO_PUSH_PULL);//板子上的灯P19_0
    gpio_init(P19_4,GPO,0,GPO_PUSH_PULL);//蜂鸣器P19_4  
    gpio_init(P06_1,GPO,0,GPO_PUSH_PULL);//MOter
  
    //定时器初始化
    timer_init(TC_TIME2_CH1, TIMER_US);//打开定时器测速单位us   
    timer_init(TC_TIME2_CH0, TIMER_US);//打开定时器测速单位us   

    adc_init(ADC0_CH00_P06_0, ADC_12BIT); //检测电池电压
    while(true)
    {
        timer_clear(TC_TIME2_CH1);
        timer_start(TC_TIME2_CH1);
        SCB_CleanInvalidateDCache_by_Addr(&m7_1_data, sizeof(m7_1_data));//更新RAM数据
        Baterry_ChecK();                                                  //检测电池电压
        ///////////////////////////////////////测速区间///////////////////////////////////////////
        Battery_V = (float)adc_convert(ADC0_CH00_P06_0)/4096 * 3.3*4.1;
        
        //////////////////////////////////////测速区间//////////////////////////////////////////
        M1_speed = timer_get(TC_TIME2_CH1);
        timer_stop(TC_TIME2_CH1);
    }
}

// **************************** 代码区域 ****************************