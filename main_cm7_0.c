/*********************************************************************************************************************
* CYT4BB Opensourec Library ���� CYT4BB ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� CYT4BB ��Դ���һ����
*
* CYT4BB ��Դ�� ��������
* �����Ը��������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ���֤Э�� �����������Ϊ���İ汾
* �������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ���֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          main_cm7_0
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 9.40.1
* ����ƽ̨          CYT4BB
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "moter.h"
#include "trace.h"

/////////////////����///////////
#define UART_INDEX              (DEBUG_UART_INDEX   )                           // Ĭ�� UART_0
#define UART_BAUDRATE           (DEBUG_UART_BAUDRATE)                           // Ĭ�� 115200
#define UART_TX_PIN             (DEBUG_UART_TX_PIN  )                           // Ĭ�� UART0_TX_P00_1
#define UART_RX_PIN             (DEBUG_UART_RX_PIN  )                           // Ĭ�� UART0_RX_P00_0

uint8 uart_get_data[64];                                                        // ���ڽ������ݻ�����
uint8 fifo_get_data[64];                                                        // fifo �������������

uint8  get_data = 0;                                                            // �������ݱ���
uint32 fifo_data_count = 0;                                                     // fifo ���ݸ���

fifo_struct uart_data_fifo;
/////////////////����///////////

//#define M0_speed m7_1_data[0]//��һ�������ٶ�
#define M1_speed m7_1_data[1]//�ڶ��������ٶ�
#define Battery_V m7_1_data[3] //��ص�ѹ
 
uint32_t M0_speed;
uint16 ArrowPos = 128;//arrow position
int8 threshold_add = 0;//�ع�� 
uint8_t threshold_gate = 120;
uint16 motor_base = 400;//�����ʼ�ٶ�
int8 PID_ = 0;//�ڼ���PID
uint32_t CPU_Speed=0;
////////��־λ////////


////////������////////
int16 Max_encoderL = 0;//����������ٶ�
int16 Max_encoderR = 0;
int16 EncoderL;
int16 EncoderR;

///////����PID����/////
PID PID_Steering;
PID PID_Steering_turn;
PID PID_Speed_R;//�ұߵ��
PID PID_Speed_L;//��ߵ��

Rectangle_Struct REC;//���νṹ��

#pragma location = 0x28001000  
__no_init float m7_1_data[20];

void key_even(){
    //������������ҵ���رյ�ʱ�����ͷ�ƶ�
    if(gpio_get_level(P20_2) == 0 && gpio_get_level(P06_1) == 0)
    {
        while (gpio_get_level(P20_2) == 0)
        ips200_Printf(60,ArrowPos,(ips200_font_size_enum)0,"   ");
        ArrowPos+=8;
        if(ArrowPos>=192)ArrowPos=128;
    }
    if(gpio_get_level(P06_1) == 0){//�������ر�
      switch(ArrowPos){
         case 128://PIDѡ��
           if(gpio_get_level(P20_0) == 0) {
             while(gpio_get_level(P20_0)==0); 
             PID_++;
           }
           else if(gpio_get_level(P20_3) == 0){
             while(gpio_get_level(P20_3)==0);
             PID_--;
           }
           //��λ����
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
         case 160://�ع��
            if(gpio_get_level(P20_0) == 0 && threshold_add<125)threshold_add++;
            else if(gpio_get_level(P20_3) == 0 && threshold_add>-20)threshold_add--;
            break;
         case 168://����ٶ�
            if(gpio_get_level(P20_0) == 0)motor_base+=100;
            else if(gpio_get_level(P20_3) == 0)motor_base-=100;
            break;
         case 176://���ʹ��
            if(gpio_get_level(P20_0) == 0)gpio_set_level(P06_1,0);
            else if(gpio_get_level(P20_3) == 0){
              gpio_set_level(P06_1,1);
              ips200_Printf(46,176,(ips200_font_size_enum)0,"1");// ���ʹ��
            }
            break;
         case 184://���ʹ��
            if(gpio_get_level(P20_0) == 0)threshold_gate++;
            else if(gpio_get_level(P20_3) == 0)threshold_gate--;
            break;
      }
    }else{//��������
      if(gpio_get_level(P20_0) == 0)gpio_set_level(P06_1,0);
      else if(gpio_get_level(P20_3) == 0)gpio_set_level(P06_1,1);  
    }
}

void Scrren_Init(){
    ips200_init(IPS200_TYPE_PARALLEL8);//��Ļ��ʼ��
    ips200_pen_color(RGB565_BLUE);
    ips200_show_chinese(190,0, 16, dianya[0], 2, RGB565_RED);//��ʾfont�ļ������
    //ips200_Printf(190,0,(ips200_font_size_enum)0,"Battery");
    ips200_Printf(8,128,(ips200_font_size_enum)0,"pid:");//�����ʼ�ٶ�
    ips200_Printf(0,136,(ips200_font_size_enum)0,"Kp:");//kp
    ips200_Printf(0,144,(ips200_font_size_enum)0,"Ki:");//ki
    ips200_Printf(0,152,(ips200_font_size_enum)0,"Kd:");//kd
    ips200_Printf(0,160,(ips200_font_size_enum)0,"light:");//����ͷ�ع��
    ips200_Printf(0,168,(ips200_font_size_enum)0,"M_V:");//�����ʼ�ٶ�
    ips200_Printf(0,176,(ips200_font_size_enum)0,"motor:");// ���ʹ��
    ips200_Printf(0,184,(ips200_font_size_enum)0,"gate:");// ���ʹ��
    ips200_Printf(0,216,(ips200_font_size_enum)0,"Linepid:");//��ʾPIDת��
    ips200_Printf(0,224,(ips200_font_size_enum)0,"Turnpid:");//��ʾPIDת��
    ips200_Printf(0,280,(ips200_font_size_enum)0,"targetL:");//��ʾL�ߵ��pwmֵ
    ips200_Printf(122,280,(ips200_font_size_enum)0,"targetR:");//��ʾL�ߵ��pwmֵ
    ips200_Printf(0,288,(ips200_font_size_enum)0,"EncoderL:");//��߱�����ֵ
    ips200_Printf(120,288,(ips200_font_size_enum)0,"EncoderR:");//�ұ߱�����ֵ
    ips200_Printf(0,300,(ips200_font_size_enum)0,"M1:");       //��һ�����ĵ��ٶ�
    ips200_Printf(80,300,(ips200_font_size_enum)0,"M2:");//�ڶ������������ٶ���ʾ�����ٶ�
    ips200_draw_rectangle(120,160,220,240,RGB565_GRAY);
    ips200_pen_color(RGB565_RED);
}

static void bubbleSort(uint32_t arr[], uint8_t n) {
    uint8_t i, j;
    uint32_t temp;
    for (i = 0; i < n - 1; i++) {
        for (j = 0; j < n - i - 1; j++) {
            if (arr[j] > arr[j + 1]) {
                // ���� arr[j] �� arr[j+1]
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
    clock_init(SYSTEM_CLOCK_250M); 	// ʱ�����ü�ϵͳ��ʼ��<��ر���>
    debug_init();                       // ���Դ�����Ϣ��ʼ��
   
    //�����ʼ��
    mt9v03x_init();//����ͷ��ʼ��
    //imu660ra_init();//��̬��������ʼ��
    moter_init();//�����ʼ��
        //��������ʼ��
    encoder_quad_init(TC_CH07_ENCODER, TC_CH07_ENCODER_CH1_P07_6, TC_CH07_ENCODER_CH2_P07_7);
    encoder_quad_init(TC_CH20_ENCODER, TC_CH20_ENCODER_CH1_P08_1, TC_CH20_ENCODER_CH2_P08_2);
    
    //PID��ʼ��
    PID_init(&PID_Steering,95,0,200,0);//ֱ��PID
    PID_init(&PID_Steering_turn,450,0,2,0);//���PID
    //PID_init(&PID_Speed_L,2.5,1,0.5,motor_base);//�����ջ�,��������õ�Ŀ��ֵ,���Encoder_speed 100
    //PID_init(&PID_Speed_R,2.5,1,0.5,motor_base);//�ҵ���ջ�,��������õ�Ŀ��ֵ,���ΪEncoder_speed 100
    PID_init(&PID_Speed_L,2,0.5,0.5,motor_base);//�����ջ�,��������õ�Ŀ��ֵ,���Encoder_speed 100
    PID_init(&PID_Speed_R,2,0.5,0.5,motor_base);//�ҵ���ջ�,��������õ�Ŀ��ֵ,���ΪEncoder_speed 100
    //��ʱ���ж�
    pit_ms_init(PIT_CH0, 10);//��ʼ�� PIT0 Ϊ�����ж� 10ms
    //��Ļ��ʼ��
    Scrren_Init();
    adc_init(ADC0_CH00_P06_0, ADC_12BIT); //����ص�ѹ
    
    while(true)
    {
        timer_clear(TC_TIME2_CH0) ;
        timer_start(TC_TIME2_CH0) ;
        key_even();
        thread++;
        ///////////////////////////////////////////��������/////////////////////////////////////////////////
        
        
        ////////////////////ѭ������////////////////////
        if(thread%2){
          threshold = otsu_threshold(mt9v03x_image[0]) + threshold_add;//�������Ӧ�㷨5ms
          if(threshold<=threshold_gate)threshold = threshold_gate;//�����ع�
        }
        
        binarizeImage(2,threshold);
        //Trace_middleLine();//��ͨɨ��
        //Trace_Eight_fields_new();//�Ż�������
        WhitePoint_amount(&REC);//�׵��㷨
        
        if(gpio_get_level(P06_1) == 0){//������û�д�������Ļ����
            //Screen_Add(threshold);//��ʾ����ӻ�ͼ��
            Show_Binaray_map();//��ʾʵ�ʶ�ֵ��ͼ��
            ips200_Printf(60,ArrowPos,(ips200_font_size_enum)0,"<<");
            ips200_Printf(188,18,(ips200_font_size_enum)1,"%.2fV",Battery_V);//��ʾ��ص�ѹ  
            ips200_Printf(36,128,(ips200_font_size_enum)0,"%d",PID_);//�л�PID
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
            ips200_Printf(36,160,(ips200_font_size_enum)0,"%d ",threshold);//����ͷ�ع��
            ips200_Printf(28,168,(ips200_font_size_enum)0,"%d ",motor_base);//�����ʼ�ٶ�
            ips200_Printf(46,176,(ips200_font_size_enum)0,"%d",gpio_get_level(P06_1));// ���ʹ��
            ips200_Printf(30,184,(ips200_font_size_enum)0,"%d",threshold_gate);//�ع�����
            ips200_Printf(50,216,(ips200_font_size_enum)0,"%.1f ",PID_Steering.OUT);//��ʾPIDת��
            ips200_Printf(50,224,(ips200_font_size_enum)0,"%.1f ",PID_Steering_turn.OUT);//��ʾPIDת��
            ips200_Printf(20,300,(ips200_font_size_enum)0,"%d ",(uint32_t)M0_speed);//��ʾ��һ�����������ٶ�
            ips200_Printf(100,300,(ips200_font_size_enum)0,"%d ",(uint32_t)M1_speed);//��ʾ�ڶ������������ٶ�
            ips200_Printf(126,168,(ips200_font_size_enum)0,"M0Speed:%d ",CPU_Speed/10);
            ips200_Printf(126,176,(ips200_font_size_enum)0,"point:%d ",REC.L+REC.R+REC.LL+REC.RR);
        }
        else{//��¼CPU�ٶ�
            static uint32_t Speed_array[20];
            static uint8_t i = 0;
            static uint8_t once_flag = 1;
            if(i < 20){//��¼����
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
        
        if((REC.R + REC.L + REC.RR + REC.LL) >=340){//����ͣ��
          gpio_set_level(P06_1,0);
        }
        else if(REC.LL <= 3 && REC.RR <= 3 && (REC.R + REC.L)>=4){//ֱ��ת��
          Cascade_FeedBack(&PID_Steering,&PID_Speed_L,&PID_Speed_R,Sum_of_Dif_near(REC.L,REC.R));//����PID
          gpio_set_level(P19_4,0);
        }
        else if(REC.LL > 3 || REC.RR > 3){//���ת��
          ips200_Printf(126,186,(ips200_font_size_enum)0,"error:%.2f ",Sum_of_Dif(REC.L,REC.R,REC.LL,REC.RR));
          //Steering_FeedBack(&PID_Steering_turn,Sum_of_Dif(REC.L,REC.R,REC.LL,REC.RR));
          Cascade_FeedBack(&PID_Steering_turn,&PID_Speed_L,&PID_Speed_R,Sum_of_Dif(REC.L,REC.R,REC.LL,REC.RR));//����PID
          //Last_Error = Sum_of_Dif(REC.L,REC.R,REC.LL,REC.RR);//��¼����ε����
          //gpio_set_level(P19_4,1);
        }
        else if((REC.R + REC.L + REC.RR + REC.LL) <=4){//����
          //gpio_set_level(P19_4,0);//���߾���
          //Steering_FeedBack(&PID_Steering_turn,Last_Error);//ʹ����һ�ε����
        }
        
        ///////���Ժ���///////
        //Encoder_loop_Test(&PID_Speed_L,&PID_Speed_R);//���ڲ����ٶȱջ�����������,��ƽ�PID->OUTӦ�ñƽ�Moter_Base
        //Encoder_Test();//���ڱ���������,ȡ��ע��֮�󣬵������������ٶ�ǰ�������Ҽ�¼�±�����������ٶ�,����ӡ����
        /////////////////////////////////////////��������/////////////////////////////////////////////////
        SCB_CleanInvalidateDCache_by_Addr(&m7_1_data, sizeof(m7_1_data));//����RAM������
        M0_speed = timer_get(TC_TIME2_CH0);//��ȡ��һ�����ĵ�����ʱ��
        timer_stop(TC_TIME2_CH0);
    }
}
