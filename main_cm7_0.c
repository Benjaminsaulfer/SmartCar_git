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

/*/////////////////����///////////
#define UART_INDEX              (DEBUG_UART_INDEX   )                           // Ĭ�� UART_0
#define UART_BAUDRATE           (DEBUG_UART_BAUDRATE)                           // Ĭ�� 115200
#define UART_TX_PIN             (DEBUG_UART_TX_PIN  )                           // Ĭ�� UART0_TX_P00_1
#define UART_RX_PIN             (DEBUG_UART_RX_PIN  )                           // Ĭ�� UART0_RX_P00_0

uint8 uart_get_data[64];                                                        // ���ڽ������ݻ�����
uint8 fifo_get_data[64];                                                        // fifo �������������

uint8  get_data = 0;                                                            // �������ݱ���
uint32 fifo_data_count = 0;                                                     // fifo ���ݸ���

fifo_struct uart_data_fifo;
*//////////////////����///////////

#define M0_speed m7_1_data[0]
#define M1_speed m7_1_data[1]
#define motor_flag m7_1_data[2]//�����־λ 
#define Battery_V m7_1_data[3] //��ص�ѹ
#define EncoderL m7_1_data[4]  //������
#define EncoderR m7_1_data[5]  //������

uint16 ArrowPos = 128;//arrow position
uint8 threshold_add = 30;//�ع�� 
uint16 motor_base = 2500;//�����ʼ�ٶ�

////////������////////
int16 Max_encoderL = 0;//����������ٶ�
int16 Max_encoderR = 0;

///////����PID����/////
PID PID_Steering;
PID PID_Steering_turn;
PID PID_Speed_R;//�ұߵ��
PID PID_Speed_L;//��ߵ��

#pragma location = 0x28001000  
__no_init float m7_1_data[20];

void key_even(){
    //������������ҵ���رյ�ʱ�����ͷ�ƶ�
    if(gpio_get_level(P20_2) == 0 && motor_flag == 0)
    {
        while (gpio_get_level(P20_2) == 0)
        ips200_Printf(80,ArrowPos,(ips200_font_size_enum)0,"   ");
        ArrowPos+=8;
        if(ArrowPos>=176)ArrowPos=128;
    }
    if(motor_flag == 0){//�������ر�
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
         case 152://�ع��
            if(gpio_get_level(P20_0) == 0 && threshold_add<200)threshold_add++;
            if(gpio_get_level(P20_3) == 0 && threshold_add>0)threshold_add--;
            break;
         case 160://����ٶ�
            if(gpio_get_level(P20_0) == 0)motor_base+=100;
            if(gpio_get_level(P20_3) == 0)motor_base-=100;
            break;
         case 168://���ʹ��
            if(gpio_get_level(P20_0) == 0)motor_flag = 0;
            if(gpio_get_level(P20_3) == 0){
              motor_flag = 1;
              ips200_Printf(0,168,(ips200_font_size_enum)0,"motor:  %d",motor_flag);// ���ʹ��
            }
            break;
      }
    }else{//��������
      if(gpio_get_level(P20_0) == 0)motor_flag = 0;
         if(gpio_get_level(P20_3) == 0){
          motor_flag = 1;
          ips200_Printf(0,168,(ips200_font_size_enum)0,"motor:  %d ",(uint32_t)motor_flag);// ���ʹ��
      }
    }
}

int main(void)
{     
    static uint8_t threshold=0;
    uint8_t thread = 0;
    clock_init(SYSTEM_CLOCK_250M); 	// ʱ�����ü�ϵͳ��ʼ��<��ر���>
    debug_init();                       // ���Դ�����Ϣ��ʼ��
   
    //�����ʼ��
    mt9v03x_init();//����ͷ��ʼ��
    //imu660ra_init();//��̬��������ʼ��
    moter_init();//�����ʼ��
    
    //PID��ʼ��
    //PID_init(&PID_Steering,5,0,2,94);//ֱ��PID
    PID_init(&PID_Steering,1500,0,200,0);//ֱ��PID
    PID_init(&PID_Speed_L,12,0.5,2,Encoder_speed(5));//�����ջ�,��������õ�Ŀ��ֵ,���Encoder_speed 100
    PID_init(&PID_Speed_R,12,0.5,2,Encoder_speed(5));//�ҵ���ջ�,��������õ�Ŀ��ֵ,���ΪEncoder_speed 100

    //��Ļ��ʼ��
    ips200_init(IPS200_TYPE_PARALLEL8);//��Ļ��ʼ��
    ips200_Printf(190,0,(ips200_font_size_enum)0,"Battery");
    ips200_Printf(0,300,(ips200_font_size_enum)0,"M1:");       //��һ�����ĵ��ٶ�
    ips200_Printf(80,300,(ips200_font_size_enum)0,"M2:");//�ڶ������������ٶ���ʾ�����ٶ�
    ips200_Printf(0,152,(ips200_font_size_enum)0,"threshold:");//����ͷ�ع��
    ips200_Printf(0,160,(ips200_font_size_enum)0,"Moter_V:");//�����ʼ�ٶ�
    ips200_Printf(0,168,(ips200_font_size_enum)0,"motor:");// ���ʹ��
    ips200_Printf(0,216,(ips200_font_size_enum)0,"pid:");//��ʾPIDת��
    ips200_Printf(0,288,(ips200_font_size_enum)0,"EncoderL:");//��߱�����ֵ
    ips200_Printf(120,288,(ips200_font_size_enum)0,"EncoderR:");//�ұ߱�����ֵ
    adc_init(ADC0_CH00_P06_0, ADC_12BIT); //����ص�ѹ
    while(true)
    {
        timer_clear(TC_TIME2_CH0) ;
        timer_start(TC_TIME2_CH0) ;
        key_even();
        thread++;
        //Encoder_Test();//���ڱ���������,ȡ��ע��֮�󣬵������������ٶ�ǰ�������Ҽ�¼�±�����������ٶ�,����ӡ����
        ///////////////////////////////////////////��������/////////////////////////////////////////////////
        
        ////////////////////ѭ������////////////////////
        if(thread%2)
            threshold = otsu_threshold(mt9v03x_image[0]) + threshold_add;//�������Ӧ�㷨5ms
        binarizeImage(2,threshold);//(ѹ����,�ع��)��ֵ��ͼ��
        //Trace_middleLine();//��ͨɨ��
        //Trace_Eight_fields_new();//�Ż�������
        ////////////////////ѭ������///////////////////

        //�׵�����
        uint8_t Lnum = White_amount(rectangleL);
        uint8_t Rnum = White_amount(rectangleR);
        uint8_t LLnum = White_amount(rectangleLL);
        uint8_t RRnum = White_amount(rectangleRR);
        
        ips200_Printf(180,180,(ips200_font_size_enum)0,"%.2f ",Sum_of_Dif(Lnum,Rnum,LLnum,RRnum));
        ips200_Printf(160,200,(ips200_font_size_enum)0,"%d ",LLnum);
        ips200_Printf(200,200,(ips200_font_size_enum)0,"%d ",RRnum);
        if(motor_flag == 0){//������û�д�������Ļ����
            //Screen_Add(threshold);//��ʾ����ӻ�ͼ��
            Show_Binaray_map();//��ʾʵ�ʶ�ֵ��ͼ��
            ips200_Printf(80,ArrowPos,(ips200_font_size_enum)0,"<<");
            ips200_Printf(188,8,(ips200_font_size_enum)1,"%.2fV",Battery_V);//��ʾ��ص�ѹ
            ips200_Printf(50,168,(ips200_font_size_enum)0,"%d",(uint32_t)motor_flag);// ���ʹ��
            ips200_Printf(0,128,(ips200_font_size_enum)0,"Kp:%.1f ",PID_Steering.Kp);//kp
            ips200_Printf(0,136,(ips200_font_size_enum)0,"Ki:%.1f ",PID_Steering.Ki);//ki
            ips200_Printf(0,144,(ips200_font_size_enum)0,"Kd:%.1f ",PID_Steering.Kd);//kd
            ips200_Printf(60,152,(ips200_font_size_enum)0,"%d ",threshold);//����ͷ�ع��
            ips200_Printf(50,160,(ips200_font_size_enum)0,"%d ",motor_base);//�����ʼ�ٶ�
            ips200_Printf(24,216,(ips200_font_size_enum)0,"%.1f ",PID_Steering.OUT);//��ʾPIDת��
            if(PID_Steering.OUT>0)
              ips200_Printf(100,200,(ips200_font_size_enum)1,"<<<");//��ʾת����
            else
              ips200_Printf(100,200,(ips200_font_size_enum)1,">>>");//��ʾת����
        }
        
        Steering_FeedBack(&PID_Steering,Sum_of_Dif(Lnum,Rnum,LLnum,RRnum));//ת��
        //Speed_FeedBack(&PID_Speed_R,Right);//�ҵ���ٶȻ�
        //Speed_FeedBack(&PID_Speed_L,Left);//�����ٶȻ�
        //Cascade_FeedBack(&PID_Steering,&PID_Speed_L,&PID_Speed_R);//����PID

        if (motor_flag == 1)
        {
            if (Lnum== 0 && Rnum== 0 && LLnum== 0 && RRnum== 0)
            {
              motor_flag = 0;
            }
        
        }
        
        /////////////////////////////////////////��������/////////////////////////////////////////////////
        ips200_Printf(20,300,(ips200_font_size_enum)0,"%d ",(uint32_t)M0_speed);//��ʾ��һ�����������ٶ�
        ips200_Printf(100,300,(ips200_font_size_enum)0,"%d ",(uint32_t)M1_speed);//��ʾ�ڶ������������ٶ�
        ips200_Printf(24,216,(ips200_font_size_enum)0,"%.1f ",PID_Steering.OUT);//��ʾPIDת��
        SCB_CleanInvalidateDCache_by_Addr(&m7_1_data, sizeof(m7_1_data));//����RAM������
        M0_speed = timer_get(TC_TIME2_CH0);
        timer_stop(TC_TIME2_CH0);
    }
}

