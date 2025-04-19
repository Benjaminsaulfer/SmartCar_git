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
* �ļ�����          main_cm7_1
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
#include "trace.h"

#pragma location = 0x28001000
float m7_1_data[20] = {0};
//��ַ�䶯50*4=200byte,0x28001000 + 200(C8)

#define M0_speed m7_1_data[0]
#define M1_speed m7_1_data[1]
#define motor_flag m7_1_data[2]//�����־λ 
#define Battery_V m7_1_data[3]
#define EncoderL m7_1_data[4]
#define EncoderR m7_1_data[5]

void Baterry_ChecK(){
  if(Battery_V>= 6  &&  Battery_V <= 7.3 )//����2S��ض���(6~732����)
    gpio_toggle_level(P19_4);
  else if(Battery_V >= 9  &&  Battery_V <= 10.8 )//����3S��ض���9~10.8����)
    gpio_toggle_level(P19_4);
  else
    gpio_set_level(P19_4,0);
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// ʱ�����ü�ϵͳ��ʼ��<��ر���>
    debug_info_init();                  // ���Դ�����Ϣ��ʼ��
    
    //GPIO��ʼ��
    gpio_init(P20_0,GPI,1,GPI_PULL_UP);//����
    gpio_init(P20_3,GPI,1,GPI_PULL_UP);//����
    gpio_init(P20_2,GPI,1,GPI_PULL_UP);//����
    gpio_init(P19_0,GPO,1,GPO_PUSH_PULL);//�����ϵĵ�P19_0
    gpio_init(P19_4,GPO,0,GPO_PUSH_PULL);//������P19_4  
  
    //��ʱ����ʼ��
    timer_init(TC_TIME2_CH1, TIMER_US);//�򿪶�ʱ�����ٵ�λus   
    timer_init(TC_TIME2_CH0, TIMER_US);//�򿪶�ʱ�����ٵ�λus   

    //��������ʼ��
    encoder_quad_init(TC_CH07_ENCODER, TC_CH07_ENCODER_CH1_P07_6, TC_CH07_ENCODER_CH2_P07_7);
    encoder_quad_init(TC_CH20_ENCODER, TC_CH20_ENCODER_CH1_P08_1, TC_CH20_ENCODER_CH2_P08_2);
    
    //�жϳ�ʼ��
    pit_ms_init(PIT_CH0, 10);//��ʼ�� PIT0 Ϊ�����ж� 10ms

    adc_init(ADC0_CH00_P06_0, ADC_12BIT); //����ص�ѹ
    while(true)
    {
        timer_clear(TC_TIME2_CH1);
        timer_start(TC_TIME2_CH1);
        SCB_CleanInvalidateDCache_by_Addr(&m7_1_data, sizeof(m7_1_data));//����RAM����
        Baterry_ChecK();                                                  //����ص�ѹ
        ///////////////////////////////////////��������///////////////////////////////////////////
        
        Battery_V = (float)adc_convert(ADC0_CH00_P06_0)/4096 * 3.3*4.1;
        
        if(motor_flag == 1){//������˵��

        }
        
        //////////////////////////////////////��������//////////////////////////////////////////
        M1_speed = timer_get(TC_TIME2_CH1);
        timer_stop(TC_TIME2_CH1);
    }
}

// **************************** �������� ****************************
