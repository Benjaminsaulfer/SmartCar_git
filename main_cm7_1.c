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

#pragma location = 0x28001000
float m7_1_data[20] = {0};
//��ַ�䶯50*4=200byte,0x28001000 + 200(C8)

uint8_t scrren_flag = 0;

void Baterry_ChecK(){
  if(m7_1_data[10]>= 6  &&  m7_1_data[10] <= 7.2 )//����2S��ض���(6~7.2����)
    gpio_toggle_level(P19_4);
  else if(m7_1_data[10]>= 9  &&  m7_1_data[10] <= 10.8 )//����3S��ض���9~10.8����)
    gpio_toggle_level(P19_4);
}

int main(void)
{
    static uint32_t speed;              //m7_1_data�������ٶ�
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
    timer_init(TC_TIME2_CH0, TIMER_US);//us

    adc_init(ADC0_CH00_P06_0, ADC_12BIT); //����ص�ѹ
    while(true)
    {
        timer_clear(TC_TIME2_CH1);
        timer_start(TC_TIME2_CH1);
        SCB_CleanInvalidateDCache_by_Addr(&m7_1_data, sizeof(m7_1_data));//����RAM����
        Baterry_ChecK();                                //����ص�ѹ
        ///////////////////////////////////////��������///////////////////////////////////////////
        
        m7_1_data[3] = (float)adc_convert(ADC0_CH00_P06_0)/4096 * 3.3*4.1;
        
        if(m7_1_data[2] == 1){//������˵������ô�ڶ������Ľ��������Ļ
            if(scrren_flag == 0){
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
              scrren_flag = 1;
            }
            ips200_Printf(190,8,(ips200_font_size_enum)1,"%.2fV ",(float)adc_convert(ADC0_CH00_P06_0)/4096 * 3.3*4.1);//��ʾ��ص�ѹ
            ips200_Printf(24,216,(ips200_font_size_enum)0,"%.1f ",m7_1_data[7]);//��ʾPIDת��
            ips200_Printf(0,272,(ips200_font_size_enum)0,"pidL:%d ",(uint32_t)m7_1_data[8]);//��ʾL�ߵ��pwmֵ
            ips200_Printf(120,272,(ips200_font_size_enum)0,"pidR:%d ",(uint32_t)m7_1_data[9]);//��ʾR�ߵ��pwmֵ  
            ips200_Printf(20,300,(ips200_font_size_enum)0,"%d ",(uint32_t)m7_1_data[0]);//��ʾ�����ٶ�
            ips200_Printf(100,300,(ips200_font_size_enum)0,"M2:%d ",speed);//�ڶ������������ٶ���ʾ�����ٶ�
        }
        
        //////////////////////////////////////��������//////////////////////////////////////////
        speed = timer_get(TC_TIME2_CH1);
        m7_1_data[1] = speed;
        timer_stop(TC_TIME2_CH1);
    }
}

// **************************** �������� ****************************
