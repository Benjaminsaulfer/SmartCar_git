#pragma once

#include "zf_common_headfile.h"
#include "math.h"

////////����Ϊ��������(x1,y1,x2,y2)//////
#define rectangleL 46,90,92,100     // 94
#define rectangleR 96,90,136,100
#define rectangleLL 47-12,60,55-12,86+14  // (60-30)*(86-78) = 240  ���� ��51+- 4��76 +- 15��  160
#define rectangleRR 129+12,60,137+12,86+14  // (158-128)*(8) = 240   ���� ��133+- 4 , 76  +- 15��

extern uint8 Binary_map[120][188];//binary map
extern uint8_t remenber_point;//��¼������ɨ�������е�

//���ε����ṹ��
typedef struct _Rectangle_Struct{
  uint16_t R;
  uint16_t L;
  uint16_t RR;
  uint16_t LL;
}Rectangle_Struct;

////////////��ʾ������//////////////
void Show_Binaray_map();//��ʾ��ֵ������
void Screen_Add(uint8 threshold);   

////////////��ֵ��ͼ��//////////////
void binarizeImage(uint8_t ZIP,uint8 threshold);//Convert to binarizeImage
uint16_t otsu_threshold(unsigned char *image);

/////////////ѭ������///////////////
uint16_t White_amount(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2);//�׵�����
void WhitePoint_amount(Rectangle_Struct* this);//�׵��㷨����
float Sum_of_Dif(float L1,float R1,float L2,float R2);//4����Ⱥ�
float Sum_of_Dif_near(float L1,float R1);//2����Ⱥ�
void Trace_middleLine();//Trace_middleLine
void Trace_Eight_fields_new();//�Ż�������

