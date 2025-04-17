#pragma once

#include "zf_common_headfile.h"
#include "math.h"

//һ��Ϊ��������
#define rectangleL 68,90,92,100     // 94
#define rectangleR 96,90,120,100
#define rectangleLL 47,60,55,86  // (60-30)*(86-78) = 240  ���� ��51+- 4��76 +- 15��  160
#define rectangleRR 129,60,137,86  // (158-128)*(8) = 240   ���� ��133+- 4 , 76  +- 15��

extern uint8 Binary_map[188][120];//binary map
extern uint8_t remenber_point;//��¼������ɨ�������е�

////////////��ʾ������//////////////
void Screen_Add(uint8 threshold);   

////////////��ֵ��ͼ��//////////////
void binarizeImage(uint8_t ZIP,uint8 threshold);//Convert to binarizeImage
uint16_t otsu_threshold(unsigned char *image);

/////////////ɨ�߷���///////////////
uint8_t White_amount(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2);//�׵�����
float turn_plus(float L1,float R1);//�Ӵ�ת��
float Sum_of_Dif(float L1,float R1,float L2,float R2);
void Trace_middleLine();//Trace_middleLine
void Trace_Eight_fields_new();//�Ż�������
void Trace_rectangle();//����ѭ��

