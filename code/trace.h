#pragma once

#include "zf_common_headfile.h"
#include "math.h"

//һ��Ϊ��������
#define rectangleL 68,90,92,100
#define rectangleR 96,90,120,100
#define rectangleLL 30,78,60,86
#define rectangleRR 128,78,158,86

extern uint8 Binary_map[188][120];//binary map
extern uint8_t remenber_point;//��¼������ɨ�������е�

////////////��ʾ������//////////////
void Screen_Add(uint8 threshold);   

////////////��ֵ��ͼ��//////////////
void binarizeImage(uint8_t ZIP,uint8 threshold);//Convert to binarizeImage
uint16_t otsu_threshold(unsigned char *image);

/////////////ɨ�߷���///////////////
uint8_t White_amount(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2);//�׵�����
float Sum_of_Dif(float L1,float R1,float L2,float R2);
void Trace_middleLine();//Trace_middleLine
void Trace_Eight_fields_new();//�Ż�������
void Trace_rectangle();//����ѭ��

