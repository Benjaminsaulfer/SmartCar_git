#pragma once

#include "zf_common_headfile.h"
#include "math.h"

//һ��Ϊ��������
#define rectangleL 60,90,90,100
#define rectangleR 98,90,128,100
#define rectangleLL 20,70,60,76
#define rectangleRR 128,70,168,76

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

