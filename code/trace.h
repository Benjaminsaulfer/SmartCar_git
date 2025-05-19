#pragma once

#include "zf_common_headfile.h"
#include "math.h"

////////����Ϊ��������(x1,y1,x2,y2)//////
#define rectangleL 46,90,93,96 
#define rectangleR 93,90,136,96
#define rectangleLL 34,80,84,82
#define rectangleRR 104,80,154,82

#define guideLL 70,80,90,82
#define guideRR 98,80,114,82
extern uint8 Binary_map[120][188];//binary map

//���ε����ṹ��
typedef struct _Rectangle_Struct{
  uint16_t R;
  uint16_t L;
  uint16_t RR;
  uint16_t LL;
  uint8_t GR;
  uint8_t GL;
  uint16_t total;
}Rectangle_Struct;

////////////��ʾ������//////////////
void Show_Binaray_map();//��ʾ��ֵ������
void Screen_Add(uint8 threshold);   

////////////��ֵ��ͼ��//////////////
void binarizeImage(uint8_t ZIP,uint8 threshold);//Convert to binarizeImage
uint16_t otsu_threshold(unsigned char *image);//���

/////////////ѭ������///////////////
void WhitePoint_amount(Rectangle_Struct* this);//�׵��㷨����
float Sum_of_Dif(uint16 L1,uint16 R1);//2����Ⱥ�

