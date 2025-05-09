#pragma once

#include "zf_common_headfile.h"
#include "math.h"

////////����Ϊ��������(x1,y1,x2,y2)//////
#define rectangleL 46,90,92,100 
#define rectangleR 96,90,136,100
#define rectangleLL 34,60,42,100
#define rectangleRR 141,60,149,100
#define rectangleLLL 3,57,42,60
#define rectangleRRR 141,57,180,60

extern uint8 Binary_map[120][188];//binary map
extern uint8_t remenber_point;//��¼������ɨ�������е�

//���ε����ṹ��
typedef struct _Rectangle_Struct{
  uint16_t R;
  uint16_t L;
  uint16_t RR;
  uint16_t LL;
  uint16_t RRR;
  uint16_t LLL;
  uint16_t total;
}Rectangle_Struct;

////////////��ʾ������//////////////
void Show_Binaray_map();//��ʾ��ֵ������
void Screen_Add(uint8 threshold);   

////////////��ֵ��ͼ��//////////////
void binarizeImage(uint8_t ZIP,uint8 threshold);//Convert to binarizeImage
uint16_t otsu_threshold(unsigned char *image);

/////////////ѭ������///////////////
void WhitePoint_amount(Rectangle_Struct* this);//�׵��㷨����
float Sum_of_Dif(float L1,float R1,float L2,float R2);//4����Ⱥ�
float Sum_of_Dif_near(float L1,float R1);//2����Ⱥ�

