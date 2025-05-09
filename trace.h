#pragma once

#include "zf_common_headfile.h"
#include "math.h"

////////以下为矩形坐标(x1,y1,x2,y2)//////
#define rectangleL 46,90,92,100 
#define rectangleR 96,90,136,100
#define rectangleLL 34,60,42,100
#define rectangleRR 141,60,149,100
#define rectangleLLL 3,57,42,60
#define rectangleRRR 141,57,180,60

extern uint8 Binary_map[120][188];//binary map
extern uint8_t remenber_point;//记录八邻域扫出来的中点

//矩形点数结构体
typedef struct _Rectangle_Struct{
  uint16_t R;
  uint16_t L;
  uint16_t RR;
  uint16_t LL;
  uint16_t RRR;
  uint16_t LLL;
  uint16_t total;
}Rectangle_Struct;

////////////显示屏叠加//////////////
void Show_Binaray_map();//显示二值化数组
void Screen_Add(uint8 threshold);   

////////////二值化图像//////////////
void binarizeImage(uint8_t ZIP,uint8 threshold);//Convert to binarizeImage
uint16_t otsu_threshold(unsigned char *image);

/////////////循迹方案///////////////
void WhitePoint_amount(Rectangle_Struct* this);//白点算法数量
float Sum_of_Dif(float L1,float R1,float L2,float R2);//4数差比和
float Sum_of_Dif_near(float L1,float R1);//2数差比和

