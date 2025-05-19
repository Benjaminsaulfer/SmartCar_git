#pragma once

#include "zf_common_headfile.h"
#include "math.h"

////////以下为矩形坐标(x1,y1,x2,y2)//////
#define rectangleL 46,90,93,96 
#define rectangleR 93,90,136,96
#define rectangleLL 34,80,84,82
#define rectangleRR 104,80,154,82

#define guideLL 70,80,90,82
#define guideRR 98,80,114,82
extern uint8 Binary_map[120][188];//binary map

//矩形点数结构体
typedef struct _Rectangle_Struct{
  uint16_t R;
  uint16_t L;
  uint16_t RR;
  uint16_t LL;
  uint8_t GR;
  uint8_t GL;
  uint16_t total;
}Rectangle_Struct;

////////////显示屏叠加//////////////
void Show_Binaray_map();//显示二值化数组
void Screen_Add(uint8 threshold);   

////////////二值化图像//////////////
void binarizeImage(uint8_t ZIP,uint8 threshold);//Convert to binarizeImage
uint16_t otsu_threshold(unsigned char *image);//大津法

/////////////循迹方案///////////////
void WhitePoint_amount(Rectangle_Struct* this);//白点算法数量
float Sum_of_Dif(uint16 L1,uint16 R1);//2数差比和

