#pragma once

#include "zf_common_headfile.h"
#include "math.h"

////////以下为矩形坐标(x1,y1,x2,y2)//////
#define rectangleL 38,90,92,100     // 94
#define rectangleR 96,90,150,100
#define rectangleLL 47,60,55,86  // (60-30)*(86-78) = 240  中心 （51+- 4，76 +- 15）  160
#define rectangleRR 129,60,137,86  // (158-128)*(8) = 240   中心 （133+- 4 , 76  +- 15）

extern uint8 Binary_map[120][188];//binary map
extern uint8_t remenber_point;//记录八邻域扫出来的中点

////////////显示屏叠加//////////////
void Show_Binaray_map();//显示二值化数组
void Screen_Add(uint8 threshold);   

////////////二值化图像//////////////
void binarizeImage(uint8_t ZIP,uint8 threshold);//Convert to binarizeImage
uint16_t otsu_threshold(unsigned char *image);

/////////////扫线方案///////////////
uint8_t White_amount(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2);//白点数量
float Sum_of_Dif(float L1,float R1,float L2,float R2);
void Trace_middleLine();//Trace_middleLine
void Trace_Eight_fields_new();//优化八邻域

