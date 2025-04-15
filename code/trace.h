#pragma once

#include "zf_common_headfile.h"
#include "math.h"

//一下为矩形坐标
#define rectangleL 60,90,90,100
#define rectangleR 98,90,128,100
#define rectangleLL 20,70,60,76
#define rectangleRR 128,70,168,76

extern uint8 Binary_map[188][120];//binary map
extern uint8_t remenber_point;//记录八邻域扫出来的中点

////////////显示屏叠加//////////////
void Screen_Add(uint8 threshold);   

////////////二值化图像//////////////
void binarizeImage(uint8_t ZIP,uint8 threshold);//Convert to binarizeImage
uint16_t otsu_threshold(unsigned char *image);

/////////////扫线方案///////////////
uint8_t White_amount(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2);//白点数量
float Sum_of_Dif(float L1,float R1,float L2,float R2);
void Trace_middleLine();//Trace_middleLine
void Trace_Eight_fields_new();//优化八邻域
void Trace_rectangle();//矩形循迹

