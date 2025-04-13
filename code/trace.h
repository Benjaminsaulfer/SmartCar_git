#pragma once

#include "zf_common_headfile.h"
#include "math.h"

extern uint8 Binary_map[188][120];//binary map
extern int16_t remenber_point;//记录八邻域扫出来的中点

////////////显示屏叠加//////////////
void Screen_Add(uint8 threshold);   

////////////二值化图像//////////////
void binarizeImage(uint8 threshold);//Convert to binarizeImage
void binarizeImage_ZIP(uint8 threshold);//压缩图像
int otsu_threshold(unsigned char *image);

/////////////扫线方案///////////////
void Trace_middleLine();//Trace_middleLine
void Trace_Eight_fields_new();//优化八邻域
