#include "trace.h"
////普通扫线//////
uint8 Middle_line[110] = {60};//middle line
uint8 PointX_L[120] = {0};//记录左边点
uint8 PointX_R[120] = {0};//记录右边点
uint8 lengthX_L = 0;//记录点数
uint8 lengthX_R = 0;//记录点数

///////八领域///////
uint16 Fileds_len = 0;//用来记录八邻域记录的点数
uint8 eight_fields_x[300] = {0};//八邻域X点的坐标
uint8 eight_fields_y[300] = {0};//八邻域Y点的坐标
typedef enum Diraction{
  Right,
  UP,
  Down,
  Left
}Diraction;
///////中点////////
uint8_t remenber_point = 94;

uint8 Binary_map[120][188];//binary map

void Show_Binaray_map(){//显示二值化数组
  //画出二值化数组
  ips200_show_binary_map(Binary_map[0]);
   //直线循迹
  ips200_draw_rectangle(rectangleL,RGB565_RED);
  ips200_draw_rectangle(rectangleR,RGB565_RED);
  //弯道循迹
  ips200_draw_rectangle(rectangleLL,RGB565_RED);
  ips200_draw_rectangle(rectangleRR,RGB565_RED);
};

//在屏幕上显示出其它数据
void Screen_Add(uint8 threshold){
  //Middle basic point基准点
  ips200_draw_big_point(94,80,RGB565_BLUE);//中心基准点(94,80)
  
  //画出扫线扫到的中点(x,80)
  if(remenber_point>1 && remenber_point<187)
    ips200_draw_big_point(remenber_point,80,RGB565_PURPLE);
  else if(remenber_point<=1)//限位保护
    ips200_draw_big_point(0,80,RGB565_PURPLE);
  else if(remenber_point>=187)//限位保护
    ips200_draw_big_point(187,80,RGB565_PURPLE);
  //ips200_draw_line(34,110,154,110,RGB565_YELLOW);//画出扫描基准线X 34~110

  if(mt9v03x_finish_flag){
       mt9v03x_finish_flag= 0;
       ips200_show_gray_image(0, 0, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, 188, 120, threshold);//循环刷新出二值化图形
  }
  //////////////////////矩形扫描/////////////////////////////
  //直线循迹
  ips200_draw_rectangle(rectangleL,RGB565_RED);
  ips200_draw_rectangle(rectangleR,RGB565_RED);
  //弯道循迹
  ips200_draw_rectangle(rectangleLL,RGB565_RED);
  ips200_draw_rectangle(rectangleRR,RGB565_RED);

   //////////////////////扫线线扫描/////////////////////////////
  /*
  for(uint8_t x = 0,y=110;x<lengthX_L;x++,y--){
    ips200_draw_point(PointX_L[x],y,RGB565_RED);
  }
  for(uint8_t x = 0,y=110;x<lengthX_R;x++,y--){
    ips200_draw_point(PointX_R[x],y,RGB565_RED);
  }

   */
   //////////////////////八邻域扫描///////////////////////// 
  /*
  for(int i = 0;i<Fileds_len;i++)//画出边界线
  {
    ips200_draw_point(eight_fields_x[i],eight_fields_y[i],RGB565_RED);
  }
  for(int i = 0;i<Fileds_len/2;i++)//draw middle line画出中线
  {
    ips200_draw_point((eight_fields_x[i]+eight_fields_x[(uint8)(Fileds_len/2) + i])/2
                      ,(eight_fields_y[i]+eight_fields_y[(uint8)(Fileds_len/2) + i])/2,RGB565_GREEN); //draw middle line
  }
  */
}
////////////////////////////////////////////二值化图像/////////////////////////////////////////////////
//直接阈值二值化
void binarizeImage(uint8_t ZIP,uint8 threshold) {
  //scan all the gray map then convert it intp binary map
  for (uint8_t y = 0; y < 120; y+=ZIP) {
    for(uint8_t x = 0;x<188;x+=ZIP){
       //if lager than threshold we will turn this pixel to white else black
       if(mt9v03x_image[y][x] >= threshold)Binary_map[y][x] = 1;   //  white 1
       else Binary_map[y][x] = 0;          //  black 0
    }
  }
}

//动态自适应算法:大津法
//image:传入图像
uint16_t otsu_threshold(unsigned char *image) {
    int hist[256] = {0};
    double sum = 0;
    double sumB = 0;
    int wB = 0;
    int wF = 0;
    double varMax = 0;
    int threshold = 0;

    // 计算直方图
    for (int i = 0; i < 22560; i++)//22560 = 188*120图像大小
        hist[image[i]]++;
    
    for (int i = 0; i < 256; i++) // 计算像素值总和
        sum += i * hist[i];

    for (int t = 0; t < 256; t++) {// 遍历所有可能的阈值
        wB += hist[t];
        if (wB == 0) continue;
        wF = 22560 - wB;
        if (wF == 0) break;

        sumB += t * hist[t];
        double mB = sumB / wB;
        double mF = (sum - sumB) / wF;

        double varBetween = (double)wB * (double)wF * (mB - mF) * (mB - mF);

        if (varBetween > varMax) {
            varMax = varBetween;
            threshold = t;
        }
    }

    return threshold;
}
////////////////////////////////////循迹方案////////////////////////////////////////////////
//计算白点数量:
static uint16_t White_amount(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2){
  uint16_t counter = 0;
  for(uint8_t Y =y1;Y<y2;Y++){
    for(uint8_t X = x1;X<x2;X++){
      if(Binary_map[Y][X]==1)counter++;
    }
  }
  return counter;
}

void WhitePoint_amount(Rectangle_Struct* this){
  this->L = White_amount(rectangleL);
  this->R = White_amount(rectangleR);
  this->LL = White_amount(rectangleLL);
  this->RR = White_amount(rectangleRR);
  this->LLL = White_amount(rectangleLLL);
  this->RRR = White_amount(rectangleRRR);
}

float turn_plus(float L1,float R1)
{
  uint8_t sum_amount = 160;
    if (L1 == 0 && R1 >= (1/2)*sum_amount)
    {
      return 0.3;
    }
    if (R1 == 0 && L1 >= (1/2)*sum_amount)
    {
      return -0.3;
    }
    return 0;
}

/*差比和算法
L1:左边矩形(靠近中线)
L2:左边矩形
R1:右边矩形(靠近中线)
R2:右边矩形
*/
float Sum_of_Dif(float L1,float R1,float L2,float R2){
  static float weight = 10;
  if((L1 + R1 + L2 + R2)==0)return 0;//如果分母为0
  return ((R1 + R2*weight) - (L1 + L2*weight)) / (L1 + R1 + L2*weight + R2*weight);
}

float Sum_of_Dif_near(float L1,float R1){
  if((L1 + R1)==0)return 0;//如果分母为0
  return (R1  - L1 ) / (L1 + R1 );
}