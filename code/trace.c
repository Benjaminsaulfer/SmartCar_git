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
uint8_t S_GetOSTU(uint8_t tmImage[MT9V03X_H][MT9V03X_W])
{
    int16_t i, j;
    uint32_t Amount = 0;
    uint32_t PixelBack = 0;
    uint32_t PixelIntegralBack = 0;
    uint32_t PixelIntegral = 0;
    int32_t PixelIntegralFore = 0;
    int32_t PixelFore = 0;
    double OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
    int16_t MinValue, MaxValue;
    uint8_t Threshold = 0;
    uint8_t HistoGram[256];
    
    for (j = 0; j < 256; j++)  
      HistoGram[j] = 0; //初始化灰度直方图
    
    for (j = 0; j < MT9V03X_H; j++)
    {
        for (i = 0; i <MT9V03X_W; i++)
        {
            HistoGram[tmImage[j][i]]++; //统计灰度级中每个像素在整幅图像中的个数
        }
    }

    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++);        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--); //获取最大灰度的值

    if (MaxValue == MinValue)     
      return (uint8)MaxValue;         // 图像中只有一个颜色
    if (MinValue + 1 == MaxValue)  
      return (uint8)MinValue;        // 图像中只有二个颜色

    for (j = MinValue; j <= MaxValue; j++)    
      Amount += HistoGram[j];        //  像素总数

    PixelIntegral = 0;
    for (j = MinValue; j <= MaxValue; j++)
    {
        PixelIntegral += HistoGram[j] * j;//灰度值总数
    }
    SigmaB = -1;
    for (j = MinValue; j < MaxValue; j++)
    {
        PixelBack = PixelBack + HistoGram[j];    //前景像素点数
        PixelFore = Amount - PixelBack;         //背景像素点数
        OmegaBack = (double)PixelBack / Amount;//前景像素百分比
        OmegaFore = (double)PixelFore / Amount;//背景像素百分比
        PixelIntegralBack += HistoGram[j] * j;  //前景灰度值
        PixelIntegralFore = PixelIntegral - PixelIntegralBack;//背景灰度值
        MicroBack = (double)PixelIntegralBack / PixelBack;   //前景灰度百分比
        MicroFore = (double)PixelIntegralFore / PixelFore;   //背景灰度百分比
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//计算类间方差
        if (Sigma > SigmaB)                    //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
        {
            SigmaB = Sigma;
            Threshold = (uint8)j;
        }
    }
    return Threshold;                        //返回最佳阈值;
}

////////////////////////////////////循迹方案////////////////////////////////////////////////
//普通扫线
void Trace_middleLine(){
    uint8 Right_scan_flag = 1;//允许左边扫线
    uint8 Left_scan_flag = 1;//允许右边扫线
    uint8_t LeftX = 34;//左边点初始化
    uint8_t RightX = 154;//右边点初始化
    lengthX_L = 0;//重新初始化左边数组长度
    lengthX_R = 0;//重新初始化右边数组长度
    static uint8_t Last_remenber_point=0;//记录上一次得到的中点

    for(uint8_t i = 34;i <= 154;i++){//寻找八邻域起点
      if(Binary_map[i][110] < Binary_map[i+1][110]){//如果左边小于右边
        LeftX = i;
        i = i + 6;//跳点加速
      }
      else if(Binary_map[i][110] > Binary_map[i+1][110]){//如果左边大于右边
        RightX = i+1;
        break;
      }     
    }
    uint8_t midpoint = (RightX+ LeftX)/2;//起始中点
    
    for(uint8 y = 110;y>20;y--){//从Y=110往上到Y=20
      for (uint8_t x = midpoint; x > 20 && Left_scan_flag; x--) {//中点往X=20找
        if(Binary_map[x-1][y] < Binary_map[x][y]){//找到左边点
          if(y == 80)LeftX = x-1;//记录左点求中点
          PointX_L[lengthX_L] = x-1;
          lengthX_L++;
          break;
        }
        else if(x == 21){//如果越界没找到
          Left_scan_flag = 0;
          break;
        }
      }
      for (int x = midpoint ; x < 168 && Right_scan_flag; x++) {//中点往X=168找
        if(Binary_map[x+1][y] < Binary_map[x][y]){//找右边点
          if(y == 80)RightX = x-1;//记录右点求中点
          PointX_R[lengthX_R] = x+1;
          lengthX_R++;
          break;
        }
        else if(x == 167){//如果越界没找到
          Right_scan_flag = 0;
        }
      }
    }
    remenber_point = (RightX + LeftX) / 2;//左点加右点求中点
    if(lengthX_L < 30 && (lengthX_R > lengthX_L))  remenber_point -= 40;//左转
    else if(lengthX_R < 30 && (lengthX_L > lengthX_R))  remenber_point += 40;//右转
    else if(lengthX_L ==0 && lengthX_R==0)  remenber_point = Last_remenber_point;
    
    Last_remenber_point = remenber_point;
}

/////优化八邻域扫线//////
uint8_t passed_pointX[200]={0};
uint8_t passed_pointY[200]={0};
void Trace_Eight_fields_new(){
    /*
    从中间开始往两边扫(begin with (94,100)),扫描到黑点之后，直接进入八邻域.
    八邻域扫描顺序(左，右):
     5 4 3              3 4 5
     6 * 2              2 * 6
     7 8 1              1 8 7
    */
    static uint8_t Lx[8] = { 1 ,  1,  1,  0, -1, -1, -1, 0 };//向左
    //static uint8_t Rx[8] = {-1 , -1, -1,  0,  1,  1,  1, 0 };//向右
    static uint8_t dy[8] = { 1 ,  0, -1, -1, -1,  0,  1, 1 };
    
    //uint8 remenber_Rpoint = 0;
    //uint8 remenber_Lpoint = 0;
    
    uint8_t LeftX = 34;
    uint8_t RightX = 154;
    uint8_t beginY = 110;
    Fileds_len = 0;//记录点数
    uint8_t visited[188][120] = {false};//走过的X和Y
    
    for(int i = 34;i <= 154;i++){//寻找八邻域起点
      if(Binary_map[i+1][110] > Binary_map[i][110])
         LeftX = i+1;
      else if(Binary_map[i+1][110] < Binary_map[i][110])
         RightX = i+1;
    }
    ips200_draw_big_point(LeftX,110,RGB565_BLUE);
    ips200_draw_big_point(RightX,110,RGB565_BLUE);
    
    for(uint16_t i =0;i<100;i++,Fileds_len++){
        uint8_t j =0;
        for (j = 0; j < 8; j++) {
            //开始攀爬
            uint8_t new_x = LeftX + Lx[j];
            uint8_t new_y = beginY + dy[j];
            //如果找到了黑点而且这个点不是上一个点
            if (Binary_map[new_x][new_y] == 0 && visited[new_x][new_y] == false) {
               //将新点导入数组中
               eight_fields_x[i] = new_x;
               eight_fields_y[i] = new_y;
               visited[LeftX][beginY] = true;//标记走过的点
               //记录下新的点
               LeftX = new_x;
               beginY = new_y;
               break;
             }                                       
        }
    }
}

void WhitePoint_amount(Rectangle_Struct* this){
  this->L = White_amount(rectangleL);
  this->R = White_amount(rectangleR);
  this->LL = White_amount(rectangleLL);
  this->RR = White_amount(rectangleRR);
}

//计算白点数量:
uint16_t White_amount(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2){
  uint16_t counter = 0;
  for(uint8_t Y =y1;Y<y2;Y++){
    for(uint8_t X = x1;X<x2;X++){
      if(Binary_map[Y][X]==1)counter++;
    }
  }
  return counter;
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
  static float weight = 2.5;
  if((L1 + R1 + L2 + R2)==0)return 0;//如果分母为0
  //return ((R1 + R2*weight) - (L1 + L2*weight)) / (L1 + R1 + L2*weight + R2*weight) + turn_plus(L2,R2);
  return ((R1 + R2*weight) - (L1 + L2*weight)) / (L1 + R1 + L2*weight + R2*weight); 
}

float Sum_of_Dif_near(float L1,float R1){
  if((L1 + R1)==0)return 0;//如果分母为0
  return (R1  - L1 ) / (L1 + R1 );
}