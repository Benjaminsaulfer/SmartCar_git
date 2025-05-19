#include "trace.h"

uint8 Binary_map[120][188];//binary map
extern Rectangle_Struct REC;//���νṹ�� 

void Show_Binaray_map(){//��ʾ��ֵ������
  //������ֵ������
  ips200_show_binary_map(Binary_map[0]);
   //ֱ��ѭ��
  ips200_draw_rectangle(rectangleL,RGB565_RED);
  ips200_draw_rectangle(rectangleR,RGB565_RED);  

  //���ѭ��
  if(REC.LL>=8)
    ips200_draw_rectangle(rectangleLL,RGB565_PURPLE);
  if(REC.RR>=8)
    ips200_draw_rectangle(rectangleRR,RGB565_PURPLE);
  else{
    //ָʾ��
    ips200_draw_rectangle(guideLL,RGB565_PINK);
    ips200_draw_rectangle(guideRR,RGB565_PINK);
  }

};

void fastSmoothImage(unsigned char Binary_map[120][188]) {
    int sumTable[120][188] = {0};
    unsigned char temp[120][188];
    memcpy(temp, Binary_map, sizeof(temp));
    
    // ��������ͼ
    for (int y = 0; y < 120; y++) {
        int rowSum = 0;
        for (int x = 0; x < 188; x++) {
            rowSum += temp[y][x];
            sumTable[y][x] = (y > 0 ? sumTable[y-1][x] : 0) + rowSum;
        }
    }
    
    // ʹ�û���ͼ���ټ���3��3�����
    for (int y = 1; y < 119; y++) {
        for (int x = 1; x < 187; x++) {
            int x1 = x-1, y1 = y-1;
            int x2 = x+1, y2 = y+1;
            int sum = sumTable[y2][x2] - 
                     (y1 > 0 ? sumTable[y1-1][x2] : 0) -
                     (x1 > 0 ? sumTable[y2][x1-1] : 0) +
                     (x1 > 0 && y1 > 0 ? sumTable[y1-1][x1-1] : 0);
            // ��ֵ������
            Binary_map[y][x] = (sum > 9 * 127) ? 255 : 0;
        }
    }
}

////////////////////////////////////////////��ֵ��ͼ��/////////////////////////////////////////////////
//ֱ����ֵ��ֵ��
void binarizeImage(uint8_t ZIP,uint8 threshold) {
  //scan all the gray map then convert it intp binary map
  //fastSmoothImage(mt9v03x_image);
  for (uint8_t y = 0; y < 120; y+=ZIP) {
    for(uint8_t x = 0;x<188;x+=ZIP){
       //if lager than threshold we will turn this pixel to white else black
       if(mt9v03x_image[y][x] >= threshold)Binary_map[y][x] = 1;   //  white 1
       else Binary_map[y][x] = 0;          //  black 0
    }
  }
}

//��̬����Ӧ�㷨:���
//image:����ͼ��
uint16_t otsu_threshold(unsigned char *image) {
    int hist[256] = {0};
    double sum = 0;
    double sumB = 0;
    int wB = 0;
    int wF = 0;
    double varMax = 0;
    int16 threshold = 0;

    // ����ֱ��ͼ
    for (uint16 i = 0; i < 22560; i++)//22560 = 188*120ͼ���С
        hist[image[i]]++;
    
    for (uint8 i = 0; i < 255; i++) // ��������ֵ�ܺ�
        sum += i * hist[i];

    for (uint8 t = 0; t < 255; t++) {// �������п��ܵ���ֵ
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
////////////////////////////////////ѭ������////////////////////////////////////////////////
//����׵�����:
static uint16_t White_amount(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2){
  uint16_t counter = 0;
  for(uint8_t Y =y1;Y<y2;Y++){
    for(uint8_t X = x1;X<x2;X++){
      if(Binary_map[Y][X]==1)counter++;
    }
  }
  return counter;
}
//����׵�����
void WhitePoint_amount(Rectangle_Struct* this){
  this->L = White_amount(rectangleL);
  this->R = White_amount(rectangleR);
  //�ȼ���ָʾ��
  /*
  this->GL = White_amount(guideLL);
  this->GR = White_amount(guideRR);
  if(this->GL>=8) 
    this->LL = White_amount(rectangleLL);
  if(this->GR>=8)
    this->RR = White_amount(rectangleRR);
  else{
    this->LL = 0;
    this->RR = 0;
  }
  */
  this->LL = White_amount(rectangleLL);
  this->RR = White_amount(rectangleRR);
}

/*��Ⱥ��㷨
L1:��߾���(��������)
L2:��߾���
R1:�ұ߾���(��������)
R2:�ұ߾���
*/
float Sum_of_Dif(uint16_t L1,uint16_t R1){
  if((L1 + R1)==0)return 0;//�����ĸΪ0
  return (float)(R1  - L1 ) / (L1 + R1 );
}