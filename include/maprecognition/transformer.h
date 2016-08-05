#ifndef TRANSFORMER_H
#define TRANSFORMER_H

#include "maprecognition/base.h"
#include <cv_bridge/cv_bridge.h>

#define THRESH_RED 0.4
#define COLORL_GROUND_RED 20
#define COLORH_GROUND_RED 80

//Canny边缘检测两个阈值（0~255）
#define CANNY_THL 50
#define CANNY_THH 255

//红色区域FLAG
#define FLAG_TRAVELING 0
#define FLAG_ARRIVE 1
#define FLAG_LOCATE 2
#define FLAG_TARGET 3
#define FLAG_CALIBRATE 4
#define FLAG_RETURN 5

//获取成员变量
#define MEMBER_IMG 0
#define MEMBER_CANNY 1
#define MEMBER_IMG_HSV 0
#define MEMBER_OPEN_HSV 1
#define MEMBER_CLOSE_HSV 2

//红色区域FLAG
#define FLAG_TRAVELING 0
#define FLAG_ARRIVE 1
#define FLAG_LOCATE 2
#define FLAG_TARGET 3
#define FLAG_CALIBRATE 4
#define FLAG_RETURN 5

class Transformer{
public:

    Transformer();   /* 构造函数，输入图片src，初始化得到图像size，图像中心坐标 */
    Point gCenter();
    Mat gImage(int key);
    void gTransformer(Mat &dst, int channel, int key);
    void sTransformer(const Mat image);

    /* areaTwo Dingshan Sun*/
    void detectTwo(const int current_direction, double &direction, bool &state);

    /* apriltag Jianhao Jiao*/
    void detectApriltag(const int current_direction, double &direction, bool &state);

    /* areaThree*/
    void detectThree(const int current_direction, double &direction, bool &state);

    /* areaFour Dingshan Sun*/
    void detectFour(const int current_direction, double &direction, bool &state);

    /* areaRed Lu Fan*/
    void detectRed(const int current_direction, double &direction, bool &state);

    static int number;

private:
    Point center;
    Mat img_Origin, img_Canny, img_Result;
    vector<Mat> img_HSV;   /*原图HSV三通道图像img_HSV */
    vector<Mat> open_HSV; /* 开运算HSV三通道图像open_HSV */
    vector<Mat> close_HSV; /* 闭运算HSV三通道图像close_HSV */

    int open_size;
    int open_operator;
    int close_size;
    int close_operator;
};


#endif // TRANSFORMER_H
