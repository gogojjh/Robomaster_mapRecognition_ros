// different operation
#include "maprecognition/transformer.h"
#include "maprecognition/apriltags.h"
#include "maprecognition/griddetect.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

Transformer::Transformer()
{
    close_size=17;
    close_operator=0;
    open_size=6;
    open_operator=0;
}

void Transformer::sTransformer(const Mat image)
{
    resize(image,img_Origin,Size(image.cols/IMAGE_SIZE_COL, image.rows/IMAGE_SIZE_ROW));
    img_Origin.copyTo(img_Result);
    center.x=img_Origin.cols/2;
    center.y=img_Origin.rows/2;

    split(img_Origin,img_HSV);

    Mat img_Open;
    Mat element = getStructuringElement( 0, Size( 2*open_size +3, 2*open_size+10 ), Point(open_size, open_size+5 ));
    morphologyEx( img_Origin, img_Open,MORPH_OPEN, element );
    split(img_Open,open_HSV);

    Mat img_Close;
    element = getStructuringElement( 0, Size( 2*close_size-5, 2*close_size-5 ), Point(close_size, close_size ) );
    morphologyEx( img_Origin, img_Close,  MORPH_CLOSE, element );
    split(img_Close,close_HSV);

    Canny(img_Origin, img_Canny, CANNY_THL, CANNY_THH);
}

Point Transformer::gCenter()
{
    Point p(center.x,center.y);
    return p;
}

Mat Transformer::gImage(int key)
{
    switch(key)
    {
        case 0: return img_Origin; break;
        case 1: return img_Canny; break;
        case 2: return img_Result; break;
    default: break;
    }
}

void Transformer::gTransformer(Mat &dst, int channel, int key)
{
    switch(key)
    {
        case 1: open_HSV[channel].copyTo(dst); break;
        case 2: close_HSV[channel].copyTo(dst); break;
        default: img_HSV[channel].copyTo(dst); break;
    }
}

void Transformer::detectApriltag(const int current_direction, double &next_direction, bool &state)
{
    apritags(img_Origin, current_direction, next_direction, state, img_Result);
}

void Transformer::detectTwo(const int current_direction, double &next_direction, bool &state)
{
    //griddetect(img_Origin, current_direction, next_direction, state, img_Result);
}

void Transformer::detectThree(const int current_direction, double &next_direction, bool &state)
{

}

void Transformer::detectFour(const int current_direction, double &next_direction, bool &state)
{

}

void Transformer::detectRed(const int current_direction, double &next_direction, bool &state)
{

    Mat temp;
    medianBlur(open_HSV[1], temp, 9);
    Mat red_filter_1, red_filter_2;
    threshold(temp, red_filter_1, COLORL_GROUND_RED,255,CV_THRESH_BINARY);
    threshold(temp, red_filter_2, COLORH_GROUND_RED,255,CV_THRESH_BINARY_INV);

    Mat red_result;
    bitwise_and(red_filter_1, red_filter_2, red_result);

    Vector<double>area = sum(red_result) / 256/ (red_result.cols * red_result.rows);
    red_result.copyTo(img_Result);

    if(area[0] >THRESH_RED)
    {
        next_direction = current_direction * 90;
        state = true;
    }
    else
    {
        next_direction = current_direction * 90;
        state = false;
    }
}
