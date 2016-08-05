#include "maprecognition/base.h"
#include "maprecognition/transformer.h"
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

using namespace cv;
using namespace cv_bridge;

Mat img_mat;
cv_bridge::CvImagePtr cv_ptr;

static const std::string OPENCV_WINDOW = "Image window";
static const std::string RESULT_WINDOW = "Result window";
int current_direction;
int Transformer::number = 0;

void ControlStreamCallback(const sensor_msgs::ImageConstPtr& msg)
{
  
    cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);

    // fill out cv code
    Transformer trans;

    int choice = 0;
    double next_direction;
    bool next_state;
    Mat frame;

    current_direction = UP;
    next_direction = current_direction * 90;
    next_state = false;

    //undistort the images
    //Mat image_undistorted;
    //imageCalibration(frame, image_undistorted);

    trans.sTransformer(cv_ptr->image);
    Transformer::number ++;

    switch (choice)
    {
        case 0: trans.detectApriltag(current_direction, next_direction, next_state); break;
        case 1: trans.detectTwo(current_direction, next_direction, next_state); break;
        case 2: trans.detectThree(current_direction, next_direction, next_state); break;
        case 3: trans.detectFour(current_direction, next_direction, next_state); break;
        case 4: trans.detectRed(current_direction, next_direction, next_state); break;
        default:break;
    }

    cout << "Angle: " << next_direction << endl;

    cv::imshow(RESULT_WINDOW, trans.gImage(2));

    if ((Transformer::number % 5) == 0) 
    {
        char saveFileName[50];
        sprintf(saveFileName, "/home/gogojjh/catkin_ws/src/maprecognition/image/%d.jpg", Transformer::number/5);
        cv::imwrite(saveFileName, trans.gImage((2)));
    }

    cv::waitKey(100);

    //sleep(3);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "maprecognition");
    ros::NodeHandle n;

    cv::namedWindow(OPENCV_WINDOW, 2);
    cv::namedWindow(RESULT_WINDOW, 2);

    ros::Subscriber sub = n.subscribe("/usb_cam/image_raw", 10, ControlStreamCallback);
    ros::spin();

    return 0;
}

