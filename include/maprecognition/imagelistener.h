#include <ros/ros.h>
#include <stdio.h>
#include <cstdlib>
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "maprecognition/imagelistener.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <image_transport/image_transport.h>
using namespace cv;
using namespace cv_bridge;


void ControlStreamCallback(const sensor_msgs::Image& msg);


int listener(int argc, char **argv);



    

 

