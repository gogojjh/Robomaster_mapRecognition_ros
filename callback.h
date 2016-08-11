#ifndef __CONTROL_STREAM__
#define __CONTROL_STREAM__

#include <maprecognition/base.h>
#include <maprecognition/transformer.h>
#include <maprecognition/controldata.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/Float32.h"


class  CONTROLSTREAM
{
   private:
   maprecognition::controldata controlstream;
   cv::Mat img_mat;
   //Publisher
    ros::Publisher controlstream_publisher;
   //Subscriber
   ros::Subscriber controlstream_subscriber;
   //CALLBACK
   void ControlStreamCallback(const sensor_msgs::ImageConstPtr& msg);

    void init_publishers(ros::NodeHandle& n)
    {
    controlstream_publisher = n.advertise<maprecognition::controldata>("dji_sdk_controlstream/controlstream", 10);
    }
    void init_subscribers(ros::NodeHandle& n)
    {
     controlstream_subscriber = n.subscribe("/usb_cam/image_raw", 1, &CONTROLSTREAM::ControlStreamCallback, this);
    }

   public:
   CONTROLSTREAM(ros::NodeHandle& n);
};

#endif
