#include "maprecognition/callback.h"
#include "maprecognition/base.h"
#include "maprecognition/transformer.h"

int Transformer::gridfourdetect_threshold = 66;

CONTROLSTREAM::CONTROLSTREAM(ros::NodeHandle& n) 
{
    init_publishers(n);
    init_subscribers(n);
}

void setting(Point2f &goal, float &distance, bool &find_object, Mat src)
{
    goal.x = 0; goal.y = 0;
    distance = src.cols * src.rows;
    find_object = false;
}

void CONTROLSTREAM::ControlStreamCallback(const sensor_msgs::ImageConstPtr& msg)
{
    static const std::string OPENCV_WINDOW = "Image window";
    static const std::string RESULT_WINDOW = "Result window";
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    this->img_mat=cv_ptr->image;
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    
    namedWindow(OPENCV_WINDOW, 2);
    namedWindow(RESULT_WINDOW, 2);

    int choice = 3;
    float distance;        /* the distance of object */
    bool find_object;   /* if find object */
    Mat frame;
    Point2f goal;
    int countF1 = 0;
    int countF2 = 0;
    int flag;
    trans.sTransformer(cv_ptr->image);
    Transformer::number ++;
    switch (choice)
    {
       case 0: setting(goal, distance, find_object, trans.gImage(0)); trans.detectApriltag(goal, distance, find_object); break; 
       case 1: setting(goal, distance, find_object, trans.gImage(0)); trans.detectTwo(goal, distance, find_object); break; 
       case 2: setting(goal, distance, find_object, trans.gImage(0)); trans.detectThree(goal, distance, find_object); break; 
       case 3: setting(goal, distance, find_object, trans.gImage(0)); trans.detectFour(goal, distance, find_object); break;
       case 4: setting(goal, distance, find_object, trans.gImage(0)); trans.detectRed(goal, distance, find_object); break;
       default:break;
   }
   int ACK=3;
   int thresholdD=20;

   if (find_object&countF1<3)    countF1++;
   else
   {
       if(!find_object)
       {
           countF1 = 0;
           cout<<"missing target"<<endl;
           Transformer::gridfourdetect_threshold  = 66;
       }
   }
   if (countF1 == ACK)
   {
       cout << "***********object_targeted**********" << endl;
       Transformer::gridfourdetect_threshold  = 80;
       if(distance<thresholdD)
       {
           countF2++;
       }
       else
           countF2=0;
       if(countF2==ACK)
       {
           cout<<"///////////////////I wanna drop WAWA/////////////////"<<endl;
           countF2=0;
           countF1=0;
           Transformer::gridfourdetect_threshold = 66;
       }
    }
    imshow(RESULT_WINDOW, trans.gImage(2));
    imshow(OPENCV_WINDOW, trans.gImage(0));

    cv::waitKey(100);    
    controlstream_publisher.publish( this->controlstream);
}

 

