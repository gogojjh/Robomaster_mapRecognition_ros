#include "maprecognition/apriltags.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

int min(double x, double y)
{
    if (x<y) return x;
    else	return y;
}

int max(double x, double y)
{
    if (x>y) return x;
    else	return y;
}

void setROI(Mat image, vector<Mat> &imageAp, const Rect rects)
{
    Mat imageTemp;
    image(rects).copyTo(imageTemp);
    imageAp.push_back(imageTemp);
}

void apritags(const Mat src,
              const int current_direction,
              double &direction,
              bool &state,
              Mat &result)
{
        Histogram hc;
        MatND colorhist;
        Mat thresholded;
        Mat imageBinary;

        src.copyTo(result);

        Mat imageFilter;
        for (int i=1; i<9; i=i+2) GaussianBlur(src, imageFilter, Size(i, i), 0, 0);

        //imshow("img", src);
        //imshow("img_Gaussian", imageFilter);

        //createTrackbar("alpha", "camera", &alpha, 3, on_track);
        //on_track(alpha, 0);

        Mat imageL = imageFilter - Scalar(20,20,20);

        hc.getHueHistogram(imageL);
        equalizeHist(hc.v[2], hc.v[2]);

        //imshow("v[2]",hc.v[2]);

        threshold(hc.v[2], imageBinary, COLOR_BLACK_TH, 255, 1);
        //imshow("img_binary", imageBinary);

        Mat imageClosed;
        Mat element = getStructuringElement(MORPH_CROSS, Size(7,7), Point(0,0));
        morphologyEx(imageBinary, imageClosed, MORPH_CLOSE,  element);
        dilate(imageClosed, imageClosed, element);
        //imshow("img_open", imageClosed);

        vector<vector<Point> > contours;
        findContours(imageClosed, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0, 0));
        Mat imageContours(imageClosed.size(), CV_8U, Scalar(255));
        drawContours(imageContours, contours, -1, Scalar(0), 2);

        /* calculate the ju*/        
        vector<Moments> mu(contours.size());
        for (int i=0; i<contours.size(); i++) mu[i] = moments(contours[i], false);
        //imshow("contours", imageContours);

        //imageContours.copyTo(result);

        vector<vector<Point> > apcontours; //AprilTags' contours
        vector<RotatedRect> rotatedRects;

        /* number of apriltags */
        int countAp = 0;
        float angle[20]; //the rotating angle of each Apriltage
        double area, length, p;
        double d = src.cols * src.rows;
        Point center(src.cols/2, src.rows/2);
        state = false;
        double maxp = PROPERTY;
        cout << "X: " << center.x << " Y: " << center.y << endl;
        for ( int i=0; i<contours.size(); i++)
        {
            area = abs(contourArea( contours[i] ));
            length = abs(arcLength( contours[i], true ));
            p = 1.0*area/length;
            if (p>PROPERTY)
            {
                cout << "Area: " << area << "  Length: " << length << "  Property: " << int(p) << endl;
                countAp++;
                apcontours.push_back(contours.at(i));
                vector<Point> p = contours.at(i);
                rotatedRects.push_back(minAreaRect(Mat(p)));
                angle[countAp-1] = rotatedRects[countAp-1].angle;

                Point2f mc;
                mc = Point2f(mu[i].m10/mu[i].m00, mu[i].m01/mu[i].m00);
                circle(result, mc, 2, Scalar::all(255));
                cout << "X: " << mc.x << " Y: " << mc.y << endl;

                if ((((current_direction == UP) && (mc.y<=center.y)) ||
                        ((current_direction == DOWN) && (mc.y>center.y)) ||
                        ((current_direction == LEFT) && (mc.x<=center.x)) ||
                        ((current_direction == RIGHT) && (mc.x>center.x))))
                {
                    if (sqrt(pow(mc.x-center.x, 2.0) + pow(mc.y-center.y, 2.0)) < d)
                    {
                        d = sqrt(pow(mc.x-center.x, 2.0) + pow(mc.y-center.y, 2.0));
                        //direction = atan2(mc.x-center.x, mc.y-center.y);
                        //direction = direction * 180 / PI;
                        //maxp = p;
                        if (center.y > mc.y)
                        {
                            if ( mc.x>center.x )    //第一象限
                            {
                               direction=180*atan(float ((mc.x-center.x)/(center.y-mc.y))) / PI;
                            }
                            else            //第四象限
                            {
                                direction=360 + 180*atan(float ((mc.x-center.x)/(center.y-mc.y))) / PI;
                            }
                        }
                        else if (center.y < mc.y)
                        {
                             if (mc.x>center.x )            //第er象限
                            {
                               direction=180 - 180*atan(float ((mc.x-center.x)/(mc.y-center.y))) / PI;
                            }
                            else                 //第san象限
                            {
                                direction=180 + 180*atan(float ((center.x-mc.x)/(mc.y-center.y))) / PI;
                            }
                        }
                        state = true;
                    }
                }
                // apriltag is near
                if ( d<20 )
                {
                    state = false;
                    return;
                }
            }
            //cout << "A:" << area << "  L:" << length << "  P:" << p << endl;
        }

        drawContours(result, apcontours, -1, Scalar(255), 5);

        //waitKey(0);
        /*
        vector<Mat> imageAp;
        Rect rects;
        for (int i=0; i<countAp; i++)
        {
            Point2f rect_points[4];
            rotatedRects[i].points(rect_points);
            cout << rect_points[0].x << "  " << rect_points[0].y << "  " <<  rect_points[1].x << "  " <<  rect_points[1].y << "  " <<  rect_points[2].x << "  " << rect_points[2].y << "  " <<  rect_points[3].x << "  " <<  rect_points[3].y << endl;
            int x = min(min(min(rect_points[0].x, rect_points[1].x), rect_points[2].x), rect_points[3].x);
            int y = min(min(min(rect_points[0].y, rect_points[1].y), rect_points[2].y), rect_points[3].y);
            int rows = max(max(max(rect_points[0].y, rect_points[1].y), rect_points[2].y), rect_points[3].y)-y;
            int cols = max(max(max(rect_points[0].x, rect_points[1].x), rect_points[2].x), rect_points[3].x)-x;
            if (x + cols>image.cols) cols = image.cols-x;
            if (y + rows>image.rows) rows = image.rows-y;
            rects = Rect(max(x, 0), max(y, 0), cols, rows);
            setROI(image, imageAp, rects);
        }

        for (int i=0; i<imageAp.size(); i++)
        {
            Mat imageGray;
            cvtColor(imageAp[i], imageGray, CV_RGB2GRAY);
            Mat imageAdaptive;
            adaptiveThreshold(imageGray, imageAdaptive, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY , 3, 5);
            //imshow("6", imageAdaptive);
            waitKey(0);
        }*/
}


