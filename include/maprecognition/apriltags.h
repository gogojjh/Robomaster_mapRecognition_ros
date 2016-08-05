#ifndef APRILTAGS_H
#define APRILTAGS_H

#include "maprecognition/base.h"
#include "maprecognition/histogram.h"
#include <cv_bridge/cv_bridge.h>

#define COLOR_BLACK_TH 30
#define PROPERTY 20

void apritags(const Mat src,
              const int current_direction,
              double &direction,
              bool &state,
              Mat &result);


#endif // APRILTAGS_H
