#ifndef GRIDDETECT_H
#define GRIDDETECT_H

#include "maprecognition/base.h"
#include <cv_bridge/cv_bridge.h>

void griddetect(const Mat src,
              const int current_direction,
              double &direction,
              bool &state,
              Mat &frame_result);

#endif // GRIDDETECT_H
