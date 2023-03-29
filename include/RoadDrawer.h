#ifndef ROADDRAWER_H
#define ROADDRAWER_H

#include <pangolin/pangolin.h>

#include <mutex>

#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"

namespace ORB_SLAM2 {

class RoadDrawer {
   public:
    RoadDrawer(Map* pMap);

    Map* mpMap;

    cv::Mat DrawRoad();

   private:
};

}  // namespace ORB_SLAM2

#endif
