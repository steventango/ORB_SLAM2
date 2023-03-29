#include "RoadDrawer.h"

#include <pangolin/pangolin.h>

#include <mutex>

#include "KeyFrame.h"
#include "MapPoint.h"

namespace ORB_SLAM2 {

RoadDrawer::RoadDrawer(Map *pMap) : mpMap(pMap) {

}


cv::Mat RoadDrawer::DrawRoad() {
    const vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();

    // get pt correspondences
    for (size_t i = 0; i < vpKFs.size() && i < 1; i++) {
        KeyFrame *pKF = vpKFs[i];
        vector<MapPoint *> vpMPs = pKF->GetMapPointMatches();
        for (size_t j = 0; j < vpMPs.size(); j++) {
            if (!vpMPs[j]) {
                continue;
            }
            if (vpMPs[j]->isBad()) {
                continue;
            }
            MapPoint *mp = vpMPs[j];
            const cv::KeyPoint kp = pKF->mvKeys[j];
            cv::Mat pos = mp->GetWorldPos();
            if (pos.at<float>(2) < 0) {
                continue;
            }
            cout << "debug--a" << endl;
            cout << kp.pt.x << " " << kp.pt.y << endl;
            cout << pos << endl;
        }
    }
    // findHomography from pts
    // warpPerspective

    // rescale to winsize
    // output to Pangolin
    // try a devcontainer so we can use make cache
    return cv::Mat();

}

}
