/****************************************************************
 *****                                                      *****
 ***** Name: Tracker.hpp                                    *****
 ***** Ver.: 1.00                                           *****
 ***** Date: 2016-03-11                                     *****
 ***** Auth: Kevin Rueter                                   *****
 ***** University of Osnabrueck                             *****
 ***** Germany                                              *****
 ***** Func: implements a tracker based on Visp             *****
 *****                                                      *****
 ******************************************************************/

#ifndef VISP_EXAMPLES_LINETRACKER_H
#define VISP_EXAMPLES_LINETRACKER_H

#include <vector>
#include <openni2/OpenNI.h>
#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_MODULE_SENSOR
#include <visp3/sensor/vpOpenCVGrabber.h>
#endif
#include <visp3/klt/vpKltOpencv.h>
#include <visp3/blob/vpDot2.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/me/vpMeLine.h>
#include <visp3/me/vpMeEllipse.h>

#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/vision/vpPose.h>

using namespace std;
using namespace openni;

class Tracker {

public:
    Tracker();
    ~Tracker();
    int initKeypointTrack(cv::Mat frame);
    void keypointTrack(cv::Mat frame);
    int trackPoints();
    map<vpPoint,int> ransacFinding(vector<vpPoint> features, vector<int> featureIds);
    bool computePose(vector<vpPoint> &point, bool init, vpHomogeneousMatrix &cMo);
    //int keypointTrackTest();

private:
    //vpImage<unsigned char> I;
    //cv::VideoCapture g;
    //cv::Mat frame;
    vpKltOpencv m_tracker;
    bool initTracking;
    bool initPose;
    float worldX;
    float worldY;
    float worldZ;
};


#endif //VISP_EXAMPLES_LINETRACKER_H
