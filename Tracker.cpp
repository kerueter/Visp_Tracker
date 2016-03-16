#include "Tracker.hpp"

#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms

/**
 * Constructor for the tracker
 */
Tracker::Tracker() {
    initTracking = false;
    //g = cv::VideoCapture(0);
}

/**
 * Destructor for the tracker
 */
Tracker::~Tracker() {}

/*int Tracker::keypointTrackTest() {
        try {
            if(!g.isOpened()) { // check if we succeeded
                std::cout << "Failed to open the camera" << std::endl;
                return -1;
            }
            g >> frame; // get a new frame from camera
            vpImageConvert::convert(frame, I);
            vpImageConvert::convert(I, frame);

            vpDisplayOpenCV d(I, 0, 0, "Camera view");
            vpDisplay::display(I);
            vpDisplay::flush(I);
            m_tracker.setMaxFeatures(10);
            m_tracker.setWindowSize(10);
            m_tracker.setQuality(0.6);
            m_tracker.setMinDistance(15);
            m_tracker.setHarrisFreeParameter(0.04);
            m_tracker.setBlockSize(9);
            m_tracker.setUseHarris(1);
            m_tracker.setPyramidLevels(3);

            // Initialise the tracking
            m_tracker.initTracking(frame);
            std::cout << "Tracker initialized with " << m_tracker.getNbFeatures() << " features" << std::endl;

            while(1) {
                g >> frame;
                vpDisplay::display(I);
                vpImageConvert::convert(frame, I);
                vpImageConvert::convert(I, frame);
                m_tracker.track(frame);
                m_tracker.display(I, vpColor::red);
                vpDisplay::flush(I);
                //m_tracker.initTracking(frame);
                std::cout << m_tracker.getNbFeatures() << std::endl;
            }
        }
        catch(vpException &e) {
            std::cout << "Catch an exception: " << e << std::endl;
            keypointTrackTest();
        }
}*/

/**
 * Initializes the keypoint tracker
 */
int Tracker::initKeypointTrack(cv::Mat frame) {
    try {
        // ab hier Initialisierung
        m_tracker.setMaxFeatures(100);
        m_tracker.setWindowSize(10);
        m_tracker.setQuality(0.1);
        m_tracker.setMinDistance(15);
        m_tracker.setHarrisFreeParameter(0.04);
        m_tracker.setBlockSize(9);
        m_tracker.setUseHarris(1);
        m_tracker.setPyramidLevels(3);

        // Initialise the tracking
        m_tracker.initTracking(frame);
    }
    catch(vpException &e) {
        std::cout << "Catch an exception: " << e << std::endl;
    }


}

/**
 * Tracks the keypoints from the given camera frame
 */
void Tracker::keypointTrack(cv::Mat frame) {
    try {
        m_tracker.track(frame);
    }
    catch(vpException &e) {
        std::cout << "Catch an exception: " << e << std::endl;
        initTracking = false;
    }
}

/**
 * Contains the tracking algorithm
 */
int Tracker::trackPoints() {
    Device device;
    VideoStream depth, color;

    OpenNI::initialize();
    device.open(ANY_DEVICE);

    depth.create(device, SENSOR_DEPTH);
    color.create(device, SENSOR_COLOR);

    VideoMode vm = color.getVideoMode();
    int cols, rows;
    cols = vm.getResolutionX();
    rows = vm.getResolutionY();

    VideoFrameRef refColor;
    VideoFrameRef refDepth;

    color.start();
    depth.start();

    Status rc;
    while(1) {
        int changedStreamDummy;
        VideoStream* colorStream = &color;
        rc = OpenNI::waitForAnyStream(&colorStream, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);

        if (rc != STATUS_OK) {
            printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
            continue;
        }
        rc = color.readFrame(&refColor);
        if (rc != STATUS_OK) {
            printf("Read failed!\n%s\n", OpenNI::getExtendedError());
            continue;
        }
        rc = depth.readFrame(&refDepth);
        if (rc != STATUS_OK) {
            printf("Read failed!\n%s\n", OpenNI::getExtendedError());
            continue;
        }

        RGB888Pixel* colorData;
        DepthPixel* depthData;

        colorData = (RGB888Pixel*)refColor.getData();
        depthData = (DepthPixel*)refDepth.getData();
        DepthPixel* depthDataBegin = depthData;

        cv::Mat srcColor(rows, cols, CV_8UC3, colorData);
        cv::Mat matColor;
        cv::cvtColor(srcColor, matColor, CV_RGB2GRAY);

        if (!initTracking) {
            initKeypointTrack(matColor);
            initTracking = true;
        }
        else {
            keypointTrack(matColor);
        }

        std::vector<vpPoint> features;
        std::vector<vpPoint> point;
        vpHomogeneousMatrix cMo;

        // convert all feature points
        for (unsigned int i = 0; i < m_tracker.getFeatures().size(); i++) {
            for(int y = 0; y < (int)m_tracker.getFeatures()[i].y; ++y) {
                for(int x = 0; x <(int)m_tracker.getFeatures()[i].x; ++x) {
                    ++depthData;
                }
            }
            openni::CoordinateConverter::convertDepthToWorld(depth, (int)m_tracker.getFeatures()[i].x, (int)m_tracker.getFeatures()[i].y, *depthData, &worldX, &worldY, &worldZ);

            if(!(fabsf(worldX) == 0 || fabsf(worldY) == 0 || fabsf(worldZ) == 0))
                features.push_back(vpPoint(worldX, worldY, worldZ));
            depthData = depthDataBegin;
        }

        std::vector<vpPoint> result;

        if(features.size() > 3) {
            result = findCoplanarPoints(features);
            if(result.size() > 0) {
                computePose(result, initPose, cMo);
                if (initPose)
                    initPose = false;
                vpTranslationVector trans = cMo.getTranslationVector();
                for(unsigned int i = 0; i < trans.size(); i++) {
                    std::cout << (trans[i] / 1000) << std::endl;
                }
                std::cout << "\n";
            }
            else
                std::cout << "no coplanar points" << std::endl;
        }
        else
            std::cout << "not enough features" << std::endl;
    }
}

/**
 * Pose estimation for given position
 */
void Tracker::computePose(std::vector<vpPoint> &point, bool init, vpHomogeneousMatrix &cMo) {

    vpPose pose;

    try {
        for (unsigned int i = 0; i < point.size(); i++) {
            pose.addPoint(point[i]);
        }

        if (init) {
            vpHomogeneousMatrix cMo_dem;
            vpHomogeneousMatrix cMo_lag;

            pose.computePose(vpPose::DEMENTHON, cMo_dem);
            pose.computePose(vpPose::LAGRANGE, cMo_lag);
            double residual_dem = pose.computeResidual(cMo_dem);
            double residual_lag = pose.computeResidual(cMo_lag);
            if (residual_dem < residual_lag)
                cMo = cMo_dem;
            else
                cMo = cMo_lag;
        }
        pose.computePose(vpPose::VIRTUAL_VS, cMo);
    }
    catch(vpException &e) {
        std::cout << "Catch an exception: " << e << std::endl;
    }
}

/**
 * Algorithm for finding coplanar points
 */
std::vector<vpPoint> Tracker::findCoplanarPoints(std::vector<vpPoint> features) {

    std::vector<vpPoint> result;

    for(unsigned int a = 0; a < features.size(); a++) {
        for(unsigned int b = 1; b < features.size(); b++) {
            for(unsigned int c = 2; c < features.size(); c++) {
                for(unsigned int d = 3; d < features.size(); d++) {
                    if((a != b) && (a != c) && (a != d) && (b != c) && (b != d) && (c != d)) {
                        // Richtungsvektoren bestimmen
                        vpPoint vecAB(features[b].get_oX() - features[a].get_oX(),
                                     features[b].get_oY() - features[a].get_oY(),
                                     features[b].get_oZ() - features[a].get_oZ());
                        vpPoint vecAC(features[c].get_oX() - features[a].get_oX(),
                                     features[c].get_oY() - features[a].get_oY(),
                                     features[c].get_oZ() - features[a].get_oZ());
                        vpPoint vecAD(features[d].get_oX() - features[a].get_oX(),
                                      features[d].get_oY() - features[a].get_oY(),
                                      features[d].get_oZ() - features[a].get_oZ());

                        // Normalenvektor bestimmen
                        vpPoint vecNorm((vecAC.get_oY()*vecAD.get_oZ()) - (vecAC.get_Z()*vecAD.get_oY()),
                                       (vecAC.get_oZ()*vecAD.get_oX()) - (vecAC.get_X()*vecAD.get_oZ()),
                                       (vecAC.get_oX()*vecAD.get_oY()) - (vecAC.get_Y()*vecAD.get_oX()));

                        // Koordinatenform bestimmen
                        double scalar = (vecAB.get_oX()*vecNorm.get_oX()) + (vecAB.get_oY()*vecNorm.get_oY()) + (vecAB.get_oZ()*vecNorm.get_oZ());

                        // Punkt in Ebene - Überprüfung
                        if (scalar == 0) {
                            result.push_back(features[a]);
                            result.push_back(features[b]);
                            result.push_back(features[c]);
                            result.push_back(features[d]);

                            return result;
                        }
                    }
                }
            }
        }
    }
    return result;
}
