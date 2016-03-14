#include "Tracker.hpp"

#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms

/**
 * Constructor for the tracker
 */
Tracker::Tracker() {
    initTracking = false;
}

/**
 * Destructor for the tracker
 */
Tracker::~Tracker() {}

/**
 * Initializes the keypoint tracker
 */
int Tracker::initKeypointTrack(cv::Mat frame) {
    try {
        // ab hier Initialisierung
        m_tracker.setMaxFeatures(100);
        m_tracker.setWindowSize(10);
        m_tracker.setQuality(0.6);
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
        m_tracker.display(I, vpColor::red);
        vpDisplay::flush(I);
        m_tracker.initTracking(frame);
    }
    catch(vpException &e) {
        std::cout << "Catch an exception: " << e << std::endl;
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

    VideoFrameRef frame;
    color.start();

    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);

    Status rc;
    while(1) {

        int changedStreamDummy;
        VideoStream* pStream = &color;
        rc = OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);

        if (rc != STATUS_OK)
        {
            printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
            continue;
        }
        rc = color.readFrame(&frame);
        if (rc != STATUS_OK)
        {
            printf("Read failed!\n%s\n", OpenNI::getExtendedError());
            continue;
        }

        colorData = (RGB888Pixel *) frame.getData();
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

        // convert all feature points
        /*for (unsigned int i = 0; i < m_tracker.getFeatures().size(); i++) {
            std::cout << "track x: " << (int) m_tracker.getFeatures()[i].x << " track y: " <<
            (int) m_tracker.getFeatures()[i].y << std::endl;
            //openni::CoordinateConverter::convertDepthToWorld(depth, (int)m_tracker.getFeatures()[i].x, (int)m_tracker.getFeatures()[i].y, *depthPixels, &worldX, &worldY, &worldZ);
            // transform world parameters from millimeters to meters
            //worldX /= 1000;
            //worldY /= 1000;
            //worldZ /= 1000;
        }
        for(int y = 0; y < tdepth.getVideoMode().getResolutionY(); ++y)
        {
            for(int x = 0; x < tdepth.getVideoMode().getResolutionX(); ++x, ++pDepth) {
                CoordinateConverter::convertDepthToWorld(tdepth, x, y, *pDepth, &worldX, &worldY, &worldZ);
                // transform world parameters from millimeters to meters
                worldX /= 1000;
                worldY /= 1000;
                worldZ /= 1000;

                if(!(fabsf(worldX) == 0 || fabsf(worldY) == 0 || fabsf(worldZ) == 0))
                    std::cout << worldX << " " << worldY << " " << worldZ << std::endl;
            }
        }*/
    }
}

/**
 * Algorithm for finding coplanar points
 */
std::vector<Vector> Tracker::findCoplanarPoints(std::vector<Vector> features) {

    std::vector<Vector> result;

    for(unsigned int a = 0; a < features.size(); a++) {
        for(unsigned int b = 0; b < features.size(); b++) {
            for(unsigned int c = 0; c < features.size(); c++) {
                for(unsigned int d = 0; d < features.size(); d++) {
                    // Richtungsvektoren bestimmen
                    Vector vecAB(features[b].getX() - features[a].getX(),
                                 features[b].getY() - features[a].getY(),
                                 features[b].getZ() - features[a].getZ());
                    Vector vecAC(features[c].getX() - features[a].getX(),
                                 features[c].getY() - features[a].getY(),
                                 features[c].getZ() - features[a].getZ());

                    // Normalenvektor bestimmen
                    Vector vecNorm((vecAB.getY()*vecAC.getZ()) - (vecAB.getZ()*vecAC.getY()),
                                   (vecAB.getZ()*vecAC.getX()) - (vecAB.getX()*vecAC.getZ()),
                                   (vecAB.getX()*vecAC.getY()) - (vecAB.getY()*vecAC.getX()));

                    // Koordinatenform bestimmen
                    float normA =  vecNorm.getX()*features[a].getX() +
                                    vecNorm.getY()*features[a].getY() +
                                    vecNorm.getZ()*features[a].getZ();

                    float normD =  vecNorm.getX()*features[d].getX() +
                                   vecNorm.getY()*features[d].getY() +
                                   vecNorm.getZ()*features[d].getZ();

                    // Punkt in Ebene - Überprüfung
                    if(normA == normD)
                        result.push_back(features[d]);
                }
            }
        }
    }
    return result;
}
