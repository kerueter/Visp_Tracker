#include "Tracker.hpp"

#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms

/**
 * Constructor for the tracker
 */
Tracker::Tracker() {
    g = cv::VideoCapture(0);
}

/**
 * Destructor for the tracker
 */
Tracker::~Tracker() {}

/*int Tracker::lineTrack() {
    try {
        if(!g.isOpened()) { // check if we succeeded
            std::cout << "Failed to open the camera" << std::endl;
            return -1;
        }
        g >> frame; // get a new frame from camera
        vpImageConvert::convert(frame, I);

        vpDisplayOpenCV d(I, 0, 0, "Camera view");
        vpDisplay::display(I);
        vpDisplay::flush(I);

        vpMe me;
        me.setRange(25);
        me.setThreshold(15000);
        me.setSampleStep(10);

        vpMeLine line;
        line.setMe(&me);
        line.setDisplay(vpMeSite::RANGE_RESULT);
        line.initTracking(I);

        while(1) {
            g >> frame;
            vpImageConvert::convert(frame, I);
            vpDisplay::display(I);
            line.track(I);
            line.display(I, vpColor::red);
            vpDisplay::flush(I);
        }
    }
    catch(vpException &e) {
        std::cout << "Catch an exception: " << e << std::endl;
    }
}*/

/*int Tracker::ellipseTrack() {
    try {
        if(!g.isOpened()) { // check if we succeeded
            std::cout << "Failed to open the camera" << std::endl;
            return -1;
        }
        g >> frame; // get a new frame from camera
        vpImageConvert::convert(frame, I);

        vpDisplayOpenCV d(I, 0, 0, "Camera view");
        vpDisplay::display(I);
        vpDisplay::flush(I);

        vpMe me;
        me.setRange(25);
        me.setThreshold(15000);
        me.setSampleStep(10);
        vpMeEllipse ellipse;
        ellipse.setMe(&me);
        ellipse.setDisplay(vpMeSite::RANGE_RESULT);
        ellipse.initTracking(I);
        while(1) {
            g >> frame;
            vpImageConvert::convert(frame, I);
            vpDisplay::display(I);
            ellipse.track(I);
            ellipse.display(I, vpColor::red);
            vpDisplay::flush(I);
        }
    }
    catch(vpException &e) {
        std::cout << "Catch an exception: " << e << std::endl;
    }
}*/

/*int Tracker::blobTrack() {

    if(!g.isOpened()) { // check if we succeeded
        std::cout << "Failed to open the camera" << std::endl;
        return -1;
    }
    g >> frame; // get a new frame from camera
    vpImageConvert::convert(frame, I);

    vpDisplayOpenCV d(I, 0, 0, "Camera view");

    vpDot2 blob;
    blob.setGraphics(true);
    blob.setGraphicsThickness(2);
    vpImagePoint germ;
    bool init_done = false;
    std::cout << "Click!!!" << std::endl;
    while(1) {
        try {
            g >> frame;
            vpImageConvert::convert(frame, I);
            vpDisplay::display(I);
            if (!init_done) {
                vpDisplay::displayText(I, vpImagePoint(10,10), "Click in the blob to initialize the tracker", vpColor::red);
                if (vpDisplay::getClick(I, germ, false)) {
                    blob.initTracking(I, germ);
                    init_done = true;
                }
            }
            else {
                blob.track(I);
            }
            vpDisplay::flush(I);
        }
        catch(...) {
            init_done = false;
        }
    }
}*/

/**
 * Tracks the keypoints from the given camera frame
 */
int Tracker::initKeypointTrack(cv::Mat& frame) {
    try {
        // nur zu Testzwecken
        if(!g.isOpened()) { // check if we succeeded
            std::cout << "Failed to open the camera" << std::endl;
            return -1;
        }
        g >> frame;
        vpImageConvert::convert(frame, I);
        vpImageConvert::convert(I, frame);

        m_window = vpDisplayOpenCV(I, 0, 0, "Camera view");
        vpDisplay::display(I);
        vpDisplay::flush(I);

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

void Tracker::keypointTrack(cv::Mat& frame) {
    try {
        // nur zu Testzwecken
        g >> frame;
        vpDisplay::display(I);
        vpImageConvert::convert(frame, I);
        vpImageConvert::convert(I, frame);

        // ab hier Tracking
        m_tracker.track(frame);
        m_tracker.display(I, vpColor::red);
        vpDisplay::flush(I);
        m_tracker.initTracking(frame);
    }
    catch(vpException &e) {
        std::cout << "Catch an exception: " << e << std::endl;
    }
}

int Tracker::trackPoints(cv::Mat& depth, cv::Mat& frame) {
    Status rc = OpenNI::initialize();
    initKeypointTrack(frame);

    if (rc != STATUS_OK)
    {
        printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
        return 1;
    }

    Device device;
    rc = device.open(ANY_DEVICE);
    if (rc != STATUS_OK)
    {
        printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
        return 2;
    }

    VideoStream tdepth;
    if (device.getSensorInfo(SENSOR_DEPTH) != NULL)
    {
        rc = tdepth.create(device, SENSOR_DEPTH);
        if (rc != STATUS_OK)
        {
            printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
            return 3;
        }
    }
    rc = tdepth.start();
    if (rc != STATUS_OK)
    {
        printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
        return 4;
    }

    VideoFrameRef tframe;

    float worldX;
    float worldY;
    float worldZ;

    while(1) {
        int changedStreamDummy;
        VideoStream* pStream = &tdepth;
        rc = OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);
        if (rc != STATUS_OK)
        {
            printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
            continue;
        }
        rc = tdepth.readFrame(&tframe);
        if (rc != STATUS_OK)
        {
            printf("Read failed!\n%s\n", OpenNI::getExtendedError());
            continue;
        }

        DepthPixel* pDepth = (DepthPixel*)tframe.getData();

        keypointTrack(frame);

        // convert all feature points
        for(unsigned int i = 0; i < m_tracker.getFeatures().size(); i++)
        {

        }
        /*for(int y = 0; y < tdepth.getVideoMode().getResolutionY(); ++y) {
            for(int x = 0; x < tdepth.getVideoMode().getResolutionX(); ++x, ++pDepth) {
                CoordinateConverter::convertDepthToWorld(tdepth, x, y, *pDepth, &worldX, &worldY, &worldZ);
                // transform world parameters from millimeters to meters
                worldX /= 1000;
                worldY /= 1000;
                worldZ /= 1000;
                std::cout << worldX << " " << worldY << " " << worldZ << std::endl;
            }
        }*/
    }
}

bool Tracker::findCoplanarPoints(std::vector<Quarternion> features) {

    for(unsigned int a = 0; a < features.size(); a++) {
        for(unsigned int b = 0; b < features.size(); b++) {
            for(unsigned int c = 0; c < features.size(); c++) {
                for(unsigned int d = 0; d < features.size(); d++) {
                    Quarternion firstCalc(features[c].getX() - features[a].getX(),
                                          features[c].getY() - features[a].getY(),
                                          features[c].getZ() - features[a].getZ(),
                                          features[c].getW() - features[a].getW());
                    Quarternion secondCalc(features[b].getX() - features[a].getX(),
                                          features[b].getY() - features[a].getY(),
                                          features[b].getZ() - features[a].getZ(),
                                          features[b].getW() - features[a].getW());
                    Quarternion thirdCalc(features[d].getX() - features[c].getX(),
                                          features[d].getY() - features[c].getY(),
                                          features[d].getZ() - features[c].getZ(),
                                          features[d].getW() - features[c].getW());
                    Quarternion crossSecondThird((secondCalc.getY()*thirdCalc.getZ()) - (secondCalc.getZ()*thirdCalc.getY()),
                                                 (secondCalc.getZ()*thirdCalc.getX()) - (secondCalc.getX()*thirdCalc.getZ()),
                                                 (secondCalc.getX()*thirdCalc.getY()) - (secondCalc.getY()*thirdCalc.getX()),
                                                 0);
                    Quarternion result(firstCalc.getX() * crossSecondThird.getX(),
                                       firstCalc.getY() * crossSecondThird.getY(),
                                       firstCalc.getZ() * crossSecondThird.getZ(),
                                       firstCalc.getW() * crossSecondThird.getW());

                    if((result.getX() == 0) && (result.getY() == 0) && (result.getZ() == 0) && (result.getW() == 0))
                        return true;
                }
            }
        }
    }
    return false;
}
