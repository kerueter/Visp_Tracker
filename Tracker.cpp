#include <set>
#include <map>
#include "Tracker.hpp"

#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms

/**
 * Constructor for the tracker
 */
Tracker::Tracker() {
    initTracking = false;
    initPose = true;
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
        m_tracker.setQuality(0.01);
        m_tracker.setMinDistance(15);
        m_tracker.setHarrisFreeParameter(0.04);
        m_tracker.setBlockSize(9);
        m_tracker.setUseHarris(1);
        m_tracker.setPyramidLevels(3);

        // Initialise the tracking
        m_tracker.initTracking(frame);
        m_tracker.track(frame);
        initTracking = true;
    }
    catch(vpException &e) {
        cout << "Catch an exception: " << e << endl;
        initTracking = false;
    }


}

/**
 * Tracks the keypoints from the given camera frame
 */
void Tracker::keypointTrack(cv::Mat frame) {
    try {
        //m_tracker.initTracking(frame);
        m_tracker.track(frame);
    }
    catch(vpException &e) {
        cout << "Catch an exception: " << e << endl;
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
        }
        else {
            keypointTrack(matColor);
        }

        vector<vpPoint> features;
        vector<int> featureIds;
        vpHomogeneousMatrix cMo;

        // convert all feature points
        for(int i = 0; i < m_tracker.getFeatures().size(); i++) {
            for(int y = 0; y < (int)m_tracker.getFeatures()[i].y; ++y) {
                for(int x = 0; x <(int)m_tracker.getFeatures()[i].x; ++x) {
                    ++depthData;
                }
            }
            CoordinateConverter::convertDepthToWorld(depth, (int)m_tracker.getFeatures()[i].x, (int)m_tracker.getFeatures()[i].y, *depthData, &worldX, &worldY, &worldZ);
            if(!(fabsf(worldX) == 0 || fabsf(worldY) == 0 || fabsf(worldZ) == 0))
                features.push_back(vpPoint(worldX, worldY, worldZ));
                featureIds.push_back((int)m_tracker.getFeaturesId()[i]);
            depthData = depthDataBegin;
        }

        if(features.size() > 3)
            map<vpPoint,int> ransacRes = ransacFinding(features, featureIds);

        /*std::vector<vpPoint> result;

        if(features.size() > 3) {
            result = findCoplanarPoints(features);
            if(result.size() > 0) {
                if(computePose(features, initPose, cMo)) {
                    if (initPose)
                        initPose = false;
                    vpTranslationVector trans = cMo.getTranslationVector();
                    for (unsigned int i = 0; i < trans.size(); i++) {
                        std::cout << (trans[i] / 1000) << std::endl;
                    }
                    std::cout << "\n";
                }
                std::cout << "coplanar points, yay" << std::endl;
            }
            else {
                std::cout << "no coplanar points" << std::endl;
            }
        }*/
    }
}

/**
 * Pose estimation for given position
 */
bool Tracker::computePose(std::vector<vpPoint> &point, bool init, vpHomogeneousMatrix &cMo) {

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
        return true;
    }
    catch(vpException &e) {
        cout << "Catch an exception: " << e << endl;
        return false;
    }
}

/**
 * Finding best level for keypoint tracking
 */
map<vpPoint,int> Tracker::ransacFinding(vector<vpPoint> features, vector<int> featureIds) {

    map<vpPoint,int> bestlevel;
    vpPoint point1;
    vpPoint point2;
    vpPoint point3;
    int pointId1;
    int pointId2;
    int pointId3;


    // Distance values
    float bestdist = numeric_limits<float>::max();
    float dist = 0;

    int iterations = 0;
    int nonimproving_iterations = 0;
    int max_iterations = 10;

    cout << "-----------------------------" << endl;
    while((nonimproving_iterations < 5) && (iterations < max_iterations)) {

        int rand_num;
        std::set<int> ids;

        do {
            rand_num = (int)(rand() % features.size());
            ids.insert(rand_num);
        }
        while(ids.size() < 3);

        // copy set into vector
        vector<unsigned long> sample_ids(ids.size());
        std::copy(ids.begin(), ids.end(), sample_ids.begin());

        // get the random points
        point1 = vpPoint(features[sample_ids[0]].get_oX(), features[sample_ids[0]].get_oY(), features[sample_ids[0]].get_oZ());
        point2 = vpPoint(features[sample_ids[1]].get_oX(), features[sample_ids[1]].get_oY(), features[sample_ids[1]].get_oZ());
        point3 = vpPoint(features[sample_ids[2]].get_oX(), features[sample_ids[2]].get_oY(), features[sample_ids[2]].get_oZ());
        pointId1 = featureIds[sample_ids[0]];
        pointId2 = featureIds[sample_ids[1]];
        pointId3 = featureIds[sample_ids[2]];


        // Richtungsvektoren bestimmen
        vpPoint vecAB(point2.get_oX() - point1.get_oX(),
                      point2.get_oY() - point1.get_oY(),
                      point2.get_oZ() - point1.get_oZ());
        vpPoint vecAC(point3.get_oX() - point1.get_oX(),
                      point3.get_oY() - point1.get_oY(),
                      point3.get_oZ() - point1.get_oZ());

        // Normalenvektor bestimmen
        vpPoint vecNorm((vecAB.get_oY()*vecAC.get_oZ()) - (vecAB.get_Z()*vecAC.get_oY()),
                        (vecAB.get_oZ()*vecAC.get_oX()) - (vecAB.get_X()*vecAC.get_oZ()),
                        (vecAB.get_oX()*vecAC.get_oY()) - (vecAB.get_Y()*vecAC.get_oX()));

        // Vektor normalisieren
        double norm = sqrt((vecNorm.get_oX()*vecNorm.get_oX()) + (vecNorm.get_oY()*vecNorm.get_oY()) + (vecNorm.get_oZ()*vecNorm.get_oZ()));

        for(vpPoint feature : features) {
            double scalar = (vecNorm.get_oX()*feature.get_oX()) + (vecNorm.get_oY()*feature.get_oY()) + (vecNorm.get_oZ()*feature.get_oZ());
            double a = (vecNorm.get_oX()*point1.get_oX()) + (vecNorm.get_oY()*point1.get_oY()) + (vecNorm.get_oZ()*point1.get_oZ());

            dist += fabs(scalar - a);
            if(norm != 0)
                dist /= norm;
        }

        if(dist < bestdist) {
            bestdist = dist;
            nonimproving_iterations = 0;

            bestlevel.clear();
            bestlevel.insert(pair<vpPoint,int>(point1,pointId1));
            bestlevel.insert(pair<vpPoint,int>(point2,pointId2));
            bestlevel.insert(pair<vpPoint,int>(point3,pointId3));
        }
        else {
            nonimproving_iterations++;
        }
        cout << bestdist << endl;
        iterations++;
    }

    return bestlevel;
}
