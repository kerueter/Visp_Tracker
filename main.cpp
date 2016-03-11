#include <iostream>
#include "Tracker.hpp"

using namespace std;

int main() {

    int choice = 0;
    Tracker tracker = Tracker();
    cv::Mat test1, test2;

    cout << "1. Line Track\n2. Ellipse Track\n3. Blob Track\n4. Keypoint Track\n5. NI Test\nAuswahl: ";
    cin >> choice;

    switch(choice) {
        case 1:
            tracker.lineTrack();
            break;
        case 2:
            tracker.ellipseTrack();
            break;
        case 3:
            tracker.blobTrack();
            break;
        case 4:
            tracker.keypointTrack();
            break;
        case 5:
            tracker.niTest(test1, test2);
            break;
        default:
            cout << "Falsche Auswahl!" << endl;
            break;
    }
    return 0;
}