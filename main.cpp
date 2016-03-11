#include <iostream>
#include "Tracker.hpp"

using namespace std;

int main() {

    int choice = 0;
    Tracker tracker = Tracker();
    cv::Mat test1, test2;

    tracker.trackPoints(test1, test2);
    return 0;
}