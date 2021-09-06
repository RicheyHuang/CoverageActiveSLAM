#include "coverage_navigator/global_traverser.h"
#include "opencv2/opencv.hpp"

const cv::Vec3b COVERAGE = cv::Vec3b(203, 192, 255);

int main(int argc, char** argv){

    ros::init(argc, argv, "coverage_navigator_node");

    GlobalTraverser global_traverser;
    global_traverser.robotPowerOn();
    global_traverser.setup();
    global_traverser.run();
    global_traverser.shutDown();
    global_traverser.robotPowerOff();

    return 0;
}