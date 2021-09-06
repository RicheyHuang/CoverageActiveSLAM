#include "coverage_mapper/coverage_mapper.h"

const cv::Scalar COVERAGE = cv::Scalar(203, 192, 255);

int main(int argc, char** argv){

    ros::init(argc, argv, "coverage_mapper_node");
    CoverageMapper coverage_mapper;

    return 0;
}