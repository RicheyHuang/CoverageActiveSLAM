#ifndef COVERAGE_MAPPER_COVERAGE_MAPPER_H
#define COVERAGE_MAPPER_COVERAGE_MAPPER_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"

#include "boost/thread/recursive_mutex.hpp"
#include "opencv2/opencv.hpp"

extern const cv::Scalar COVERAGE;

extern std::string PARAM_NAME_robot_radius;
extern std::string PARAM_NAME_world_min_x;
extern std::string PARAM_NAME_world_max_x;
extern std::string PARAM_NAME_world_min_y;
extern std::string PARAM_NAME_world_max_y;
extern std::string PARAM_NAME_map_resolution;

extern std::string PARAM_NAME_map_topic;
extern std::string PARAM_NAME_occupancy_map_topic;
extern std::string PARAM_NAME_odom_topic;
extern std::string PARAM_NAME_trajectory_topic;
extern std::string PARAM_NAME_coverage_map_topic;
extern std::string PARAM_NAME_coverage_image_topic;

extern std::string PARAM_NAME_trajectory_frame;
extern std::string PARAM_NAME_map_frame;

class CoverageMapper{
public:
    CoverageMapper();

    ~CoverageMapper();

private:
    void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy);

    bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my);

    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);

    void occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& occupancy_grid_msg);

    boost::shared_ptr<ros::NodeHandle> _n;

    ros::Subscriber _odom_subscriber;

    ros::Publisher _trajectory_publisher;
    ros::Subscriber _trajectory_subscriber;

    ros::Subscriber _occupancy_grid_subscriber;
    ros::Publisher _occupancy_map_publisher;

    ros::Publisher _coverage_map_publisher;
    ros::Publisher _coverage_image_publisher;

    std::string _map_topic;
    std::string _occupancy_map_topic;
    std::string _odom_topic;
    std::string _trajectory_topic;
    std::string _coverage_map_topic;
    std::string _coverage_image_topic;

    std::string _trajectory_frame;
    std::string _map_frame;

    nav_msgs::Path _trajectory;

    boost::recursive_mutex _mutex;

    cv::Mat _occupancy_mask; // 8UC3
    cv::Mat _occupancy_map; // 8UC3
    cv::Mat _coverage_layer; // 8UC3
    cv::Mat _coverage_map; // 8UC3
    cv::Mat _coverage_image; // 8UC3

    float _world_min_x;
    float _world_max_x;
    float _world_min_y;
    float _world_max_y;
    float _map_resolution;

    unsigned int _map_width;
    unsigned int _map_height;
    float _origin_x;
    float _origin_y;
    unsigned int _size_x;
    unsigned int _size_y;

    float _robot_radius;
    unsigned int _robot_radius_pix;

    bool _initialized;
};

#endif //COVERAGE_MAPPER_COVERAGE_MAPPER_H
