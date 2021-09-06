#ifndef COVERAGE_NAVIGATOR_ZIGZAG_WALKER_H
#define COVERAGE_NAVIGATOR_ZIGZAG_WALKER_H

#include <deque>

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "cv_bridge/cv_bridge.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/time_synchronizer.h"

#include "sensor_msgs/Image.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "custom_msgs/DetectionState.h"

#include "coverage_navigator/utilities.h"

#include "opencv2/opencv.hpp"


extern const cv::Vec3b COVERAGE;

namespace zigzag_walker {

    extern std::string PARAM_NAME_robot_size;
    extern std::string PARAM_NAME_high_speed;
    extern std::string PARAM_NAME_normal_speed;
    extern std::string PARAM_NAME_low_speed;
    extern std::string PARAM_NAME_speed_increment;
    extern std::string PARAM_NAME_odom_topic;
    extern std::string PARAM_NAME_states_topic;
    extern std::string PARAM_NAME_coverage_map_topic;
    extern std::string PARAM_NAME_occupancy_map_topic;
    extern std::string PARAM_NAME_yaw_buffer;
    extern std::string PARAM_NAME_yaw_tolerance;
    extern std::string PARAM_NAME_position_tolerance;

}

typedef message_filters::sync_policies::ApproximateTime<custom_msgs::DetectionState, nav_msgs::Odometry> syncStateOdomPolicy;

class ZigzagWalker : public Robot{
public:
    ZigzagWalker();
    ~ZigzagWalker();
    void setup(const cv::Mat& boundary_map);
    void run(bool in_refine_mode, DIRECTION turning_direction, float base_yaw);
    void robotRotate(DIRECTION rotating_direction, const float& wheel_speed) override;
    void robotTurning(DIRECTION turning_direction, const float& wheel_speed) override;
    void robotRotate(DIRECTION rotating_direction, SPEED_LEVEL speed_level) override;
    void robotTurning(DIRECTION turning_direction, SPEED_LEVEL speed_level) override;
    void updateRecentPositions();
    bool switchTuningDirection();
    void updateBaseYaw();
    bool isVicinitiesCovered(const nav_msgs::Odometry& odom, int search_range_pix=0);
    void syncStateOdomCallback(const custom_msgs::DetectionState::ConstPtr& states_msg, const nav_msgs::Odometry::ConstPtr& odom_msg);
    void shutDown();
private:
    boost::shared_ptr<ros::NodeHandle> _n;
    message_filters::Subscriber<custom_msgs::DetectionState> _sync_state_sub;
    message_filters::Subscriber<nav_msgs::Odometry> _sync_odom_sub;
    boost::shared_ptr<message_filters::Synchronizer<syncStateOdomPolicy>> _state_odom_sync;

    std::string _coverage_map_topic;
    std::string _occupancy_map_topic;
    std::string _odom_topic;
    std::string _states_topic;

    int _position_queue_size;
    std::deque<geometry_msgs::Pose> _recent_positions;

    float _robot_size;

    bool _near_obstacle;
    bool _collided;

    float _position_tolerance;
    float _yaw_tolerance;

    bool _is_turning;
    bool _is_interrupted;
    bool _finish_turning;
    unsigned int _turning_num;
    bool _make_first_turning;
    bool _make_second_turning;
    bool _in_refine_mode;

    bool _finish_zigzagging;

    float _base_yaw;
    float _yaw_buffer; // degree
    DIRECTION _robot_turning_direction;
    DIRECTION _robot_rotating_direction;

    SPEED_LEVEL _speed_level;
    float _high_speed;
    float _normal_speed;
    float _low_speed;
    float _speed_increment;

    cv::Mat _boundary_map;
};


#endif //COVERAGE_NAVIGATOR_ZIGZAG_WALKER_H
