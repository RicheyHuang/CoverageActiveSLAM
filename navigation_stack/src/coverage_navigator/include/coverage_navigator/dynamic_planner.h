#ifndef COVERAGE_NAVIGATOR_DYNAMIC_PLANNER_H
#define COVERAGE_NAVIGATOR_DYNAMIC_PLANNER_H

#include <vector>
#include <numeric>

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "cv_bridge/cv_bridge.h"
#include "move_base/move_base.h"
#include "actionlib/client/simple_action_client.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/time_synchronizer.h"

#include "custom_msgs/Zone.h"
#include "custom_msgs/Obstacles.h"
#include "custom_msgs/DetectionState.h"
#include "sensor_msgs/Image.h"
#include "nav_msgs/Odometry.h"
#include "move_base_msgs/MoveBaseGoal.h"

#include "custom_srvs/Zone.h"
#include "custom_srvs/Obstacle.h"

#include "opencv2/opencv.hpp"

#include "coverage_navigator/utilities.h"
#include "coverage_navigator/wall_follower.h"
#include "coverage_navigator/zigzag_walker.h"
#include "coverage_navigator/stuck_escaper.h"

extern const cv::Vec3b COVERAGE;

typedef message_filters::sync_policies::ApproximateTime<custom_msgs::DetectionState, nav_msgs::Odometry> syncStateOdomPolicy;

namespace dynamic_planner{

    extern std::string PARAM_NAME_robot_radius;
    extern std::string PARAM_NAME_robot_size;

    extern std::string PARAM_NAME_high_speed;
    extern std::string PARAM_NAME_normal_speed;
    extern std::string PARAM_NAME_low_speed;
    extern std::string PARAM_NAME_speed_increment;

    extern std::string PARAM_NAME_path_plan_frame;
    extern std::string PARAM_NAME_odom_topic;
    extern std::string PARAM_NAME_states_topic;
    extern std::string PARAM_NAME_coverage_map_topic;
    extern std::string PARAM_NAME_occupancy_map_topic;
    extern std::string PARAM_NAME_make_zone_service;
    extern std::string PARAM_NAME_make_obstacles_service;
    extern std::string PARAM_NAME_make_plan_service;
    extern std::string PARAM_NAME_move_action;

    extern std::string PARAM_NAME_yaw_buffer;
    extern std::string PARAM_NAME_yaw_tolerance;
    extern std::string PARAM_NAME_position_tolerance;
    extern std::string PARAM_NAME_time_tolerance;

}

class DynamicPlanner : public Robot {
public:

    void setup();
    void run(const custom_msgs::Zone& zone);
    void shutDown();

    void robotRotate(DIRECTION rotating_direction, const float& wheel_speed) override;
    void robotRotate(DIRECTION rotating_direction, SPEED_LEVEL speed_level) override;

    void clearViableZone();
    void setViableZone();
    void setBoundary();
    void clearBoundary();
    void lookAroundCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
    void lookAround(const nav_msgs::Odometry::ConstPtr& odom_msg);
    void setupZoneContour();
    void updateBoundaryAndObstacles(const sensor_msgs::ImageConstPtr& occupancy_grid_map_msg);
    void findMainDirection(const sensor_msgs::ImageConstPtr& occupancy_grid_map_msg);
    void clearObstacles();
    void setObstacles();
    bool isPathFound(const nav_msgs::Odometry& start_odom, const geometry_msgs::Pose& target_pose);
    bool getOutOfStuck(const geometry_msgs::Pose& goal_pose);
    bool moveToGoal(const move_base_msgs::MoveBaseGoal& goal);
    void parallelAtWallCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
    void aimAtWallCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
    void travelToWallCallback(const custom_msgs::DetectionState::ConstPtr& states_msg, const nav_msgs::Odometry::ConstPtr& odom_msg);
    void headingToWall(const nav_msgs::Odometry::ConstPtr& odom_msg, const custom_msgs::DetectionState::ConstPtr& states_msg);
    void alignWithWall(const nav_msgs::Odometry::ConstPtr& odom_msg);
    void setupZigzaggingInitTurningDirection();
    void setupZigzaggingNewTurningDirection();
    bool queryNextWaypoint();
    void updateNextWaypoints();
    bool coverNextWaypoints();

private:
    boost::shared_ptr<ros::NodeHandle> _n;
    ros::Subscriber _odom_subscriber;
    message_filters::Subscriber<custom_msgs::DetectionState> _sync_state_sub;
    message_filters::Subscriber<nav_msgs::Odometry> _sync_odom_sub;
    boost::shared_ptr<message_filters::Synchronizer<syncStateOdomPolicy>> _state_odom_sync;
    boost::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> _ac;
    ros::ServiceClient _sc;

    std::string _odom_topic;
    std::string _states_topic;
    std::string _coverage_map_topic;
    std::string _occupancy_map_topic;
    std::string _make_zone_service;
    std::string _make_obstacles_service;
    std::string _make_plan_service;

    std::string _move_action;

    float _robot_radius;
    float _robot_size;

    SPEED_LEVEL _speed_level;
    float _high_speed;
    float _normal_speed;
    float _low_speed;
    float _speed_increment;

    std::string _path_plan_frame;

    DIRECTION _robot_turning_direction;
    DIRECTION _robot_rotating_direction;

    bool _start_rotation;
    bool _finish_rotation;

    float _target_yaw;
    int _main_direction; // degree [0,180)
    int _inverse_main_direction; // degree [180,360)

    float _yaw_buffer; // degree
    float _yaw_tolerance; // degree
    float _position_tolerance; // m
    float _time_tolerance; // second

    bool _collided;

    cv::Mat _boundary_map;
    cv::Mat _uncovered_areas_map;

    geometry_msgs::Pose _next_uncovered_position;
    std::deque<geometry_msgs::Point> _uncovered_positions;

    custom_msgs::Zone _viable_zone;
    custom_msgs::Zone _boundary;
    custom_msgs::Obstacles _obstacles;

    std::vector<std::vector<cv::Point>> _contours; // [boundary, obstacle1, ... , obstacleN] contour形式
    std::vector<std::vector<cv::Point>> _polys; // [boundary, obstacle1, ... , obstacleN] polygon形式

    std::vector<cv::Point> _zone_contour;
    std::vector<cv::Point> _wall_contour; // 靠近boundary的可行轮廓区域
    std::vector<cv::Point> _boundary_contour;
    std::vector<std::vector<cv::Point>> _obstacle_contours;

};


#endif //COVERAGE_NAVIGATOR_DYNAMIC_PLANNER_H
