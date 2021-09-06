
#ifndef COVERAGE_NAVIGATOR_STUCK_ESCAPER_H
#define COVERAGE_NAVIGATOR_STUCK_ESCAPER_H

#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "tf/transform_listener.h"
#include "nav_msgs/Odometry.h"
#include "move_base/move_base.h"
#include "move_base_msgs/MoveBaseAction.h"

#include "coverage_navigator/utilities.h"

namespace stuck_escaper {

    extern std::string PARAM_NAME_high_speed;
    extern std::string PARAM_NAME_normal_speed;
    extern std::string PARAM_NAME_low_speed;
    extern std::string PARAM_NAME_yaw_buffer;
    extern std::string PARAM_NAME_yaw_tolerance;
    extern std::string PARAM_NAME_position_tolerance;
    extern std::string PARAM_NAME_time_tolerance;
    extern std::string PARAM_NAME_explore_time;
    extern std::string PARAM_NAME_path_plan_frame;
    extern std::string PARAM_NAME_odom_topic;
    extern std::string PARAM_NAME_global_path_topic;
    extern std::string PARAM_NAME_local_path_topic;
    extern std::string PARAM_NAME_make_plan_service;
    extern std::string PARAM_NAME_move_action;

}

class StuckEscaper : public Robot{
public:
    StuckEscaper();
    ~StuckEscaper();

    void robotRotate(DIRECTION rotating_direction, const float& wheel_speed) override;
    void robotRotate(DIRECTION rotating_direction, SPEED_LEVEL speed_level) override;
    void robotForward(SPEED_LEVEL speed_level) override;

    void setup();
    void escapeCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
    bool run(const geometry_msgs::Pose& goal_pose);
    void shutDown();

private:
    boost::shared_ptr<ros::NodeHandle> _n;
    ros::Subscriber _odom_subscriber;
    boost::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> _ac;

    std::string _path_plan_frame;
    std::string _odom_topic;
    std::string _global_path_topic;
    std::string _local_path_topic;

    std::string _make_plan_service;
    std::string _move_action;

    SPEED_LEVEL _speed_level;
    DIRECTION _robot_rotating_direction;

    bool _finish_rotation;
    float _yaw_buffer; // degree
    float _yaw_tolerance; // degree
    float _position_tolerance; // m
    float _time_tolerance; // second

    float _explore_time;

    float _next_yaw;

    float _high_speed;
    float _normal_speed;
    float _low_speed;

};

#endif //COVERAGE_NAVIGATOR_STUCK_ESCAPER_H
