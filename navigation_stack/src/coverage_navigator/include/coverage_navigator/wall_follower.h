#ifndef COVERAGE_NAVIGATOR_WALL_FOLLOWER_H
#define COVERAGE_NAVIGATOR_WALL_FOLLOWER_H

#include "ros/ros.h"
#include "tf/transform_listener.h"

#include "nav_msgs/Odometry.h"
#include "custom_msgs/DetectionState.h"

#include "coverage_navigator/utilities.h"

namespace wall_follower {

    extern std::string PARAM_NAME_robot_size;
    extern std::string PARAM_NAME_odom_topic;
    extern std::string PARAM_NAME_states_topic;
    extern std::string PARAM_NAME_high_speed;
    extern std::string PARAM_NAME_normal_speed;
    extern std::string PARAM_NAME_low_speed;

}

class WallFollower : public Robot{
public:
    WallFollower();
    ~WallFollower();
    void setup();
    void run();
    void shutDown();
private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
    void statesCallback(const custom_msgs::DetectionState::ConstPtr& states_msg);

    boost::shared_ptr<ros::NodeHandle> _n;
    ros::Subscriber _odom_subscriber;
    ros::Subscriber _states_subscriber;
    nav_msgs::Odometry _odom;

    std::string _odom_topic;
    std::string _states_topic;

    bool _collided;

    geometry_msgs::Pose _origin;
    float _origin_yaw; // degree
    bool _do_leave;
    bool _do_return;
    bool _finish_following;

    float _robot_size;

    float _position_tolerance;

    float _high_speed;
    float _normal_speed;
    float _low_speed;
};


#endif //COVERAGE_NAVIGATOR_WALL_FOLLOWER_H
