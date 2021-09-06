#include "coverage_navigator/wall_follower.h"

std::string wall_follower::PARAM_NAME_robot_size = "robot_size";
std::string wall_follower::PARAM_NAME_odom_topic = "wall_following/odom_topic";
std::string wall_follower::PARAM_NAME_states_topic = "wall_following/states_topic";
std::string wall_follower::PARAM_NAME_high_speed = "wall_following/high_speed";
std::string wall_follower::PARAM_NAME_normal_speed = "wall_following/normal_speed";
std::string wall_follower::PARAM_NAME_low_speed = "wall_following/low_speed";

WallFollower::WallFollower() {}

WallFollower::~WallFollower() {}

void WallFollower::setup() {
    
    _n.reset(new ros::NodeHandle("~"));
    
    _n->getParam(wall_follower::PARAM_NAME_robot_size,_robot_size);
    _n->getParam(wall_follower::PARAM_NAME_odom_topic,_odom_topic);
    _n->getParam(wall_follower::PARAM_NAME_states_topic,_states_topic);
    _n->getParam(wall_follower::PARAM_NAME_high_speed,_high_speed);
    _n->getParam(wall_follower::PARAM_NAME_normal_speed,_normal_speed);
    _n->getParam(wall_follower::PARAM_NAME_low_speed,_low_speed);
    _position_tolerance = _robot_size;
    _collided = false;
    _do_leave = false;
    _do_return = false;
    _finish_following = false;
}

void WallFollower::run() {
    _collided = false;
    _do_leave = false;
    _do_return = false;
    _finish_following = false;

    auto _origin_msg = ros::topic::waitForMessage<nav_msgs::Odometry>(_odom_topic, *(_n.get()));
    _origin.position.x = _origin_msg->pose.pose.position.x;
    _origin.position.y = _origin_msg->pose.pose.position.y;
    _origin.position.z = 0;
    _origin.orientation = _origin_msg->pose.pose.orientation;
    _origin_yaw = tf::getYaw(_origin_msg->pose.pose.orientation)/M_PI*180.0;
    // _origin_yaw: [0, 360)
    if(_origin_yaw<0){
        _origin_yaw+=360.0;
    }

    _odom_subscriber = _n->subscribe(_odom_topic,10, &WallFollower::odomCallback, this);
    _states_subscriber = _n->subscribe(_states_topic, 10, &WallFollower::statesCallback, this);

    ros::Duration _interval(0.001);
    while (ros::ok()) {

        if(_finish_following){
            break;
        }
        ros::spinOnce();
        _interval.sleep();
    }
}

void WallFollower::shutDown() {
    _odom_subscriber.shutdown();
    _states_subscriber.shutdown();
}

void WallFollower::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg){
    _odom = *odom_msg;

    float position_error = compute_position_error(_origin,_odom.pose.pose);
    float yaw = tf::getYaw(odom_msg->pose.pose.orientation)/M_PI*180.0;
    // yaw: [0, 360)
    if(yaw<0){
        yaw+=360.0;
    }
    // yaw-origin_yaw
    float yaw_offset = compute_relative_yaw(_origin_yaw, yaw);

    if(!_do_leave){
        if(position_error > _position_tolerance){
            _do_leave = true;
        }
    }else{
        if(position_error <= _position_tolerance
           && yaw_offset > 45){
            _do_return = true;
        }
    }

    if(_do_return){
        ROS_INFO("RETURN TO ORIGIN ALREADY!");
        robotStop();
        _finish_following = true;
        return;
    }
}

void WallFollower::statesCallback(const custom_msgs::DetectionState::ConstPtr& states_msg){
    custom_msgs::DetectionState detection_states = *states_msg;

    if(detection_states.heading== detection_states.COLLIDED|| detection_states.heading_left==detection_states.COLLIDED || detection_states.heading_right==detection_states.COLLIDED){
        _collided = true;
    }else{
        _collided = false;
    }

    // 右沿墙
    if(_collided){
        robotMove(-_high_speed,_high_speed); // 碰撞即原地左转
//        ROS_INFO("COLLIDED!");
    }else if(detection_states.right==detection_states.AWAY_FROM_WALL){
        robotMove(_high_speed, _normal_speed); // 沿墙贴紧
//        ROS_INFO("AWAY FROM WALL!");
    }else if(detection_states.right==detection_states.NO_WALL){
        robotMove(_high_speed, _low_speed); // 拐角处拐弯
//        ROS_INFO("AT CORNER!");
    }else{
        robotMove(_high_speed, _high_speed);
//        ROS_INFO("ALONG WITH WALL!");
    }
}