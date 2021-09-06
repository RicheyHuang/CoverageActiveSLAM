#include "coverage_navigator/stuck_escaper.h"

std::string stuck_escaper::PARAM_NAME_high_speed = "stuck_escaping/high_speed";
std::string stuck_escaper::PARAM_NAME_normal_speed = "stuck_escaping/normal_speed";
std::string stuck_escaper::PARAM_NAME_low_speed = "stuck_escaping/low_speed";
std::string stuck_escaper::PARAM_NAME_yaw_buffer = "stuck_escaping/yaw_buffer";
std::string stuck_escaper::PARAM_NAME_yaw_tolerance = "stuck_escaping/yaw_tolerance";
std::string stuck_escaper::PARAM_NAME_position_tolerance = "stuck_escaping/position_tolerance";
std::string stuck_escaper::PARAM_NAME_time_tolerance = "stuck_escaping/time_tolerance";
std::string stuck_escaper::PARAM_NAME_explore_time = "stuck_escaping/explore_time";
std::string stuck_escaper::PARAM_NAME_path_plan_frame = "stuck_escaping/path_plan_frame";
std::string stuck_escaper::PARAM_NAME_odom_topic = "stuck_escaping/odom_topic";
std::string stuck_escaper::PARAM_NAME_global_path_topic = "stuck_escaping/global_path_topic";
std::string stuck_escaper::PARAM_NAME_local_path_topic = "stuck_escaping/local_path_topic";
std::string stuck_escaper::PARAM_NAME_make_plan_service = "stuck_escaping/make_plan_service";
std::string stuck_escaper::PARAM_NAME_move_action = "stuck_escaping/move_action";

StuckEscaper::StuckEscaper() {}

StuckEscaper::~StuckEscaper() {}

void StuckEscaper::robotRotate(DIRECTION rotating_direction, const float& wheel_speed){
    switch(rotating_direction){
        case LEFT:
            robotMove(-wheel_speed,wheel_speed);
            break;
        case RIGHT:
            robotMove(wheel_speed,-wheel_speed);
            break;
        default:
            robotStop();
            break;
    }
}

void StuckEscaper::robotRotate(DIRECTION rotating_direction, SPEED_LEVEL speed_level){
    switch(speed_level){
        case HIGH:
            robotRotate(rotating_direction, _high_speed);
            break;
        case NORMAL:
            robotRotate(rotating_direction, _normal_speed);
            break;
        case LOW:
            robotRotate(rotating_direction, _low_speed);
            break;
        default:
            robotStop();
            break;
    }
}

void StuckEscaper::robotForward(SPEED_LEVEL speed_level) {
    switch(speed_level){
        case HIGH:
            robotMove(_high_speed, _high_speed);
            break;
        case NORMAL:
            robotMove(_normal_speed, _normal_speed);
            break;
        case LOW:
            robotMove(_low_speed, _low_speed);
            break;
        default:
            robotStop();
            break;
    }
}

void StuckEscaper::setup() {

    _n.reset(new ros::NodeHandle("~"));
    
    _n->getParam(stuck_escaper::PARAM_NAME_high_speed,_high_speed);
    _n->getParam(stuck_escaper::PARAM_NAME_normal_speed,_normal_speed);
    _n->getParam(stuck_escaper::PARAM_NAME_low_speed,_low_speed);
    _n->getParam(stuck_escaper::PARAM_NAME_yaw_buffer,_yaw_buffer);
    _n->getParam(stuck_escaper::PARAM_NAME_yaw_tolerance,_yaw_tolerance);
    _n->getParam(stuck_escaper::PARAM_NAME_position_tolerance,_position_tolerance);
    _n->getParam(stuck_escaper::PARAM_NAME_time_tolerance,_time_tolerance);
    _n->getParam(stuck_escaper::PARAM_NAME_explore_time, _explore_time);
    _n->getParam(stuck_escaper::PARAM_NAME_path_plan_frame,_path_plan_frame);
    _n->getParam(stuck_escaper::PARAM_NAME_odom_topic,_odom_topic);
    _n->getParam(stuck_escaper::PARAM_NAME_global_path_topic, _global_path_topic);
    _n->getParam(stuck_escaper::PARAM_NAME_local_path_topic, _local_path_topic);
    _n->getParam(stuck_escaper::PARAM_NAME_make_plan_service,_make_plan_service);
    _n->getParam(stuck_escaper::PARAM_NAME_move_action,_move_action);

    _speed_level = HIGH;
    _finish_rotation = false;

}

void StuckEscaper::escapeCallback(const nav_msgs::Odometry::ConstPtr &odom_msg){

    robotRotate(_robot_rotating_direction, _speed_level);

    auto odom = *odom_msg;
    float yaw = tf::getYaw(odom.pose.pose.orientation)/M_PI*180.0;
    // yaw: [0, 360)
    if(yaw<0){
        yaw+=360.0;
    }

    float yaw_offset = compute_relative_yaw(_next_yaw, yaw);

    if(std::abs(yaw_offset)<_yaw_buffer){
        _speed_level=NORMAL;
    }
    if((std::abs(yaw_offset) < _yaw_tolerance)
       || (_robot_rotating_direction == LEFT && yaw_offset > 0 && yaw_offset < _yaw_buffer) // 左转转过头了 及时停下
       || (_robot_rotating_direction == RIGHT && yaw_offset < 0 && yaw_offset > -_yaw_buffer)){ // 右转转过头了 及时停下
        _finish_rotation = true;
        robotStop();
        _speed_level=HIGH;
    }
}

bool StuckEscaper::run(const geometry_msgs::Pose& goal_pose){
    ROS_INFO("START TO GET OUT OF STUCK");
    _ac.reset(new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(_move_action, true)); // global namespace
    ros::ServiceClient makeplan_client = _n->serviceClient<nav_msgs::GetPlan>(_make_plan_service, true);
    nav_msgs::GetPlan makeplan_service;
    makeplan_service.request.tolerance = getMapResolution(); // minimal unit
    makeplan_service.request.goal.header.frame_id = _path_plan_frame;
    makeplan_service.request.goal.pose = goal_pose;
    makeplan_service.request.start.header.frame_id = _path_plan_frame;

    while(true){
        auto init_odom_msg = ros::topic::waitForMessage<nav_msgs::Odometry>(_odom_topic, *(_n.get()));
        auto init_odom = *init_odom_msg;
        makeplan_service.request.start.pose = init_odom.pose.pose;

        _ac->waitForServer();
        bool is_makeplan_succeed = makeplan_client.call(makeplan_service);

        if(is_makeplan_succeed){
            if(makeplan_service.response.plan.poses.empty()){
                ROS_INFO("MAKE PLAN FAILED");
                return false;
            }else{
                ROS_INFO("MAKE PLAN SUCCESSFULLY");
                auto path = makeplan_service.response.plan.poses;
                for(auto& next_pose:path){

                    auto current_odom_msg = ros::topic::waitForMessage<nav_msgs::Odometry>(_odom_topic, *(_n.get()));
                    auto current_pose = current_odom_msg->pose.pose;

                    auto yaw = tf::getYaw(current_pose.orientation)/M_PI*180.0;
                    // yaw: [0, 360)
                    if(yaw<0){
                        yaw+=360.0;
                    }

                    auto q = tf::createQuaternionFromYaw(atan2((next_pose.pose.position.y-current_pose.position.y),(next_pose.pose.position.x-current_pose.position.x))).normalized();
                    next_pose.pose.orientation.x = q.x();
                    next_pose.pose.orientation.y = q.y();
                    next_pose.pose.orientation.z = q.z();
                    next_pose.pose.orientation.w = q.w();

                    _next_yaw = tf::getYaw(next_pose.pose.orientation)/M_PI*180.0;
                    // next_yaw: [0, 360)
                    if(_next_yaw<0){
                        _next_yaw+=360.0;
                    }
                    // next_yaw-yaw
                    auto yaw_offset = compute_relative_yaw(yaw, _next_yaw);
                    if(yaw_offset >= 0){
                        _robot_rotating_direction = LEFT;
                    }else{
                        _robot_rotating_direction = RIGHT;
                    }
                    robotRotate(_robot_rotating_direction,_speed_level);

                    _finish_rotation = false;
                    _odom_subscriber = _n->subscribe(_odom_topic, 10, &StuckEscaper::escapeCallback, this);

                    ros::Duration interval(_explore_time);
                    while (ros::ok()) {
                        if(_finish_rotation){
                            _odom_subscriber.shutdown();
                            _finish_rotation = false;
                            ROS_INFO("DIRECTION CORRECTED.");
                            break;
                        }
                        ros::spinOnce();
                        interval.sleep();
                    }

//                    auto start_odom_msg = ros::topic::waitForMessage<nav_msgs::Odometry>(_odom_topic, *(_n.get()));
//                    auto start_time = ros::Time::now().toSec();
//                    robotForward(_speed_level);
//                    ros::Duration calculation_interval(0.1);
//                    while (ros::ok()) {
//                        auto now_odom_msg = ros::topic::waitForMessage<nav_msgs::Odometry>(_odom_topic, *(_n.get()));
//                        auto now_time = ros::Time::now().toSec();
//                        auto goal_dist = compute_position_error(now_odom_msg->pose.pose, next_pose.pose);
//                        auto nav_dist = compute_position_error(now_odom_msg->pose.pose, start_odom_msg->pose.pose);
//
//                        if(goal_dist < _position_tolerance){
//                            ROS_INFO("GET ONE MORE STEP.");
//                            robotStop();
//                            break;
//                        }
//
//                        if(std::abs(now_time-start_time)>_time_tolerance){
//                            if(nav_dist < _position_tolerance){
//                                ROS_INFO("GET ONE MORE STEP FAILED.");
//                                robotStop();
//                                return false;
//                            }
//                            start_odom_msg = ros::topic::waitForMessage<nav_msgs::Odometry>(_odom_topic, *(_n.get()));
//                            start_time = ros::Time::now().toSec();
//                        }
//                    }
                    ROS_INFO("CHECKING WHETHER DWA PLANNER WORKS");
                    auto dwa_init_odom_msg = ros::topic::waitForMessage<nav_msgs::Odometry>(_odom_topic, *(_n.get()));
                    auto dwa_init_odom = *dwa_init_odom_msg;
                    makeplan_service.request.start.pose = dwa_init_odom.pose.pose;
                    _ac->waitForServer();
                    auto dwa_make_global_plan_success = makeplan_client.call(makeplan_service);
                    if(dwa_make_global_plan_success){
                        if(makeplan_service.response.plan.poses.empty()){
                            return false;
                        }else{
                            move_base_msgs::MoveBaseGoal goal;
                            goal.target_pose.header.stamp = ros::Time::now();
                            goal.target_pose.header.frame_id = _path_plan_frame;
                            goal.target_pose.pose = goal_pose;
                            _ac->sendGoal(goal);

                            auto dwa_global_path = ros::topic::waitForMessage<nav_msgs::Path>(_global_path_topic, ros::Duration(1.0));
                            auto dwa_local_path = ros::topic::waitForMessage<nav_msgs::Path>(_local_path_topic, ros::Duration(1.0));

                            if(dwa_global_path!=nullptr && dwa_local_path!=nullptr){
                                ROS_INFO("DWA PLANNER RECOVERED.");
                                _ac->cancelAllGoals();
                                return true;
                            }else{
                                _ac->cancelAllGoals();
                                continue;
                            }
                        }
                    }else{
                        return false;
                    }
                }
                return true;
            }
        }else{
            return false;
        }
    }
}

void StuckEscaper::shutDown() {
    _odom_subscriber.shutdown();
    _ac.get()->cancelAllGoals();
}