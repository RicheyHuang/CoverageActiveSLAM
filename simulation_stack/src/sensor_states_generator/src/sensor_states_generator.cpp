#include "sensor_states_generator/sensor_states_generator.h"

std::string PARAM_NAME_robot_radius="robot_radius";
std::string PARAM_NAME_robot_size="robot_size";
std::string PARAM_NAME_collided_range="collided_range";
std::string PARAM_NAME_obstacle_detected_range="obstacle_detected_range";
std::string PARAM_NAME_along_wall_range="along_wall_range";
std::string PARAM_NAME_free_space_range="free_space_range";
std::string PARAM_NAME_unilateral_detect_angle="unilateral_detect_angle";
std::string PARAM_NAME_beam_size="beam_size";
std::string PARAM_NAME_states_topic="states_topic";
std::string PARAM_NAME_scan_topic="scan_topic";
std::string PARAM_NAME_bumper_topic="bumper_topic";


SensorStatesGenerator::SensorStatesGenerator() {
    _n.reset(new ros::NodeHandle("~"));
    
    _n->getParam(PARAM_NAME_robot_radius,_robot_radius);
    _n->getParam(PARAM_NAME_robot_size,_robot_size);
    _n->getParam(PARAM_NAME_collided_range,_collided_range);
    _n->getParam(PARAM_NAME_obstacle_detected_range,_obstacle_detected_range);
    _n->getParam(PARAM_NAME_along_wall_range,_along_wall_range);
    _n->getParam(PARAM_NAME_free_space_range,_free_space_range);
    _n->getParam(PARAM_NAME_unilateral_detect_angle,_unilateral_detect_angle);
    _n->getParam(PARAM_NAME_beam_size,_beam_size);
    _n->getParam(PARAM_NAME_states_topic,_states_topic);
    _n->getParam(PARAM_NAME_scan_topic,_scan_topic);
    _n->getParam(PARAM_NAME_bumper_topic,_bumper_topic);

    _bilateral_detect_angles[0] = _unilateral_detect_angle;
    _bilateral_detect_angles[1] = 360-_unilateral_detect_angle;

    _states_publisher = _n->advertise<custom_msgs::DetectionState>(_states_topic, 1);
    _bumper_subscriber = _n->subscribe(_bumper_topic, 1, &SensorStatesGenerator::bumperCallback, this);
    _scan_subscriber = _n->subscribe(_scan_topic, 1, &SensorStatesGenerator::scanCallback, this);

    ros::Duration interval(0.001);
    while (ros::ok()) {
        ros::spinOnce();
        interval.sleep();
    }
}

SensorStatesGenerator::~SensorStatesGenerator() {}

void SensorStatesGenerator::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg){
    boost::unique_lock<boost::recursive_mutex> lock(_scan_mutex);

    custom_msgs::DetectionState detect_states;
    detect_states.header = scan_msg->header;

    float heading_range = FLT_MAX;
    float heading_left_range = FLT_MAX;
    float heading_right_range = FLT_MAX;
    float left_range = FLT_MAX;
    float right_range = FLT_MAX;
    std::vector<float> heading_ranges;
    std::vector<float> heading_left_ranges;
    std::vector<float> heading_right_ranges;
    std::vector<float> left_ranges;
    std::vector<float> right_ranges;

    if(!scan_msg->ranges.empty()){

        auto scan = *scan_msg;
        std::vector<float> laser_ranges;
        laser_ranges = scan.ranges;

        if(_unilateral_detect_angle<=0 || _unilateral_detect_angle>=90){
            ROS_INFO("UNILATERAL DETECTING ANGLE SHOULD LIES FROM (0, 90)! \n");
            return;
        }

        if(_beam_size>=_unilateral_detect_angle){
            _beam_size = _unilateral_detect_angle-1;
        }else if(_beam_size<0){
            _beam_size = 0;
        }

        /* 激光雷达为逆时针旋转，范围360度:0-360度，分辨率是1度，正前方是0度，range的第一个元素就是正前方的距离值 */
        /* ranges有360个range值 */

        // 0度为正前方
        heading_ranges.clear();
        if(_beam_size == 0){
            if(std::isnan(laser_ranges[0])){
                heading_range = FLT_MAX;
            }else{
                heading_range = laser_ranges[0];
            }
        }else{
            if(!std::isnan(laser_ranges[0])){
                heading_ranges.emplace_back(laser_ranges[0]);
            }
            for(size_t i = 1; i <= _beam_size; ++i){
                if(!std::isnan(laser_ranges[0+i])){
                    heading_ranges.emplace_back(laser_ranges[0+i]);
                }
                if(!std::isnan(laser_ranges[360-i])){
                    heading_ranges.emplace_back(laser_ranges[360-i]);
                }
            }
            if(!heading_ranges.empty()){
                heading_range = std::accumulate(heading_ranges.begin(), heading_ranges.end(), 0.0)/float(heading_ranges.size());
            }else{
                heading_range = FLT_MAX;
            }
        }

        // 左前方
        heading_left_ranges.clear();
        if(_beam_size == 0){
            if(std::isnan(laser_ranges[_bilateral_detect_angles[0]])){
                heading_left_range = FLT_MAX;
            }else{
                heading_left_range = laser_ranges[_bilateral_detect_angles[0]];
            }
        }else{
            if(!std::isnan(laser_ranges[_bilateral_detect_angles[0]])){
                heading_left_ranges.emplace_back(laser_ranges[_bilateral_detect_angles[0]]);
            }
            for(size_t i = 1; i <= _beam_size; ++i){
                if(!std::isnan(laser_ranges[_bilateral_detect_angles[0]+i])){
                    heading_left_ranges.emplace_back(laser_ranges[_bilateral_detect_angles[0]+i]);
                }
                if(!std::isnan(laser_ranges[_bilateral_detect_angles[0]-i])){
                    heading_left_ranges.emplace_back(laser_ranges[_bilateral_detect_angles[0]-i]);
                }
            }
            if(!heading_left_ranges.empty()){
                heading_left_range = std::accumulate(heading_left_ranges.begin(), heading_left_ranges.end(), 0.0)/float(heading_left_ranges.size());
            }else{
                heading_left_range = FLT_MAX;
            }
        }

        // 右前方
        heading_right_ranges.clear();
        if(_beam_size == 0){
            if(std::isnan(laser_ranges[_bilateral_detect_angles[1]])){
                heading_right_range = FLT_MAX;
            }else{
                heading_right_range = laser_ranges[_bilateral_detect_angles[1]];
            }
        }else{
            if(!std::isnan(laser_ranges[_bilateral_detect_angles[1]])){
                heading_right_ranges.emplace_back(laser_ranges[_bilateral_detect_angles[1]]);
            }
            for(size_t i = 1; i <= _beam_size; ++i){
                if(!std::isnan(laser_ranges[_bilateral_detect_angles[1]+i])){
                    heading_right_ranges.emplace_back(laser_ranges[_bilateral_detect_angles[1]+i]);
                }
                if(!std::isnan(laser_ranges[_bilateral_detect_angles[1]-i])){
                    heading_right_ranges.emplace_back(laser_ranges[_bilateral_detect_angles[1]-i]);
                }
            }
            if(!heading_right_ranges.empty()){
                heading_right_range = std::accumulate(heading_right_ranges.begin(), heading_right_ranges.end(), 0.0)/float(heading_right_ranges.size());
            }else{
                heading_right_range = FLT_MAX;
            }
        }

        // 90度为正左侧
        left_ranges.clear();
        if(_beam_size == 0){
            if(std::isnan(laser_ranges[90])){
                left_range = FLT_MAX;
            }else{
                left_range = laser_ranges[90];
            }
        }else{
            if(!std::isnan(laser_ranges[90])){
                left_ranges.emplace_back(laser_ranges[90]);
            }
            for(size_t i = 1; i <= _beam_size; ++i){
                if(!std::isnan(laser_ranges[90+i])){
                    left_ranges.emplace_back(laser_ranges[90+i]);
                }
                if(!std::isnan(laser_ranges[90-i])){
                    left_ranges.emplace_back(laser_ranges[90-i]);
                }
            }
            if(!left_ranges.empty()){
                left_range = std::accumulate(left_ranges.begin(), left_ranges.end(), 0.0)/float(left_ranges.size());
            }else{
                left_range = FLT_MAX;
            }
        }

        // 270度为正右侧
        right_ranges.clear();
        if(_beam_size == 0){
            if(std::isnan(laser_ranges[270])){
                right_range = FLT_MAX;
            }else{
                right_range = laser_ranges[270];
            }
        }else{
            if(!std::isnan(laser_ranges[270])){
                right_ranges.emplace_back(laser_ranges[270]);
            }
            for(size_t i = 1; i <= _beam_size; ++i){
                if(!std::isnan(laser_ranges[270+i])){
                    right_ranges.emplace_back(laser_ranges[270+i]);
                }
                if(!std::isnan(laser_ranges[270-i])){
                    right_ranges.emplace_back(laser_ranges[270-i]);
                }
            }
            if(!right_ranges.empty()){
                right_range = std::accumulate(right_ranges.begin(), right_ranges.end(), 0.0)/float(right_ranges.size());
            }else{
                right_range = FLT_MAX;
            }
        }

    }

    detect_states.heading_range = heading_range;
    detect_states.heading_left_range = heading_left_range;
    detect_states.heading_right_range = heading_right_range;
    detect_states.left_range = left_range;
    detect_states.right_range = right_range;

    if(detect_states.heading_range<_collided_range){
        detect_states.heading = detect_states.COLLIDED;
    }else if(detect_states.heading_range<_obstacle_detected_range){
        detect_states.heading = detect_states.NEAR_OBSTACLE;
    }else{
        detect_states.heading = detect_states.NO_OBSTACLE;
    }
    if(detect_states.heading_left_range<_collided_range){
        detect_states.heading_left = detect_states.COLLIDED;
    }else if(detect_states.heading_left_range<_obstacle_detected_range){
        detect_states.heading_left = detect_states.NEAR_OBSTACLE;
    }else{
        detect_states.heading_left = detect_states.NO_OBSTACLE;
    }
    if(detect_states.heading_right_range<_collided_range){
        detect_states.heading_right = detect_states.COLLIDED;
    }else if(detect_states.heading_right_range<_obstacle_detected_range){
        detect_states.heading_right = detect_states.NEAR_OBSTACLE;
    }else{
        detect_states.heading_right = detect_states.NO_OBSTACLE;
    }

    if(detect_states.left_range<_along_wall_range){
        detect_states.left = detect_states.ALONG_WALL;
    }else if(detect_states.left_range<_free_space_range){
        detect_states.left = detect_states.AWAY_FROM_WALL;
    }else{
        detect_states.left = detect_states.NO_WALL;
    }
    if(detect_states.right_range<_along_wall_range){
        detect_states.right = detect_states.ALONG_WALL;
    }else if(detect_states.right_range<_free_space_range){
        detect_states.right = detect_states.AWAY_FROM_WALL;
    }else{
        detect_states.right = detect_states.NO_WALL;
    }

    if(!_bumper_state.states.empty()){
        detect_states.heading = detect_states.COLLIDED;
        detect_states.heading_left = detect_states.COLLIDED;
        detect_states.heading_right = detect_states.COLLIDED;
    }
    _states_publisher.publish(detect_states);

    lock.unlock();
}

void SensorStatesGenerator::bumperCallback(const gazebo_msgs::ContactsState::ConstPtr &bumper_msg){
    boost::unique_lock<boost::recursive_mutex> lock(_bumper_mutex);
    _bumper_state = *bumper_msg;
    lock.unlock();
}