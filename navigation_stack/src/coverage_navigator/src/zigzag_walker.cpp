#include "coverage_navigator/zigzag_walker.h"

std::string zigzag_walker::PARAM_NAME_robot_size = "robot_size";
std::string zigzag_walker::PARAM_NAME_high_speed = "zigzag_walking/high_speed";
std::string zigzag_walker::PARAM_NAME_normal_speed = "zigzag_walking/normal_speed";
std::string zigzag_walker::PARAM_NAME_low_speed = "zigzag_walking/low_speed";
std::string zigzag_walker::PARAM_NAME_speed_increment = "zigzag_walking/speed_increment";
std::string zigzag_walker::PARAM_NAME_odom_topic = "zigzag_walking/odom_topic";
std::string zigzag_walker::PARAM_NAME_states_topic = "zigzag_walking/states_topic";
std::string zigzag_walker::PARAM_NAME_coverage_map_topic = "zigzag_walking/coverage_map_topic";
std::string zigzag_walker::PARAM_NAME_occupancy_map_topic = "zigzag_walking/occupancy_map_topic";
std::string zigzag_walker::PARAM_NAME_yaw_buffer = "zigzag_walking/yaw_buffer";
std::string zigzag_walker::PARAM_NAME_yaw_tolerance = "zigzag_walking/yaw_tolerance";
std::string zigzag_walker::PARAM_NAME_position_tolerance = "zigzag_walking/position_tolerance";

ZigzagWalker::ZigzagWalker() {}

ZigzagWalker::~ZigzagWalker() {}

void ZigzagWalker::setup(const cv::Mat& boundary_map) {
    
    _n.reset(new ros::NodeHandle("~"));
    
    _n->getParam(zigzag_walker::PARAM_NAME_robot_size, _robot_size);
    _n->getParam(zigzag_walker::PARAM_NAME_high_speed, _high_speed);
    _n->getParam(zigzag_walker::PARAM_NAME_normal_speed, _normal_speed);
    _n->getParam(zigzag_walker::PARAM_NAME_low_speed, _low_speed);
    _n->getParam(zigzag_walker::PARAM_NAME_speed_increment, _speed_increment);
    _n->getParam(zigzag_walker::PARAM_NAME_odom_topic, _odom_topic);
    _n->getParam(zigzag_walker::PARAM_NAME_states_topic, _states_topic);
    _n->getParam(zigzag_walker::PARAM_NAME_coverage_map_topic, _coverage_map_topic);
    _n->getParam(zigzag_walker::PARAM_NAME_occupancy_map_topic, _occupancy_map_topic);
    _n->getParam(zigzag_walker::PARAM_NAME_yaw_buffer, _yaw_buffer);
    _n->getParam(zigzag_walker::PARAM_NAME_yaw_tolerance, _yaw_tolerance);
    _n->getParam(zigzag_walker::PARAM_NAME_position_tolerance, _position_tolerance);

    _boundary_map = boundary_map.clone();

    _position_queue_size = 4;
    _near_obstacle = false;
    _collided = false;
    _is_turning = false;
    _is_interrupted = false;
    _finish_turning = false;
    _turning_num = 0;
    _make_first_turning = false;
    _make_second_turning = false;
    _in_refine_mode = false;
    _finish_zigzagging = false;
    _robot_turning_direction = RIGHT;
    _robot_rotating_direction = RIGHT;
    _speed_level = HIGH;
    _base_yaw = 0.0;
}

void ZigzagWalker::robotRotate(DIRECTION rotating_direction, const float& wheel_speed){
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

void ZigzagWalker::robotTurning(DIRECTION turning_direction, const float& wheel_speed){
    switch(turning_direction){
        case LEFT:
            robotMove(0,wheel_speed);
            break;
        case RIGHT:
            robotMove(wheel_speed,0);
            break;
        default:
            robotStop();
            break;
    }
}

void ZigzagWalker::robotRotate(DIRECTION rotating_direction, SPEED_LEVEL speed_level){
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

void ZigzagWalker::robotTurning(DIRECTION turning_direction, SPEED_LEVEL speed_level){
    switch(speed_level){
        case HIGH:
            robotTurning(turning_direction, _high_speed);
            break;
        case NORMAL:
            robotTurning(turning_direction, _normal_speed);
            break;
        case LOW:
            robotTurning(turning_direction, _low_speed);
            break;
        default:
            robotStop();
            break;
    }
}

bool ZigzagWalker::isVicinitiesCovered(const nav_msgs::Odometry& odom, int search_range_pix){

//    test
//    cv::Mat boundary_img = boundary_map.clone();
//    cv::cvtColor(boundary_img, boundary_img, cv::COLOR_GRAY2BGR);
//    cv::flip(boundary_img, boundary_img,1);
//    cv::rotate(boundary_img,boundary_img,cv::ROTATE_90_CLOCKWISE);
//    cv::imwrite(data_folder+"boundary_map.jpg",boundary_img);

    auto coverage_map_msg = ros::topic::waitForMessage<sensor_msgs::Image>(_coverage_map_topic, *(_n.get()));
    cv_bridge::CvImageConstPtr coverage_map_image = cv_bridge::toCvShare(coverage_map_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat coverage_map = coverage_map_image->image.clone();

    auto occupancy_map_msg = ros::topic::waitForMessage<sensor_msgs::Image>(_occupancy_map_topic, *(_n.get()));
    cv_bridge::CvImageConstPtr occupancy_map_image = cv_bridge::toCvShare(occupancy_map_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat occupancy_map = occupancy_map_image->image.clone();
    cv::cvtColor(occupancy_map,occupancy_map,cv::COLOR_BGR2GRAY);
    cv::threshold(occupancy_map, occupancy_map, 250, 255, cv::THRESH_BINARY);

    double wx = odom.pose.pose.position.x;
    double wy = odom.pose.pose.position.y;
    unsigned int mx, my;

    worldToMap(wx,wy,mx,my);

    auto robot_image_size = static_cast<unsigned int>(_robot_size/(getMapResolution()));
    ROS_INFO("VICINITIES SIZE: %d", robot_image_size);

    int search_range;
    if(search_range_pix==0){
        search_range = 2*robot_image_size;
    }else{
        search_range = search_range_pix;
    }

    bool uncleaned_position_found = false;
    for(size_t x = mx-search_range; x <= mx+search_range; ++x){
        for(size_t y = my-search_range; y <= my+search_range; ++y){
            if(x>=0 && x<coverage_map.cols && y>=0 && y<coverage_map.rows){
                if(coverage_map.at<cv::Vec3b>(cv::Point(x,y))!=COVERAGE
                   && occupancy_map.at<uchar>(cv::Point(x,y))==255
                   && _boundary_map.at<uchar>(cv::Point(mx,my))==255){
                    uncleaned_position_found = true;
                    break;
                }
            }
        }
    }

    ROS_INFO("CHECKED VICINITIES");
    return (!uncleaned_position_found);
}

void ZigzagWalker::run(bool in_refine_mode, DIRECTION turning_direction, float base_yaw){

    ROS_INFO("START ZIGZAGGING.");
    _finish_zigzagging = false;

    _is_turning = false;
    _is_interrupted = false;
    _finish_turning = false;
    _turning_num = 0;
    _make_first_turning = false;
    _make_second_turning = false;
    _in_refine_mode = in_refine_mode; //refine mode: 补扫

    if(_in_refine_mode){
        auto odom_msg = ros::topic::waitForMessage<nav_msgs::Odometry>(_odom_topic, *(_n.get()));
        auto odom = *odom_msg;
        if(isVicinitiesCovered(odom)){
            robotStop();
            ROS_INFO("NO NEW POSITION TO GO. FINISH ZIGZAGGING.");
        }
    }

    _recent_positions.clear();

    if(turning_direction == LEFT){
        _robot_turning_direction = LEFT;
        _robot_rotating_direction = LEFT;
    }else if(turning_direction == RIGHT){
        _robot_turning_direction = RIGHT;
        _robot_rotating_direction = RIGHT;
    }

    _base_yaw = base_yaw;
    // _base_yaw: [0, 360)
    if(_base_yaw<0){
        _base_yaw+=360.0;
    }
    ROS_INFO("ZIGZAG BASE YAW: %f",_base_yaw);

    _sync_state_sub.subscribe(*(_n.get()),_states_topic,10,ros::TransportHints().tcpNoDelay());
    _sync_odom_sub.subscribe(*(_n.get()),_odom_topic,10,ros::TransportHints().tcpNoDelay());
    _state_odom_sync.reset(new message_filters::Synchronizer<syncStateOdomPolicy>(syncStateOdomPolicy(10), _sync_state_sub, _sync_odom_sub));
    _state_odom_sync->registerCallback(boost::bind(&ZigzagWalker::syncStateOdomCallback, this, _1, _2));

    ros::Duration interval(0.001);
    while (ros::ok()) {
        if(_finish_zigzagging){
            break;
        }
        ros::spinOnce();
        interval.sleep();
    }
}

void ZigzagWalker::shutDown() {
    _sync_state_sub.unsubscribe();
    _sync_odom_sub.unsubscribe();
}

void ZigzagWalker::updateRecentPositions(){
    if(_recent_positions.size()>_position_queue_size){
        _recent_positions.pop_front();
    }
}

bool ZigzagWalker::switchTuningDirection(){

    if(_robot_turning_direction == LEFT){
        _robot_turning_direction = RIGHT;
        _robot_rotating_direction = RIGHT;
    }else if(_robot_turning_direction == RIGHT){
        _robot_turning_direction = LEFT;
        _robot_rotating_direction = LEFT;
    }else{
        return false;
    }
    return true;
}

void ZigzagWalker::updateBaseYaw(){
    // _base_yaw: [0, 360)
    _base_yaw += 180.0;
    if(_base_yaw >= 360.0){
        _base_yaw -= 360.0;
    }
    ROS_INFO("BASE_YAW: %f",_base_yaw);
}

void ZigzagWalker::syncStateOdomCallback(const custom_msgs::DetectionState::ConstPtr& states_msg, const nav_msgs::Odometry::ConstPtr& odom_msg){

    auto odom = *odom_msg;
    auto states = *states_msg;

    if(states.heading==states.NEAR_OBSTACLE || states.heading_left==states.NEAR_OBSTACLE || states.heading_right==states.NEAR_OBSTACLE){
        _near_obstacle = true;
    }else{
        _near_obstacle = false;
    }

    if(states.heading==states.COLLIDED || states.heading_left==states.COLLIDED || states.heading_right==states.COLLIDED){
        _collided = true;
    }else{
        if(states.left==states.COLLIDED && states.right==states.COLLIDED){
            _collided = true;
        }else{
            _collided = false;
        }
    }

    if(!_in_refine_mode){
        if(!_make_first_turning){
            ROS_INFO("EXECUTE FIRST TURNING");
            _near_obstacle = true;
            _collided = false;
            _is_turning = false;
            _make_first_turning = true;
        }
        if(!_make_second_turning){
            if( (_near_obstacle||_collided) && _turning_num>0){
                ROS_INFO("EXECUTE SECOND TURNING");
                _near_obstacle = false;
                _collided = true;
                _is_turning = false;
                _make_second_turning = true;
            }
        }
    }

    if(!_is_turning){
        if(_near_obstacle){
            ROS_INFO("NEAR OBSTACLE!!");
            robotStop();
            _is_turning = true;
            updateBaseYaw();
            switchTuningDirection();

            if(_in_refine_mode){
                if(isVicinitiesCovered(odom)){
                    robotStop();
                    ROS_INFO("NO NEW POSITION TO GO. FINISH ZIGZAGGING.");
                    _finish_zigzagging = true;
                    return;
                }
            }
            geometry_msgs::Pose turning_position = odom.pose.pose;
            bool in_old_position = false;
            for(auto position : _recent_positions){
                if(compute_position_error(turning_position, position)<_position_tolerance){
                    in_old_position = true;
                    break;
                }
            }
            if(in_old_position){
                ROS_INFO("NO NEW POSITION TO GO. FINISH ZIGZAGGING.");
                _finish_zigzagging = true;
                return;
            }else{
                _recent_positions.emplace_back(turning_position);
                updateRecentPositions();
            }

        }else if(_collided){
            ROS_INFO("COLLIDED!!");
            robotStop();
            _is_turning = true;
            updateBaseYaw();
            switchTuningDirection();
            _is_interrupted = true;

            if(_in_refine_mode){
                if(isVicinitiesCovered(odom)){
                    robotStop();
                    ROS_INFO("NO NEW POSITION TO GO. FINISH ZIGZAGGING.");
                    _finish_zigzagging = true;
                    return;
                }
            }
            geometry_msgs::Pose turning_position = odom.pose.pose;
            bool in_old_position = false;
            for(auto position : _recent_positions){
                if(compute_position_error(turning_position, position)<_position_tolerance){
                    in_old_position = true;
                    break;
                }
            }
            if(in_old_position){
                ROS_INFO("NO NEW POSITION TO GO. FINISH ZIGZAGGING.");
                _finish_zigzagging = true;
                return;
            }else{
                _recent_positions.emplace_back(turning_position);
                updateRecentPositions();
            }

        }else{
            float yaw = tf::getYaw(odom.pose.pose.orientation)/M_PI*180.0;
            // yaw: [0, 360)
            if(yaw<0){
                yaw+=360.0;
            }
            float yaw_offset = compute_relative_yaw(_base_yaw, yaw);

            if(std::abs(yaw_offset)<_yaw_tolerance){
                if(yaw_offset<0){
//                    ROS_INFO("HEADING RIGHT, SHEER LEFT.");
                    robotMove(_high_speed,(_high_speed+_speed_increment));
                }else if(yaw_offset>0){
//                    ROS_INFO("HEADING LEFT, SHEER RIGHT.");
                    robotMove((_high_speed+_speed_increment),_high_speed);
                } else{
                    robotMove(_high_speed,_high_speed);
                }
            }else{
                _is_turning = true;
                _finish_turning = false;
                _is_interrupted = true;
                return;
            }
        }
    }else{
        float yaw = tf::getYaw(odom.pose.pose.orientation)/M_PI*180.0;
        // yaw: [0, 360)
        if(yaw<0){
            yaw+=360.0;
        }
        float yaw_offset = compute_relative_yaw(_base_yaw, yaw);
//        ROS_INFO("CURR YAW: %f,    BASELINE YAW: %f,    YAW OFFSET: %f", yaw, _base_yaw, yaw_offset);

        if(std::abs(yaw_offset)<_yaw_buffer){
            _speed_level=NORMAL;
        }

        if((std::abs(yaw_offset) < _yaw_tolerance)){
            _finish_turning = true;
            robotStop();
        }else if(_robot_rotating_direction == LEFT && yaw_offset > 0 && yaw_offset < _yaw_buffer){
            robotStop();
            _speed_level=LOW;
            _robot_rotating_direction = RIGHT;

        }else if(_robot_rotating_direction == RIGHT && yaw_offset < 0 && yaw_offset > -_yaw_buffer){
            robotStop();
            _speed_level=LOW;
            _robot_rotating_direction = LEFT;
        }

        if(!_is_interrupted){
            if(_collided){
                ROS_INFO("COLLIDED!!");
                robotStop();
                _is_interrupted = true;

                if(_in_refine_mode){
                    if(isVicinitiesCovered(odom)){
                        robotStop();
                        ROS_INFO("NO NEW POSITION TO GO. FINISH ZIGZAGGING.");
                        _finish_zigzagging = true;
                        return;
                    }
                }
                geometry_msgs::Pose turning_position = odom.pose.pose;
                bool in_old_position = false;
                for(auto position : _recent_positions){
                    if(compute_position_error(turning_position, position)<_position_tolerance){
                        in_old_position = true;
                        break;
                    }
                }
                if(in_old_position){
                    ROS_INFO("NO NEW POSITION TO GO. FINISH ZIGZAGGING.");
                    _finish_zigzagging = true;
                    return;
                }else{
                    _recent_positions.emplace_back(turning_position);
                    updateRecentPositions();
                }

            }else{
                if(_finish_turning){
                    robotStop();
                    _speed_level=HIGH;
                    _finish_turning = false;
                    _is_turning = false;
                    _turning_num++;
                }else{
                    robotTurning(_robot_turning_direction, _speed_level);
                }
            }
        }else{
            if(_finish_turning){
                robotStop();
                _speed_level=HIGH;
                _finish_turning = false;
                _is_turning = false;
                _turning_num++;
                _is_interrupted = false;
            }else{
                robotRotate(_robot_rotating_direction, _speed_level);
            }
        }
    }
}