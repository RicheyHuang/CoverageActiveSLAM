#include "virtual_layer_processor/virtual_layer_processor.h"

std::string PARAM_NAME_robot_size="robot_size";
std::string PARAM_NAME_collided_range="collided_range";
std::string PARAM_NAME_obstacle_detected_range="obstacle_detected_range";
std::string PARAM_NAME_along_wall_range="along_wall_range";
std::string PARAM_NAME_free_space_range="free_space_range";
std::string PARAM_NAME_unilateral_detect_angle="unilateral_detect_angle";
std::string PARAM_NAME_beam_size="beam_size";
std::string PARAM_NAME_data_source="data_source";

std::string PARAM_NAME_raw_states_topic="raw_states_topic";
std::string PARAM_NAME_raw_beams_topic="raw_beams_topic";
std::string PARAM_NAME_processed_states_topic="processed_states_topic";
std::string PARAM_NAME_processed_beams_topic="processed_beams_topic";
std::string PARAM_NAME_odom_topic="odom_topic";
std::string PARAM_NAME_zone_topic="zone_topic";
std::string PARAM_NAME_obstacles_topic="obstacles_topic";
std::string PARAM_NAME_make_zone_service="make_zone_service";
std::string PARAM_NAME_make_obstacles_service="make_obstacles_service";

VirtualLayerProcessor::VirtualLayerProcessor() {
    
    _n.reset(new ros::NodeHandle("~"));

    _n->getParam(PARAM_NAME_robot_size, _robot_size);
    _n->getParam(PARAM_NAME_collided_range, _collided_range);
    _n->getParam(PARAM_NAME_obstacle_detected_range, _obstacle_detected_range);
    _n->getParam(PARAM_NAME_along_wall_range, _along_wall_range);
    _n->getParam(PARAM_NAME_free_space_range, _free_space_range);
    _n->getParam(PARAM_NAME_unilateral_detect_angle, _unilateral_detect_angle);
    _n->getParam(PARAM_NAME_beam_size, _beam_size);
    _n->getParam(PARAM_NAME_data_source, _data_source_type);
    _n->getParam(PARAM_NAME_raw_states_topic, _raw_states_topic);
    _n->getParam(PARAM_NAME_processed_states_topic, _processed_states_topic);
    _n->getParam(PARAM_NAME_odom_topic, _odom_topic);
    _n->getParam(PARAM_NAME_zone_topic, _zone_topic);
    _n->getParam(PARAM_NAME_obstacles_topic, _obstacles_topic);
    _n->getParam(PARAM_NAME_make_zone_service, _make_zone_service);
    _n->getParam(PARAM_NAME_make_obstacles_service, _make_obstacles_service);

    _data_source = DATA_SOURCE(_data_source_type);

    _bilateral_detect_angles[0] = _unilateral_detect_angle;
    _bilateral_detect_angles[1] = 360-_unilateral_detect_angle;

    _states_publisher = _n->advertise<custom_msgs::DetectionState>(_processed_states_topic, 10);

    _zone_subscriber = _n->subscribe(_zone_topic,10,&VirtualLayerProcessor::zoneCallback, this);
    _obstacles_subscriber = _n->subscribe(_obstacles_topic,10,&VirtualLayerProcessor::obstaclesCallback,this);
    _odom_subscriber = _n->subscribe(_odom_topic,10,&VirtualLayerProcessor::odomCallback,this);
    _states_subscriber = _n->subscribe(_raw_states_topic,10,&VirtualLayerProcessor::statesCallback, this);

    bool is_latched = true;

    _zone_maker_server = _n->advertiseService<custom_srvs::ZoneRequest, custom_srvs::ZoneResponse>(_make_zone_service, boost::bind(&VirtualLayerProcessor::makeZone, this, _1, _2));
    _zone_publisher = _n->advertise<custom_msgs::Zone>(_zone_topic, 1, is_latched);

    _obstacles_maker_server = _n->advertiseService<custom_srvs::ObstacleRequest, custom_srvs::ObstacleResponse>(_make_obstacles_service, boost::bind(&VirtualLayerProcessor::makeObstacles, this, _1, _2));
    _obstacles_publisher = _n->advertise<custom_msgs::Obstacles>(_obstacles_topic, 1, is_latched);

//    ros::AsyncSpinner spinner(5);
//    spinner.start();
//    ros::waitForShutdown();

    ros::Duration interval(0.001);
    while (ros::ok()) {
        ros::spinOnce();
        interval.sleep();
    }
}

VirtualLayerProcessor::~VirtualLayerProcessor() {}

// zone里面是世界点
void VirtualLayerProcessor::zoneCallback(const custom_msgs::Zone::ConstPtr& zone_msg){
    boost::unique_lock<boost::recursive_mutex> lock(_zone_mutex);
    _zone = *zone_msg;
    _virtual_walls.clear();

    for(const auto& point : _zone.area.form){
        _virtual_walls.push_back(CGAL::Point_2<K>(point.x, point.y));
    }
    lock.unlock();
}

// obstacles里面是世界点
void VirtualLayerProcessor::obstaclesCallback(const custom_msgs::Obstacles::ConstPtr& obstacles_msg){
    boost::unique_lock<boost::recursive_mutex> lock(_obstacles_mutex);
    _obstacles = *obstacles_msg;
    _virtual_obstacles.clear();

    for(const auto& obstacle:_obstacles.list){
        CGAL::Polygon_2<K> virtual_obstacle;
        for(const auto& point:obstacle.form){
            virtual_obstacle.push_back(CGAL::Point_2<K>(point.x, point.y));
        }
        _virtual_obstacles.push_back(virtual_obstacle);
    }
    lock.unlock();
}

void VirtualLayerProcessor::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg){
    boost::unique_lock<boost::recursive_mutex> lock(_odom_mutex);

    auto odom = *odom_msg;

    float heading_range = FLT_MAX;
    float heading_left_range = FLT_MAX;
    float heading_right_range = FLT_MAX;
    float left_range = FLT_MAX;
    float right_range = FLT_MAX;

    std::vector<std::pair<int,float>> degree2ranges;
    degree2ranges.emplace_back(std::make_pair(0,heading_range));
    degree2ranges.emplace_back(std::make_pair(_bilateral_detect_angles[0],heading_left_range));
    degree2ranges.emplace_back(std::make_pair(_bilateral_detect_angles[1],heading_right_range));
    degree2ranges.emplace_back(std::make_pair(90,left_range));
    degree2ranges.emplace_back(std::make_pair(270,right_range));

    for(auto& degree2range:degree2ranges){
        int deg = degree2range.first;
        float range = degree2range.second;

        double robot_yaw = tf::getYaw(odom.pose.pose.orientation);
        double laser_yaw = robot_yaw + double(deg)/180.0*M_PI;

        _ray = CGAL::Ray_2<K>(CGAL::Point_2<K>(odom.pose.pose.position.x,odom.pose.pose.position.y), CGAL::Direction_2<K>(cos(laser_yaw),sin(laser_yaw)));

        double min_virtual_range = range;
        for(auto virtual_wall = _virtual_walls.edges_begin(); virtual_wall != _virtual_walls.edges_end(); ++virtual_wall){
            auto hit_point = CGAL::intersection(_ray, *virtual_wall);
            if(hit_point){
                if(auto p = boost::get<CGAL::Point_2<K>>(&*hit_point)){
                    CGAL::Segment_2<K> ray_path(CGAL::Point_2<K>(_ray.source()), CGAL::Point_2<K>(*p));
                    double virtual_range = sqrt(CGAL::to_double(ray_path.squared_length()));
                    if(virtual_range < min_virtual_range){
                        min_virtual_range = virtual_range;
                    }
                }
            }
        }

        for(auto& virtual_obstacle:_virtual_obstacles){
            for(auto virtual_obstacle_edge = virtual_obstacle.edges_begin(); virtual_obstacle_edge != virtual_obstacle.edges_end(); ++virtual_obstacle_edge){
                auto hit_point = CGAL::intersection(_ray, *virtual_obstacle_edge);
                if(hit_point){
                    if(auto p = boost::get<CGAL::Point_2<K>>(&*hit_point)){
                        CGAL::Segment_2<K> ray_path(CGAL::Point_2<K>(_ray.source()), CGAL::Point_2<K>(*p));
                        double virtual_range = sqrt(CGAL::to_double(ray_path.squared_length()));
                        if(virtual_range < min_virtual_range){
                            min_virtual_range = virtual_range;
                        }
                    }
                }
            }
        }

        if(deg==0){
            _virtual_states.heading_range = min_virtual_range;
        }else if(deg==_bilateral_detect_angles[0]){
            _virtual_states.heading_left_range = min_virtual_range;
        }else if(deg==_bilateral_detect_angles[1]){
            _virtual_states.heading_right_range = min_virtual_range;
        }else if(deg==90){
            _virtual_states.left_range = min_virtual_range;
        }else if(deg==270){
            _virtual_states.right_range = min_virtual_range;
        }
    }

//    ROS_INFO("VIRTUAL BEAMS: [HEAD:%f] [HEAD-LEFT:%f] [HEAD-RIGHT:%f] [LEFT:%f] [RIGHT:%f]\n", _virtual_beams.heading_range, _virtual_beams.heading_left_range, _virtual_beams.heading_right_range, _virtual_beams.left_range, _virtual_beams.right_range);

    if(_virtual_states.heading_range<_collided_range){
        _virtual_states.heading = _virtual_states.COLLIDED;
    }else if(_virtual_states.heading_range<_obstacle_detected_range){
        _virtual_states.heading = _virtual_states.NEAR_OBSTACLE;
    }else{
        _virtual_states.heading = _virtual_states.NO_OBSTACLE;
    }
    if(_virtual_states.heading_left_range<_collided_range){
        _virtual_states.heading_left = _virtual_states.COLLIDED;
    }else if(_virtual_states.heading_left_range<_obstacle_detected_range){
        _virtual_states.heading_left = _virtual_states.NEAR_OBSTACLE;
    }else{
        _virtual_states.heading_left = _virtual_states.NO_OBSTACLE;
    }
    if(_virtual_states.heading_right_range<_collided_range){
        _virtual_states.heading_right = _virtual_states.COLLIDED;
    }else if(_virtual_states.heading_right_range<_obstacle_detected_range){
        _virtual_states.heading_right = _virtual_states.NEAR_OBSTACLE;
    }else{
        _virtual_states.heading_right = _virtual_states.NO_OBSTACLE;
    }

    if(_virtual_states.left_range<_along_wall_range){
        _virtual_states.left = _virtual_states.ALONG_WALL;
    }else if(_virtual_states.left_range<_free_space_range){
        _virtual_states.left = _virtual_states.AWAY_FROM_WALL;
    }else{
        _virtual_states.left = _virtual_states.NO_WALL;
    }
    if(_virtual_states.right_range<_along_wall_range){
        _virtual_states.right = _virtual_states.ALONG_WALL;
    }else if(_virtual_states.right_range<_free_space_range){
        _virtual_states.right = _virtual_states.AWAY_FROM_WALL;
    }else{
        _virtual_states.right = _virtual_states.NO_WALL;
    }

    lock.unlock();
}

void VirtualLayerProcessor::statesCallback(const custom_msgs::DetectionState::ConstPtr &states_msg) {
    boost::unique_lock<boost::recursive_mutex> lock(_states_mutex);
    auto sensor_states = *states_msg;
    switch (_data_source){
        case SENSOR_INFO:
        {
            _states_publisher.publish(sensor_states);
            break;
        }
        case VIRTUAL_INFO:
        {
            _states_publisher.publish(_virtual_states);
            break;
        }
        case MERGED_INFO:
        {
            custom_msgs::DetectionState merged_states;
            merged_states.header.frame_id=sensor_states.header.frame_id;
            merged_states.header.stamp=sensor_states.header.stamp;

            merged_states.heading_range = std::min(sensor_states.heading_range, _virtual_states.heading_range);
            merged_states.heading_left_range = std::min(sensor_states.heading_left_range, _virtual_states.heading_left_range);
            merged_states.heading_right_range = std::min(sensor_states.heading_right_range, _virtual_states.heading_right_range);
            merged_states.left_range = std::min(sensor_states.left_range, _virtual_states.left_range);
            merged_states.right_range = std::min(sensor_states.right_range, _virtual_states.right_range);

            if(sensor_states.heading==sensor_states.COLLIDED || _virtual_states.heading==_virtual_states.COLLIDED){
                merged_states.heading = merged_states.COLLIDED;
            }else if(sensor_states.heading==sensor_states.NEAR_OBSTACLE || _virtual_states.heading==_virtual_states.NEAR_OBSTACLE){
                merged_states.heading = merged_states.NEAR_OBSTACLE;
            }else{
                merged_states.heading = merged_states.NO_OBSTACLE;
            }

            if(sensor_states.heading_left==sensor_states.COLLIDED || _virtual_states.heading_left==_virtual_states.COLLIDED){
                merged_states.heading_left = merged_states.COLLIDED;
            }else if(sensor_states.heading_left==sensor_states.NEAR_OBSTACLE || _virtual_states.heading_left==_virtual_states.NEAR_OBSTACLE){
                merged_states.heading_left = merged_states.NEAR_OBSTACLE;
            }else{
                merged_states.heading_left = merged_states.NO_OBSTACLE;
            }

            if(sensor_states.heading_right==sensor_states.COLLIDED || _virtual_states.heading_right==_virtual_states.COLLIDED){
                merged_states.heading_right = merged_states.COLLIDED;
            }else if(sensor_states.heading_right==sensor_states.NEAR_OBSTACLE || _virtual_states.heading_right==_virtual_states.NEAR_OBSTACLE){
                merged_states.heading_right = merged_states.NEAR_OBSTACLE;
            }else{
                merged_states.heading_right = merged_states.NO_OBSTACLE;
            }

            if(sensor_states.left==sensor_states.ALONG_WALL || _virtual_states.left==_virtual_states.ALONG_WALL){
                merged_states.left = merged_states.ALONG_WALL;
            }else if(sensor_states.left==sensor_states.AWAY_FROM_WALL || _virtual_states.left==_virtual_states.AWAY_FROM_WALL){
                merged_states.left = merged_states.AWAY_FROM_WALL;
            }else{
                merged_states.left = merged_states.NO_WALL;
            }

            if(sensor_states.right==sensor_states.ALONG_WALL || _virtual_states.right==_virtual_states.ALONG_WALL){
                merged_states.right = merged_states.ALONG_WALL;
            }else if(sensor_states.right==sensor_states.AWAY_FROM_WALL || _virtual_states.right==_virtual_states.AWAY_FROM_WALL){
                merged_states.right = merged_states.AWAY_FROM_WALL;
            }else{
                merged_states.right = merged_states.NO_WALL;
            }

            _states_publisher.publish(merged_states);
            break;
        }
        default:
            break;
    }
    lock.unlock();
}

bool VirtualLayerProcessor::makeZone(custom_srvs::ZoneRequest& req, custom_srvs::ZoneResponse& res){
    ROS_INFO("UPDATE VIABLE ZONE");
    _zone.area = req.area;
    _zone_publisher.publish(_zone); // latching-mode下发布一次即可
}

bool VirtualLayerProcessor::makeObstacles(custom_srvs::ObstacleRequest& req, custom_srvs::ObstacleResponse& res){
    _obstacles.list = req.list;
    _obstacles_publisher.publish(_obstacles);
}
