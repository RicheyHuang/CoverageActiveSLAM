#include "coverage_mapper/coverage_mapper.h"

std::string PARAM_NAME_robot_radius="robot_radius";
std::string PARAM_NAME_world_min_x="world_min_x";
std::string PARAM_NAME_world_max_x="world_max_x";
std::string PARAM_NAME_world_min_y="world_min_y";
std::string PARAM_NAME_world_max_y="world_max_y";
std::string PARAM_NAME_map_resolution="map_resolution";

std::string PARAM_NAME_map_topic="map_topic";
std::string PARAM_NAME_occupancy_map_topic="occupancy_map_topic";
std::string PARAM_NAME_odom_topic="odom_topic";
std::string PARAM_NAME_trajectory_topic="trajectory_topic";
std::string PARAM_NAME_coverage_map_topic="coverage_map_topic";
std::string PARAM_NAME_coverage_image_topic="coverage_image_topic";

std::string PARAM_NAME_trajectory_frame="trajectory_frame";
std::string PARAM_NAME_map_frame="map_frame";

CoverageMapper::CoverageMapper() {

    _n.reset(new ros::NodeHandle("~"));

    _n->getParam(PARAM_NAME_robot_radius,_robot_radius);
    _n->getParam(PARAM_NAME_world_min_x,_world_min_x);
    _n->getParam(PARAM_NAME_world_max_x,_world_max_x);
    _n->getParam(PARAM_NAME_world_min_y,_world_min_y);
    _n->getParam(PARAM_NAME_world_max_y,_world_max_y);
    _n->getParam(PARAM_NAME_map_resolution,_map_resolution);
    _n->getParam(PARAM_NAME_map_topic,_map_topic);
    _n->getParam(PARAM_NAME_occupancy_map_topic,_occupancy_map_topic);
    _n->getParam(PARAM_NAME_odom_topic,_odom_topic);
    _n->getParam(PARAM_NAME_trajectory_topic,_trajectory_topic);
    _n->getParam(PARAM_NAME_coverage_map_topic,_coverage_map_topic);
    _n->getParam(PARAM_NAME_coverage_image_topic,_coverage_image_topic);
    _n->getParam(PARAM_NAME_trajectory_frame,_trajectory_frame);
    _n->getParam(PARAM_NAME_map_frame,_map_frame);

    _map_width = static_cast<unsigned int>((_world_max_x-_world_min_x)/_map_resolution);
    _map_height = static_cast<unsigned int>((_world_max_y-_world_min_y)/_map_resolution);
    _origin_x = _world_min_x;
    _origin_y = _world_min_y;
    _size_x = _map_width;
    _size_y = _map_height;
    _robot_radius_pix = static_cast<unsigned int>(std::round(_robot_radius/_map_resolution));
    _initialized = false;

    auto init_occupancy_grid_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(_map_topic, *(_n.get()));
    auto occupancy_grid_width = init_occupancy_grid_msg->info.width;
    auto occupancy_grid_height = init_occupancy_grid_msg->info.height;

    _occupancy_map = cv::Mat(occupancy_grid_height, occupancy_grid_width,CV_8UC3);
    _occupancy_map.setTo(255);
    cv::cvtColor(_occupancy_map, _occupancy_mask, cv::COLOR_BGR2GRAY);
    cv::threshold(_occupancy_mask, _occupancy_mask, 128, 255, cv::THRESH_BINARY); // 将灰度值为205的unknown视为free space
    cv::cvtColor(_occupancy_mask, _occupancy_mask, cv::COLOR_GRAY2BGR);

    _coverage_layer = cv::Mat(occupancy_grid_height, occupancy_grid_width, CV_8UC3);
    _coverage_layer.setTo(255);
    cv::bitwise_and(_coverage_layer, _occupancy_mask, _coverage_map);

    _trajectory_publisher = _n->advertise<nav_msgs::Path>(_trajectory_topic, 1);
    _odom_subscriber = _n->subscribe<nav_msgs::Odometry>(_odom_topic, 1, &CoverageMapper::odomCallback, this);

    _occupancy_map_publisher = _n->advertise<sensor_msgs::Image>(_occupancy_map_topic, 1);
    _coverage_map_publisher = _n->advertise<sensor_msgs::Image>(_coverage_map_topic, 1);
    _coverage_image_publisher = _n->advertise<sensor_msgs::Image>(_coverage_image_topic, 1);
    _occupancy_grid_subscriber = _n->subscribe<nav_msgs::OccupancyGrid>(_map_topic, 1, &CoverageMapper::occupancyGridCallback, this);

    ros::Duration interval(0.001);
    while (ros::ok()) {
        ros::spinOnce();
        interval.sleep();
    }
}

CoverageMapper::~CoverageMapper() {}

void CoverageMapper::mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy)
{
    wx = _origin_x + (mx + 0.5) * _map_resolution;
    wy = _origin_y + (my + 0.5) * _map_resolution;
}

bool CoverageMapper::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my)
{
    if (wx < _origin_x || wy < _origin_y){
        return false;
    }
    mx = (int)((wx - _origin_x) / _map_resolution);
    my = (int)((wy - _origin_y) / _map_resolution);
    if (mx < _size_x && my < _size_y){
        return true;
    }else{
        return false;
    }
}

void CoverageMapper::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg){
    boost::unique_lock<boost::recursive_mutex> lock(_mutex);
    unsigned int mx, my;

    nav_msgs::Odometry odom = *odom_msg;

    if(_trajectory.poses.empty()){
        geometry_msgs::PoseStamped curr_pose;
        curr_pose.header.stamp = odom.header.stamp;
        curr_pose.pose = odom.pose.pose;
        _trajectory.poses.emplace_back(curr_pose);

        worldToMap(curr_pose.pose.position.x, curr_pose.pose.position.y, mx, my);
//        cv::circle(_coverage_layer, cv::Point(mx,my), _robot_radius_pix, COVERAGE, -1);
        cv::rectangle(_coverage_layer, cv::Point(mx-_robot_radius_pix, my-_robot_radius_pix), cv::Point(mx+_robot_radius_pix, my+_robot_radius_pix), COVERAGE, -1);

    }else if(!(odom.pose.pose.position.x == _trajectory.poses.back().pose.position.x
               && odom.pose.pose.position.y == _trajectory.poses.back().pose.position.y)){
        geometry_msgs::PoseStamped curr_pose;
        curr_pose.header.stamp = odom.header.stamp;
        curr_pose.pose = odom.pose.pose;
        _trajectory.poses.emplace_back(curr_pose);

        worldToMap(curr_pose.pose.position.x, curr_pose.pose.position.y, mx, my);
//        cv::circle(_coverage_layer, cv::Point(mx,my), _robot_radius_pix, COVERAGE, -1);
        cv::rectangle(_coverage_layer, cv::Point(mx-_robot_radius_pix, my-_robot_radius_pix), cv::Point(mx+_robot_radius_pix, my+_robot_radius_pix), COVERAGE, -1);
    }

    _trajectory.header.stamp = ros::Time::now();
    _trajectory.header.frame_id = _trajectory_frame;

    _trajectory_publisher.publish(_trajectory);


    cv_bridge::CvImage occupancy_map_ros;
    occupancy_map_ros.header.stamp = odom.header.stamp;
    occupancy_map_ros.header.frame_id = _map_frame;
    occupancy_map_ros.encoding = "bgr8"; // mono8:CV_8UC1, bgr8:CV_8UC3
    occupancy_map_ros.image = _occupancy_map.clone();
    sensor_msgs::ImagePtr occupancy_map_ros_img = occupancy_map_ros.toImageMsg();
    if(_initialized){
        _occupancy_map_publisher.publish(occupancy_map_ros_img);
    }

    cv::cvtColor(_occupancy_map, _occupancy_mask, cv::COLOR_BGR2GRAY);
    cv::threshold(_occupancy_mask, _occupancy_mask, 128, 255, cv::THRESH_BINARY); // 将灰度值为205的unknown视为free space
    cv::cvtColor(_occupancy_mask, _occupancy_mask, cv::COLOR_GRAY2BGR);
    cv::bitwise_and(_coverage_layer, _occupancy_mask, _coverage_map);

    cv_bridge::CvImage coverage_map;
    coverage_map.header.stamp = odom.header.stamp;
    coverage_map.header.frame_id = _map_frame;
    coverage_map.encoding = "bgr8"; // mono8:CV_8UC1, bgr8:CV_8UC3
    coverage_map.image = _coverage_map.clone();
    sensor_msgs::ImagePtr coverage_map_msg = coverage_map.toImageMsg();
    if(_initialized){
        _coverage_map_publisher.publish(coverage_map_msg);
    }

    _coverage_image = _coverage_map.clone();
    cv::flip(_coverage_image, _coverage_image, 1); // 上下翻转
    cv::rotate(_coverage_image, _coverage_image, cv::ROTATE_90_CLOCKWISE);

    cv_bridge::CvImage coverage_image;
    coverage_image.header.stamp = odom.header.stamp;
    coverage_image.header.frame_id = _map_frame;
    coverage_image.encoding = "bgr8"; // mono8:CV_8UC1, bgr8:CV_8UC3
    coverage_image.image = _coverage_image.clone();
    sensor_msgs::ImagePtr coverage_image_msg = coverage_image.toImageMsg();
    if(_initialized){
        _coverage_image_publisher.publish(coverage_image_msg);
    }

    lock.unlock();
}

void CoverageMapper::occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& occupancy_grid_msg){
    boost::unique_lock<boost::recursive_mutex> lock(_mutex);

    nav_msgs::OccupancyGrid occupancy_grid = *occupancy_grid_msg;
    std::vector<int8_t> occupancy_grid_mapdata = occupancy_grid.data;

    int mapdata_index = 0;
    for(size_t row = 0; row < _occupancy_map.rows; ++row){
        for(size_t col = 0; col < _occupancy_map.cols; ++col){
            if(occupancy_grid_mapdata.at(mapdata_index)==100){ // wall
                _occupancy_map.at<cv::Vec3b>(row, col) = cv::Vec3b(0,0,0);
            }else if(occupancy_grid_mapdata.at(mapdata_index)==0){ // free space
                _occupancy_map.at<cv::Vec3b>(row, col) = cv::Vec3b(255,255,255);
            }else if(occupancy_grid_mapdata.at(mapdata_index)==-1){ // unknown space
                _occupancy_map.at<cv::Vec3b>(row, col) = cv::Vec3b(205,205,205);
            }else{
                _occupancy_map.at<cv::Vec3b>(row, col) = cv::Vec3b(0,0,0);
            }
            mapdata_index++;
        }
    }
    if(!_initialized){
        _initialized = true;
    }

    lock.unlock();
}