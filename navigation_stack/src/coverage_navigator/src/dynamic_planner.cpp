#include "coverage_navigator/dynamic_planner.h"

std::string dynamic_planner::PARAM_NAME_robot_radius="robot_radius";
std::string dynamic_planner::PARAM_NAME_robot_size="robot_size";

std::string dynamic_planner::PARAM_NAME_high_speed="dynamic_navigating/high_speed";
std::string dynamic_planner::PARAM_NAME_normal_speed="dynamic_navigating/normal_speed";
std::string dynamic_planner::PARAM_NAME_low_speed="dynamic_navigating/low_speed";
std::string dynamic_planner::PARAM_NAME_speed_increment="dynamic_navigating/speed_increment";

std::string dynamic_planner::PARAM_NAME_path_plan_frame = "dynamic_navigating/path_plan_frame";
std::string dynamic_planner::PARAM_NAME_odom_topic="dynamic_navigating/odom_topic";
std::string dynamic_planner::PARAM_NAME_states_topic="dynamic_navigating/states_topic";
std::string dynamic_planner::PARAM_NAME_coverage_map_topic="dynamic_navigating/coverage_map_topic";
std::string dynamic_planner::PARAM_NAME_occupancy_map_topic="dynamic_navigating/occupancy_map_topic";
std::string dynamic_planner::PARAM_NAME_make_zone_service="dynamic_navigating/make_zone_service";
std::string dynamic_planner::PARAM_NAME_make_obstacles_service="dynamic_navigating/make_obstacles_service";
std::string dynamic_planner::PARAM_NAME_make_plan_service="dynamic_navigating/make_plan_service";
std::string dynamic_planner::PARAM_NAME_move_action="dynamic_navigating/move_action";

std::string dynamic_planner::PARAM_NAME_yaw_buffer="dynamic_navigating/yaw_buffer";
std::string dynamic_planner::PARAM_NAME_yaw_tolerance="dynamic_navigating/yaw_tolerance";
std::string dynamic_planner::PARAM_NAME_position_tolerance="dynamic_navigating/position_tolerance";
std::string dynamic_planner::PARAM_NAME_time_tolerance="dynamic_navigating/time_tolerance";


void DynamicPlanner::setup() {

    _n.reset(new ros::NodeHandle("~"));

    _n->getParam(dynamic_planner::PARAM_NAME_robot_radius, _robot_radius);
    _n->getParam(dynamic_planner::PARAM_NAME_robot_size,_robot_size);
    _n->getParam(dynamic_planner::PARAM_NAME_high_speed,_high_speed);
    _n->getParam(dynamic_planner::PARAM_NAME_normal_speed,_normal_speed);
    _n->getParam(dynamic_planner::PARAM_NAME_low_speed,_low_speed);
    _n->getParam(dynamic_planner::PARAM_NAME_speed_increment,_speed_increment);
    _n->getParam(dynamic_planner::PARAM_NAME_path_plan_frame,_path_plan_frame);
    _n->getParam(dynamic_planner::PARAM_NAME_odom_topic,_odom_topic);
    _n->getParam(dynamic_planner::PARAM_NAME_states_topic,_states_topic);
    _n->getParam(dynamic_planner::PARAM_NAME_coverage_map_topic,_coverage_map_topic);
    _n->getParam(dynamic_planner::PARAM_NAME_occupancy_map_topic,_occupancy_map_topic);
    _n->getParam(dynamic_planner::PARAM_NAME_make_zone_service,_make_zone_service);
    _n->getParam(dynamic_planner::PARAM_NAME_make_obstacles_service,_make_obstacles_service);
    _n->getParam(dynamic_planner::PARAM_NAME_make_plan_service,_make_plan_service);
    _n->getParam(dynamic_planner::PARAM_NAME_move_action,_move_action);
    _n->getParam(dynamic_planner::PARAM_NAME_yaw_buffer,_yaw_buffer);
    _n->getParam(dynamic_planner::PARAM_NAME_yaw_tolerance,_yaw_tolerance);
    _n->getParam(dynamic_planner::PARAM_NAME_position_tolerance,_position_tolerance);
    _n->getParam(dynamic_planner::PARAM_NAME_time_tolerance,_time_tolerance);

    _speed_level = HIGH;
    _robot_turning_direction = RIGHT;
    _robot_rotating_direction = RIGHT;

    _start_rotation = false;
    _finish_rotation = false;

    _target_yaw = 0.0;
    _main_direction = 0;
    _inverse_main_direction = 0;

    _collided = false;

    _boundary_map = cv::Mat();
    _uncovered_areas_map = cv::Mat();

    _next_uncovered_position = geometry_msgs::Pose();
    _uncovered_positions.clear();

    _viable_zone.area.form.clear();
    _boundary.area.form.clear();
    _obstacles.list.clear();

    _contours.clear();
    _polys.clear();

    _zone_contour.clear();
    _wall_contour.clear();
    _boundary_contour.clear();
    _obstacle_contours.clear();

}

void DynamicPlanner::run(const custom_msgs::Zone& zone) {

    _viable_zone.area = zone.area;
    setupZoneContour();

    // 当传感器不是360度的工作范围时(如TOF)才使用look around, 当使用laser就不需要使用look around
//    auto init_odom = ros::topic::waitForMessage<nav_msgs::Odometry>(_odom_topic, _n);
//    look_around(init_odom);

    auto init_occupancy_map = ros::topic::waitForMessage<sensor_msgs::Image>(_occupancy_map_topic,*(_n.get()));
    findMainDirection(init_occupancy_map);

    auto odom = ros::topic::waitForMessage<nav_msgs::Odometry>(_odom_topic, *(_n.get()));
    auto states = ros::topic::waitForMessage<custom_msgs::DetectionState>(_states_topic,*(_n.get()));
    headingToWall(odom, states);


    WallFollower wall_follower;
    wall_follower.robotPowerOn();
    wall_follower.setup();
    wall_follower.run();
    wall_follower.shutDown();
    wall_follower.robotPowerOff();

    auto updated_occupancy_map = ros::topic::waitForMessage<sensor_msgs::Image>(_occupancy_map_topic,*(_n.get()));
    clearBoundary();
    clearObstacles();
    updateBoundaryAndObstacles(updated_occupancy_map);
    setBoundary();
    setObstacles();

    odom = ros::topic::waitForMessage<nav_msgs::Odometry>(_odom_topic, *(_n.get()));
    alignWithWall(odom);

    setupZigzaggingInitTurningDirection();

    odom = ros::topic::waitForMessage<nav_msgs::Odometry>(_odom_topic, *(_n.get()));
    float yaw = tf::getYaw(odom->pose.pose.orientation)/M_PI*180.0;
    // yaw: [0, 360)
    if(yaw<0){
        yaw+=360.0;
    }

    int zigzagging_main_direction;
    if(std::abs(compute_relative_yaw(_main_direction, yaw))<=std::abs(compute_relative_yaw(_inverse_main_direction, yaw))){
        zigzagging_main_direction = _main_direction;
    }else{
        zigzagging_main_direction = _inverse_main_direction;
    }
    DIRECTION zigzagging_turning_direction = _robot_turning_direction;
    bool do_zigzagging_refinement = false;

    ZigzagWalker zigzag_walker;
    zigzag_walker.robotPowerOn();
    zigzag_walker.setup(_boundary_map);
    zigzag_walker.run(do_zigzagging_refinement, zigzagging_turning_direction, zigzagging_main_direction);
    zigzag_walker.shutDown();
    zigzag_walker.robotPowerOff();

    while(true){
        bool found = queryNextWaypoint();
        if(!found){
            break;
        }
        ROS_INFO("FOUND UNCOVERED POINTS");

        coverNextWaypoints();

        clearViableZone();
        setBoundary();
        setObstacles();

        odom = ros::topic::waitForMessage<nav_msgs::Odometry>(_odom_topic, *(_n.get()));
        alignWithWall(odom);

        setupZigzaggingNewTurningDirection();

        odom = ros::topic::waitForMessage<nav_msgs::Odometry>(_odom_topic, *(_n.get()));
        float curr_yaw = tf::getYaw(odom->pose.pose.orientation)/M_PI*180.0;
        // curr_yaw: [0, 360)
        if(curr_yaw<0){
            curr_yaw+=360.0;
        }

        if(std::abs(compute_relative_yaw(_main_direction, curr_yaw))<=std::abs(compute_relative_yaw(_inverse_main_direction, curr_yaw))){
            zigzagging_main_direction = _main_direction;
        }else{
            zigzagging_main_direction = _inverse_main_direction;
        }

        zigzagging_turning_direction = _robot_turning_direction;
        do_zigzagging_refinement = true;

        zigzag_walker.robotPowerOn();
        zigzag_walker.setup(_boundary_map);
        zigzag_walker.run(do_zigzagging_refinement, zigzagging_turning_direction, zigzagging_main_direction);
        zigzag_walker.shutDown();
        zigzag_walker.robotPowerOff();

    }
    ROS_INFO("FINISH COVERAGE.");

    clearBoundary();
    clearObstacles();

}

void DynamicPlanner::shutDown() {
    _odom_subscriber.shutdown();
    _ac->cancelAllGoals();
}

void DynamicPlanner::robotRotate(DIRECTION rotating_direction, const float& wheel_speed){
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

void DynamicPlanner::robotRotate(DIRECTION rotating_direction, SPEED_LEVEL speed_level){
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

void DynamicPlanner::clearViableZone() {

    _sc = _n->serviceClient<custom_srvs::Zone>(_make_zone_service);

    uint32_t map_width = getMapWidth();
    uint32_t map_height = getMapHeight();

    double min_wx, max_wx, min_wy, max_wy;
    mapToWorld(0, 0, min_wx, min_wy);
    mapToWorld(map_width-1, map_height-1, max_wx, max_wy);

    geometry_msgs::Point w_top_left, w_top_right, w_bottom_left, w_bottom_right;
    w_top_left.x = max_wx;
    w_top_left.y = max_wy;
    w_top_left.z = 0.0;

    w_top_right.x = max_wx;
    w_top_right.y = min_wy;
    w_top_right.z = 0.0;

    w_bottom_right.x = min_wx;
    w_bottom_right.y = min_wy;
    w_bottom_right.z = 0.0;

    w_bottom_left.x = min_wx;
    w_bottom_left.y = max_wy;
    w_bottom_left.z = 0.0;

    // 逆时针排序
    custom_msgs::Zone clear_zone;
    clear_zone.area.form.clear();
    clear_zone.area.form.emplace_back(w_top_left);
    clear_zone.area.form.emplace_back(w_bottom_left);
    clear_zone.area.form.emplace_back(w_bottom_right);
    clear_zone.area.form.emplace_back(w_top_right);

    custom_srvs::Zone clear_zone_srv;
    clear_zone_srv.request.area = clear_zone.area;

    _sc.call(clear_zone_srv);
}

void DynamicPlanner::setViableZone(){
    _sc = _n->serviceClient<custom_srvs::Zone>(_make_zone_service);
    custom_srvs::Zone zone_srv;
    zone_srv.request.area = _viable_zone.area;
    _sc.call(zone_srv);
}

void DynamicPlanner::setBoundary(){

    _sc = _n->serviceClient<custom_srvs::Zone>(_make_zone_service);

    _boundary.area.form.clear();

    for(auto& boundary_img_point:_boundary_contour){
        double boundary_point_x, boundary_point_y;
        mapToWorld(boundary_img_point.x, boundary_img_point.y, boundary_point_x, boundary_point_y);
        geometry_msgs::Point boundary_point;
        boundary_point.x = boundary_point_x;
        boundary_point.y = boundary_point_y;
        boundary_point.z = 0.0;
        _boundary.area.form.push_back(boundary_point);
    }

    custom_srvs::Zone boundary_srv;
    boundary_srv.request.area = _boundary.area;
    _sc.call(boundary_srv);
}

void DynamicPlanner::clearBoundary(){
    clearViableZone();
}

void DynamicPlanner::lookAroundCallback(const nav_msgs::Odometry::ConstPtr &odom_msg){

    robotRotate(RIGHT, NORMAL);

    ROS_INFO("[ROBOT] LOOKING AROUND! \n");

    auto odom = *odom_msg;
    float yaw = tf::getYaw(odom.pose.pose.orientation)/M_PI*180.0;
    // yaw: [0, 360)
    if(yaw<0){
        yaw+=360.0;
    }

    float yaw_offset = compute_relative_yaw(_target_yaw, yaw);

    if(!_start_rotation){
        if(std::abs(yaw_offset) > _yaw_buffer){
            _start_rotation = true;
        }
    }else{
        if((std::abs(yaw_offset) < _yaw_tolerance)
           || (yaw_offset < 0 && yaw_offset > -_yaw_buffer)){
            _finish_rotation = true;
            _start_rotation = false;
            ROS_INFO("FINISH ROTATION.");
            robotStop();
        }
    }
}

void DynamicPlanner::lookAround(const nav_msgs::Odometry::ConstPtr &odom_msg){
    ROS_INFO("LOOK AROUND.");

    auto odom = *odom_msg;
    _target_yaw = tf::getYaw(odom.pose.pose.orientation)/M_PI*180.0;
    // yaw: [0, 360)
    if(_target_yaw<0){
        _target_yaw+=360.0;
    }

    _start_rotation = false;
    _finish_rotation = false;

    robotRotate(RIGHT, NORMAL);
    ROS_INFO("[ROBOT] LOOKING AROUND! \n");

    _odom_subscriber = _n->subscribe(_odom_topic, 1, &DynamicPlanner::lookAroundCallback, this);

    ros::Duration interval(0.001);
    while (ros::ok()) {
        if(_finish_rotation){
            _odom_subscriber.shutdown();
            return;
        }
        ros::spinOnce();
        interval.sleep();
    }
}

void DynamicPlanner::setupZoneContour(){
    unsigned int zone_point_img_x, zone_point_img_y; // pix
    _zone_contour.clear();
    for(auto zone_point : _viable_zone.area.form){ // m
        worldToMap(zone_point.x, zone_point.y, zone_point_img_x, zone_point_img_y);
        _zone_contour.emplace_back(cv::Point(zone_point_img_x, zone_point_img_y));
    }
}

void DynamicPlanner::updateBoundaryAndObstacles(const sensor_msgs::ImageConstPtr &occupancy_grid_map_msg){

    cv_bridge::CvImageConstPtr img_ptr = cv_bridge::toCvShare(occupancy_grid_map_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat img = img_ptr->image.clone();

    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    std::vector<std::vector<cv::Point>> zone_contours = {_zone_contour};

    cv::Mat mask = cv::Mat::zeros(gray.rows,gray.cols,CV_8UC3);
    cv::fillPoly(mask, zone_contours, cv::Scalar(255,255,255));
    cv::cvtColor(mask,mask,cv::COLOR_BGR2GRAY);
    mask.convertTo(mask, CV_8UC1);
    cv::bitwise_and(gray, mask, gray);

    cv::Mat thresh = gray.clone();
    cv::threshold(thresh, thresh, 128, 255, 0); // 将灰度值为205的unknown视为free space

    auto robot_image_radius = static_cast<unsigned int>(std::round((_robot_radius/(getMapResolution()))));
    unsigned int robot_image_size = 2*robot_image_radius;

//    cv::Mat erode_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(robot_image_size+1,robot_image_size+1), cv::Point(-1,-1)); // size: default:10
//    cv::morphologyEx(thresh, thresh, cv::MORPH_ERODE, erode_kernel);

    cv::Mat open_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2*robot_image_size+1,2*robot_image_size+1), cv::Point(-1,-1)); // size: default:5
    cv::morphologyEx(thresh, thresh, cv::MORPH_OPEN, open_kernel);

    std::vector<std::vector<cv::Point>> cnts;
    std::vector<cv::Vec4i> hierarchy; // index: next, prev, first_child, parent
    cv::findContours(thresh, cnts, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    std::vector<int> cnt_indices(cnts.size());
    std::iota(cnt_indices.begin(), cnt_indices.end(), 0);
    std::sort(cnt_indices.begin(), cnt_indices.end(), [&cnts](int lhs, int rhs){return cv::contourArea(cnts[lhs]) > cv::contourArea(cnts[rhs]);});
    
    auto odom = ros::topic::waitForMessage<nav_msgs::Odometry>(_odom_topic, *(_n.get()));
    double robot_wx = odom->pose.pose.position.x;
    double robot_wy = odom->pose.pose.position.y;
    unsigned int robot_mx, robot_my;
    worldToMap(robot_wx,robot_wy,robot_mx,robot_my);

    int ext_cnt_idx = cnt_indices.front();
    for(auto& cnt_idx:cnt_indices){
        auto cnt = cnts[cnt_idx];
        auto p = cv::Point2f(robot_mx,robot_my);
        auto dist = cv::pointPolygonTest(cnt, p, false); //measureDist设置为false时，返回 -1、0、1三个固定值。若返回值为+1，表示点在多边形内部，返回值为-1，表示在多边形外部，返回值为0，表示在多边形上
        if(dist==1){
            ext_cnt_idx = cnt_idx;
            break;
        }
    }

    _contours.clear();
    _contours.emplace_back(cnts[ext_cnt_idx]);

    // find all the contours of obstacle
    for(int i = 0; i < hierarchy.size(); i++){
        if(hierarchy[i][3]==ext_cnt_idx){ // parent contour's index equals to external contour's index
            _contours.emplace_back(cnts[i]);
        }
    }

    _polys.clear();
    std::vector<cv::Point> poly;
    for(auto& contour : _contours){
        cv::approxPolyDP(contour, poly, 3, true);
        _polys.emplace_back(poly);
        poly.clear();
    }

    _obstacle_contours.clear();
    for(size_t i = 1; i < _polys.size(); ++i){
        _obstacle_contours.emplace_back(_polys[i]);
    }
    _boundary_contour.clear();
    _boundary_contour = _polys[0];

    cv::Mat boundary_area = cv::Mat(img.rows, img.cols, CV_8UC3);
    boundary_area.setTo(0);
    std::vector<std::vector<cv::Point>> boundary_contours = {_boundary_contour};
    cv::fillPoly(boundary_area, boundary_contours, cv::Scalar(255,255,255));
    cv::cvtColor(boundary_area, boundary_area, cv::COLOR_BGR2GRAY);

    cv::Mat erode_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2*robot_image_size+1,2*robot_image_size+1), cv::Point(-1,-1)); // size: default:10
    cv::morphologyEx(boundary_area, boundary_area, cv::MORPH_ERODE, erode_kernel);

    std::vector<std::vector<cv::Point>> inner_cnts;
    std::vector<cv::Vec4i> inner_hierarchy; // index: next, prev, first_child, parent
    cv::findContours(boundary_area, inner_cnts, inner_hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

    std::vector<int> inner_cnt_indices(inner_cnts.size());
    std::iota(inner_cnt_indices.begin(), inner_cnt_indices.end(), 0);
    std::sort(inner_cnt_indices.begin(), inner_cnt_indices.end(), [&inner_cnts](int lhs, int rhs){return cv::contourArea(inner_cnts[lhs]) > cv::contourArea(inner_cnts[rhs]);});
    int inner_ext_cnt_idx = inner_cnt_indices.front();

    _wall_contour.clear();
    _wall_contour = inner_cnts[inner_ext_cnt_idx];


    // test
    cv::Mat boundary_image = img.clone();
    cv::drawContours(boundary_image, _polys, 0, cv::Scalar(0,0,255));
    std::vector<std::vector<cv::Point>> wall_cnt = {_wall_contour};
    cv::drawContours(boundary_image, wall_cnt, 0, cv::Scalar(0,255,0));
    for(size_t i = 1; i < _polys.size(); ++i){
        cv::drawContours(boundary_image, _polys, i, cv::Scalar(255,0,0));
    }
    cv::flip(boundary_image, boundary_image,1);
    cv::rotate(boundary_image,boundary_image,cv::ROTATE_90_CLOCKWISE);
    cv::imwrite("boundary.jpg", boundary_image);
    //
}

void DynamicPlanner::findMainDirection(const sensor_msgs::ImageConstPtr &occupancy_grid_map_msg){

    updateBoundaryAndObstacles(occupancy_grid_map_msg);

    /// compute main direction ///

    // 坐标系: costmap图像坐标系; 角度范围: [0,180)
    std::vector<int> line_deg_histogram(180);
    double line_len; // weight
    double line_deg;
    int line_deg_idx;

    auto ext_poly = _polys.front();
    ext_poly.emplace_back(ext_poly.front());
    for(int i = 1; i < ext_poly.size(); i++){
        line_len = std::sqrt(std::pow((ext_poly[i].x-ext_poly[i-1].x),2)+std::pow((ext_poly[i].y-ext_poly[i-1].y),2));
        // y-axis towards up, x-axis towards right, theta is from x-axis to y-axis
        line_deg = std::round(atan2(ext_poly[i].y-ext_poly[i-1].y,ext_poly[i].x-ext_poly[i-1].x)/M_PI*180.0); // atan2: (-180, 180]
        line_deg_idx = (int(line_deg) + 180) % 180; // [0, 180)
        line_deg_histogram[line_deg_idx] += int(line_len);
    }

    auto it = std::max_element(line_deg_histogram.begin(), line_deg_histogram.end());
    _main_direction = (it-line_deg_histogram.begin());
    ROS_INFO("[MAIN DIRECTION] %d degree", _main_direction);

}

void DynamicPlanner::clearObstacles(){

    _sc = _n->serviceClient<custom_srvs::Obstacle>(_make_obstacles_service);

    double corner_wx, corner_wy; // 角落上的点
    mapToWorld(0, 0, corner_wx, corner_wy);

    geometry_msgs::Point corner;
    corner.x = corner_wx;
    corner.y = corner_wy;
    corner.z = 0.0;

    _obstacles.list.clear();
    custom_msgs::Form obstacle;
    obstacle.form.push_back(corner);
    _obstacles.list.push_back(obstacle);

    custom_srvs::Obstacle obstacle_srv;
    obstacle_srv.request.list = _obstacles.list;

    _sc.call(obstacle_srv);
}

void DynamicPlanner::setObstacles(){

    _sc = _n->serviceClient<custom_srvs::Obstacle>(_make_obstacles_service);

    _obstacles.list.clear();
    for(auto& obstacle_contour:_obstacle_contours){
        custom_msgs::Form obstacle;
        double obstacle_point_x, obstacle_point_y;
        for(auto& obstacle_img_point:obstacle_contour){
            mapToWorld(obstacle_img_point.x, obstacle_img_point.y, obstacle_point_x, obstacle_point_y);
            geometry_msgs::Point obstacle_point;
            obstacle_point.x = obstacle_point_x;
            obstacle_point.y = obstacle_point_y;
            obstacle_point.z = 0.0;
            obstacle.form.push_back(obstacle_point);
        }
        _obstacles.list.push_back(obstacle);
    }

    custom_srvs::Obstacle obstacle_srv;
    obstacle_srv.request.list = _obstacles.list;
    _sc.call(obstacle_srv);
}

bool DynamicPlanner::isPathFound(const nav_msgs::Odometry &start_odom, const geometry_msgs::Pose &target_pose){

    ros::ServiceClient makeplan_client = _n->serviceClient<nav_msgs::GetPlan>(_make_plan_service, true);
    nav_msgs::GetPlan makeplan_service;
    makeplan_service.request.tolerance = getMapResolution(); //单位m; 数值和yaml文件中一致
    makeplan_service.request.start.header.frame_id = _path_plan_frame;
    makeplan_service.request.start.pose = start_odom.pose.pose;
    makeplan_service.request.goal.header.frame_id = _path_plan_frame;
    makeplan_service.request.goal.pose = target_pose;

    bool is_makeplan_succeed = makeplan_client.call(makeplan_service);

    if(is_makeplan_succeed){
        if(!makeplan_service.response.plan.poses.empty()){
            ROS_INFO("MAKE LOCAL PLAN SUCCESSFULLY.");
            return true;
        }else{
            ROS_INFO("MAKE EMPTY LOCAL PLAN");
            return false;
        }
    }else{
        ROS_INFO("MAKE LOCAL PLAN FAILED");
        return false;
    }
}

bool DynamicPlanner::getOutOfStuck(const geometry_msgs::Pose &goal_pose){
    StuckEscaper escaper;
    escaper.robotPowerOn();
    escaper.setup();
    escaper.run(goal_pose);
    escaper.shutDown();
    escaper.robotPowerOff();
}

bool DynamicPlanner::moveToGoal(const move_base_msgs::MoveBaseGoal &goal){

    _ac.reset(new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(_move_action, true)); // global namespace
    _ac->waitForServer();

    double start_sec=0, now_sec=0;
    nav_msgs::Odometry::ConstPtr start_odom, now_odom;

    while(true){
        _ac->waitForServer();
        _ac->sendGoal(goal);
        start_sec = ros::Time::now().toSec();
        start_odom = ros::topic::waitForMessage<nav_msgs::Odometry>(_odom_topic, *(_n.get()));
        while (true) {
            now_sec = ros::Time::now().toSec();
            now_odom = ros::topic::waitForMessage<nav_msgs::Odometry>(_odom_topic, *(_n.get()));
            auto goal_dist = compute_position_error(now_odom->pose.pose, goal.target_pose.pose);
            auto nav_dist = compute_position_error(now_odom->pose.pose, start_odom->pose.pose);
            if(_ac->getState()==actionlib::SimpleClientGoalState::SUCCEEDED){
                _ac->cancelAllGoals();
                ROS_INFO("GET ON TARGET POINT");
                return true;
            }
            if(std::abs(now_sec-start_sec)>_time_tolerance){
//                ROS_INFO("DURING: %f, NAV DIST: %f, GOAL DIST: %f \n", std::abs(now_sec-start_sec), nav_dist, goal_dist);
                if(nav_dist < _position_tolerance){
                    _ac->cancelAllGoals();
                    if(!getOutOfStuck(goal.target_pose.pose)){
                        robotStop();
                        ROS_INFO("CANNOT GET ON TARGET POINT");
                        return false;
                    }else{
                        break;
                    }
                }
                start_sec = ros::Time::now().toSec();
                start_odom = ros::topic::waitForMessage<nav_msgs::Odometry>(_odom_topic, *(_n.get()));
            }
        }
    }
}

void DynamicPlanner::parallelAtWallCallback(const nav_msgs::Odometry::ConstPtr &odom_msg){

    if(_robot_rotating_direction == LEFT){
        robotRotate(LEFT,_speed_level);
    }else if(_robot_rotating_direction == RIGHT){
        robotRotate(RIGHT,_speed_level);
    }else{
        return;
    }

    auto odom = *odom_msg;
    float yaw = tf::getYaw(odom.pose.pose.orientation)/M_PI*180.0;
    // yaw: [0, 360)
    if(yaw<0){
        yaw+=360.0;
    }

    float yaw_offset = compute_relative_yaw(_target_yaw, yaw);


    if(std::abs(yaw_offset)<_yaw_buffer){
        _speed_level=NORMAL;
    }

    if((std::abs(yaw_offset) < _yaw_tolerance)
       || (_robot_rotating_direction == LEFT && yaw_offset > 0 && yaw_offset < _yaw_buffer)
       || (_robot_rotating_direction == RIGHT && yaw_offset < 0 && yaw_offset > -_yaw_buffer)
            ){
        _finish_rotation = true;
        ROS_INFO("PARALLEL AT WALL.");
        robotStop();
        _speed_level=HIGH;
    }

}

void DynamicPlanner::aimAtWallCallback(const nav_msgs::Odometry::ConstPtr &odom_msg){

    if(_robot_rotating_direction == LEFT){
        robotRotate(LEFT, _speed_level);
    }else if(_robot_rotating_direction == RIGHT){
        robotRotate(RIGHT, _speed_level);
    }else{
        return;
    }

    auto odom = *odom_msg;
    float yaw = tf::getYaw(odom.pose.pose.orientation)/M_PI*180.0;
    // yaw: [0, 360)
    if(yaw<0){
        yaw+=360.0;
    }

    float yaw_offset = compute_relative_yaw(_target_yaw, yaw);

    if(std::abs(yaw_offset)<_yaw_buffer){
        _speed_level=NORMAL;
    }

    if((std::abs(yaw_offset) < _yaw_tolerance)
       || (_robot_rotating_direction == LEFT && yaw_offset > 0 && yaw_offset < _yaw_buffer)
       || (_robot_rotating_direction == RIGHT && yaw_offset < 0 && yaw_offset > -_yaw_buffer)){
        _finish_rotation = true;
        robotStop();
        _speed_level=HIGH;
    }

}

void DynamicPlanner::travelToWallCallback(const custom_msgs::DetectionState::ConstPtr &states_msg,
                                          const nav_msgs::Odometry::ConstPtr &odom_msg){

    auto states = *states_msg;
    auto odom = *odom_msg;

    if(states.heading==states.COLLIDED || states.heading_left==states.COLLIDED || states.heading_right==states.COLLIDED){
        _collided = true;
//        ROS_INFO("COLLIDED!");
    }else{
        _collided = false;
    }

    if(_collided){
        robotStop();
//        ROS_INFO("STOP AT WALL.");
        return;
    }else{
        float yaw = tf::getYaw(odom.pose.pose.orientation)/M_PI*180.0;
        // yaw: [0, 360)
        if(yaw<0){
            yaw+=360.0;
        }
        float yaw_offset = compute_relative_yaw(_target_yaw, yaw);

        if(yaw_offset<0){
            robotMove(_high_speed, _high_speed+_speed_increment);
        }else if(yaw_offset>0){
            robotMove(_high_speed+_speed_increment, _high_speed);
        }else{
            robotForward(_high_speed);
        }
    }
}

void DynamicPlanner::headingToWall(const nav_msgs::Odometry::ConstPtr &odom_msg,
                                   const custom_msgs::DetectionState::ConstPtr &states_msg){

    // 看sweep的两个方向哪个近就转到哪边去, 与墙平行后判断两边哪边离墙远就转个90度朝向墙,nan说明太远,都是nan的话往右边转,太近的话是有数值的,只是不准

    _collided = false;

    auto odom = *odom_msg;
    float yaw = tf::getYaw(odom.pose.pose.orientation)/M_PI*180.0;
    // yaw: [0, 360)
    if(yaw<0){
        yaw+=360.0;
    }

    _inverse_main_direction = _main_direction + 180;

    float main_dir_offset = compute_relative_yaw(yaw, _main_direction);
    float inv_main_dir_offset = compute_relative_yaw(yaw, _inverse_main_direction);

    if(std::abs(main_dir_offset)<=std::abs(inv_main_dir_offset)){
        _target_yaw = _main_direction;
        if(main_dir_offset >= 0){
            _robot_rotating_direction = LEFT;
        }else{
            _robot_rotating_direction = RIGHT;
        }
    }else{
        _target_yaw = _inverse_main_direction;
        if(_inverse_main_direction >= 0){
            _robot_rotating_direction = LEFT;
        }else{
            _robot_rotating_direction = RIGHT;
        }
    }

    _finish_rotation = false;

    robotRotate(_robot_rotating_direction, _speed_level);

    _odom_subscriber = _n->subscribe(_odom_topic, 10, &DynamicPlanner::parallelAtWallCallback, this);

    ros::Duration interval(0.001);
    while (ros::ok()) {
        if(_finish_rotation){
            _odom_subscriber.shutdown();
            break;
        }
        ros::spinOnce();
        interval.sleep();
    }

    auto states = *states_msg;

    if(states.left_range==INFINITY && states.right_range!=INFINITY){
        _robot_rotating_direction = RIGHT;
    }else if(states.left_range!=INFINITY && states.right_range==INFINITY){
        _robot_rotating_direction = LEFT;
    }else if(states.left_range==INFINITY && states.right_range==INFINITY){
        _robot_rotating_direction = RIGHT;
    }else{
        if(states.left_range>states.right_range){
            _robot_rotating_direction = LEFT;
        }else{
            _robot_rotating_direction = RIGHT;
        }
    }

    _finish_rotation = false;

    // target_yaw: [0, 360)
    if(_robot_rotating_direction == LEFT){
        _target_yaw += 90.0;
        if(_target_yaw >= 360.0){
            _target_yaw -= 360.0;
        }
    }else if(_robot_rotating_direction == RIGHT){
        _target_yaw -= 90.0;
        if(_target_yaw < 0.0){
            _target_yaw += 360.0;
        }
    }else{
        return;
    }
    robotRotate(_robot_rotating_direction, _speed_level);

    _odom_subscriber = _n->subscribe(_odom_topic, 10, &DynamicPlanner::aimAtWallCallback, this);

    while (ros::ok()) {
        if(_finish_rotation){
            _odom_subscriber.shutdown();
            break;
        }
        ros::spinOnce();
        interval.sleep();
    }

    auto curr_odom_msg = ros::topic::waitForMessage<nav_msgs::Odometry>(_odom_topic, *(_n.get()));
    auto curr_odom = *curr_odom_msg;

    unsigned int curr_mx, curr_my;
    double curr_wx = curr_odom_msg->pose.pose.position.x, curr_wy = curr_odom_msg->pose.pose.position.y;
    worldToMap(curr_wx, curr_wy, curr_mx, curr_my);

    double wall_point_wx, wall_point_wy;

    auto min_yaw_offset = DBL_MAX;
    geometry_msgs::Pose best_wall_goal;

    for(auto& wall_point:_wall_contour){

        mapToWorld(wall_point.x,wall_point.y,wall_point_wx,wall_point_wy);

        auto wall_point_yaw = atan2((wall_point_wy-curr_wy),(wall_point_wx-curr_wx))/M_PI*180.0; // atan2: (-180, 180]
        // wall_point_yaw: [0, 360)
        if(wall_point_yaw<0){
            wall_point_yaw+=360.0;
        }
        auto yaw_offset = std::abs(compute_relative_yaw(wall_point_yaw, _target_yaw));
//        ROS_INFO("WALL YAW: %f, YAW OFFSET: %f, MIN YAW OFFSET: %f", wall_point_yaw, yaw_offset, min_yaw_offset);

        if(yaw_offset < min_yaw_offset){

            geometry_msgs::Pose goal_at_wall;
            goal_at_wall.position.x = wall_point_wx;
            goal_at_wall.position.y = wall_point_wy;
            goal_at_wall.position.z = 0.0;
            goal_at_wall.orientation = curr_odom.pose.pose.orientation;

            if(isPathFound(curr_odom,goal_at_wall)){
                min_yaw_offset = yaw_offset;
                best_wall_goal = goal_at_wall;
            }
        }
    }

    move_base_msgs::MoveBaseGoal wall_goal;
    wall_goal.target_pose.header.stamp = ros::Time::now();
    wall_goal.target_pose.header.frame_id = _path_plan_frame;
    wall_goal.target_pose.pose.position = best_wall_goal.position;
    wall_goal.target_pose.pose.orientation = best_wall_goal.orientation;
    moveToGoal(wall_goal);

    clearViableZone();
    setBoundary();
    setObstacles();

    _finish_rotation = false;
    curr_odom_msg = ros::topic::waitForMessage<nav_msgs::Odometry>(_odom_topic, *(_n.get()));
    yaw = tf::getYaw(curr_odom_msg->pose.pose.orientation)/M_PI*180.0;
    // yaw: [0, 360)
    if(yaw<0){
        yaw+=360.0;
    }
    float yaw_offset = compute_relative_yaw(yaw, _target_yaw);
    if(yaw_offset >= 0){
        _robot_rotating_direction = LEFT;
    }else{
        _robot_rotating_direction = RIGHT;
    }
    robotRotate(_robot_rotating_direction, _speed_level);

    _odom_subscriber = _n->subscribe(_odom_topic, 10, &DynamicPlanner::aimAtWallCallback, this);

    while (ros::ok()) {
        if(_finish_rotation){
            _odom_subscriber.shutdown();
            break;
        }
        ros::spinOnce();
        interval.sleep();
    }
    
    _sync_state_sub.subscribe(*(_n.get()), _states_topic, 10, ros::TransportHints().tcpNoDelay());
    _sync_odom_sub.subscribe(*(_n.get()), _odom_topic, 10, ros::TransportHints().tcpNoDelay());

    _state_odom_sync.reset(new message_filters::Synchronizer<syncStateOdomPolicy>(syncStateOdomPolicy(10), _sync_state_sub, _sync_odom_sub));
    _state_odom_sync->registerCallback(boost::bind(&DynamicPlanner::travelToWallCallback, this,_1,_2));

    while (ros::ok()) {
        if(_collided){
            _sync_state_sub.unsubscribe();
            _sync_odom_sub.unsubscribe();
            return;
        }
        ros::spinOnce();
        interval.sleep();
    }
}

void DynamicPlanner::alignWithWall(const nav_msgs::Odometry::ConstPtr &odom_msg){

    auto odom = *odom_msg;
    float yaw = tf::getYaw(odom.pose.pose.orientation)/M_PI*180.0;
    // yaw: [0, 360)
    if(yaw<0){
        yaw+=360.0;
    }

    _inverse_main_direction = _main_direction + 180;

    float main_dir_offset = compute_relative_yaw(yaw, _main_direction);
    float inv_main_dir_offset = compute_relative_yaw(yaw, _inverse_main_direction);

    if(std::abs(main_dir_offset)<=std::abs(inv_main_dir_offset)){
        _target_yaw = _main_direction;
        if(main_dir_offset >= 0){
            _robot_rotating_direction = LEFT;
        }else{
            _robot_rotating_direction = RIGHT;
        }
    }else{
        _target_yaw = _inverse_main_direction;
        if(inv_main_dir_offset >= 0){
            _robot_rotating_direction = LEFT;
        }else{
            _robot_rotating_direction = RIGHT;
        }
    }

    _finish_rotation = false;

    if(_robot_rotating_direction == LEFT){
        robotRotate(LEFT, _speed_level);
    }else if(_robot_rotating_direction == RIGHT){
        robotRotate(RIGHT, _speed_level);
    }else{
        return;
    }

    _odom_subscriber = _n->subscribe(_odom_topic, 10, &DynamicPlanner::parallelAtWallCallback, this);

    ros::Duration interval(0.001);
    while (ros::ok()) {
        if(_finish_rotation){
            _odom_subscriber.shutdown();
            return;
        }
        ros::spinOnce();
        interval.sleep();
    }
}

void DynamicPlanner::setupZigzaggingInitTurningDirection(){

    // 因为默认右沿墙，所以起始都是左转
    // 碰墙之后切换转向,朝向空闲的方向(左边)转,但是切换转向之前,是朝向墙壁(右边)的,所以这里是选择朝向墙壁的一侧
    _robot_turning_direction = RIGHT;
}

void DynamicPlanner::setupZigzaggingNewTurningDirection(){
    auto odom = ros::topic::waitForMessage<nav_msgs::Odometry>(_odom_topic, *(_n.get()));

    cv::threshold(_uncovered_areas_map,_uncovered_areas_map,128,255,cv::THRESH_BINARY);

    cv::Mat labels, stats, centroids;
    cv::connectedComponentsWithStats(_uncovered_areas_map, labels, stats, centroids);

    unsigned int robot_mx, robot_my;
    worldToMap(odom->pose.pose.position.x,odom->pose.pose.position.y,robot_mx,robot_my);

    int uncleaned_label = labels.at<int>(cv::Point(robot_mx, robot_my));

    geometry_msgs::Point robot_position;
    robot_position = odom->pose.pose.position;

    float yaw = tf::getYaw(odom->pose.pose.orientation)/M_PI*180.0;
    // yaw: [0, 360)
    if(yaw<0){
        yaw+=360.0;
    }

    int left_count = 0;
    int right_count = 0;

    unsigned int mx, my;
    float yaw_candidate;
    for(auto& uncovered_position:_uncovered_positions){
        worldToMap(uncovered_position.x, uncovered_position.y, mx, my);
        if(labels.at<int>(cv::Point(mx,my))==uncleaned_label){
            yaw_candidate = atan2((uncovered_position.y-robot_position.y),(uncovered_position.x-robot_position.x))/M_PI*180.0;
            // yaw_candidate: [0, 360)
            if(yaw_candidate<0){
                yaw_candidate+=360.0;
            }
            if(compute_relative_yaw(yaw, yaw_candidate)>=0){
                left_count++;
            }else{
                right_count++;
            }
        }
    }

    if(left_count > right_count){ // 碰撞墙后左转,碰撞墙前即是右转
        _robot_turning_direction = RIGHT;
    }else{ // 碰撞墙后右转,碰撞墙前即是左转
        _robot_turning_direction = LEFT;
    }
}

bool DynamicPlanner::queryNextWaypoint(){

    ROS_INFO("QUERY NEXT POINT");

    _uncovered_positions.clear();

    float map_resolution = getMapResolution();
    uint32_t map_width = getMapWidth();
    uint32_t map_height = getMapHeight();

    auto odom_msg = ros::topic::waitForMessage<nav_msgs::Odometry>(_odom_topic, *(_n.get()));
    nav_msgs::Odometry odom = *odom_msg;

    auto coverage_map_msg = ros::topic::waitForMessage<sensor_msgs::Image>(_coverage_map_topic, *(_n.get()));
    cv_bridge::CvImageConstPtr coverage_map_image = cv_bridge::toCvShare(coverage_map_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat coverage_map = coverage_map_image->image.clone();

    auto occupancy_map_msg = ros::topic::waitForMessage<sensor_msgs::Image>(_occupancy_map_topic, *(_n.get()));
    cv_bridge::CvImageConstPtr occupancy_map_image = cv_bridge::toCvShare(occupancy_map_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat occupancy_map = occupancy_map_image->image.clone();
    cv::cvtColor(occupancy_map,occupancy_map,cv::COLOR_BGR2GRAY);
    cv::threshold(occupancy_map, occupancy_map, 250, 255, cv::THRESH_BINARY);

    cv::Mat labels, stats, centroids;
    int connected_components = cv::connectedComponentsWithStats(occupancy_map, labels, stats, centroids);
    unsigned int robot_image_x = 0, robot_image_y = 0;
    worldToMap(odom.pose.pose.position.x, odom.pose.pose.position.y, robot_image_x, robot_image_y);
    int movable_label = labels.at<int>(cv::Point(robot_image_x, robot_image_y));

    cv::Mat distance_map = cv::Mat(map_height, map_width, CV_8UC1);
    distance_map.setTo(0);

    _boundary_map = cv::Mat::zeros(map_height, map_width, CV_8UC3);
    std::vector<std::vector<cv::Point>> boundary_contours = {_boundary_contour};
    cv::fillPoly(_boundary_map, boundary_contours, cv::Scalar(255,255,255));
    cv::fillPoly(_boundary_map, _obstacle_contours, cv::Scalar(0,0,0));

    auto robot_image_radius = static_cast<unsigned int>(std::round((_robot_radius/map_resolution)));
    unsigned int robot_image_size = 2*robot_image_radius;
    cv::Mat erode_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2*robot_image_size+1,2*robot_image_size+1), cv::Point(-1,-1)); // size: default:10
    cv::morphologyEx(_boundary_map, _boundary_map, cv::MORPH_ERODE, erode_kernel);

    cv::cvtColor(_boundary_map, _boundary_map, cv::COLOR_BGR2GRAY);
    cv::threshold(_boundary_map, _boundary_map, 128, 255, cv::THRESH_BINARY);

    cv::Mat coverage_mask = coverage_map.clone();
    cv::cvtColor(coverage_mask,coverage_mask,cv::COLOR_BGR2GRAY);
    cv::threshold(coverage_mask,coverage_mask,250,255,cv::THRESH_BINARY);

    cv::bitwise_and(coverage_mask, _boundary_map, _uncovered_areas_map);

    // test
//    cv::Mat uncleaned_areas_img = uncleaned_areas_map.clone();
//    cv::flip(uncleaned_areas_img, uncleaned_areas_img,1);
//    cv::rotate(uncleaned_areas_img,uncleaned_areas_img,cv::ROTATE_90_CLOCKWISE);
//    cv::imwrite(data_folder+"uncleaned_areas_map.jpg",uncleaned_areas_img);
    //

    for(size_t x = 0; x < _boundary_map.cols; ++x){
        for(size_t y = 0; y < _boundary_map.rows; ++y){
            if(_boundary_map.at<uchar>(cv::Point(x,y))==255){
                if(occupancy_map.at<uchar>(cv::Point(x,y))==255 && labels.at<int>(cv::Point(x,y))==movable_label){
                    distance_map.at<uchar>(cv::Point(x,y)) = 255;
                }
            }
        }
    }

    cv::Mat score_map = cv::Mat(distance_map.rows, distance_map.cols, CV_32FC1);
    cv::distanceTransform(distance_map, score_map, cv::DIST_L2, cv::DIST_MASK_PRECISE);

    std::vector<double> xs,ys;
    for(auto zone_point : _viable_zone.area.form){
        xs.emplace_back(zone_point.x);
        ys.emplace_back(zone_point.y);
    }
    std::sort(xs.begin(), xs.end(), std::greater<>());
    std::sort(ys.begin(), ys.end(), std::greater<>());
    double max_zone_x = xs.front(), min_zone_x = xs.back(), max_zone_y = ys.front(), min_zone_y = ys.back();

    unsigned int mx, my;
    double searching_unit = _robot_radius; // m
    for(double x = min_zone_x+_robot_size; x <= max_zone_x-_robot_size;){
        for(double y = min_zone_y+_robot_size; y <= max_zone_y-_robot_size;){
            worldToMap(x,y,mx,my);
            auto area_label = labels.at<int>(cv::Point(mx,my));
            if(_boundary_map.at<uchar>(cv::Point(mx,my))==255){
                if(coverage_map.at<cv::Vec3b>(cv::Point(mx,my))!=COVERAGE
                   && occupancy_map.at<uchar>(cv::Point(mx,my))==255
                   && stats.at<int>(area_label,cv::CC_STAT_AREA) > 2*(_robot_size/map_resolution)*(_robot_size/map_resolution)
                   && score_map.at<float>(cv::Point(mx,my)) > 0.5*(_robot_size/map_resolution) //最近的障碍物距离机器人都至少有半个机器人直径
                        ){
                    geometry_msgs::Point p;
                    p.x = x;
                    p.y = y;
                    p.z = 0;
                    _uncovered_positions.emplace_back(p);
                }
            }
            y+=searching_unit;
        }
        x+=searching_unit;
    }

    if(_uncovered_positions.empty()){
        ROS_INFO("NO UNVISITED POINT FOUND!");
        return false;
    }

    ROS_INFO("UNVISITED POINT FOUND!");

    odom_msg = ros::topic::waitForMessage<nav_msgs::Odometry>(_odom_topic, *(_n.get()));
    double robot_world_x, robot_world_y;
    robot_world_x = odom_msg->pose.pose.position.x;
    robot_world_y = odom_msg->pose.pose.position.y;

    // 按照距离当前位置从近到远排序
    std::sort(_uncovered_positions.begin(),
              _uncovered_positions.end(),
              [&robot_world_x,&robot_world_y](const geometry_msgs::Point& lhs, const geometry_msgs::Point& rhs){
                  return (pow((lhs.x-robot_world_x),2)+pow((lhs.y-robot_world_y),2))<(pow((rhs.x-robot_world_x),2)+pow((rhs.y-robot_world_y),2));
              }
    );
    return true;
}

void DynamicPlanner::updateNextWaypoints(){
    std::deque<geometry_msgs::Point> updated_uncovered_positions;

    auto coverage_map_msg = ros::topic::waitForMessage<sensor_msgs::Image>(_coverage_map_topic, *(_n.get()));
    cv_bridge::CvImageConstPtr coverage_map_image = cv_bridge::toCvShare(coverage_map_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat coverage_map = coverage_map_image->image.clone();

    double wx, wy;
    unsigned int mx, my;

    for(auto p:_uncovered_positions){
        wx = p.x;
        wy = p.y;
        worldToMap(wx, wy, mx, my);
        if(coverage_map.at<cv::Vec3b>(cv::Point(mx,my))!=COVERAGE){
            updated_uncovered_positions.emplace_back(p);
        }
    }

    _uncovered_positions.clear();
    _uncovered_positions = updated_uncovered_positions;
}

bool DynamicPlanner::coverNextWaypoints(){
    
    while(!_uncovered_positions.empty()){

        auto odom_msg = ros::topic::waitForMessage<nav_msgs::Odometry>(_odom_topic, *(_n.get()));
        nav_msgs::Odometry odom = *odom_msg;

        _next_uncovered_position.position.x = _uncovered_positions.front().x;
        _next_uncovered_position.position.y = _uncovered_positions.front().y;
        _next_uncovered_position.position.z = 0.0;

        auto q = tf::createQuaternionFromYaw(atan2((_next_uncovered_position.position.y-odom.pose.pose.position.y),(_next_uncovered_position.position.x-odom.pose.pose.position.x))).normalized();
        _next_uncovered_position.orientation.x = q.x();
        _next_uncovered_position.orientation.y = q.y();
        _next_uncovered_position.orientation.z = q.z();
        _next_uncovered_position.orientation.w = q.w();

        if(isPathFound(odom, _next_uncovered_position)){

            //test
//            cv::Mat boundary_canvas = boundary_map.clone();
//            cv::cvtColor(boundary_canvas,boundary_canvas,cv::COLOR_GRAY2BGR);
//            unsigned int img_x, img_y;
//            worldToMap(next_uncovered_position.position.x, next_uncovered_position.position.y, img_x, img_y);
//            cv::circle(boundary_canvas, cv::Point(img_x,img_y), 5, cv::Scalar(0,255,0), -1);
//            worldToMap(odom.pose.pose.position.x,odom.pose.pose.position.y,img_x,img_y);
//            cv::circle(boundary_canvas, cv::Point(img_x,img_y), 5, cv::Scalar(0,0,255), -1);
//            cv::flip(boundary_canvas,boundary_canvas,1);
//            cv::rotate(boundary_canvas,boundary_canvas,cv::ROTATE_90_CLOCKWISE);
//            cv::imwrite(data_folder+"uncleaned_nav.jpg", boundary_canvas);
            //


            ROS_INFO("WALK TO NEXT WAYPOINT");
            move_base_msgs::MoveBaseGoal next_goal;
            next_goal.target_pose.header.stamp = ros::Time::now();
            next_goal.target_pose.header.frame_id = _path_plan_frame;
            next_goal.target_pose.pose.position = _next_uncovered_position.position;
            next_goal.target_pose.pose.orientation = _next_uncovered_position.orientation;

            clearViableZone();
            clearBoundary();
            clearObstacles();
            moveToGoal(next_goal);
            setViableZone();
            setBoundary();
            setObstacles();

            break;
        }

        _uncovered_positions.pop_front();// 如果是无法到达的点 就不能通过update函数去除 所以在这里直接丢掉
        updateNextWaypoints();
    }
    return true;
}



