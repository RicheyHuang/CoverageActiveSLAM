#include "coverage_navigator/global_traverser.h"

std::string global_traverser::PARAM_NAME_area_radius="area_radius";
std::string global_traverser::PARAM_NAME_area_size="area_size";
std::string global_traverser::PARAM_NAME_robot_size="robot_size";
std::string global_traverser::PARAM_NAME_yaw_buffer="global_traversing/yaw_buffer";
std::string global_traverser::PARAM_NAME_yaw_tolerance="global_traversing/yaw_tolerance";
std::string global_traverser::PARAM_NAME_position_tolerance="global_traversing/position_tolerance";
std::string global_traverser::PARAM_NAME_time_tolerance="global_traversing/time_tolerance";
std::string global_traverser::PARAM_NAME_max_move_trials="global_traversing/max_move_trials";
std::string global_traverser::PARAM_NAME_high_speed="global_traversing/high_speed";
std::string global_traverser::PARAM_NAME_normal_speed="global_traversing/normal_speed";
std::string global_traverser::PARAM_NAME_low_speed="global_traversing/low_speed";
std::string global_traverser::PARAM_NAME_path_plan_frame = "global_traversing/path_plan_frame";
std::string global_traverser::PARAM_NAME_odom_topic="global_traversing/odom_topic";
std::string global_traverser::PARAM_NAME_occupancy_map_topic="global_traversing/occupancy_map_topic";
std::string global_traverser::PARAM_NAME_make_zone_service="global_traversing/make_zone_service";
std::string global_traverser::PARAM_NAME_make_obstacles_service="global_traversing/make_obstacles_service";
std::string global_traverser::PARAM_NAME_make_plan_service="global_traversing/make_plan_service";
std::string global_traverser::PARAM_NAME_clear_costmap_service="global_traversing/clear_costmap_service";
std::string global_traverser::PARAM_NAME_move_action="global_traversing/move_action";

AreaNode::AreaNode() {
    isVisited = false;
    isCovered = false;
    parentIndex = INT_MAX;
    nodeIndex = INT_MAX;
    listIndex = INT_MAX;
}

GlobalTraverser::GlobalTraverser() {}

GlobalTraverser::~GlobalTraverser() {}

void GlobalTraverser::setup() {

    _n.reset(new ros::NodeHandle("~"));

    _n->getParam(global_traverser::PARAM_NAME_area_radius,_area_radius);
    _n->getParam(global_traverser::PARAM_NAME_area_size,_area_size);
    _n->getParam(global_traverser::PARAM_NAME_robot_size,_robot_size);
    _n->getParam(global_traverser::PARAM_NAME_yaw_buffer,_yaw_buffer);
    _n->getParam(global_traverser::PARAM_NAME_yaw_tolerance,_yaw_tolerance);
    _n->getParam(global_traverser::PARAM_NAME_position_tolerance,_position_tolerance);
    _n->getParam(global_traverser::PARAM_NAME_time_tolerance,_time_tolerance);
    _n->getParam(global_traverser::PARAM_NAME_max_move_trials,_max_move_trials);
    _n->getParam(global_traverser::PARAM_NAME_high_speed,_high_speed);
    _n->getParam(global_traverser::PARAM_NAME_normal_speed,_normal_speed);
    _n->getParam(global_traverser::PARAM_NAME_low_speed,_low_speed);
    _n->getParam(global_traverser::PARAM_NAME_path_plan_frame,_path_plan_frame);
    _n->getParam(global_traverser::PARAM_NAME_odom_topic,_odom_topic);
    _n->getParam(global_traverser::PARAM_NAME_occupancy_map_topic,_occupancy_map_topic);
    _n->getParam(global_traverser::PARAM_NAME_make_zone_service,_make_zone_service);
    _n->getParam(global_traverser::PARAM_NAME_make_obstacles_service,_make_obstacles_service);
    _n->getParam(global_traverser::PARAM_NAME_make_plan_service,_make_plan_service);
    _n->getParam(global_traverser::PARAM_NAME_clear_costmap_service,_clear_costmap_service);
    _n->getParam(global_traverser::PARAM_NAME_move_action,_move_action);

    _areas.clear();
    _area_graph.clear();

    _max_area_x=0;
    _min_area_x=0;
    _max_area_y=0;
    _min_area_y=0;
    _start_rotation = false;
    _finish_rotation = false;
    _robot_rotating_direction = RIGHT;
    _speed_level = HIGH;
}

void GlobalTraverser::robotRotate(DIRECTION rotating_direction, const float& wheel_speed){
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

void GlobalTraverser::robotRotate(DIRECTION rotating_direction, SPEED_LEVEL speed_level){
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


void GlobalTraverser::walkThroughGraph(std::vector<AreaNode>& node_graph, int node_index, int& unvisited_counter, std::deque<AreaNode>& path)
{
    if(!node_graph[node_index].isVisited){
        node_graph[node_index].isVisited = true;
        unvisited_counter--;
    }
    path.emplace_front(node_graph[node_index]);

//    for debugging
//    std::cout<< "area: " <<node_graph[node_index].nodeIndex<<std::endl;
//

    AreaNode neighbor;
    int neighbor_idx = INT_MAX;

    for(int i = 0; i < node_graph[node_index].neighborIndices.size(); i++){
        neighbor = node_graph[node_graph[node_index].neighborIndices[i]];
        neighbor_idx = node_graph[node_index].neighborIndices[i];
        if(!neighbor.isVisited){
            break;
        }
    }

    // unvisited neighbor found
    if(!neighbor.isVisited){
        node_graph[neighbor_idx].parentIndex = node_graph[node_index].nodeIndex;
        walkThroughGraph(node_graph, neighbor_idx, unvisited_counter, path);
    }
        // unvisited neighbor not found
    else{
        // cannot go on back-tracking
        if (node_graph[node_index].parentIndex == INT_MAX){
            return;
        }else if(unvisited_counter == 0){
            return;
        }else{
            walkThroughGraph(node_graph, node_graph[node_index].parentIndex, unvisited_counter, path);
        }
    }
}

std::deque<int> GlobalTraverser::getTravellingPath(const std::vector<AreaNode>& node_graph, int first_node_index)
{
    std::deque<int> travelling_path;

    std::deque<AreaNode> _node_path;
    std::vector<AreaNode> _node_graph = node_graph;

    if(_node_graph.size()==1){
        travelling_path.emplace_back(0);
    }else{
        int unvisited_counter = _node_graph.size();
        walkThroughGraph(_node_graph, first_node_index, unvisited_counter, _node_path);
        std::reverse(_node_path.begin(), _node_path.end());
    }

    for(auto& node : _node_path){
        travelling_path.emplace_back(node.nodeIndex);
    }

    return travelling_path;
}

// [floor, ceiling]
void GlobalTraverser::clampImageCoordinate(int& img_coord, const unsigned int& floor, const unsigned int& ceiling){
    int _floor = static_cast<signed int>(floor);
    int _ceiling = static_cast<signed int>(ceiling);
    if(img_coord < _floor){
        img_coord = _floor;
    }
    if(img_coord > _ceiling){
        img_coord = _ceiling;
    }
}

// 原始的边界 没有扩展
void GlobalTraverser::partitionAreas(){
    ROS_INFO("START PARTITIONING");

    auto odom = ros::topic::waitForMessage<nav_msgs::Odometry>(_odom_topic, *(_n.get()));
    double origin_wx = odom->pose.pose.position.x;
    double origin_wy = odom->pose.pose.position.y;

    _areas.clear();

    unsigned int mh = getMapHeight();
    unsigned int mw = getMapWidth();
    ROS_INFO("H: %d, W: %d", mh, mw);

    unsigned int origin_mx, origin_my;
    worldToMap(origin_wx, origin_wy, origin_mx, origin_my);

    unsigned int max_mx = mw-1, min_mx = 0;
    unsigned int max_my = mh-1, min_my = 0;

    _max_area_x = (static_cast<signed int>(max_mx)-static_cast<signed int>(origin_mx)) / static_cast<signed int>(_area_size);
    _min_area_x = (static_cast<signed int>(min_mx)-static_cast<signed int>(origin_mx)) / static_cast<signed int>(_area_size);
    _max_area_y = (static_cast<signed int>(max_my)-static_cast<signed int>(origin_my)) / static_cast<signed int>(_area_size);
    _min_area_y = (static_cast<signed int>(min_my)-static_cast<signed int>(origin_my)) / static_cast<signed int>(_area_size);

    if(_max_area_x + _area_radius > max_mx){
        _max_area_x -= 1;
    }
    if(_max_area_y + _area_radius > max_my){
        _max_area_y -= 1;
    }
    if(_min_area_x - _area_radius < min_mx){
        _min_area_x += 1;
    }
    if(_min_area_y - _area_radius < min_my){
        _min_area_y += 1;
    }

    //test
    ROS_INFO("AREA X: [%d, %d]", _min_area_x, _max_area_x);
    ROS_INFO("AREA Y: [%d, %d]", _min_area_y, _max_area_y);
    //

    int list_index = 0;

    for(int x = _min_area_x; x <= _max_area_x; ++x){
        for(int y = _min_area_y; y <= _max_area_y; ++y){

            AreaNode area;
            area.isCovered = false;
            area.isVisited = false;
            cv::Point p; // x朝右, y朝上
            p.x = origin_mx + x*(_area_size) - _area_radius;
            p.y = origin_my + y*(_area_size) - _area_radius;
            clampImageCoordinate(p.x, min_mx, max_mx);
            clampImageCoordinate(p.y, min_my, max_my);
            area.areaCorners.emplace_back(p);

            p.x = origin_mx + x*(_area_size) - _area_radius;
            p.y = origin_my + y*(_area_size) + _area_radius;
            clampImageCoordinate(p.x, min_mx, max_mx);
            clampImageCoordinate(p.y, min_my, max_my);
            area.areaCorners.emplace_back(p);

            p.x = origin_mx + x*(_area_size) + _area_radius;
            p.y = origin_my + y*(_area_size) + _area_radius;
            clampImageCoordinate(p.x, min_mx, max_mx);
            clampImageCoordinate(p.y, min_my, max_my);
            area.areaCorners.emplace_back(p);

            p.x = origin_mx + x*(_area_size) + _area_radius;
            p.y = origin_my + y*(_area_size) - _area_radius;
            clampImageCoordinate(p.x, min_mx, max_mx);
            clampImageCoordinate(p.y, min_my, max_my);
            area.areaCorners.emplace_back(p);

            area.listIndex = list_index;
            _areas.emplace_back(area);

            list_index++;
        }
    }
    ROS_INFO("FINISH PARTITIONING");

    ROS_INFO("%d AREAS WERE LISTED", static_cast<int>(_areas.size()));
}

bool GlobalTraverser::updateAreaGraph(){
    ROS_INFO("STARTING UPDATING GRAPH");

    _area_graph.clear();

    auto occupancy_grid_map_msg = ros::topic::waitForMessage<sensor_msgs::Image>(_occupancy_map_topic, *(_n.get()));

    uint32_t map_width = getMapWidth();
    uint32_t map_height = getMapHeight();

    cv_bridge::CvImageConstPtr occupancy_map_image = cv_bridge::toCvShare(occupancy_grid_map_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat occupancy_map = occupancy_map_image->image.clone();
    cv::cvtColor(occupancy_map, occupancy_map, cv::COLOR_BGR2GRAY);
    cv::threshold(occupancy_map, occupancy_map, 250, 255, cv::THRESH_BINARY);

    cv::Mat labels, stats, centroids;
    int connected_components = cv::connectedComponentsWithStats(occupancy_map, labels, stats, centroids);

    unsigned int robot_image_x, robot_image_y;
    worldToMap(_origin_odom.pose.pose.position.x, _origin_odom.pose.pose.position.y, robot_image_x, robot_image_y);
    int free_space_label = labels.at<int>(cv::Point(robot_image_x, robot_image_y));


    int index = 0;
    bool jump_out = false;
    std::vector<unsigned int>xs, ys;

    for(auto& area : _areas){

        xs.clear();
        ys.clear();
        for(auto& corner:area.areaCorners){
            xs.emplace_back(corner.x);
            ys.emplace_back(corner.y);
        }
        std::sort(xs.begin(),xs.end(),std::greater<>());
        std::sort(ys.begin(),ys.end(),std::greater<>());
        unsigned int max_zone_mx = xs.front(), min_zone_mx = xs.back();
        unsigned int max_zone_my = ys.front(), min_zone_my = ys.back();

        for(int x = min_zone_mx; x <= max_zone_mx; ++x){
            if(jump_out){
                break;
            }
            for(int y = min_zone_my; y <= max_zone_my; ++y){
                if(
                        occupancy_map.at<uchar>(cv::Point(x,y)) == 255
                        && labels.at<int>(cv::Point(x, y)) == free_space_label
                        ){
//                    ROS_INFO("ADDING GRAPH NODE");
                    _area_graph.emplace_back(area);
                    _area_graph.back().nodeIndex = index;
                    index++;
                    jump_out = true;
                    break;
                }
            }
        }
        jump_out = false;
    }

    ROS_INFO("FINISH UPDATING GRAPH");

    ROS_INFO("%d AREA NODES WERE CONSTRUCTED", static_cast<int>(_area_graph.size()));


    // test
//        occupancy_grid_map_msg = ros::topic::waitForMessage<sensor_msgs::Image>(_occupancy_map_topic, _n);
//        occupancy_grid_map_cv_image_ptr = cv_bridge::toCvShare(occupancy_grid_map_msg, sensor_msgs::image_encodings::BGR8);
//        occupancy_grid_map_cv_mat = occupancy_grid_map_cv_image_ptr->image.clone();
//        if(!area_graph.empty()){
//            for(auto& area:area_graph){
//                auto roi_corners = area.areaCorners;
//                auto upleft = roi_corners[0];
//                auto downright = roi_corners[2];
//                if(areas[area.listIndex].iscovered){
//                    cv::rectangle(occupancy_grid_map_cv_mat, upleft, downright, cv::Scalar(0,0,255)); //扫过为红色
//                }else{
//                    cv::rectangle(occupancy_grid_map_cv_mat, upleft, downright, cv::Scalar(255,0,0)); //未扫过为蓝色
//                }
//            }
//        }
//
//        cv::flip(occupancy_grid_map_cv_mat, occupancy_grid_map_cv_mat,1);
//        cv::rotate(occupancy_grid_map_cv_mat,occupancy_grid_map_cv_mat,cv::ROTATE_90_CLOCKWISE);
//        cv::imwrite(data_folder+"area_map.jpg",occupancy_grid_map_cv_mat);
    //

    return (!_area_graph.empty());
}

// 判断两个区域是否可以互通
bool GlobalTraverser::areasConnected(const int& area_graph_idx1, const int& area_graph_idx2){

    auto occupancy_grid_map_msg = ros::topic::waitForMessage<sensor_msgs::Image>(_occupancy_map_topic, *(_n.get()));
    cv_bridge::CvImageConstPtr occupancy_map_image = cv_bridge::toCvShare(occupancy_grid_map_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat occupancy_map = occupancy_map_image->image.clone();
    cv::cvtColor(occupancy_map,occupancy_map,cv::COLOR_BGR2GRAY);
    cv::threshold(occupancy_map, occupancy_map, 250, 255, cv::THRESH_BINARY);

    std::vector<unsigned int> xs1, ys1, xs2, ys2;
    for(size_t i = 0; i < 4; ++i){
        xs1.emplace_back(_area_graph[area_graph_idx1].areaCorners[i].x);
        ys1.emplace_back(_area_graph[area_graph_idx1].areaCorners[i].y);
        xs2.emplace_back(_area_graph[area_graph_idx2].areaCorners[i].x);
        ys2.emplace_back(_area_graph[area_graph_idx2].areaCorners[i].y);
    }
    std::sort(xs1.begin(),xs1.end(),std::greater<>());
    std::sort(ys1.begin(),ys1.end(),std::greater<>());
    std::sort(xs2.begin(),xs2.end(),std::greater<>());
    std::sort(ys2.begin(),ys2.end(),std::greater<>());
    unsigned int max_x1=xs1.front(), min_x1=xs1.back(), max_y1=ys1.front(), min_y1=ys1.back();
    unsigned int max_x2=xs2.front(), min_x2=xs2.back(), max_y2=ys2.front(), min_y2=ys2.back();

    cv::Mat image = cv::Mat::zeros(occupancy_map.rows, occupancy_map.cols, CV_8UC1);
    for(size_t x = 0; x < occupancy_map.cols; ++x){
        for(size_t y = 0; y < occupancy_map.rows; ++y){
            if((x>=min_x1 && x<=max_x1 && y>=min_y1 && y<=max_y1)
               ||(x>=min_x2 && x<=max_x2 && y>=min_y2 && y<=max_y2)){
                image.at<uchar>(cv::Point(x,y)) = occupancy_map.at<uchar>(cv::Point(x,y));
            }
        }
    }

    cv::Mat labels, stats, centroids;
    int connected_components = cv::connectedComponentsWithStats(image, labels, stats, centroids);

    bool is_connected = false;

    std::set<int> labels1, labels2;
    for(size_t x = min_x1; x <= max_x1; ++x){
        for(size_t y = min_y1; y <= max_y1; ++y){
            if(occupancy_map.at<uchar>(cv::Point(x,y))==255){
                labels1.insert(labels.at<int>(cv::Point(x,y)));
            }
        }
    }

    for(size_t x = min_x2; x <= max_x2; ++x){
        for(size_t y = min_y2; y <= max_y2; ++y){
            if(occupancy_map.at<uchar>(cv::Point(x,y))==255){
                labels2.insert(labels.at<int>(cv::Point(x,y)));
            }
        }
    }

    for(auto& label1:labels1){
        if(labels2.find(label1)!=labels2.end()){
            is_connected = true;
            break;
        }
    }

    return is_connected;
}

// 判断两个区域是否相邻
bool GlobalTraverser::areasAdjacent(const int& area_graph_idx1, const int& area_graph_idx2){

    int list_idx1 = _area_graph[area_graph_idx1].listIndex;
    int list_idx2 = _area_graph[area_graph_idx2].listIndex;

    int x_idx1 = list_idx1/(_max_area_y-_min_area_y+1);
    int x_idx2 = list_idx2/(_max_area_y-_min_area_y+1);
    int y_idx1 = list_idx1%(_max_area_y-_min_area_y+1);
    int y_idx2 = list_idx2%(_max_area_y-_min_area_y+1);

//    if((std::abs(x_idx1-x_idx2)<=1&&(std::abs(y_idx1-y_idx2)<=1))
//       ||(std::abs(y_idx1-y_idx2)<=1&&(std::abs(x_idx1-x_idx2)<=1))){
//        return true;
//    }else{
//        return false;
//    }

    if((std::abs(x_idx1-x_idx2)<=1&&(y_idx1==y_idx2))
       ||(std::abs(y_idx1-y_idx2)<=1&&(x_idx1==x_idx2))){
        return true;
    }else{
        return false;
    }
}

bool GlobalTraverser::updateGraphConnectivity(){
    ROS_INFO("STARTING UPDATING CONNECTIVITY");

    if(_area_graph.empty()){
        return false;
    }else if(_area_graph.size()==1){
        return true;
    }

    for(auto& area_node : _area_graph){
        area_node.neighborIndices.clear();
    }

    for(int i = 0; i < _area_graph.size()-1; ++i){
        for(int j = i + 1; j < _area_graph.size(); ++j){

            if(areasAdjacent(i, j)){
//                ROS_INFO("AREA %d AND %d ARE ADJACENT", i, j);
                _area_graph[i].neighborIndices.emplace_back(j);
                _area_graph[j].neighborIndices.emplace_back(i);
            }else{
//                ROS_INFO("AREA %d AND %d AREN'T ADJACENT", i, j);
            }

//            int list_idx1 = area_graph[i].listIndex;
//            int list_idx2 = area_graph[j].listIndex;

//            int x_idx1 = list_idx1/(_max_area_y-_min_area_y+1);
//            int x_idx2 = list_idx2/(_max_area_y-_min_area_y+1);
//            int y_idx1 = list_idx1%(_max_area_y-_min_area_y+1);
//            int y_idx2 = list_idx2%(_max_area_y-_min_area_y+1);

//            if((std::abs(x_idx1-x_idx2)<=1&&(std::abs(y_idx1-y_idx2)<=1))
//            ||(std::abs(y_idx1-y_idx2)<=1&&(std::abs(x_idx1-x_idx2)<=1))){
//                if(areas_connected(i, j)){
//                    ROS_INFO("AREA %d and %d ARE CONNECTED", i, j);
//                    area_graph[i].neighbor_indices.emplace_back(j);
//                    area_graph[j].neighbor_indices.emplace_back(i);
//                }else{
//                    ROS_INFO("AREA %d and %d ARE DISCONNECTED", i, j);
//                }
//            }else{
//                ROS_INFO("AREA %d and %d ARE DISCONNECTED", i, j);
//            }
        }
    }

    // 遍历graph 对邻居排序 未扫过的在前
    for(auto& area:_area_graph){
        std::sort(area.neighborIndices.begin(), area.neighborIndices.end(), [this](const int& lhs, const int& rhs){
            bool l_covered = _areas[_area_graph[lhs].listIndex].isCovered;
            bool r_covered = _areas[_area_graph[rhs].listIndex].isCovered;
            if(!l_covered && r_covered){
                return true;
            }else if(l_covered && !r_covered){
                return false;
            }else{
                return lhs<rhs;
            }
        });
    }

    ROS_INFO("FINISH UPDATING CONNECTIVITY");

    return true;
}

// 以上的函数都是使用未扩展的area信息

void GlobalTraverser::clearViableZone(){
    ros::ServiceClient sc = _n->serviceClient<custom_srvs::Zone>(_make_zone_service);

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
    custom_msgs::Zone viable_zone;
    viable_zone.area.form.emplace_back(w_top_left);
    viable_zone.area.form.emplace_back(w_bottom_left);
    viable_zone.area.form.emplace_back(w_bottom_right);
    viable_zone.area.form.emplace_back(w_top_right);

    custom_srvs::Zone zone_srv;
    zone_srv.request.area = viable_zone.area;

    sc.call(zone_srv);
}

// 设置虚拟墙时要扩大边界
void GlobalTraverser::setViableZone(const int& area_graph_idx){

    ros::ServiceClient sc = _n->serviceClient<custom_srvs::Zone>(_make_zone_service);

    double resolution = getMapResolution();

    unsigned int max_my = getMapHeight()-1;
    unsigned int max_mx = getMapWidth()-1;
    unsigned int min_my = 0, min_mx = 0;

    custom_msgs::Zone viable_zone;
    auto corners = _area_graph[area_graph_idx].areaCorners; //pix
    std::vector<unsigned int>xs, ys;
    for(auto& corner:corners){
        xs.emplace_back(corner.x);
        ys.emplace_back(corner.y);
    }
    std::sort(xs.begin(),xs.end(),std::greater<>());
    std::sort(ys.begin(),ys.end(),std::greater<>());
    unsigned int max_zone_mx = xs.front(), min_zone_mx = xs.back();
    unsigned int max_zone_my = ys.front(), min_zone_my = ys.back();

    unsigned int robot_image_size = int(std::ceil(_robot_size/resolution));

    max_zone_mx = (max_zone_mx + robot_image_size) > max_mx ? max_mx : (max_zone_mx + robot_image_size);
    min_zone_mx = (min_zone_mx - robot_image_size) < min_mx ? min_mx : (min_zone_mx - robot_image_size);
    max_zone_my = (max_zone_my + robot_image_size) > max_my ? max_my : (max_zone_my + robot_image_size);
    min_zone_my = (min_zone_my - robot_image_size) < min_my ? min_my : (min_zone_my - robot_image_size);

    corners.clear();
    corners = {cv::Point(max_zone_mx,min_zone_my),
               cv::Point(max_zone_mx,max_zone_my),
               cv::Point(min_zone_mx,max_zone_my),
               cv::Point(min_zone_mx,min_zone_my)};

    double wx, wy;
    geometry_msgs::Point p;
    for(auto& corner:corners){
        mapToWorld(corner.x, corner.y, wx, wy);
        p.x = wx;
        p.y = wy;
        p.z = 0.0;
        viable_zone.area.form.emplace_back(p);
    }

    custom_srvs::Zone zone_srv;
    zone_srv.request.area = viable_zone.area;

    sc.call(zone_srv);
}

void GlobalTraverser::clearObstacles(){

    ros::ServiceClient sc = _n->serviceClient<custom_srvs::Obstacle>(_make_obstacles_service);

    double corner_wx, corner_wy; // 角落上的点
    mapToWorld(0, 0, corner_wx, corner_wy);

    geometry_msgs::Point corner;
    corner.x = corner_wx;
    corner.y = corner_wy;
    corner.z = 0.0;

    custom_msgs::Obstacles obstacles;
    custom_msgs::Form obstacle;
    obstacle.form.push_back(corner);
    obstacles.list.push_back(obstacle);

    custom_srvs::Obstacle obstacle_srv;
    obstacle_srv.request.list = obstacles.list;

    sc.call(obstacle_srv);
}

void GlobalTraverser::updateCoveredArea(const int& area_graph_idx){
    int area_list_idx = _area_graph[area_graph_idx].listIndex;
    _areas[area_list_idx].isCovered = _area_graph[area_graph_idx].isCovered;
}

// 使用未扩展的area进行判断
int GlobalTraverser::getRobotsAreaGraphIndex(){

    auto odom_msg = ros::topic::waitForMessage<nav_msgs::Odometry>(_odom_topic, *(_n.get()));
    nav_msgs::Odometry odom = *odom_msg;

    unsigned int robot_image_x, robot_image_y;
    worldToMap(odom.pose.pose.position.x, odom.pose.pose.position.y, robot_image_x, robot_image_y);

    int max_mx, min_mx, max_my, min_my;
    int graph_idx = INT_MAX;
    std::vector<unsigned int>xs, ys;

    for(int idx = 0; idx < _area_graph.size(); ++idx){

        xs.clear();
        ys.clear();
        for(auto& corner:_area_graph[idx].areaCorners){
            xs.emplace_back(corner.x);
            ys.emplace_back(corner.y);
        }
        std::sort(xs.begin(),xs.end(),std::greater<>());
        std::sort(ys.begin(),ys.end(),std::greater<>());
        max_mx = xs.front(), min_mx = xs.back();
        max_my = ys.front(), min_my = ys.back();

        // test
//        std::cout<<"robot x: "<<robot_image_x<<", y: "<<robot_image_y<<std::endl;
//        std::cout<<"x: ["<<min_mx<<", "<<max_mx<<"]"<<std::endl;
//        std::cout<<"y: ["<<min_my<<", "<<max_my<<"]"<<std::endl;
        //

        if(robot_image_x >= min_mx && robot_image_x <= max_mx &&
           robot_image_y >= min_my && robot_image_y <= max_my)
        {
            graph_idx = idx;
            break;
        }
    }
    return graph_idx;
}

unsigned int GlobalTraverser::countCoveredAreas(){
    if(_area_graph.empty()){
        return 0;
    }

    unsigned int num_covered_areas = 0;
    for(auto& area:_area_graph){
        if(area.isCovered){
            num_covered_areas++;
        }
    }
    return num_covered_areas;
}

bool GlobalTraverser::isAllCovered(){
    bool is_finished = true;
    for(auto& area:_area_graph){
        if(!area.isCovered){
            is_finished = false;
            return is_finished;
        }
    }
    return is_finished;
}

bool GlobalTraverser::isPathCovered(const std::deque<int>& path){
    bool is_finished = true;
    for(auto& area_graph_idx : path){

        if(!_area_graph[area_graph_idx].isCovered){
            is_finished = false;
            return is_finished;
        }
    }
    return is_finished;
}

void GlobalTraverser::aimAtGoalCallback(const nav_msgs::Odometry::ConstPtr& odom_msg){

    robotRotate(_robot_rotating_direction, _speed_level);

    auto odom = *odom_msg;
    float yaw = tf::getYaw(odom.pose.pose.orientation)/M_PI*180.0;
    // yaw: [0, 360)
    if(yaw<0){
        yaw+=360.0;
    }

    float goal_yaw = tf::getYaw(_next_uncovered_position.orientation)/M_PI*180.0;
    // goal_yaw: [0, 360)
    if(goal_yaw<0){
        goal_yaw+=360.0;
    }

    float yaw_offset = compute_relative_yaw(goal_yaw, yaw);


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

void GlobalTraverser::aimAtGoal(const geometry_msgs::Pose& next_goal){

    _next_uncovered_position.orientation = next_goal.orientation;

    auto odom_msg = ros::topic::waitForMessage<nav_msgs::Odometry>(_odom_topic, *(_n.get()));
    auto odom = *odom_msg;
    float yaw = tf::getYaw(odom.pose.pose.orientation)/M_PI*180.0;
    // yaw: [0, 360)
    if(yaw<0){
        yaw+=360.0;
    }
    float goal_yaw = tf::getYaw(_next_uncovered_position.orientation)/M_PI*180.0;
    // goal_yaw: [0, 360)
    if(goal_yaw<0){
        goal_yaw+=360.0;
    }
    // goal_yaw-yaw
    float yaw_offset = compute_relative_yaw(yaw, goal_yaw);
    if(yaw_offset >= 0){
        _robot_rotating_direction = LEFT;
    }else{
        _robot_rotating_direction = RIGHT;
    }
    robotRotate(_robot_rotating_direction, _speed_level);

    _odom_subscriber = _n->subscribe(_odom_topic, 10, &GlobalTraverser::aimAtGoalCallback, this);
    ros::Duration interval(0.001);
    while (ros::ok()) {
        if(_finish_rotation){
            _odom_subscriber.shutdown();
            _finish_rotation = false;
            break;
        }
        ros::spinOnce();
        interval.sleep();
    }

}

// 清扫时也要扩大边界 以防漏掉公共的原始边界
bool GlobalTraverser::coverArea(const int& area_graph_idx){
    if(getRobotsAreaGraphIndex() != area_graph_idx){
        return false;
    }

    double resolution = getMapResolution();

    unsigned int max_my = getMapHeight()-1;
    unsigned int max_mx = getMapWidth()-1;
    unsigned int min_my = 0, min_mx = 0;

    custom_msgs::Zone viable_zone;
    auto corners = _area_graph[area_graph_idx].areaCorners; //pix
    std::vector<unsigned int>xs, ys;
    for(auto& corner:corners){
        xs.emplace_back(corner.x);
        ys.emplace_back(corner.y);
    }
    std::sort(xs.begin(),xs.end(),std::greater<>());
    std::sort(ys.begin(),ys.end(),std::greater<>());
    unsigned int original_max_zone_mx = xs.front(), original_min_zone_mx = xs.back();
    unsigned int original_max_zone_my = ys.front(), original_min_zone_my = ys.back();

    unsigned int robot_image_size = int(std::ceil(_robot_size/resolution));

    unsigned int max_zone_mx = (original_max_zone_mx + robot_image_size) > max_mx ? max_mx : (original_max_zone_mx + robot_image_size);
    unsigned int min_zone_mx = (original_min_zone_mx - robot_image_size) < min_mx ? min_mx : (original_min_zone_mx - robot_image_size);
    unsigned int max_zone_my = (original_max_zone_my + robot_image_size) > max_my ? max_my : (original_max_zone_my + robot_image_size);
    unsigned int min_zone_my = (original_min_zone_my - robot_image_size) < min_my ? min_my : (original_min_zone_my - robot_image_size);

    corners.clear();
    corners = {cv::Point(max_zone_mx,min_zone_my),
               cv::Point(max_zone_mx,max_zone_my),
               cv::Point(min_zone_mx,max_zone_my),
               cv::Point(min_zone_mx,min_zone_my)};

    double wx, wy;
    geometry_msgs::Point p;
    for(auto& corner:corners){
        mapToWorld(corner.x, corner.y, wx, wy);
        p.x = wx;
        p.y = wy;
        p.z = 0.0;
        viable_zone.area.form.emplace_back(p);
    }

    DynamicPlanner dynamic_planner;
    dynamic_planner.robotPowerOn();
    dynamic_planner.setup();
    dynamic_planner.run(viable_zone);
    dynamic_planner.shutDown();
    dynamic_planner.robotPowerOff();

    return true;
}

bool GlobalTraverser::isPathFound(const nav_msgs::Odometry& start_odom, const geometry_msgs::Pose& target_pose){

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
            ROS_INFO("MAKE GLOBAL PLAN SUCCESSFULLY.");
            return true;
        }else{
            ROS_INFO("MAKE EMPTY GLOBAL PLAN");
            return false;
        }
    }else{
        ROS_INFO("MAKE GLOBAL PLAN FAILED");
        return false;
    }
}

bool GlobalTraverser::getOutOfStuck(const geometry_msgs::Pose& goal_pose){

    bool is_escape_succeed = false;

    StuckEscaper escaper;
    escaper.robotPowerOn();
    escaper.setup();
    is_escape_succeed = escaper.run(goal_pose);
    escaper.shutDown();
    escaper.robotPowerOff();

    return is_escape_succeed;
}

MOVE_STATE GlobalTraverser::moveToArea(const int& area_graph_idx){

    auto odom_msg = ros::topic::waitForMessage<nav_msgs::Odometry>(_odom_topic, *(_n.get()));
    nav_msgs::Odometry odom = *odom_msg;

    float map_resolution = getMapResolution();
    uint32_t map_width = getMapWidth();
    uint32_t map_height = getMapHeight();

    auto occupancy_map_msg = ros::topic::waitForMessage<sensor_msgs::Image>(_occupancy_map_topic, *(_n.get()));
    cv_bridge::CvImageConstPtr occupancy_map_image = cv_bridge::toCvShare(occupancy_map_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat occupancy_map = occupancy_map_image->image.clone();
    cv::cvtColor(occupancy_map,occupancy_map,cv::COLOR_BGR2GRAY);
    cv::threshold(occupancy_map, occupancy_map, 250, 255, cv::THRESH_BINARY);

//    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(robot_size/global_costmap_resolution,robot_size/global_costmap_resolution), cv::Point(-1,-1));
//    cv::erode(occupancy_map_cv_mat, occupancy_map_cv_mat, kernel);

    cv::Mat labels, stats, centroids;
    int connected_components = cv::connectedComponentsWithStats(occupancy_map, labels, stats, centroids);

    std::vector<cv::Point> area_image_points = _area_graph[area_graph_idx].areaCorners;
    std::vector<unsigned int> xs,ys;
    for(auto area_image_point : area_image_points){ // pix
        xs.emplace_back(area_image_point.x);
        ys.emplace_back(area_image_point.y);
    }
    std::sort(xs.begin(), xs.end(), std::greater<>());
    std::sort(ys.begin(), ys.end(), std::greater<>());

    unsigned int robot_image_x = 0, robot_image_y = 0;
    double robot_world_x = 0, robot_world_y = 0;

    worldToMap(odom.pose.pose.position.x, odom.pose.pose.position.y, robot_image_x, robot_image_y);
    int movable_label = labels.at<int>(cv::Point(robot_image_x, robot_image_y));


    unsigned int _max_area_x = xs.front(), _min_area_x = xs.back(), _max_area_y = ys.front(), _min_area_y = ys.back();

    cv::Mat distance_map = cv::Mat(map_height, map_width, CV_8UC1);
    distance_map.setTo(0);

    std::deque<cv::Point> movable_points;
    //TODO: 对于可规划路径的各个连通区域的中点都遍历一遍
    for(size_t x = _min_area_x; x <= _max_area_x; ++x){
        for(size_t y = _min_area_y; y <= _max_area_y; ++y){
            if(occupancy_map.at<uchar>(cv::Point(x,y))==255 && labels.at<int>(cv::Point(x,y))==movable_label){
                distance_map.at<uchar>(cv::Point(x,y)) = 255;
                movable_points.emplace_back(cv::Point(x,y));
            }
        }
    }

    if(movable_points.empty()){
        return NO_MOVABLE_POINT;
    }

    cv::Mat score_map = cv::Mat(distance_map.rows, distance_map.cols, CV_32FC1);
    cv::distanceTransform(distance_map, score_map, cv::DIST_L2, cv::DIST_MASK_PRECISE);


    //test
//    cv::Mat score_vizmap;
//    score_vizmap = cv::Mat::zeros(score_map.size(), CV_8UC1);
//    for (int i = 0; i<score_map.rows; i++)
//    {
//        for (int j = 0; j<score_map.cols; j++)
//        {
//            score_vizmap.at<uchar>(i, j) = score_map.at<float>(i, j);
//        }
//    }
//    cv::normalize(score_vizmap, score_vizmap, 0, 255, CV_MINMAX); //为了显示清晰，做了0~255归一
    //

    std::sort(movable_points.begin(), movable_points.end(), [&score_map](const cv::Point& lhs, const cv::Point& rhs){
        float left_score = score_map.at<float>(lhs);
        float right_score = score_map.at<float>(rhs);
        return left_score > right_score;
    });

    auto best_score_point = movable_points.front();
    auto best_score = score_map.at<float>(best_score_point); // L2 pix distance
    if(best_score <= 1.5*(_robot_size/map_resolution)){ // 只有当最近的障碍物距离机器人都至少有1.5个机器人直径那么远时 才走过去
        return NO_MOVABLE_POINT;
    }

    double goal_wx, goal_wy;
    geometry_msgs::Pose goal_pose;
    bool found_goal = false;
    for(auto& movable_point:movable_points){
        mapToWorld(movable_point.x,movable_point.y,goal_wx,goal_wy);
        goal_pose.position.x=goal_wx;
        goal_pose.position.y=goal_wy;
        goal_pose.position.z = 0.0;
        auto quat = tf::createQuaternionFromYaw(atan2((goal_pose.position.y-odom.pose.pose.position.y),(goal_pose.position.x-odom.pose.pose.position.x))).normalized();
        goal_pose.orientation.x = quat.x();
        goal_pose.orientation.y = quat.y();
        goal_pose.orientation.z = quat.z();
        goal_pose.orientation.w = quat.w();

        found_goal = isPathFound(odom, goal_pose);
        if(found_goal){
            robot_image_x = movable_point.x;
            robot_image_y = movable_point.y;
            break;
        }
    }

    if(!found_goal){
        return NO_MOVABLE_POINT;
    }

    geometry_msgs::Pose target_pose;
    mapToWorld(robot_image_x,robot_image_y,robot_world_x,robot_world_y);

    //test
//        unsigned int start_image_x, start_image_y;
//        cv::cvtColor(score_vizmap, score_vizmap, cv::COLOR_GRAY2BGR);
//        worldToMap(odom.pose.pose.position.x, odom.pose.pose.position.y, start_image_x, start_image_y);
//        cv::circle(score_vizmap, cv::Point(robot_image_x,robot_image_y), 5, cv::Scalar(0,255,0), -1);
//        cv::circle(score_vizmap, cv::Point(start_image_x,start_image_y), 5, cv::Scalar(0,0,255), -1);
//        cv::flip(score_vizmap,score_vizmap,1);
//        cv::rotate(score_vizmap,score_vizmap,cv::ROTATE_90_CLOCKWISE);
//        cv::imwrite(data_folder+"nextzone_nav.jpg", score_vizmap);
    //

    target_pose.position.x = robot_world_x;
    target_pose.position.y = robot_world_y;
    target_pose.position.z = 0.0;

    auto q = tf::createQuaternionFromYaw(atan2((target_pose.position.y-odom.pose.pose.position.y),(target_pose.position.x-odom.pose.pose.position.x))).normalized();
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();

    _ac.reset(new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(_move_action, true)); // global namespace
    _ac->waitForServer();

    ROS_INFO("MOVE TO NEXT AREA %d", area_graph_idx);
    move_base_msgs::MoveBaseGoal next_goal;
    next_goal.target_pose.header.stamp = ros::Time::now();
    next_goal.target_pose.header.frame_id = _path_plan_frame;
    next_goal.target_pose.pose.position = target_pose.position;
    next_goal.target_pose.pose.orientation = target_pose.orientation;

    double start_sec=0, now_sec=0;
    nav_msgs::Odometry::ConstPtr start_odom, now_odom;

    while(true){
        _ac->waitForServer();
        _ac->sendGoal(next_goal);
        start_sec = ros::Time::now().toSec();
        start_odom = ros::topic::waitForMessage<nav_msgs::Odometry>(_odom_topic, *(_n.get()));
        while (true) {
            now_sec = ros::Time::now().toSec();
            now_odom = ros::topic::waitForMessage<nav_msgs::Odometry>(_odom_topic, *(_n.get()));
            auto goal_dist = compute_position_error(now_odom->pose.pose, target_pose);
            auto nav_dist = compute_position_error(now_odom->pose.pose, start_odom->pose.pose);
            if(_ac->getState()==actionlib::SimpleClientGoalState::SUCCEEDED){
                _ac->cancelAllGoals();
                return MOVE_SUCCESS;
            }
            if(std::abs(now_sec-start_sec)>_time_tolerance){
//                ROS_INFO("DURING: %f, NAV DIST: %f, GOAL DIST: %f \n", std::abs(now_sec-start_sec), nav_dist, goal_dist);
                if(nav_dist < _position_tolerance){
                    _ac->cancelAllGoals();
                    if(!getOutOfStuck(target_pose)){
                        robotStop();
                        return MOVE_FAILED;
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

void GlobalTraverser::printPath(const std::deque<int>& path){
    std::cout<<"START->";
    for(auto& idx : path){
        std::cout<<idx<<"->";
    }
    std::cout<<"END"<<std::endl;
}

void GlobalTraverser::clearCostmap(){
    ros::ServiceClient clr_client = _n->serviceClient<std_srvs::Empty>(_clear_costmap_service);
    std_srvs::Empty clr_srv;
    clr_client.call(clr_srv);
}

void GlobalTraverser::run() {

    auto origin_odom_msg = ros::topic::waitForMessage<nav_msgs::Odometry>(_odom_topic, *(_n.get()));
    _origin_odom = *origin_odom_msg;

    clearCostmap();
    clearViableZone();
    clearObstacles();
    partitionAreas();
    if(!updateAreaGraph()){
        ROS_INFO("FINISH COVERAGE");
        return;
    }
    if(!updateGraphConnectivity()){
        ROS_INFO("FINISH COVERAGE");
        return;
    }

    int robot_area_graph_idx = getRobotsAreaGraphIndex();
    ROS_INFO("ROBOT STARTS IN AREA %d", robot_area_graph_idx);

    std::deque<int> path;
    if(_area_graph.size()>1){
        for(auto& area:_area_graph){
            std::cout<<"AREA "<<area.nodeIndex<<"'S NEIGHBORS: ";
            for(auto& neighbor:area.neighborIndices){
                std::cout<<neighbor<<"   ";
            }
            std::cout<<std::endl;
        }
        path = getTravellingPath(_area_graph, robot_area_graph_idx);
    }else if(_area_graph.size()==1){
        path.emplace_back(robot_area_graph_idx);
    }else{
        ROS_INFO("NO AREA AVAILABLE");
    }

    printPath(path);

    bool jump_out = false;
    int trials_to_move = 0;

    unsigned int updated_num_covered_areas = 0;
    unsigned int num_covered_areas = countCoveredAreas();

    ros::Duration interval(5);
    while(true){
        if(_area_graph.empty()){
            break;
        }else if(num_covered_areas == _area_graph.size()){
            break;
        }else{
            for(auto& area_graph_idx : path){
                clearViableZone();
                if(!_area_graph[area_graph_idx].isCovered){
                    if(getRobotsAreaGraphIndex() == area_graph_idx){
                        setViableZone(area_graph_idx);
                    }else{
                        jump_out = false;
                        trials_to_move = 0;
                        while(true){
                            if(trials_to_move > _max_move_trials){
                                ROS_INFO("GIVE UP TRYING TO MOVE! ");
                                jump_out = true;
                                break;
                            }
                            clearCostmap();
                            auto move_result = moveToArea(area_graph_idx);
                            trials_to_move++;
                            if(move_result==NO_MOVABLE_POINT){
                                ROS_INFO("NO MORE MOVABLE POINT! ");
                                jump_out = true;
                                break;
                            }else{
                                if(getRobotsAreaGraphIndex() != area_graph_idx){
                                    ROS_INFO("ROBOT HASN'T MOVED INTO AREA YET! ");
                                }else{
                                    ROS_INFO("ROBOT HAS MOVED INTO AREA ALREADY! ");
                                    setViableZone(area_graph_idx);
                                    break;
                                }
                            }
                        }
                        if(jump_out){
                            continue;
                        }
                    }

                    interval.sleep();
                    // 更新地图后再移动 使机器人更接近viable zone中心
//                clear_viable_zone();
//                move_to_area(area_graph_idx);
//                set_viable_zone(area_graph_idx);

                    if(!coverArea(area_graph_idx)){
                        break;
                    }

                    // 移动回中心 避免陷在墙边
//                clear_viable_zone();
//                move_to_area(area_graph_idx);
//                set_viable_zone(area_graph_idx);

                    _area_graph[area_graph_idx].isCovered = true;
                    updateCoveredArea(area_graph_idx);

                    break;
                }
            }
            if(!updateAreaGraph()){
                ROS_INFO("FINISH COVERAGE");
                return;
            }
            if(!updateGraphConnectivity()){
                ROS_INFO("FINISH COVERAGE");
                return;
            }

            robot_area_graph_idx = getRobotsAreaGraphIndex();
            path.clear();
            if(_area_graph.size()>1){
                for(auto& area:_area_graph){
                    std::cout<<"AREA "<<area.nodeIndex<<"'S NEIGHBORS: ";
                    for(auto& neighbor:area.neighborIndices){
                        std::cout<<neighbor<<"   ";
                    }
                    std::cout<<std::endl;
                }
                path = getTravellingPath(_area_graph, robot_area_graph_idx);
            }else if(_area_graph.size()==1){
                path.emplace_back(robot_area_graph_idx);
            }else{
                ROS_INFO("NO AREA AVAILABLE");
            }

            printPath(path);

            updated_num_covered_areas = countCoveredAreas() - num_covered_areas;
            if(updated_num_covered_areas == 0){
                break;
            }else{
                num_covered_areas = countCoveredAreas();
            }
        }
    }

    ROS_INFO("FINISH COVERAGE");

}

void GlobalTraverser::shutDown() {
    _ac->cancelAllGoals();
    _odom_subscriber.shutdown();
}