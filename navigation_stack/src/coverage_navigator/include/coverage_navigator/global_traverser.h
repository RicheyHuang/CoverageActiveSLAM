#ifndef COVERAGE_NAVIGATOR_GLOBAL_TRAVERSER_H
#define COVERAGE_NAVIGATOR_GLOBAL_TRAVERSER_H

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "cv_bridge/cv_bridge.h"
#include "move_base/move_base.h"
#include "actionlib/client/simple_action_client.h"

#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Image.h"
#include "custom_msgs/Zone.h"
#include "custom_msgs/Obstacles.h"

#include "std_srvs/Empty.h"
#include "custom_srvs/Zone.h"
#include "custom_srvs/Obstacle.h"

#include "move_base_msgs/MoveBaseGoal.h"
#include "move_base_msgs/MoveBaseAction.h"

#include "coverage_navigator/utilities.h"
#include "coverage_navigator/dynamic_planner.h"
#include "coverage_navigator/stuck_escaper.h"

#include "opencv2/opencv.hpp"


namespace global_traverser{

    extern std::string PARAM_NAME_area_radius;
    extern std::string PARAM_NAME_area_size;
    extern std::string PARAM_NAME_robot_size;
    extern std::string PARAM_NAME_yaw_buffer;
    extern std::string PARAM_NAME_yaw_tolerance;
    extern std::string PARAM_NAME_position_tolerance;
    extern std::string PARAM_NAME_time_tolerance;
    extern std::string PARAM_NAME_max_move_trials;
    extern std::string PARAM_NAME_high_speed;
    extern std::string PARAM_NAME_normal_speed;
    extern std::string PARAM_NAME_low_speed;
    extern std::string PARAM_NAME_path_plan_frame;
    extern std::string PARAM_NAME_odom_topic;
    extern std::string PARAM_NAME_occupancy_map_topic;
    extern std::string PARAM_NAME_make_zone_service;
    extern std::string PARAM_NAME_make_obstacles_service;
    extern std::string PARAM_NAME_make_plan_service;
    extern std::string PARAM_NAME_clear_costmap_service;
    extern std::string PARAM_NAME_move_action;

}

enum MOVE_STATE{
    NO_MOVABLE_POINT,
    MOVE_SUCCESS,
    MOVE_FAILED
};

class AreaNode{
public:
    AreaNode();

    bool isVisited;
    bool isCovered;

    int parentIndex;
    std::deque<int> neighborIndices;
    std::vector<cv::Point> areaCorners; // four points in clockwise order: --, -+, ++, +-

    int nodeIndex; // 在graph中的index
    int listIndex; // 在list中的index
};

class GlobalTraverser : public Robot{
public:

    GlobalTraverser();
    ~GlobalTraverser();

    void robotRotate(DIRECTION rotating_direction, const float& wheel_speed) override;
    void robotRotate(DIRECTION rotating_direction, SPEED_LEVEL speed_level) override;

    void setup();
    void run();
    void shutDown();

    void walkThroughGraph(std::vector<AreaNode>& node_graph, int node_index, int& unvisited_counter, std::deque<AreaNode>& path);

    std::deque<int> getTravellingPath(const std::vector<AreaNode>& node_graph, int first_node_index);

    // [floor, ceiling]
    void clampImageCoordinate(int& img_coord, const unsigned int& floor, const unsigned int& ceiling);

    // 原始的边界 没有扩展
    void partitionAreas();

    bool updateAreaGraph();

    // 判断两个区域是否可以互通
    bool areasConnected(const int& area_graph_idx1, const int& area_graph_idx2);

    // 判断两个区域是否相邻
    bool areasAdjacent(const int& area_graph_idx1, const int& area_graph_idx2);

    bool updateGraphConnectivity();

    // 以上的函数都是使用未扩展的area信息

    void clearViableZone();

    // 设置虚拟墙时要扩大边界
    void setViableZone(const int& area_graph_idx);

    void clearObstacles();

    void updateCoveredArea(const int& area_graph_idx);

    // 使用未扩展的area进行判断
    int getRobotsAreaGraphIndex();

    unsigned int countCoveredAreas();

    bool isAllCovered();

    bool isPathCovered(const std::deque<int>& path);

    void aimAtGoalCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);

    void aimAtGoal(const geometry_msgs::Pose& next_goal);

    // 清扫时也要扩大边界 以防漏掉公共的原始边界
    bool coverArea(const int& area_graph_idx);

    bool isPathFound(const nav_msgs::Odometry& start_odom, const geometry_msgs::Pose& target_pose);

    bool getOutOfStuck(const geometry_msgs::Pose& goal_pose);

    MOVE_STATE moveToArea(const int& area_graph_idx);

    void printPath(const std::deque<int>& path);

    void clearCostmap();

private:
    boost::shared_ptr<ros::NodeHandle> _n;
    nav_msgs::Odometry _origin_odom;
    ros::Subscriber _odom_subscriber;
    boost::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> _ac;

    std::string _odom_topic;
    std::string _occupancy_map_topic;
    std::string _make_zone_service;
    std::string _make_obstacles_service;
    std::string _make_plan_service;
    std::string _clear_costmap_service;
    std::string _move_action;

    std::string _path_plan_frame;

    int _area_radius;
    int _area_size;

    std::vector<AreaNode> _areas; // 存放候选area和每个area的清扫状态的list
    std::vector<AreaNode> _area_graph; // node的index即是node在vector中的index

    signed int _max_area_x;
    signed int _min_area_x;
    signed int _max_area_y;
    signed int _min_area_y;

    float _robot_size;

    bool _start_rotation;
    bool _finish_rotation;

    float _yaw_buffer; // degree
    float _yaw_tolerance; // degree
    float _position_tolerance; // m
    float _time_tolerance; // second
    int _max_move_trials;

    DIRECTION _robot_rotating_direction;

    SPEED_LEVEL _speed_level;
    float _high_speed;
    float _normal_speed;
    float _low_speed;

    geometry_msgs::Pose _next_uncovered_position;
};

#endif //COVERAGE_NAVIGATOR_GLOBAL_TRAVERSER_H
