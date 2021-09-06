#ifndef VIRTUAL_LAYER_PROCESSOR_VIRTUAL_LAYER_PROCESSOR_H
#define VIRTUAL_LAYER_PROCESSOR_VIRTUAL_LAYER_PROCESSOR_H

#include "ros/ros.h"
#include "tf/transform_listener.h"

#include "nav_msgs/Odometry.h"
#include "custom_msgs/Zone.h"
#include "custom_msgs/Obstacles.h"
#include "custom_msgs/DetectionState.h"

#include "custom_srvs/Zone.h"
#include "custom_srvs/Obstacle.h"

#include "boost/thread/locks.hpp"
#include "boost/thread/recursive_mutex.hpp"
#include <CGAL/Point_2.h>
#include <CGAL/Segment_2.h>
#include <CGAL/Ray_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel K;

enum DATA_SOURCE{
    SENSOR_INFO=0,
    VIRTUAL_INFO=1,
    MERGED_INFO=2
};

extern std::string PARAM_NAME_robot_size;
extern std::string PARAM_NAME_collided_range;
extern std::string PARAM_NAME_obstacle_detected_range;
extern std::string PARAM_NAME_along_wall_range;
extern std::string PARAM_NAME_free_space_range;
extern std::string PARAM_NAME_unilateral_detect_angle;
extern std::string PARAM_NAME_beam_size;
extern std::string PARAM_NAME_data_source;

extern std::string PARAM_NAME_raw_states_topic;
extern std::string PARAM_NAME_processed_states_topic;
extern std::string PARAM_NAME_odom_topic;
extern std::string PARAM_NAME_zone_topic;
extern std::string PARAM_NAME_obstacles_topic;
extern std::string PARAM_NAME_make_zone_service;
extern std::string PARAM_NAME_make_obstacles_service;

class VirtualLayerProcessor{
public:
    VirtualLayerProcessor();
    ~VirtualLayerProcessor();

    // zone里面是世界点
    void zoneCallback(const custom_msgs::Zone::ConstPtr& zone_msg);

    // obstacles里面是世界点
    void obstaclesCallback(const custom_msgs::Obstacles::ConstPtr& obstacles_msg);

    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);

    void statesCallback(const custom_msgs::DetectionState::ConstPtr& states_msg);

    bool makeZone(custom_srvs::ZoneRequest& req, custom_srvs::ZoneResponse& res);

    bool makeObstacles(custom_srvs::ObstacleRequest& req, custom_srvs::ObstacleResponse& res);
private:
    boost::shared_ptr<ros::NodeHandle> _n;
    custom_msgs::Zone _zone;
    custom_msgs::Obstacles _obstacles;

    ros::ServiceServer _zone_maker_server;
    ros::Subscriber _zone_subscriber;
    ros::Publisher _zone_publisher;

    ros::ServiceServer _obstacles_maker_server;
    ros::Subscriber _obstacles_subscriber;
    ros::Publisher _obstacles_publisher;

    ros::Subscriber _odom_subscriber;
    ros::Subscriber _states_subscriber;

    ros::Publisher _states_publisher;

    std::string _raw_states_topic;
    std::string _processed_states_topic;
    std::string _odom_topic;
    std::string _zone_topic;
    std::string _obstacles_topic;
    std::string _make_zone_service;
    std::string _make_obstacles_service;

    boost::recursive_mutex _zone_mutex;
    boost::recursive_mutex _obstacles_mutex;
    boost::recursive_mutex _odom_mutex;
    boost::recursive_mutex _states_mutex;

    CGAL::Ray_2<K> _ray;
    CGAL::Polygon_2<K> _virtual_walls;
    std::vector<CGAL::Polygon_2<K>> _virtual_obstacles;

    custom_msgs::DetectionState _virtual_states;

    float _robot_size;
    float _collided_range;
    float _obstacle_detected_range;
    float _along_wall_range;
    float _free_space_range;

    int _unilateral_detect_angle;
    int _bilateral_detect_angles[2];
    int _beam_size;

    int _data_source_type;
    DATA_SOURCE _data_source;
};

#endif //VIRTUAL_LAYER_PROCESSOR_VIRTUAL_LAYER_PROCESSOR_H
