#ifndef MAP_CONVERTER_MAP_CONVERTER_H
#define MAP_CONVERTER_MAP_CONVERTER_H

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "nav_msgs/OccupancyGrid.h"

extern std::string PARAM_NAME_world_min_x;
extern std::string PARAM_NAME_world_max_x;
extern std::string PARAM_NAME_world_min_y;
extern std::string PARAM_NAME_world_max_y;
extern std::string PARAM_NAME_map_resolution;
extern std::string PARAM_NAME_map_topic;
extern std::string PARAM_NAME_occupancy_grid_topic;

class MapConverter{
public:
    MapConverter();
    ~MapConverter();

private:
    void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy);
    bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);

    boost::shared_ptr<ros::NodeHandle> _n;
    ros::Publisher _map_publisher;
    ros::Subscriber _map_subscriber;

    std::string _map_topic;
    std::string _occupancy_grid_topic;

    float _world_min_x;
    float _world_max_x;
    float _world_min_y;
    float _world_max_y;
    float _map_resolution;

    unsigned int _map_width;
    unsigned int _map_height;
    float _origin_x;
    float _origin_y;
    unsigned int _size_x;
    unsigned int _size_y;
};



#endif //MAP_CONVERTER_MAP_CONVERTER_H
