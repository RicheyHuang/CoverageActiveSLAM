#include "map_converter/map_converter.h"

std::string PARAM_NAME_world_min_x="world_min_x";
std::string PARAM_NAME_world_max_x="world_max_x";
std::string PARAM_NAME_world_min_y="world_min_y";
std::string PARAM_NAME_world_max_y="world_max_y";
std::string PARAM_NAME_map_resolution="map_resolution";
std::string PARAM_NAME_map_topic="map_topic";
std::string PARAM_NAME_occupancy_grid_topic="occupancy_grid_topic";

MapConverter::MapConverter() {
    
    _n.reset(new ros::NodeHandle("~"));

    _n->getParam(PARAM_NAME_world_min_x,_world_min_x);
    _n->getParam(PARAM_NAME_world_max_x,_world_max_x);
    _n->getParam(PARAM_NAME_world_min_y,_world_min_y);
    _n->getParam(PARAM_NAME_world_max_y,_world_max_y);
    _n->getParam(PARAM_NAME_map_resolution,_map_resolution);
    _n->getParam(PARAM_NAME_map_topic,_map_topic);
    _n->getParam(PARAM_NAME_occupancy_grid_topic,_occupancy_grid_topic);

    _map_width = static_cast<unsigned int>((_world_max_x-_world_min_x)/_map_resolution);
    _map_height = static_cast<unsigned int>((_world_max_y-_world_min_y)/_map_resolution);
    _origin_x = _world_min_x;
    _origin_y = _world_min_y;
    _size_x = _map_width;
    _size_y = _map_height;

    _map_publisher = _n->advertise<nav_msgs::OccupancyGrid>(_map_topic,1);
    _map_subscriber = _n->subscribe(_occupancy_grid_topic,1, &MapConverter::mapCallback, this);

    ros::Duration interval(0.001);
    while (ros::ok()) {
        ros::spinOnce();
        interval.sleep();
    }

}

MapConverter::~MapConverter() {}

void MapConverter::mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy)
{
    wx = _origin_x + (mx + 0.5) * _map_resolution;
    wy = _origin_y + (my + 0.5) * _map_resolution;
}

bool MapConverter::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my)
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

void MapConverter::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg){
    auto map = *map_msg;

    nav_msgs::OccupancyGrid occupancy_map;
    occupancy_map.header = map.header;
    occupancy_map.info.resolution = _map_resolution;
    occupancy_map.info.height = _map_height;
    occupancy_map.info.width = _map_width;
    occupancy_map.info.origin.position.x = _origin_x;
    occupancy_map.info.origin.position.y = _origin_y;
    occupancy_map.info.origin.position.z = 0;
    occupancy_map.info.origin.orientation.x = 0;
    occupancy_map.info.origin.orientation.y = 0;
    occupancy_map.info.origin.orientation.z = 0;
    occupancy_map.info.origin.orientation.w = 1;
    occupancy_map.data.assign(_map_width*_map_height,-1);

    auto map_data = map.data;
    unsigned int map_data_index = 0;
    unsigned int occupancy_map_data_index;
    double wx, wy;
    unsigned int mx, my;
    int8_t value;
    for(size_t carto_my = 0; carto_my < map.info.height; ++carto_my){
        for(size_t carto_mx = 0; carto_mx < map.info.width; ++carto_mx){
            auto carto_value = map_data.at(map_data_index);
            if(carto_value>=50){
                value=100;
            }else if(carto_value==-1){
                value=-1;
            }else{
                value=0;
            }
            wx = map.info.origin.position.x + (carto_mx + 0.5) * map.info.resolution;
            wy = map.info.origin.position.y + (carto_my + 0.5) * map.info.resolution;
            worldToMap(wx,wy,mx,my);
            occupancy_map_data_index = my*_map_width+mx;
            occupancy_map.data.at(occupancy_map_data_index)=value;
            map_data_index++;
        }
    }

    _map_publisher.publish(occupancy_map);
}
