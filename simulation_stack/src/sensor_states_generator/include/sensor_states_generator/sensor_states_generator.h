#ifndef SENSOR_STATES_GENERATOR_SENSOR_STATES_GENERATOR_H
#define SENSOR_STATES_GENERATOR_SENSOR_STATES_GENERATOR_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "gazebo_msgs/ContactsState.h"
#include "custom_msgs/DetectionState.h"
#include "boost/thread/recursive_mutex.hpp"
#include <numeric>

extern std::string PARAM_NAME_robot_radius;
extern std::string PARAM_NAME_robot_size;
extern std::string PARAM_NAME_collided_range;
extern std::string PARAM_NAME_obstacle_detected_range;
extern std::string PARAM_NAME_along_wall_range;
extern std::string PARAM_NAME_free_space_range;
extern std::string PARAM_NAME_unilateral_detect_angle;
extern std::string PARAM_NAME_beam_size;
extern std::string PARAM_NAME_states_topic;
extern std::string PARAM_NAME_scan_topic;
extern std::string PARAM_NAME_bumper_topic;


class SensorStatesGenerator{
public:
    SensorStatesGenerator();
    ~SensorStatesGenerator();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
    void bumperCallback(const gazebo_msgs::ContactsState::ConstPtr &bumper_msg);
private:
    boost::shared_ptr<ros::NodeHandle> _n;

    float _robot_radius;
    float _robot_size;
    float _collided_range;
    float _obstacle_detected_range;
    float _along_wall_range;
    float _free_space_range;

    int _unilateral_detect_angle;
    int _bilateral_detect_angles[2] = {_unilateral_detect_angle, 360-_unilateral_detect_angle}; // degree
    int _beam_size;

    boost::recursive_mutex _scan_mutex;
    boost::recursive_mutex _bumper_mutex;
    gazebo_msgs::ContactsState _bumper_state;

    ros::Publisher _states_publisher;
    ros::Subscriber _scan_subscriber;
    ros::Subscriber _bumper_subscriber;

    std::string _states_topic;
    std::string _scan_topic;
    std::string _bumper_topic;
};




#endif //SENSOR_STATES_GENERATOR_SENSOR_STATES_GENERATOR_H
