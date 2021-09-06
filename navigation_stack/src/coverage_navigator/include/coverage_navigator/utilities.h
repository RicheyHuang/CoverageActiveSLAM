#ifndef COVERAGE_NAVIGATOR_UTILITIES_H
#define COVERAGE_NAVIGATOR_UTILITIES_H

#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "custom_msgs/WheelSpeed.h"

namespace utilities{

    extern std::string PARAM_NAME_wheel_span;
    extern std::string PARAM_NAME_world_min_x;
    extern std::string PARAM_NAME_world_max_x;
    extern std::string PARAM_NAME_world_min_y;
    extern std::string PARAM_NAME_world_max_y;
    extern std::string PARAM_NAME_map_resolution;
    extern std::string PARAM_NAME_motion_topic;
    extern std::string PARAM_NAME_wheel_speed_topic;

}

enum DIRECTION{
    LEFT,
    RIGHT
};

enum SPEED_LEVEL{
    HIGH,
    NORMAL,
    LOW
};

class CoordinateConverter{
public:
    CoordinateConverter();
    CoordinateConverter(float xmin, float xmax, float ymin, float ymax, float resolution);
    ~CoordinateConverter();
    void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy);
    bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my);
    float getMapResolution();
    unsigned int getMapWidth();
    unsigned int getMapHeight();
    void getMapOriginWorldCoord(float& origin_x, float& origin_y);
private:
    float _xmin;
    float _xmax;
    float _ymin;
    float _ymax;
    float _resolution;
    unsigned int _map_width;
    unsigned int _map_height;
    float _origin_x;
    float _origin_y;
    unsigned int _size_x;
    unsigned int _size_y;
};

/* input yaw: [0, 360)
 * output yaw: [-180, 180]
   output>0: to_yaw在from_yaw的左边
   output<0: to_yaw在from_yaw的右边 */
float compute_relative_yaw(const double& from_yaw, const double& to_yaw);

float compute_position_error(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2);

class Robot{
public:
    Robot();
    virtual ~Robot();
    void robotPowerOn();
    void robotMove(const geometry_msgs::Twist& motion);
    void robotMove(const float& left_wheel_speed, const float& right_wheel_speed);
    virtual void robotRotate(DIRECTION rotating_direction, const float& wheel_speed);
    virtual void robotTurning(DIRECTION turning_direction, const float& wheel_speed);
    virtual void robotRotate(DIRECTION rotating_direction, SPEED_LEVEL speed_level);
    virtual void robotTurning(DIRECTION turning_direction, SPEED_LEVEL speed_level);
    void robotStop();
    virtual void robotForward(SPEED_LEVEL speed_level);
    void robotForward(const float& wheel_speed);
    void robotBackward(const float& wheel_speed);
    void robotPowerOff();
    void motionToSpeed(const geometry_msgs::Twist& motion, custom_msgs::WheelSpeed& speed);
    void speedToMotion(const custom_msgs::WheelSpeed& speed, geometry_msgs::Twist& motion);
    void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy);
    bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my);
    float getMapResolution();
    unsigned int getMapWidth();
    unsigned int getMapHeight();
    void getMapOriginWorldCoord(float& origin_x, float& origin_y);
private:
    boost::shared_ptr<ros::NodeHandle> _n;
    ros::Publisher _motion_publisher;
    ros::Publisher _wheel_speed_publisher;

    std::string _motion_topic;
    std::string _wheel_speed_topic;

    geometry_msgs::Twist _robot_motion;
    custom_msgs::WheelSpeed _robot_wheel_speed;

    CoordinateConverter _coordinate_converter;
    float _wheel_span;
    float _world_min_x;
    float _world_max_x;
    float _world_min_y;
    float _world_max_y;
    float _map_resolution;

};

#endif //COVERAGE_NAVIGATOR_UTILITIES_H
