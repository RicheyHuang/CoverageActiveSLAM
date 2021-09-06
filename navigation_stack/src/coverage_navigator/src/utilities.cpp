#include "coverage_navigator/utilities.h"

std::string utilities::PARAM_NAME_wheel_span="wheel_span";
std::string utilities::PARAM_NAME_world_min_x="world_min_x";
std::string utilities::PARAM_NAME_world_max_x="world_max_x";
std::string utilities::PARAM_NAME_world_min_y="world_min_y";
std::string utilities::PARAM_NAME_world_max_y="world_max_y";
std::string utilities::PARAM_NAME_map_resolution="map_resolution";
std::string utilities::PARAM_NAME_motion_topic="motion_topic";
std::string utilities::PARAM_NAME_wheel_speed_topic="wheel_speed_topic";

CoordinateConverter::CoordinateConverter(){}

CoordinateConverter::CoordinateConverter(float xmin, float xmax, float ymin, float ymax, float resolution) {
    _xmin=xmin;
    _xmax=xmax;
    _ymin=ymin;
    _ymax=ymax;
    _resolution=resolution;
    _map_width = static_cast<unsigned int>((_xmax-_xmin)/_resolution);
    _map_height = static_cast<unsigned int>((_ymax-_ymin)/_resolution);
    _origin_x = _xmin;
    _origin_y = _ymin;
    _size_x = _map_width;
    _size_y = _map_height;
}

CoordinateConverter::~CoordinateConverter() {}

float CoordinateConverter::getMapResolution(){
    return _resolution;
}

unsigned int CoordinateConverter::getMapWidth(){
    return _map_width;
}

unsigned int CoordinateConverter::getMapHeight(){
    return _map_height;
}

void CoordinateConverter::getMapOriginWorldCoord(float& origin_x, float& origin_y){
    origin_x = _origin_x;
    origin_y = _origin_y;
}

void CoordinateConverter::mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy)
{
    wx = _origin_x + (mx + 0.5) * _resolution;
    wy = _origin_y + (my + 0.5) * _resolution;
}

bool CoordinateConverter::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my)
{
    if (wx < _origin_x || wy < _origin_y){
        return false;
    }
    mx = (int)((wx - _origin_x) / _resolution);
    my = (int)((wy - _origin_y) / _resolution);
    if (mx < _size_x && my < _size_y){
        return true;
    }else{
        return false;
    }
}

float compute_relative_yaw(const double& from_yaw, const double& to_yaw){
    float yaw_offset = to_yaw - from_yaw;
    if(yaw_offset>180.0){
        yaw_offset-=360.0;
    }else if(yaw_offset<-180.0){
        yaw_offset+=360.0;
    }
    return yaw_offset;
}

float compute_position_error(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2){
    float x1 = p1.position.x;
    float x2 = p2.position.x;
    float y1 = p1.position.y;
    float y2 = p2.position.y;
    float e1 = powf((x1-x2),2);
    float e2 = powf((y1-y2),2);
    float err = sqrtf(e1+e2);
    return err;
}

Robot::Robot() {}

Robot::~Robot() {}

void Robot::robotRotate(DIRECTION rotating_direction, const float& wheel_speed){};
void Robot::robotTurning(DIRECTION turning_direction, const float& wheel_speed){};
void Robot::robotRotate(DIRECTION rotating_direction, SPEED_LEVEL speed_level){};
void Robot::robotTurning(DIRECTION turning_direction, SPEED_LEVEL speed_level){};
void Robot::robotForward(SPEED_LEVEL speed_level){};

void Robot::robotPowerOn() {
    
    _n.reset(new ros::NodeHandle("~"));

    _n->getParam(utilities::PARAM_NAME_motion_topic,_motion_topic);
    _n->getParam(utilities::PARAM_NAME_wheel_speed_topic,_wheel_speed_topic);
    _n->getParam(utilities::PARAM_NAME_wheel_span,_wheel_span);
    _n->getParam(utilities::PARAM_NAME_world_min_x,_world_min_x);
    _n->getParam(utilities::PARAM_NAME_world_max_x,_world_max_x);
    _n->getParam(utilities::PARAM_NAME_world_min_y,_world_min_y);
    _n->getParam(utilities::PARAM_NAME_world_max_y,_world_max_y);
    _n->getParam(utilities::PARAM_NAME_map_resolution,_map_resolution);

    _motion_publisher = _n->advertise<geometry_msgs::Twist>(_motion_topic,10);
    _wheel_speed_publisher = _n->advertise<custom_msgs::WheelSpeed>(_wheel_speed_topic,10);
    _coordinate_converter = CoordinateConverter(_world_min_x, _world_max_x, _world_min_y, _world_max_y, _map_resolution);
}

void Robot::robotPowerOff() {
    _motion_publisher.shutdown();
    _wheel_speed_publisher.shutdown();
}

void Robot::motionToSpeed(const geometry_msgs::Twist& motion, custom_msgs::WheelSpeed& speed){
    float v = motion.linear.x;
    float w = motion.angular.z;
    float l = _wheel_span;
    speed.left_wheel_speed = (2.0*v-l*w)/2.0;
    speed.right_wheel_speed = (2.0*v+l*w)/2.0;
}

void Robot::speedToMotion(const custom_msgs::WheelSpeed& speed, geometry_msgs::Twist& motion){
    float vl = speed.left_wheel_speed;
    float vr = speed.right_wheel_speed;
    float l = _wheel_span;
    motion.linear.x = (vr+vl)/2.0;
    motion.linear.y = 0;
    motion.linear.z = 0;
    motion.angular.x = 0;
    motion.angular.y = 0;
    motion.angular.z = (vr-vl)/l;
}

void Robot::robotMove(const float &left_wheel_speed, const float &right_wheel_speed) {
    _robot_wheel_speed.left_wheel_speed = left_wheel_speed;
    _robot_wheel_speed.right_wheel_speed = right_wheel_speed;
    _wheel_speed_publisher.publish(_robot_wheel_speed);

    speedToMotion(_robot_wheel_speed, _robot_motion);
    _motion_publisher.publish(_robot_motion);

    usleep(10);
}

void Robot::robotMove(const geometry_msgs::Twist &motion) {
    _robot_motion = motion;
    _motion_publisher.publish(_robot_motion);

    motionToSpeed(_robot_motion, _robot_wheel_speed);
    _wheel_speed_publisher.publish(_robot_wheel_speed);

    usleep(10);
}

void Robot::robotStop() {
    _robot_wheel_speed.left_wheel_speed = 0;
    _robot_wheel_speed.right_wheel_speed = 0;
    _wheel_speed_publisher.publish(_robot_wheel_speed);

    speedToMotion(_robot_wheel_speed, _robot_motion);
    _motion_publisher.publish(_robot_motion);

    usleep(10);
}

void Robot::robotForward(const float &wheel_speed) {
    _robot_wheel_speed.left_wheel_speed = wheel_speed;
    _robot_wheel_speed.right_wheel_speed = wheel_speed;
    _wheel_speed_publisher.publish(_robot_wheel_speed);

    speedToMotion(_robot_wheel_speed, _robot_motion);
    _motion_publisher.publish(_robot_motion);

    usleep(10);
}

void Robot::robotBackward(const float &wheel_speed) {
    _robot_wheel_speed.left_wheel_speed = -wheel_speed;
    _robot_wheel_speed.right_wheel_speed = -wheel_speed;
    _wheel_speed_publisher.publish(_robot_wheel_speed);

    speedToMotion(_robot_wheel_speed, _robot_motion);
    _motion_publisher.publish(_robot_motion);

    usleep(10);
}

bool Robot::worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my) {
    _coordinate_converter.worldToMap(wx, wy, mx, my);
}

void Robot::mapToWorld(unsigned int mx, unsigned int my, double &wx, double &wy) {
    _coordinate_converter.mapToWorld(mx, my, wx, wy);
}

float Robot::getMapResolution(){
    return _coordinate_converter.getMapResolution();
}

unsigned int Robot::getMapWidth(){
    return _coordinate_converter.getMapWidth();
}

unsigned int Robot::getMapHeight(){
    return _coordinate_converter.getMapHeight();
}

void Robot::getMapOriginWorldCoord(float& origin_x, float& origin_y){
    _coordinate_converter.getMapOriginWorldCoord(origin_x, origin_y);
}

