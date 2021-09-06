#ifndef FLAT_WORLD_IMU_NODE_H_
#define FLAT_WORLD_IMU_NODE_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#define GRAVITY 9.8

class FlatWorldImuNode
{
 public:
  FlatWorldImuNode();
  ~FlatWorldImuNode();
  bool init();

 private:
  ros::NodeHandle nh_;
  ros::Time last_published_time_;
  ros::Publisher publisher_;
  ros::Subscriber subscriber_;
  void msgCallback(const sensor_msgs::ImuConstPtr imu_in);
};

#endif // FLAT_WORLD_IMU_NODE_H_
