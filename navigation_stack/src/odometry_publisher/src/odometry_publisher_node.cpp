#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"

#include <string>
#include "boost/shared_ptr.hpp"

std::string PARAM_NAME_tf_listen_time="tf_listen_time";
std::string PARAM_NAME_tf_lookup_time="tf_lookup_time";
std::string PARAM_NAME_odom_frame="odom_frame";
std::string PARAM_NAME_robot_base_frame="robot_base_frame";
std::string PARAM_NAME_odom_topic="odom_topic";
std::string PARAM_NAME_odom_publish_rate="odom_publish_rate";

float tf_listen_time; // secs
float tf_lookup_time; // secs

boost::shared_ptr<tf::TransformListener> tf_listener;

std::string odom_frame;
std::string robot_base_frame;

std::string source_frame;
std::string target_frame;

tf::StampedTransform current_transform;
geometry_msgs::TransformStamped last_transform_msg;
geometry_msgs::TransformStamped current_transform_msg;

ros::Time last_time;
ros::Time current_time;
ros::Time lastest_time = ros::Time(0);

nav_msgs::Odometry odometry;
std::string odom_topic;
int odom_publish_rate;
ros::Publisher odom_publisher;

double delta_t;
double delta_x;
double delta_y;
double delta_yaw;

double v_x;
double v_y;
double v_yaw;

int main(int argc, char** argv){
    ros::init(argc, argv, "odometry_publisher_node");
    ros::NodeHandle n("~");

    n.getParam(PARAM_NAME_tf_listen_time, tf_listen_time);
    n.getParam(PARAM_NAME_tf_lookup_time, tf_lookup_time);
    n.getParam(PARAM_NAME_odom_frame, odom_frame);
    n.getParam(PARAM_NAME_robot_base_frame, robot_base_frame);
    n.getParam(PARAM_NAME_odom_topic, odom_topic);
    n.getParam(PARAM_NAME_odom_publish_rate, odom_publish_rate);

    target_frame = odom_frame;
    source_frame = robot_base_frame;

    ros::Rate rate(odom_publish_rate);

    odom_publisher = n.advertise<nav_msgs::Odometry>(odom_topic, 10);


    // initialization
    tf_listener.reset(new tf::TransformListener(ros::Duration(tf_listen_time)));

    while(true){
        try {
            current_time = ros::Time::now();
            tf_listener->waitForTransform(target_frame, source_frame, current_time, ros::Duration(tf_lookup_time));
            tf_listener->lookupTransform(target_frame, source_frame, current_time, current_transform);
        }catch(tf::TransformException &ex){
            continue;
        }
        if(tf_listener->canTransform(target_frame, source_frame, current_time)){
            break;
        }
    }

    tf::transformStampedTFToMsg(current_transform, current_transform_msg);

    current_time = current_transform_msg.header.stamp;
    last_transform_msg = current_transform_msg;
    last_time = current_time;

    while(n.ok()){
        try {
            current_time = ros::Time::now();
            tf_listener->waitForTransform(target_frame, source_frame, current_time, ros::Duration(tf_lookup_time));
            tf_listener->lookupTransform(target_frame, source_frame, current_time, current_transform);
        }catch(tf::TransformException &ex){
            continue;
        }

        tf::transformStampedTFToMsg(current_transform, current_transform_msg);

        current_time = current_transform_msg.header.stamp;

        delta_t = (current_time-last_time).toSec();
        delta_x = current_transform_msg.transform.translation.x-last_transform_msg.transform.translation.x;
        delta_y = current_transform_msg.transform.translation.y-last_transform_msg.transform.translation.y;

        double last_yaw = tf::getYaw(last_transform_msg.transform.rotation);
        double current_yaw = tf::getYaw(current_transform_msg.transform.rotation);
        delta_yaw = current_yaw-last_yaw;

        double yaw = (current_yaw+last_yaw)/2.0;
        double sin_yaw = std::sin(yaw);
        double cos_yaw = std::cos(yaw);

        v_x = cos_yaw*(delta_x/delta_t)+sin_yaw*(delta_y/delta_t);
        v_y = cos_yaw*(delta_y/delta_t)-sin_yaw*(delta_x/delta_t);
        v_yaw = delta_yaw/delta_t;

        odometry = nav_msgs::Odometry();
        odometry.header.stamp = current_time;
        odometry.header.frame_id = odom_frame;
        odometry.child_frame_id = robot_base_frame;

        odometry.pose.pose.position.x = current_transform_msg.transform.translation.x;
        odometry.pose.pose.position.y = current_transform_msg.transform.translation.y;
        odometry.pose.pose.position.z = current_transform_msg.transform.translation.z;

        auto quat = current_transform_msg.transform.rotation;
        auto norm_quat = tf::Quaternion(quat.x, quat.y, quat.z, quat.w).normalized();

        odometry.pose.pose.orientation.x = norm_quat.x();
        odometry.pose.pose.orientation.y = norm_quat.y();
        odometry.pose.pose.orientation.z = norm_quat.z();
        odometry.pose.pose.orientation.w = norm_quat.w();

        odometry.twist.twist.linear.x = v_x;
        odometry.twist.twist.linear.y = v_y;
        odometry.twist.twist.linear.z = 0.0;
        odometry.twist.twist.angular.x = 0.0;
        odometry.twist.twist.angular.y = 0.0;
        odometry.twist.twist.angular.z = v_yaw;

        odom_publisher.publish(odometry);

        last_transform_msg = current_transform_msg;
        last_time = current_time;

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}