cmake_minimum_required(VERSION 2.8.3)
project(custom_srvs)

find_package(catkin REQUIRED
             COMPONENTS roscpp
                        rospy
                        std_msgs
                        message_generation
                        geometry_msgs
                        sensor_msgs
                        custom_msgs
                        nav_msgs)


add_service_files(FILES
                  Escape.srv
		  Obstacle.srv
                  Zigzag.srv
                  Zone.srv)

generate_messages(DEPENDENCIES
                  std_msgs
                  geometry_msgs
                  custom_msgs
                  sensor_msgs
                  nav_msgs)

catkin_package(CATKIN_DEPENDS
               roscpp
               rospy
               message_runtime
               std_msgs
               geometry_msgs
               custom_msgs
               sensor_msgs
               nav_msgs)

include_directories(${catkin_INCLUDE_DIRS})
