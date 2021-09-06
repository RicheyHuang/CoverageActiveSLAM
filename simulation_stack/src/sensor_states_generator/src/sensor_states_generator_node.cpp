#include "sensor_states_generator/sensor_states_generator.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "sensor_states_generator_node");
    SensorStatesGenerator sensor_states_generator;

    return 0;
}