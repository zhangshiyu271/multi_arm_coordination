#include "ros/ros.h"
#include <cstdlib>
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include <time.h>

#include <sstream>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Core>

#include "franka_coord/ReadState.h"

bool read_state(franka_coord::ReadState::Request  &req,
              franka_coord::ReadState::Response &res)
{
    ROS_INFO("Read R1 state.");
    
    std::array<double, 7> q_init = {0, 0, 0, -1.5, 0, 1.5, 0};
    std::array<double, 7> dq_init = {0, 0, 0, 0, 0, 0, 0};

    franka::Robot robot("172.16.0.103");
    franka::RobotState robot_state = robot.readOnce();
    q_init = robot_state.q_d;
    dq_init = robot_state.dq_d;

    
    return true;     
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "read_state_r1");
    ros::NodeHandle node_handle;

    ROS_INFO("Waiting for read R1 state.");
    ros::ServiceServer service = node_handle.advertiseService("read_state_r1", read_state);

    ros::spin();
    

    return 0;
}