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

int robot_id;
std::array<double, 7> q_init = {0, 0, 0, -1.5, 0, 1.5, 0};
std::array<double, 7> dq_init = {0, 0, 0, 0, 0, 0, 0};

bool read_state(franka_coord::ReadState::Request  &req,
              franka_coord::ReadState::Response &res)
{
    // ROS_INFO("Read R%d state.", robot_id);

    for (size_t i = 0; i < 7; i++)
    {
        res.qinit[i] = q_init[i];
        res.dqinit[i] = dq_init[i];
    }
    return true;     
}

void update_state(const sensor_msgs::JointState& msg)
{
    for (size_t i = 0; i < 7; i++)
    {
        q_init[i] = msg.position[i];
    }
    // std::cout << "Init position updated: " 
    //     << q_init[0] << ", "
    //     << q_init[1] << ", "
    //     << q_init[2] << ", "
    //     << q_init[3] << ", "
    //     << q_init[4] << ", "
    //     << q_init[5] << ", "
    //     << q_init[6]
    //     << std::endl;   
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "read_state");
    ros::NodeHandle node_handle;
    ros::NodeHandle node_handle2("~");

    node_handle2.getParam("robot_id", robot_id);

    // ROS_INFO("Waiting for read R%d state.", robot_id);
    ros::ServiceServer service = node_handle.advertiseService("read_state_r"+std::to_string(robot_id), read_state);
    ros::Subscriber state_sub = node_handle.subscribe("joint_state_r"+std::to_string(robot_id), 1, update_state);

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    

    return 0;
}