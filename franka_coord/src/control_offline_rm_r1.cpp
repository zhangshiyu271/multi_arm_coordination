#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>

#include <sstream>
#include <iostream>
#include <fstream>

#include <algorithm>
#include <cmath>
#include <vector>
#include <time.h>

#include <franka/robot.h>
#include <franka/exception.h>

#include "motion_generation.h"
#include "franka_coord/SendTrajPara.h"
#include "franka_coord/CoordTraj.h"
#include "franka_coord/TrajPara.h"


std::array<double, 7> q_goal = {0, 0, 0, 0, 0, 0, 0};
std::array<double, 7> q_init = {0, 0, 0, -1.5, 0, 1.5, 0};
std::array<double, 7> dq_init = {0, 0, 0, 0, 0, 0, 0};
double speed_factor = 0.05;
double slow_factor = 1;
ros::Time time_start;
double sample_time = 0.01;
bool motion_finished = true;
bool new_motion = 0;
bool change_goal = 0;

MotionGeneration motion_generation(speed_factor,q_init,q_goal,dq_init);

void start_move(franka_coord::TrajPara msg)
{
    for (size_t i = 0; i < 7; i++)
    {   
      change_goal = change_goal || std::abs(q_goal[i]-msg.qgoal[i]);
    }
    change_goal = change_goal || std::abs(msg.speedfactor-speed_factor); 
 
    
    if (change_goal)
    {
        ROS_INFO("Got updated traj: R%d", msg.robotid);

        speed_factor = msg.speedfactor;
        time_start = msg.starttime;
        slow_factor = msg.slowfactor;
        for (size_t i = 0; i < 7; i++)
        {
            q_goal[i] = msg.qgoal[i];
        } 
        
        ros::NodeHandle node_handle;
        ros::Publisher state_pub = node_handle.advertise<sensor_msgs::JointState>("joint_state_r1", 1);
        motion_generation.ReInitialize2(state_pub,speed_factor,q_goal,slow_factor);

        
        franka::Robot robot("172.16.0.103");
        robot.setCollisionBehavior(
            {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
            {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
            {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{50.0, 50.0, 50.0, 50.0, 50.0, 50.0}},
            {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{50.0, 50.0, 50.0, 50.0, 50.0, 50.0}});
        robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
        robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
        // franka::RobotState robot_state;
        
        ros::Time time1;
        while (time1.now().toSec() < time_start.toSec())
        {
            /* waiting till start time */
        } 
        ROS_INFO("Start.");
        try {
            robot.control(motion_generation);    // operator()
        } catch (const franka::Exception& e) {
            std::cout << e.what() << std::endl;
        }
        ROS_INFO("Goal reached.");
        change_goal = 0;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_offline_r1");
    ros::NodeHandle node_handle;

    motion_generation.control_mode = 1;

    ROS_INFO("Waiting for traj to move R1...");
    ros::Subscriber traj_sub = node_handle.subscribe("traj_parameter_r1", 1, start_move);

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
