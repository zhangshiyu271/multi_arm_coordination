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
bool reach_goal = 0;
int robot_id;

MotionGeneration motion_generation(speed_factor,q_init,q_goal,dq_init);

void start_move(franka_coord::TrajPara msg)
{
    double time_plan_start = 0;
    double time_plan_end = 0;
    double time_calcu_plan = 0;

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

        // motion_generation.ReInitialize(speed_factor,q_goal,slow_factor);
        motion_generation.ReInitialize3(speed_factor,q_init,q_goal,dq_init);
        motion_generation.slow_factor = slow_factor;

        std::ofstream motiondataq;
        std::ofstream motiondatadq;
        std::ofstream calcutime;
        motiondataq.open ("/home/zhangshiyu/Documents/Coordination/dataq"+std::to_string(robot_id)+".txt");
        motiondatadq.open ("/home/zhangshiyu/Documents/Coordination/datadq"+std::to_string(robot_id)+".txt");
        // calcutime.open ("/home/zhangshiyu/Documents/Coordination/calcu_time.txt");


        ros::NodeHandle node_handle;
        ros::Publisher state_pub = node_handle.advertise<sensor_msgs::JointState>("joint_state_r"+std::to_string(robot_id), 1);
        sensor_msgs::JointState states;
        states.position.resize(7);
        states.velocity.resize(7);
        
        ros::Time time1;
        while (time1.now().toSec() < time_start.toSec())
        {
            /* waiting till start time */
        } 
        ROS_INFO("R%d Start.", msg.robotid);
        reach_goal = 0;

        while (!reach_goal)
        {
            motion_finished = false;
            ros::Rate loop_rate(1000);
            while(!motion_finished && ros::ok())
            {
                
                time_plan_start = ros::Time::now().toSec();
                
                motion_finished=motion_generation.generateMotion(sample_time);

                // calculation time
                time_plan_end = ros::Time::now().toSec();
                time_calcu_plan = time_plan_end - time_plan_start;
                calcutime << time_calcu_plan << std::endl;

                for (size_t i = 0; i < 7; i++) {
                    states.position[i] = motion_generation.q_current[i];
                    states.velocity[i] = motion_generation.dq_current[i];
                }
                state_pub.publish(states);
                motiondataq << motion_generation.q_current[0] << " "
                            << motion_generation.q_current[1] << " "
                            << motion_generation.q_current[2] << " "
                            << motion_generation.q_current[3] << " "
                            << motion_generation.q_current[4] << " "
                            << motion_generation.q_current[5] << " "
                            << motion_generation.q_current[6] << " "
                            << std::endl;
                motiondatadq << motion_generation.dq_current[0] << " "
                            << motion_generation.dq_current[1] << " "
                            << motion_generation.dq_current[2] << " "
                            << motion_generation.dq_current[3] << " "
                            << motion_generation.dq_current[4] << " "
                            << motion_generation.dq_current[5] << " "
                            << motion_generation.dq_current[6] << " "
                            << std::endl;

                loop_rate.sleep();
            }
            if (motion_finished)
            {
                ROS_INFO("R%d Stop.",msg.robotid);
                for (size_t i = 0; i < 7; i++)
                {
                    reach_goal = 1;
                    if (std::abs(states.position[i] - msg.qgoal[i]) > 10e-6)
                    {
                        // std::cout<<states.position[i]<<" "<<q_goal[i]<<std::endl;
                        reach_goal = 0;
                    }
                }
            }
        }

        ROS_INFO("R%d Goal reached.",msg.robotid);
        for (size_t i = 0; i < 7; i++) 
        {
            states.velocity[i] = 0;
            
            q_init[i] = states.position[i];
            dq_init[i] = states.velocity[i];
        }
        state_pub.publish(states);

        motiondataq.close();
        motiondatadq.close();

        change_goal = 0;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_offline");
    ros::NodeHandle node_handle;
    ros::NodeHandle node_handle2("~");

    node_handle2.getParam("robot_id", robot_id);

    ROS_INFO("Waiting for traj to move R%d", robot_id);
    ros::Subscriber traj_sub = node_handle.subscribe("traj_parameter_r"+std::to_string(robot_id), 1, start_move);

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
