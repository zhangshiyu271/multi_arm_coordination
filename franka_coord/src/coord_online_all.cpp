#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>

#include <sstream>
#include <iostream>
#include <fstream>
#include <string>

#include <algorithm>
#include <cmath>
#include <vector>
#include <time.h>

#include <franka/robot.h>
#include <franka/exception.h>

#include "motion_generation.h"
#include "franka_coord/SendTrajPara.h"
#include "franka_coord/SendTwoTrajs.h"
#include "franka_coord/CoordTraj.h"
#include "franka_coord/TrajPara.h"

int robot_num;

// std::vector<double> q_goal_t;  
Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> q_goal_t;
// q_goal_r1_b1,...,q_goal_r1_bn,
// q_goal_r2_b1,...,q_goal_r2_bn,
// ...
// q_goal_rn_b1,...,q_goal_rn_bn
// size: robot_num, 7*robot_num
// (ri,bi) - only consider itself (final goal)

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>  qi_goal_t;  
// qi_goal_r1_b1,...,qi_goal_r1_bn,
// qi_goal_r2_b1,...,qi_goal_r2_bn,
// ...
// qi_goal_rn_b1,...,qi_goal_rn_bn
// size: robot_num, robot_num
// (ri,bi) - only consider itself (final goal)

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>  get_temp_goal;  // if has got temp goals
// q_goal_r1_b1,...,q_goal_r1_bn,
// q_goal_r2_b1,...,q_goal_r2_bn,
// ...
// q_goal_rn_b1,...,q_goal_rn_bn
// size: robot_num, 7*robot_num
// value: 0,1

Eigen::Matrix<double, Eigen::Dynamic, 7, Eigen::ColMajor>  q_goal_tf; // temp goals based on all other robots
// q_goal_r1
// q_goal_r2
// ...
// q_goal_rn
// size: robot_num, 7

Eigen::Matrix<double, Eigen::Dynamic, 7, Eigen::ColMajor>  q_goal; // final goals
// q_goal_r1
// q_goal_r2
// ...
// q_goal_rn
// size: robot_num, 7

std::vector<double> speed_factors;
// r1, r2, ..., rn
// size: 1, robot_num


bool start_goal(franka_coord::SendTwoTrajs::Request  &req, franka_coord::SendTwoTrajs::Response &res);
bool update_goal(franka_coord::SendTwoTrajs::Request  &req, franka_coord::SendTwoTrajs::Response &res);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "coord_online_all");
    ros::NodeHandle node_handle;
    ros::NodeHandle node_handle2("~");

    robot_num = 3;
    node_handle2.getParam("robot_num", robot_num);
    speed_factors.resize(1,robot_num);
    q_goal_t.resize(robot_num, robot_num*7);
    q_goal_tf.resize(robot_num, 7);
    q_goal.resize(robot_num, 7);
    qi_goal_t.resize(robot_num, robot_num);
    get_temp_goal.resize(robot_num, robot_num);
    for (size_t i = 0; i < robot_num; i++)
    {
        for (size_t j = 0; j < robot_num; j++)
        {
            get_temp_goal(i,j) = 0;
        }       
    }    

    ROS_INFO("Waiting for inital goals from sub-coordinators.");
    ROS_INFO("robot num = %d", robot_num);
    // start goals
    ros::ServiceServer service1 = node_handle.advertiseService("initial_temp_goal", start_goal);
    ros::ServiceServer service2 = node_handle.advertiseService("update_temp_goal", update_goal);
    
    ros::AsyncSpinner spinner(8);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}

bool start_goal(franka_coord::SendTwoTrajs::Request  &req,
                 franka_coord::SendTwoTrajs::Response &res)
{
    int r1 = req.robotid1;
    int r2 = req.robotid2;
    int index_r1 = r1-1;
    int index_r2 = r2-1;
    ROS_INFO("[COORD(R%d,R%d)] Initial goals got.",r1,r2);
    
    speed_factors[index_r1]=req.speedfactor1;
    speed_factors[index_r2]=req.speedfactor2;

    for (size_t j = 0; j < 7; j++)
    {
        q_goal_t(index_r1,index_r1*7+j) = req.qgoal1[j];
        q_goal_t(index_r1,index_r2*7+j) = req.qgoalt1[j];
        q_goal_t(index_r2,index_r1*7+j) = req.qgoalt2[j];
        q_goal_t(index_r2,index_r2*7+j) = req.qgoal2[j];
    }

    qi_goal_t(index_r1,index_r1) = req.qigoal1;
    qi_goal_t(index_r1,index_r2) = req.qigoalt1;
    qi_goal_t(index_r2,index_r1) = req.qigoalt2;
    qi_goal_t(index_r2,index_r2) = req.qigoal2;

    get_temp_goal(index_r1,index_r1) = 1;
    get_temp_goal(index_r1,index_r2) = 1;
    get_temp_goal(index_r2,index_r1) = 1;
    get_temp_goal(index_r2,index_r2) = 1;
    
    // got all pairs of coordination information
    int got_all_coord = 1;
    for (size_t i = 0; i < robot_num; i++)
    {
        for (size_t j = 0; j < robot_num; j++)
        {
            got_all_coord = got_all_coord * get_temp_goal(i,j);
        }
    }
    if (got_all_coord)
    {
        ROS_INFO("Got initial goals from all sub-coordinators.");

        std::cout << "qi_goal_t: " << std::endl; 
        for (size_t i1 = 0; i1 < robot_num; i1++)
        {
            for (size_t i2 = 0; i2 < robot_num; i2++)
            {
                std::cout << qi_goal_t(i1,i2) << " ";
            }
            std::cout << std::endl;
        }

        // compare & select initial goals
        double time_cc_start = ros::Time::now().toSec();
        for (size_t i1 = 0; i1 < robot_num; i1++)  // R(i1)
        {
            // select smallest index based on all other robots
            double qi_goal = qi_goal_t(i1,i1);
            int r_goal = i1;
            for (size_t i2 = 0; i2 < robot_num; i2++)
            {
                if (qi_goal_t(i1,i2) < qi_goal)
                {
                    qi_goal = qi_goal_t(i1,i2);
                    r_goal = i2;
                }
            }
            // std::cout << "R" << i1+1 << " temp goal: " << qi_goal << std::endl;
            for (size_t j = 0; j < 7; j++)
            {
                // initial goal based on all other robots
                q_goal_tf(i1,j) = q_goal_t(i1, r_goal*7+j);
                // final goal
                q_goal(i1,j) = q_goal_t(i1,i1*7+j);
            }            
        }
        double time_cc_end = ros::Time::now().toSec();
        double time_calcu_cc = time_cc_end - time_cc_start;
        std::cout << "Calculation time: " << time_calcu_cc << std::endl;
        
        // send initial traj to control nodes (publish) 
        ros::Time time_start;
        ros::Time time_now = ros::Time::now();
        time_start.sec = time_now.toSec() + 3;

        franka_coord::TrajPara para;
        para.starttime = time_start;
        for (size_t i = 0; i < robot_num; i++)
        {
            para.robotid = i+1;
            para.speedfactor = speed_factors[i];
            for (size_t j = 0; j < 7; j++)
            {
                para.qgoalt[j] = q_goal_tf(i,j);
                para.qgoal[j]  = q_goal(i,j);
            }

            ros::NodeHandle node_handle;
            ros::Publisher pub_init_goal = node_handle.advertise<franka_coord::TrajPara>("traj_parameter_r"+std::to_string(para.robotid), 1);
            ros::Rate loop_rate(10);
            int count = 0;
            while ((ros::ok()) && (count<10))
            {
                pub_init_goal.publish(para);
                
                ros::spinOnce();
                loop_rate.sleep();
                ++count;
            }
            ROS_INFO("R%d initial goals published.", para.robotid);
        }
        
        // reset vectors
        for (size_t i = 0; i < robot_num; i++)
        {
            for (size_t j = 0; j < robot_num; j++)
            {
                get_temp_goal(i,j) = 0;
            }       
        } 

    }
       

  return true;
}

bool update_goal(franka_coord::SendTwoTrajs::Request  &req,
                 franka_coord::SendTwoTrajs::Response &res)
{
    int r_update;
    int r1 = req.robotid1;
    int r2 = req.robotid2;
    int index_r1 = r1-1;
    int index_r2 = r2-1;

    if (req.qigoalt1 > qi_goal_t(index_r1,index_r2))
    {
        r_update = r1;
        ROS_INFO("[COORD(R%d,R%d)] R%d update temp goal got.",r1,r2,r_update);
        qi_goal_t(index_r1,index_r2) = req.qigoalt1;
        for (size_t j = 0; j < 7; j++)
        {
            q_goal_t(index_r1,index_r2*7+j) = req.qgoalt1[j];
        }

        // if updated temp goal is safe for all other robots (smaller)
        bool temp_goal_update = 1;
        for (size_t i = 0; i < robot_num; i++)
        {
            if(qi_goal_t(index_r1,index_r2) > qi_goal_t(index_r1,i))
            {
                temp_goal_update = 0;
            }
        }
        if (temp_goal_update)
        {
            ROS_INFO("R%d temp goal updated: %f",r_update,qi_goal_t(index_r1,index_r2));
            // update temp goal
            for (size_t j = 0; j < 7; j++)
            {
                q_goal_tf(index_r1,j) = req.qgoalt1[j];
            }
            // send to control node (srv)
            ros::NodeHandle node_handle;
            ros::ServiceClient client_update_goal = node_handle.serviceClient<franka_coord::SendTrajPara>("temp_goal_r"+std::to_string(r_update));
            franka_coord::SendTrajPara srv_para;
            srv_para.request.robotid = r_update;
            srv_para.request.qgoal = req.qgoalt1; 

            if (client_update_goal.call(srv_para))
            {
                ROS_INFO("R%d temp goal sent.",r_update);

            } else
            {
                ROS_INFO("Failed to call service: R%d temp goal sent.",r_update);
            }

        }        
        
    } else
    {
        r_update = r2;
        ROS_INFO("[COORD(R%d,R%d)] R%d update temp goal got.",r1,r2,r_update);
        qi_goal_t(index_r2,index_r1) = req.qigoalt2;
        for (size_t j = 0; j < 7; j++)
        {
            q_goal_t(index_r2,index_r1*7+j) = req.qgoalt2[j];
        }

        // if updated temp goal is safe for all other robots (smaller)
        bool temp_goal_update = 1;
        for (size_t i = 0; i < robot_num; i++)
        {
            if(qi_goal_t(index_r2,index_r1) > qi_goal_t(index_r2,i))
            {
                temp_goal_update = 0;
            }
        }
        if (temp_goal_update)
        {
            ROS_INFO("R%d temp goal updated: %f",r_update,qi_goal_t(index_r2,index_r1));
            // update temp goal
            for (size_t j = 0; j < 7; j++)
            {
                q_goal_tf(index_r2,j) = req.qgoalt2[j];
            }
            // send to control node (srv)
            ros::NodeHandle node_handle;
            ros::ServiceClient client_update_goal = node_handle.serviceClient<franka_coord::SendTrajPara>("temp_goal_r"+std::to_string(r_update));
            franka_coord::SendTrajPara srv_para;
            srv_para.request.robotid = r_update;
            srv_para.request.qgoal = req.qgoalt2; 

            if (client_update_goal.call(srv_para))
            {
                ROS_INFO("R%d temp goal sent.",r_update);

            } else
            {
                ROS_INFO("Failed to call service: R%d temp goal sent.",r_update);
            }

        }
    }
    
    
    

    

 

  return true;
}