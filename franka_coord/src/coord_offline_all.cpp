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

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>  priority_matrix;  
// prior_r1_b1,...,prior_r1_bn,
// prior_r2_b1,...,prior_r2_bn,
// ...
// prior_rn_b1,...,prior_rn_bn
// size: robot_num, robot_num
// (ri,bi) - only consider itself (1)

Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor>  priority_overall;  
// R1 overall priority
// R2 overall priority
// ...
// Rn overall priority

Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor>  prior_t;  
// used for find a set of robots {Ri1} with highest priority in unordered robots
// R1 unordered && prior than any other unordered robot?
// R2 
// ...
// Rn 


Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>  delay_time_matrix;  
// td_r1_b1,...,td_r1_bn,
// td_r2_b1,...,td_r2_bn,
// ...
// td_rn_b1,...,td_rn_bn
// size: robot_num, robot_num
// (ri,bi) - only consider itself (0)

Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor>  delay_time;
// overall delay time
// delay_time_r1
// delay_time_r2
// ...
// delay_time_rn
// size: robot_num, 1

Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor>  delay_time_t;
// delay_time_ri_r1
// delay_time_ri_r2
// ...
// delay_time_ri_rn
// size: robot_num, 1

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>  slow_factor_matrix;  
// Cs_r1_b1,...,Cs_r1_bn,
// Cs_r2_b1,...,Cs_r2_bn,
// ...
// Cs_rn_b1,...,Cs_rn_bn
// size: robot_num, robot_num
// (ri,bi) - only consider itself (1)

Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor>  slow_factor;
// overall slow factor
// Cs_r1
// Cs_r2
// ...
// Cs_rn
// size: robot_num, 1

Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor>  slow_factor_t;
// Cs_ri_r1
// Cs_ri_r2
// ...
// Cs_ri_rn
// size: robot_num, 1

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>  get_goal;  // if has got temp goals
// r1_b1,...,r1_bn,
// r2_b1,...,r2_bn,
// ...
// rn_b1,...,rn_bn
// size: robot_num, robot_num
// value: 0,1

Eigen::Matrix<double, Eigen::Dynamic, 7, Eigen::ColMajor>  q_goal; // final goals
// q_goal_r1
// q_goal_r2
// ...
// q_goal_rn
// size: robot_num, 7

std::vector<double> speed_factors;
// r1, r2, ..., rn
// size: 1, robot_num


bool get_trajs(franka_coord::SendTwoTrajs::Request  &req, franka_coord::SendTwoTrajs::Response &res);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "coord_offline_delay_all");
    ros::NodeHandle node_handle;
    ros::NodeHandle node_handle2("~");

    robot_num = 3;
    node_handle2.getParam("robot_num", robot_num);
    speed_factors.resize(1,robot_num);
    q_goal.resize(robot_num, 7);
    priority_matrix.resize(robot_num, robot_num);
    priority_overall.resize(robot_num, 1);
    prior_t.resize(robot_num, 1);
    delay_time_matrix.resize(robot_num, robot_num);
    delay_time.resize(robot_num,1);
    delay_time_t.resize(robot_num,1);
    slow_factor_matrix.resize(robot_num, robot_num);
    slow_factor.resize(robot_num,1);
    slow_factor_t.resize(robot_num,1);
    get_goal.resize(robot_num, robot_num);
    for (size_t i = 0; i < robot_num; i++)
    {
        for (size_t j = 0; j < robot_num; j++)
        {
            get_goal(i,j) = 0;
            priority_matrix(i,j) = 0;
            slow_factor_matrix(i,j) = 1;
            delay_time_matrix(i,j) = 0;
        }
        delay_time[i] = 0;  
        delay_time_t[i] = 0;  
        slow_factor[i] = 1;  
        slow_factor_t[i] = 1;  
        priority_overall(i) = 0;
    }    

    ROS_INFO("Waiting for delay factors from sub-coordinators.");
    ROS_INFO("robot num = %d", robot_num);
    // start goals
    ros::ServiceServer service1 = node_handle.advertiseService("initial_trajectories", get_trajs);
    
    ros::AsyncSpinner spinner(8);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}

bool get_trajs(franka_coord::SendTwoTrajs::Request  &req,
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
        q_goal(index_r1,j) = req.qgoal1[j];
        q_goal(index_r2,j) = req.qgoal2[j];
    }

    priority_matrix(index_r1,index_r1) = 1;
    priority_matrix(index_r1,index_r2) = req.prior1;
    priority_matrix(index_r2,index_r1) = req.prior2;
    priority_matrix(index_r2,index_r2) = 1;

    delay_time_matrix(index_r1,index_r1) = 0;
    delay_time_matrix(index_r1,index_r2) = req.delayfactor1;
    delay_time_matrix(index_r2,index_r1) = req.delayfactor2;
    delay_time_matrix(index_r2,index_r2) = 0;

    slow_factor_matrix(index_r1,index_r1) = 1;
    slow_factor_matrix(index_r1,index_r2) = req.slowfactor1;
    slow_factor_matrix(index_r2,index_r1) = req.slowfactor2;
    slow_factor_matrix(index_r2,index_r2) = 1;

    get_goal(index_r1,index_r1) = 1;
    get_goal(index_r1,index_r2) = 1;
    get_goal(index_r2,index_r1) = 1;
    get_goal(index_r2,index_r2) = 1;
    
    // got all pairs of coordination information
    int got_all_coord = 1;
    for (size_t i = 0; i < robot_num; i++)
    {
        for (size_t j = 0; j < robot_num; j++)
        {
            got_all_coord = got_all_coord * get_goal(i,j);
        }
    }
    if (got_all_coord)
    {
        ROS_INFO("Got delay factors from all sub-coordinators.");

        std::cout << "priority: " << std::endl; 
        for (size_t i1 = 0; i1 < robot_num; i1++)
        {
            for (size_t i2 = 0; i2 < robot_num; i2++)
            {
                std::cout << priority_matrix(i1,i2) << " ";
            }
            std::cout << std::endl;
        }

        std::cout << "delay time: " << std::endl; 
        for (size_t i1 = 0; i1 < robot_num; i1++)
        {
            for (size_t i2 = 0; i2 < robot_num; i2++)
            {
                std::cout << delay_time_matrix(i1,i2) << " ";
            }
            std::cout << std::endl;
        }

        std::cout << "slow factor: " << std::endl; 
        for (size_t i1 = 0; i1 < robot_num; i1++)
        {
            for (size_t i2 = 0; i2 < robot_num; i2++)
            {
                std::cout << slow_factor_matrix(i1,i2) << " ";
            }
            std::cout << std::endl;
        }


        // calculate overall priority
        double time_cc_start = ros::Time::now().toSec();    // start time for overall coordination
        bool deadlock = 0;
        int ordered_num = 0;    // num of robots have got priority (from high to low)
        int order_index = 1;    // priority, could have more than one robots with same priority
        while ((ordered_num < robot_num) && !deadlock)     // all the robots have been ordered?
        {
            for (size_t i1 = 0; i1 < robot_num; i1++)   // find a set of robots {Ri1} with highest priority in unordered robots
            {
                prior_t[i1] = 1;
                // int prior_t = 1;
                if ((priority_overall(i1) == 0))    // Ri1 has not been ordered
                {
                    for (size_t i2 = 0; i2 < robot_num; i2++)
                    {
                        if ((priority_overall(i2) == 0) && (priority_matrix(i1,i2) == 0))   
                        {
                            prior_t[i1] = 0;  // lower than any the unordered robots
                        }
                    }
                }
                else
                {
                    prior_t[i1] = 0;  // Ri1 has been ordered
                }
            }

            deadlock = 1;
            for (size_t i1 = 0; i1 < robot_num; i1++)
            {
                if (prior_t[i1] == 1)     // Ri1 has not been ordered & higher than any other unordered robots
                {
                    priority_overall(i1) = order_index;
                    ordered_num += 1;
                    deadlock = 0;   // this priority set is not empty, no deadlock
                }    
            }
            order_index = ordered_num + 1;
            if (deadlock)
            {
                ROS_INFO("Deadlock");
            }
        }       

        // calculation overall delay/slow factor
        int order_index_max = priority_overall.maxCoeff();   // number of sets with same priority
        for (size_t ip = 1; ip <= order_index_max; ip++)        // for each set
        {
            for (size_t i1 = 0; i1 < robot_num; i1++)
            {
                if (priority_overall(i1) == ip)         // each robot in this set
                {
                    for (size_t i2 = 0; i2 < robot_num; i2++)
                    {
                        // delay: Cd_i1_i2 <- Cd_i1_i2 + Cd_i2 (if Ri1 > Ri2, Cd2 = 0)
                        delay_time_t(i2) = delay_time_matrix(i1,i2) + delay_time(i2); 
                        // slow: Cs_i1_i2 <- Cs_i1_i2 * Cs_i2 (if Ri1 > Ri2, Cs2 = 1)
                        // slow_factor_t(i2) = slow_factor_matrix(i1,i2) * slow_factor(i2);   
                    }
                    // delay: Cd_i1 = max(Cd_i1_i2 + Cd_i2) 
                    delay_time(i1) = delay_time_t.maxCoeff();
                    delay_time_t.setZero();

                    // slow: Cs_i1 = max(Cs_i1_i2 * Cs_i2) 
                    // slow_factor(i1) = slow_factor_t.maxCoeff();
                    // slow_factor_t.setOnes();

                }
            }
        }

         // calculation time for overall coordinator
        double time_cc_end = ros::Time::now().toSec();
        double time_calcu_cc = time_cc_end - time_cc_start;
        std::cout << "Calculation time: " << time_calcu_cc << std::endl;

        std::cout << "overall priority: " << std::endl; 
        for (size_t i = 0; i < robot_num; i++)
        {
            std::cout << priority_overall(i) << ", ";
        }
        std::cout << std::endl;
        std::cout << "overall delay time: " << std::endl; 
        for (size_t i = 0; i < robot_num; i++)
        {
            std::cout << delay_time(i) << " ";
        }
        std::cout << std::endl;
        // std::cout << "overall slow factor: " << std::endl; 
        // for (size_t i = 0; i < robot_num; i++)
        // {
        //     std::cout << slow_factor(i) << " ";
        // }
        // std::cout << std::endl;
        

        // send initial traj to control nodes (publish) 
        if (!deadlock)
        {
            ros::Time time_start;
            ros::Time time_now = ros::Time::now();
            time_start.sec = time_now.toSec() + 3;

            franka_coord::TrajPara para;
            for (size_t i = 0; i < robot_num; i++)
            {
                para.starttime.sec = time_start.sec + delay_time[i];
                para.robotid = i+1;
                para.speedfactor = speed_factors[i];
                para.slowfactor = 1;
                for (size_t j = 0; j < 7; j++)
                {
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
        }
        
        // reset vectors
        for (size_t i = 0; i < robot_num; i++)
        {
            for (size_t j = 0; j < robot_num; j++)
            {
                get_goal(i,j) = 0;
                priority_matrix(i,j) = 0;
                slow_factor_matrix(i,j) = 1;
                delay_time_matrix(i,j) = 0;
            }
            delay_time[i] = 0;  
            delay_time_t[i] = 0; 
            slow_factor[i] = 1;  
            slow_factor_t[i] = 1;  
            priority_overall[i] = 0;
        } 
    }
       

  return true;
}

