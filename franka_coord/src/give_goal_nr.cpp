#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <string>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include "franka_coord/GiveGoal.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "give_goal");
    ros::NodeHandle node_handle;
    ros::NodeHandle node_handle2("~");

    int robot_num;
    int robot_id;
    double speed_factor = 0.05;
    std::vector<double> q_goal = {0, 0, 0, -1.5, 0, 1.5, -0.78};
    int q_goal_num = 0;

    std::vector<double> goals = {
        0, 0, 0, -1.5, 0, 1.5, -0.783521, // 0, initial
        0.645868, 0.943308, 0.0717049, -0.307394, 0.000934956, 2.88331, -0.759619,   // 1
        0.586117, 0.750889, 0.0886796, -0.991252, 0.000361699, 3.30407, -0.712375,   // 2
        0.193814, 0.335989, 0.0497932, -1.28496, 5.07056e-05, 3.22699, -0.783521,       // 3
        0.190988, 0.373458, 0.0470603, -1.62236, 0.000378385, 3.61493, -0.785169,       // 4
        -0.193814, 0.335989, -0.0497932, -1.28496, -5.07056e-05, 3.22699, -0.783521,   // 5
        -0.190987, 0.373458, -0.0470603, -1.62236, -0.000378385, 3.61493, -0.785169,   // 6
        -0.645867, 0.943307, -0.0717049, -0.307394, -0.000934956, 2.88331, -0.759619,   // 7
        -0.586119, 0.750889, -0.0886796, -0.991252, -0.000361699, 3.30407, -0.712375,   // 8
        0.911586, 0.468886, 0.661534, -0.682394, -0.0239772, 2.65923, -1.02752,   // 9, left side
        -0.911586, 0.468886, -0.661534, -0.682394, 0.0239772, 2.65923, -1.02752  // 10, right side      
    };

    node_handle2.getParam("robot_id", robot_id);
    node_handle2.getParam("speed_factor", speed_factor);
    node_handle2.getParam("q_goal_num", q_goal_num);
    node_handle2.getParam("robot_num", robot_num);
    node_handle2.getParam("goals", goals);

    franka_coord::GiveGoal srv;
    srv.request.robotid = robot_id;
    srv.request.speedfactor = speed_factor;
    
    int target_num = goals.size()/7;
    // std::cout << robot_id << " goal: ";
    for (size_t i = 0; i < 7; i++)
    { 
        srv.request.qgoal[i] = goals[q_goal_num*7 + i];
        std::cout << srv.request.qgoal[i] << " ";
    }
    std::cout << std::endl;

    ros::ServiceClient client;
    for (size_t ir = 1; ir <= robot_num; ir++)
    {
        if (ir != robot_id)
        {
            // R(smaller) - R(larger)
            int ir1 = robot_id;
            int ir2 = ir;
            if (robot_id > ir)
            {
                ir1 = ir;
                ir2 = robot_id;
            } 
            client = node_handle.serviceClient<franka_coord::GiveGoal>("give_goal_"+std::to_string(ir1)+"_"+std::to_string(ir2));
            if (client.call(srv))
            {
                ROS_INFO("[COORD(R%d,R%d)] R%d goal sent.", ir1, ir2, srv.request.robotid);
                if (srv.response.received == 1)
                {
                    ROS_INFO("Goal received.");
                }        
            } else
            {
                ROS_ERROR("[COORD(R%d,R%d)] Failed to call R%d goal sending service.", ir1, ir2, srv.request.robotid);
            }
        }
    }
    
    


    return 0;

}