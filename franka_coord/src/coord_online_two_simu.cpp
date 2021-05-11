#include "ros/ros.h"
#include <cstdlib>
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include <time.h>

#include <sstream>
#include <iostream>
#include <fstream>
#include <string>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Core>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include "franka_coord/CoordTwoTrajs.h"
#include "franka_coord/GiveGoal.h"
#include "franka_coord/ReadState.h"
#include "franka_coord/SendTrajPara.h"
#include "franka_coord/SendTwoTrajs.h"
#include "franka_coord/TrajPara.h"

#include "motion_generation.h"

std::vector<int> pair_ir;
std::vector<double> base_position_r1;
std::vector<double> base_position_r2;

int robotid_r1 = 0;
int robotid_r2 = 0;
double slow_factor_r1 = 1;
double slow_factor_r2 = 1;
double speed_factor_r1 = 0;
double speed_factor_r2 = 0;

std::array<double, 7> q_init_r1 = {0, 0, 0, -1.5, 0, 1.5, 0};
std::array<double, 7> q_goal_r1 = {0, 0, 0, -1.5, 0, 1.5, 0};
std::array<double, 7> dq_init_r1 = {0, 0, 0, 0, 0, 0, 0}; 
std::array<double, 7> q_init_r2 = {0, 0, 0, -1.5, 0, 1.5, 0};
std::array<double, 7> q_goal_r2 = {0, 0, 0, -1.5, 0, 1.5, 0};
std::array<double, 7> dq_init_r2 = {0, 0, 0, 0, 0, 0, 0};

bool change_goal_r1 = 0;
bool change_goal_r2 = 0;
bool new_motion_r1 = 0;
bool new_motion_r2 = 0;
bool moving_r1 = 0;
bool moving_r2 = 0;


std::array<double, 7> q_current_r1;
std::array<double, 7> q_current_r2;
int qi_current_r1 = 0;
int qi_current_r2 = 0;
int qi_goal_r1 = 0;
int qi_goal_r2 = 0;
int sample_size_r1;
int sample_size_r2;
int sample_size_max = 100;
Eigen::Matrix<double, 7, Eigen::Dynamic, Eigen::ColMajor> position_joint_r1;
Eigen::Matrix<double, 7, Eigen::Dynamic, Eigen::ColMajor> position_joint_r2;
Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::ColMajor> collision_boundary_r1;
Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::ColMajor> collision_boundary_r2;
bool prior_r1 = 0;
bool prior_r2 = 0;
int num_collision;

bool get_goal(franka_coord::GiveGoal::Request  &req,
              franka_coord::GiveGoal::Response &res);
void update_temp_goal_r2(const sensor_msgs::JointState& msg);
void update_temp_goal_r1(const sensor_msgs::JointState& msg);

void printInfo();
bool collision_detection(std::array<double, 3> segment1_v1, std::array<double, 3> segment1_v2, 
                         std::array<double, 3> segment2_v1, std::array<double, 3> segment2_v2, 
                         double collision_distance);


int main(int argc, char **argv)
{

    ros::init(argc, argv, "coord_online_two");
    ros::NodeHandle node_handle;
    ros::NodeHandle node_handle2("~");
    node_handle2.getParam("sample_size_max",sample_size_max);
    // node_handle2.getParam("base_position_r1",base_position_r1);
    // node_handle2.getParam("base_position_r2",base_position_r2);
    node_handle2.getParam("pair_ir", pair_ir);
    double robot_distance;
    node_handle2.getParam("robot_distance", robot_distance);
    base_position_r1 = {0, -robot_distance*(pair_ir[0]-1), 0};
    base_position_r2 = {0, -robot_distance*(pair_ir[1]-1), 0};

    ROS_INFO("[COORD(R%d,R%d)] Waiting for goals.", pair_ir[0], pair_ir[1]);

    ros::ServiceServer service_getgoal = node_handle.advertiseService("give_goal_"+std::to_string(pair_ir[0])+"_"+std::to_string(pair_ir[1]), get_goal);
    ros::Subscriber state_sub_r1 = node_handle.subscribe("joint_state_r"+std::to_string(pair_ir[0]), 1, update_temp_goal_r2);
    ros::Subscriber state_sub_r2 = node_handle.subscribe("joint_state_r"+std::to_string(pair_ir[1]), 1, update_temp_goal_r1);

    ros::AsyncSpinner spinner(8);
    spinner.start();
    ros::waitForShutdown();
    

    return 0;
}

bool get_goal(franka_coord::GiveGoal::Request  &req,
              franka_coord::GiveGoal::Response &res)
{
  ros::NodeHandle node_handle;

  
  res.received = 1;
  // ROS_INFO("[COORD(R%d,R%d)] R%d goal got.", pair_ir[0], pair_ir[1],req.robotid);

  if (req.robotid == pair_ir[0])
  {
    robotid_r1 = req.robotid;
    for (size_t i = 0; i < 7; i++)
    {   
      change_goal_r1 = change_goal_r1 || std::abs(q_goal_r1[i]-req.qgoal[i]);
    }
    change_goal_r1 = change_goal_r1 || std::abs(speed_factor_r1-req.speedfactor); 
    
    if (change_goal_r1 == 1)
    {
        speed_factor_r1 = req.speedfactor;    
        for (size_t i = 0; i < 7; i++)
        {   
            q_goal_r1[i] = req.qgoal[i];
        }
    }
  }

  if (req.robotid == pair_ir[1])
  {
    robotid_r2 = req.robotid;

    for (size_t i = 0; i < 7; i++)
    {   
      change_goal_r2 = change_goal_r2||std::abs(q_goal_r2[i]-req.qgoal[i]);
    }
    change_goal_r2 = change_goal_r2 || std::abs(speed_factor_r2-req.speedfactor); 
    
    if (change_goal_r2 == 1)
    {
      speed_factor_r2 = req.speedfactor;
      for (size_t i = 0; i < 7; i++)
      {
        q_goal_r2[i] = req.qgoal[i];
      }
    }
  }
  
  
  // ----------------
  // get 2 robot information & goal changes
  // std::cout << robotid_r1 << ", " << robotid_r2 << ", " << change_goal_r1 << ", " << change_goal_r2 << std::endl;

  if (robotid_r1 && robotid_r2 && (change_goal_r1 || change_goal_r2))
  {
    double time_new_goals_got = ros::Time::now().toSec();
    ROS_INFO("[COORD(R%d,R%d)] New goals got.", pair_ir[0], pair_ir[1]);

    // Step 1 - Read initial state
    ros::ServiceClient client_state1 = node_handle.serviceClient<franka_coord::ReadState>("read_state_r"+std::to_string(robotid_r1));
    franka_coord::ReadState srv_state1;
    ros::ServiceClient client_state2 = node_handle.serviceClient<franka_coord::ReadState>("read_state_r"+std::to_string(robotid_r2));
    franka_coord::ReadState srv_state2;
  
    srv_state1.request.robotid = robotid_r1;
    if (client_state1.call(srv_state1))
    {
      // ROS_INFO("Got R%d state.",robotid_r1);
      for (size_t i = 0; i < 7; i++)
      {
        q_init_r1[i] = srv_state1.response.qinit[i];
        dq_init_r1[i] = srv_state1.response.dqinit[i];
      }
    } else
    {
      ROS_ERROR("Failed to call read R%d state service",robotid_r1);
    }

    srv_state2.request.robotid = robotid_r2;
    if (client_state2.call(srv_state2))
    {
      // ROS_INFO("Got R%d state.",robotid_r2);
      for (size_t i = 0; i < 7; i++)
      {
        q_init_r2[i] = srv_state2.response.qinit[i];
        dq_init_r2[i] = srv_state2.response.dqinit[i];
      }
    } else
    {
      ROS_ERROR("Failed to call read R%d state service",robotid_r2);
    }

    // printInfo();

    // Step 2 - Collision check  
    
    double time_cc_start = ros::Time::now().toSec();
    ROS_INFO("[COORD(R%d,R%d)] Calculating coordinated trajetories...",pair_ir[0],pair_ir[1]);

    num_collision = 1;
    bool deadlock = 0;
    double sample_time = 0.1;

    MotionGeneration motion_generation_r1(speed_factor_r1, q_init_r1, q_goal_r1, dq_init_r1);
    MotionGeneration motion_generation_r2(speed_factor_r2, q_init_r2, q_goal_r2, dq_init_r2);
    motion_generation_r1.motionInitialize();
    motion_generation_r2.motionInitialize();

    sample_time = std::max(motion_generation_r1.traj_para[7],motion_generation_r2.traj_para[7])/sample_size_max;
    // std::cout << "sample_time: " << sample_time << std::endl;

    // 1 - Generate path (joint space and Cartesian space)
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("panda_arm");


    // 1.1 - R1
    bool motion_finished_r1 = false;
    sample_size_r1 = ceil(motion_generation_r1.traj_para[7]/sample_time) + 2;

    // Joint space
    position_joint_r1.resize(7,sample_size_r1);
    // Cartesian space
    Eigen::Matrix<double, 27, Eigen::Dynamic, Eigen::ColMajor> position_cartesian_r1;
    position_cartesian_r1.resize(27,sample_size_r1);

    int sample_count_r1 = 0;
    while (!motion_finished_r1) {
      // Joint space
      motion_finished_r1 = motion_generation_r1.generateMotion(sample_time);

      for (size_t i = 0; i < 7; i++) {
        position_joint_r1(i, sample_count_r1) = motion_generation_r1.q_current[i];
      }
      // std::cout << "R1 Sample " << sample_count_r1+1 << ": "
      //   << position_joint_r1(0, sample_count_r1) << ", "
      //   << position_joint_r1(1, sample_count_r1) << ", "
      //   << position_joint_r1(2, sample_count_r1) << ", "
      //   << position_joint_r1(3, sample_count_r1) << ", "
      //   << position_joint_r1(4, sample_count_r1) << ", "
      //   << position_joint_r1(5, sample_count_r1) << ", "
      //   << position_joint_r1(6, sample_count_r1)
      //   << std::endl;

      // Cartesian space
      std::vector<double> joint_values;
      joint_values.resize(7);
      for (size_t i = 0; i < 7; i++)
      {
        joint_values[i] = motion_generation_r1.q_current[i];
      }          
      kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
      // kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
      const Eigen::Isometry3d& state_joint1 = kinematic_state->getGlobalLinkTransform("panda_link1");
      const Eigen::Isometry3d& state_joint2 = kinematic_state->getGlobalLinkTransform("panda_link2");
      const Eigen::Isometry3d& state_joint3 = kinematic_state->getGlobalLinkTransform("panda_link3");
      const Eigen::Isometry3d& state_joint4 = kinematic_state->getGlobalLinkTransform("panda_link4");
      const Eigen::Isometry3d& state_joint5 = kinematic_state->getGlobalLinkTransform("panda_link5");
      const Eigen::Isometry3d& state_joint6 = kinematic_state->getGlobalLinkTransform("panda_link6");
      const Eigen::Isometry3d& state_joint7 = kinematic_state->getGlobalLinkTransform("panda_link7");
      const Eigen::Isometry3d& state_joint8 = kinematic_state->getGlobalLinkTransform("panda_link8");
      
      // joint 1
      position_cartesian_r1(0, sample_count_r1) = state_joint1.translation()[0] + base_position_r1[0];
      position_cartesian_r1(1, sample_count_r1) = state_joint1.translation()[1] + base_position_r1[1];
      position_cartesian_r1(2, sample_count_r1) = state_joint1.translation()[2] + base_position_r1[2];
      // joint 2
      position_cartesian_r1(3, sample_count_r1) = state_joint2.translation()[0] + base_position_r1[0];
      position_cartesian_r1(4, sample_count_r1) = state_joint2.translation()[1] + base_position_r1[1];
      position_cartesian_r1(5, sample_count_r1) = state_joint2.translation()[2] + base_position_r1[2];
      // joint 3
      position_cartesian_r1(6, sample_count_r1) = state_joint3.translation()[0] + base_position_r1[0];
      position_cartesian_r1(7, sample_count_r1) = state_joint3.translation()[1] + base_position_r1[1];
      position_cartesian_r1(8, sample_count_r1) = state_joint3.translation()[2] + base_position_r1[2];
      // joint 4
      position_cartesian_r1(9, sample_count_r1) = state_joint4.translation()[0] + base_position_r1[0];
      position_cartesian_r1(10, sample_count_r1) = state_joint4.translation()[1] + base_position_r1[1];
      position_cartesian_r1(11, sample_count_r1) = state_joint4.translation()[2] + base_position_r1[2];
      // joint 5
      position_cartesian_r1(12, sample_count_r1) = state_joint5.translation()[0] + base_position_r1[0];
      position_cartesian_r1(13, sample_count_r1) = state_joint5.translation()[1] + base_position_r1[1];
      position_cartesian_r1(14, sample_count_r1) = state_joint5.translation()[2] + base_position_r1[2];
      // joint 6
      position_cartesian_r1(15, sample_count_r1) = state_joint6.translation()[0] + base_position_r1[0];
      position_cartesian_r1(16, sample_count_r1) = state_joint6.translation()[1] + base_position_r1[1];
      position_cartesian_r1(17, sample_count_r1) = state_joint6.translation()[2] + base_position_r1[2];
      // joint 7
      position_cartesian_r1(18, sample_count_r1) = state_joint7.translation()[0] + base_position_r1[0];
      position_cartesian_r1(19, sample_count_r1) = state_joint7.translation()[1] + base_position_r1[1];
      position_cartesian_r1(20, sample_count_r1) = state_joint7.translation()[2] + base_position_r1[2];
      // joint 8
      position_cartesian_r1(21, sample_count_r1) = state_joint8.translation()[0] + base_position_r1[0];
      position_cartesian_r1(22, sample_count_r1) = state_joint8.translation()[1] + base_position_r1[1];
      position_cartesian_r1(23, sample_count_r1) = state_joint8.translation()[2] + base_position_r1[2];
      // joint 9
      position_cartesian_r1(24, sample_count_r1) = state_joint8.translation()[0] + base_position_r1[0];
      position_cartesian_r1(25, sample_count_r1) = state_joint8.translation()[1] + base_position_r1[1];
      position_cartesian_r1(26, sample_count_r1) = state_joint8.translation()[2] + base_position_r1[2];
      

      // std::cout << "R1 J1 Cartesian " << sample_count_r1+1 << ": "
      //   << position_cartesian_r1(0, sample_count_r1) << " "
      //   << position_cartesian_r1(1, sample_count_r1) << " "
      //   << position_cartesian_r1(2, sample_count_r1) << " "
        
      //   << position_cartesian_r1(3, sample_count_r1) << " "
      //   << position_cartesian_r1(4, sample_count_r1) << " "
      //   << position_cartesian_r1(5, sample_count_r1) << " "
        
      //   << position_cartesian_r1(6, sample_count_r1) << " "
      //   << position_cartesian_r1(7, sample_count_r1) << " "
      //   << position_cartesian_r1(8, sample_count_r1) << " "

      //   << position_cartesian_r1(9, sample_count_r1) << " "
      //   << position_cartesian_r1(10, sample_count_r1) << " "
      //   << position_cartesian_r1(11, sample_count_r1) << " "

      //   << position_cartesian_r1(12, sample_count_r1) << " "
      //   << position_cartesian_r1(13, sample_count_r1) << " "
      //   << position_cartesian_r1(14, sample_count_r1) << " "

      //   << position_cartesian_r1(15, sample_count_r1) << " "
      //   << position_cartesian_r1(16, sample_count_r1) << " "
      //   << position_cartesian_r1(17, sample_count_r1) << " "

      //   << position_cartesian_r1(18, sample_count_r1) << " "
      //   << position_cartesian_r1(19, sample_count_r1) << " "
      //   << position_cartesian_r1(20, sample_count_r1) << " "

      //   << position_cartesian_r1(21, sample_count_r1) << " "
      //   << position_cartesian_r1(22, sample_count_r1) << " "
      //   << position_cartesian_r1(23, sample_count_r1) << " "

      //   << position_cartesian_r1(24, sample_count_r1) << " "
      //   << position_cartesian_r1(25, sample_count_r1) << " "
      //   << position_cartesian_r1(26, sample_count_r1) << " "
      //   << std::endl;

      sample_count_r1 += 1;
    }

    // 1.2 - R2
    bool motion_finished_r2 = false;
    sample_size_r2 = ceil(motion_generation_r2.traj_para[7]/sample_time) + 2;

    // Joint space
    position_joint_r2.resize(7,sample_size_r2);
    // Cartesian space
    Eigen::Matrix<double, 27, Eigen::Dynamic, Eigen::ColMajor> position_cartesian_r2;
    position_cartesian_r2.resize(27,sample_size_r2);

    int sample_count_r2 = 0;
    while (!motion_finished_r2) {
      // Joint space
      motion_finished_r2 = motion_generation_r2.generateMotion(sample_time);
      for (size_t i = 0; i < 7; i++) {
        position_joint_r2(i, sample_count_r2) = motion_generation_r2.q_current[i];
      }
      // std::cout << "R2 Sample " << sample_count_r2+1 << ": "
      //   << position_joint_r2(0, sample_count_r2) << ", "
      //   << position_joint_r2(1, sample_count_r2) << ", "
      //   << position_joint_r2(2, sample_count_r2) << ", "
      //   << position_joint_r2(3, sample_count_r2) << ", "
      //   << position_joint_r2(4, sample_count_r2) << ", "
      //   << position_joint_r2(5, sample_count_r2) << ", "
      //   << position_joint_r2(6, sample_count_r2)
      //   << std::endl;

      // Cartesian space
      std::vector<double> joint_values;
      joint_values.resize(7);
      for (size_t i = 0; i < 7; i++)
      {
        joint_values[i] = motion_generation_r2.q_current[i];
      }          
      kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
      // kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
      const Eigen::Isometry3d& state_joint1 = kinematic_state->getGlobalLinkTransform("panda_link1");
      const Eigen::Isometry3d& state_joint2 = kinematic_state->getGlobalLinkTransform("panda_link2");
      const Eigen::Isometry3d& state_joint3 = kinematic_state->getGlobalLinkTransform("panda_link3");
      const Eigen::Isometry3d& state_joint4 = kinematic_state->getGlobalLinkTransform("panda_link4");
      const Eigen::Isometry3d& state_joint5 = kinematic_state->getGlobalLinkTransform("panda_link5");
      const Eigen::Isometry3d& state_joint6 = kinematic_state->getGlobalLinkTransform("panda_link6");
      const Eigen::Isometry3d& state_joint7 = kinematic_state->getGlobalLinkTransform("panda_link7");
      const Eigen::Isometry3d& state_joint8 = kinematic_state->getGlobalLinkTransform("panda_link8");
      
      // joint 1
      position_cartesian_r2(0, sample_count_r2) = state_joint1.translation()[0] + base_position_r2[0];
      position_cartesian_r2(1, sample_count_r2) = state_joint1.translation()[1] + base_position_r2[1];
      position_cartesian_r2(2, sample_count_r2) = state_joint1.translation()[2] + base_position_r2[2];
      // joint 2
      position_cartesian_r2(3, sample_count_r2) = state_joint2.translation()[0] + base_position_r2[0];
      position_cartesian_r2(4, sample_count_r2) = state_joint2.translation()[1] + base_position_r2[1];
      position_cartesian_r2(5, sample_count_r2) = state_joint2.translation()[2] + base_position_r2[2];
      // joint 3
      position_cartesian_r2(6, sample_count_r2) = state_joint3.translation()[0] + base_position_r2[0];
      position_cartesian_r2(7, sample_count_r2) = state_joint3.translation()[1] + base_position_r2[1];
      position_cartesian_r2(8, sample_count_r2) = state_joint3.translation()[2] + base_position_r2[2];
      // joint 4
      position_cartesian_r2(9, sample_count_r2) = state_joint4.translation()[0] + base_position_r2[0];
      position_cartesian_r2(10, sample_count_r2) = state_joint4.translation()[1] + base_position_r2[1];
      position_cartesian_r2(11, sample_count_r2) = state_joint4.translation()[2] + base_position_r2[2];
      // joint 5
      position_cartesian_r2(12, sample_count_r2) = state_joint5.translation()[0] + base_position_r2[0];
      position_cartesian_r2(13, sample_count_r2) = state_joint5.translation()[1] + base_position_r2[1];
      position_cartesian_r2(14, sample_count_r2) = state_joint5.translation()[2] + base_position_r2[2];
      // joint 6
      position_cartesian_r2(15, sample_count_r2) = state_joint6.translation()[0] + base_position_r2[0];
      position_cartesian_r2(16, sample_count_r2) = state_joint6.translation()[1] + base_position_r2[1];
      position_cartesian_r2(17, sample_count_r2) = state_joint6.translation()[2] + base_position_r2[2];
      // joint 7
      position_cartesian_r2(18, sample_count_r2) = state_joint7.translation()[0] + base_position_r2[0];
      position_cartesian_r2(19, sample_count_r2) = state_joint7.translation()[1] + base_position_r2[1];
      position_cartesian_r2(20, sample_count_r2) = state_joint7.translation()[2] + base_position_r2[2];
      // joint 8
      position_cartesian_r2(21, sample_count_r2) = state_joint8.translation()[0] + base_position_r2[0];
      position_cartesian_r2(22, sample_count_r2) = state_joint8.translation()[1] + base_position_r2[1];
      position_cartesian_r2(23, sample_count_r2) = state_joint8.translation()[2] + base_position_r2[2];
      // joint 9
      position_cartesian_r2(24, sample_count_r2) = state_joint8.translation()[0] + base_position_r2[0];
      position_cartesian_r2(25, sample_count_r2) = state_joint8.translation()[1] + base_position_r2[1];
      position_cartesian_r2(26, sample_count_r2) = state_joint8.translation()[2] + base_position_r2[2];
      

      // std::cout << "R2 Cartesian " << sample_count_r2+1 << ": "
      //   << position_cartesian_r2(0, sample_count_r2) << " "
      //   << position_cartesian_r2(1, sample_count_r2) << " "
      //   << position_cartesian_r2(2, sample_count_r2) << " "
      
      //   << position_cartesian_r2(3, sample_count_r2) << " "
      //   << position_cartesian_r2(4, sample_count_r2) << " "
      //   << position_cartesian_r2(5, sample_count_r2) << " "
        
      //   << position_cartesian_r2(6, sample_count_r2) << " "
      //   << position_cartesian_r2(7, sample_count_r2) << " "
      //   << position_cartesian_r2(8, sample_count_r2) << " "

      //   << position_cartesian_r2(9, sample_count_r2) << " "
      //   << position_cartesian_r2(10, sample_count_r2) << " "
      //   << position_cartesian_r2(11, sample_count_r2) << " "

      //   << position_cartesian_r2(12, sample_count_r2) << " "
      //   << position_cartesian_r2(13, sample_count_r2) << " "
      //   << position_cartesian_r2(14, sample_count_r2) << " "

      //   << position_cartesian_r2(15, sample_count_r2) << " "
      //   << position_cartesian_r2(16, sample_count_r2) << " "
      //   << position_cartesian_r2(17, sample_count_r2) << " "

      //   << position_cartesian_r2(18, sample_count_r2) << " "
      //   << position_cartesian_r2(19, sample_count_r2) << " "
      //   << position_cartesian_r2(20, sample_count_r2) << " "

      //   << position_cartesian_r2(21, sample_count_r2) << " "
      //   << position_cartesian_r2(22, sample_count_r2) << " "
      //   << position_cartesian_r2(23, sample_count_r2) << " "

      //   << position_cartesian_r2(24, sample_count_r2) << " "
      //   << position_cartesian_r2(25, sample_count_r2) << " "
      //   << position_cartesian_r2(26, sample_count_r2) << " "
      //   << std::endl;

      sample_count_r2 += 1;
    }

    sample_size_r1 = sample_count_r1;
    sample_size_r2 = sample_count_r2;

    // 2 - Generate collision matrix
    const double collision_threshold = 0.18;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> collision_matrix;
    collision_matrix.resize(sample_size_r1, sample_size_r2);
    
    collision_boundary_r1.resize(3, sample_size_r1);
    collision_boundary_r2.resize(3, sample_size_r2);
    
    std::array<double, 3> s1_v1;
    std::array<double, 3> s1_v2;
    std::array<double, 3> s2_v1;
    std::array<double, 3> s2_v2;

    // each pair of samples
    for (size_t i_sample_r1 = 0; i_sample_r1 < sample_size_r1; ++i_sample_r1)
    {
      for (size_t i_sample_r2 = 0; i_sample_r2 < sample_size_r2; ++i_sample_r2)
      {
        // each pair of segments
        bool flag_collision = 0;

        for (size_t i = 0; i < 7; ++i)
        {
          for (size_t j = 0; j < 7; ++j)
          {
            if ((i!=3)&&(j!=3))
            {
              if (flag_collision == 0)
              {
                s1_v1 = {position_cartesian_r1(3*(8-i)+0, i_sample_r1), 
                        position_cartesian_r1(3*(8-i)+1, i_sample_r1), 
                        position_cartesian_r1(3*(8-i)+2, i_sample_r1)};
                s1_v2 = {position_cartesian_r1(3*(7-i)+0, i_sample_r1), 
                        position_cartesian_r1(3*(7-i)+1, i_sample_r1), 
                        position_cartesian_r1(3*(7-i)+2, i_sample_r1)};
                s2_v1 = {position_cartesian_r2(3*(8-j)+0, i_sample_r2), 
                        position_cartesian_r2(3*(8-j)+1, i_sample_r2), 
                        position_cartesian_r2(3*(8-j)+2, i_sample_r2)};
                s2_v2 = {position_cartesian_r2(3*(7-j)+0, i_sample_r2), 
                        position_cartesian_r2(3*(7-j)+1, i_sample_r2), 
                        position_cartesian_r2(3*(7-j)+2, i_sample_r2)};
                if (collision_detection(s1_v1, s1_v2, s2_v1, s2_v2, collision_threshold)==1)
                {
                  flag_collision = 1;
                  collision_matrix(i_sample_r1, i_sample_r2) = 1;
                } else
                {
                  collision_matrix(i_sample_r1, i_sample_r2) = 0;
                }  
              }
            }
          }
        }
      }
    }

    // 3 - Boundaries of collision matrix
    // R1
    for (size_t i_sample_r1 = 0; i_sample_r1 < sample_size_r1; ++i_sample_r1)
    {
      // initialization
      collision_boundary_r1(0, i_sample_r1) = 0;
      collision_boundary_r1(1, i_sample_r1) = sample_size_r2 - 1;
      collision_boundary_r1(2, i_sample_r1) = 0;

      for (size_t i_sample_r2 = 0; i_sample_r2 < sample_size_r2; ++i_sample_r2)
      {
        if (collision_matrix(i_sample_r1, i_sample_r2) == 1)
        {
          collision_boundary_r1(0, i_sample_r1) = 1;
          // update lower boundary
          if (collision_boundary_r1(1,i_sample_r1) > i_sample_r2)
          {
            collision_boundary_r1(1,i_sample_r1) = i_sample_r2;
          }
          // update upper boundary
          if (collision_boundary_r1(2,i_sample_r1) < i_sample_r2)
          {
            collision_boundary_r1(2,i_sample_r1) = i_sample_r2;
          }         
        }  
      }
    } 

    // R2
    for (size_t i_sample_r2 = 0; i_sample_r2 < sample_size_r2; ++i_sample_r2)
    {
      // initialization
      collision_boundary_r2(0, i_sample_r2) = 0;
      collision_boundary_r2(1, i_sample_r2) = sample_size_r1 - 1;
      collision_boundary_r2(2, i_sample_r2) = 0;

      for (size_t i_sample_r1 = 0; i_sample_r1 < sample_size_r1; ++i_sample_r1)
      {
        if (collision_matrix(i_sample_r1, i_sample_r2) == 1)
        {
          collision_boundary_r2(0, i_sample_r2) = 1;
          // update lower boundary
          if (collision_boundary_r2(1,i_sample_r2) > i_sample_r1)
          {
            collision_boundary_r2(1,i_sample_r2) = i_sample_r1;
          }
          // update upper boundary
          if (collision_boundary_r2(2,i_sample_r2) < i_sample_r1)
          {
            collision_boundary_r2(2,i_sample_r2) = i_sample_r1;
          }         
        }  
      } 
    }

    // print collision matrix
    // std::cout << "Collision Matrix" << std::endl;
    // std::ofstream filecm;
    // filecm.open ("/home/zhangshiyu/Documents/Coordination/CM_"+std::to_string(pair_ir[0])+"_"+std::to_string(pair_ir[1])+".txt");
    // for (size_t i_sample_r1 = 0; i_sample_r1 < sample_size_r1; ++i_sample_r1)
    // {
    //   for (size_t i_sample_r2 = 0; i_sample_r2 < sample_size_r2; ++i_sample_r2)
    //   {
    //     // std::cout << collision_matrix(i_sample_r1, i_sample_r2) << " ";   
    //     filecm << collision_matrix(i_sample_r1, i_sample_r2) << " "; 
    //   }
    //   filecm << std::endl;
    // }
    // filecm.close();


    // std::cout << "Boundary R1" << std::endl;
    // for (size_t i = 0; i < 3; i++)
    // {
    //   for (size_t i_sample_r1 = 0; i_sample_r1 < sample_size_r1; ++i_sample_r1)
    //   {
    //     std::cout << collision_boundary_r1(i,i_sample_r1) << " ";
    //   }
    //   std::cout << std::endl;
    // }   

    // std::cout << "Boundary R2" << std::endl;
    // for (size_t i = 0; i < 3; i++)
    // {
    //   for (size_t i_sample_r2 = 0; i_sample_r2 < sample_size_r2; ++i_sample_r2)
    //   {
    //     std::cout << collision_boundary_r2(i,i_sample_r2) << " ";
    //   }
    //   std::cout << std::endl;
    // }
    // std::cout << "sample_size_r1: " << sample_size_r1 <<std::endl;
    // std::cout << "sample_size_r2: " << sample_size_r2 <<std::endl;
    // std::cout << "sample_count_r1: " << sample_count_r1 <<std::endl;
    // std::cout << "sample_count_r2: " << sample_count_r2 <<std::endl;


    // 4 - Priority and deadlock determination
    prior_r1 = 0;
    prior_r2 = 0;
    if ((collision_boundary_r1(0, sample_size_r1-1) == 0) && (collision_boundary_r2(0, 0) == 0))
    {
      prior_r1 = 1;
    }
    if ((collision_boundary_r2(0, sample_size_r2-1) == 0) && (collision_boundary_r1(0, 0) == 0))
    {
      prior_r2 = 1;
    }

    if ((prior_r1 || prior_r2) == 0)
    {
      ROS_INFO("[COORD(R%d,R%d)] Deadlock. Cannot coordinate two trajectories.",pair_ir[0],pair_ir[1]);
      deadlock = 1;
    } else
    {
      // collision exist?
      int num_diagonal = std::min(sample_size_r1, sample_size_r2);  // number of diagonal elements
      num_collision = 0;
      for (size_t i = 0; i < num_diagonal; i++)
      {
        if (collision_matrix(i,i) == 1)
        {
          num_collision += 1;
          //std::cout << "Collision: " << i << std::endl;
        }
      }
     
        double collision_avoidance_margin = ceil(3/sample_time);
        if (prior_r1 == 1)
        { 
          // calculate slow factor
          double max_collision_index_r1;  // in most critical boundary point, index of r1 and r2
          double max_collision_index_r2;
          double max_collision_value_r1 = 0;  // largest exceed
          double max_collision_value_r2 = 0;
          // R1 prior, slow down R2
          for (size_t i_sample_r1 = 0; i_sample_r1 < sample_size_r1; ++i_sample_r1)
          {
            if (i_sample_r1 - collision_boundary_r1(1,i_sample_r1) > max_collision_value_r2)
            {
              max_collision_value_r2 = i_sample_r1 - collision_boundary_r1(1,i_sample_r1);
              max_collision_index_r2 = collision_boundary_r1(1,i_sample_r1);
              max_collision_index_r1 = i_sample_r1;
            }  
          }
          slow_factor_r2 = (max_collision_index_r1 + collision_avoidance_margin)/ max_collision_index_r2;
          // std::cout << "[COORD(R"<< pair_ir[0] <<",R"<< pair_ir[1] <<")] ";
          // std::cout << "Slow factor R"<<robotid_r2<<": " << slow_factor_r2 << std::endl;
        }

        if (prior_r2 == 1)
        { 
          // calculate slow factor
          double max_collision_index_r1;  // in most critical boundary point, index of r1 and r2
          double max_collision_index_r2;
          double max_collision_value_r1 = 0;  // largest exceed
          double max_collision_value_r2 = 0;
          // R2 prior, slow down R1
          for (size_t i_sample_r2 = 0; i_sample_r2 < sample_size_r2; ++i_sample_r2)
          {
            if (i_sample_r2 - collision_boundary_r2(1,i_sample_r2) > max_collision_value_r1)
            {
              max_collision_value_r1 = i_sample_r2 - collision_boundary_r2(1,i_sample_r2);
              max_collision_index_r1 = collision_boundary_r2(1,i_sample_r2);
              max_collision_index_r2 = i_sample_r2;
            }  
          }
          slow_factor_r1 = (max_collision_index_r2 + collision_avoidance_margin)/ max_collision_index_r1;
          // std::cout << "[COORD(R"<< pair_ir[0] <<",R"<< pair_ir[1] <<")] ";
          // std::cout << "Slow factor R"<<robotid_r1<<": " << slow_factor_r1 << std::endl;
        }

        if (prior_r1 && prior_r2)
        {
          if (slow_factor_r1 < slow_factor_r2)
          {
            slow_factor_r2 = 1;
            prior_r1 = 0;
          } else
          {
            slow_factor_r1 = 1;
            prior_r2 = 0;
          }     
        }
        // std::cout << "Collision num: " << num_collision << std::endl;        
        // std::cout << "Prior R"<<robotid_r1<<": " << prior_r1 << std::endl;
        // std::cout << "Prior R"<<robotid_r2<<": " << prior_r2 << std::endl;
        if (prior_r1)
        {
          qi_goal_r1 = sample_size_r1-1;
          qi_goal_r2 = sample_size_r2-1;
          for (size_t i = 0; i < sample_size_r1; i++)
          {
            if (collision_boundary_r1(1, i) < qi_goal_r2)
            {
              qi_goal_r2 = collision_boundary_r1(1, i);
            }
          }
          for (size_t i = 0; i < 7; i++)
          {
            q_goal_r2[i] = position_joint_r2(i,qi_goal_r2);
          }
        }
        if (prior_r2)
        {
          qi_goal_r1 = sample_size_r1-1;
          qi_goal_r2 = sample_size_r2-1;
          for (size_t i = 0; i < sample_size_r2; i++)
          {
            if (collision_boundary_r2(1, i) < qi_goal_r1)
            {
              qi_goal_r1 = collision_boundary_r2(1, i);
            }
          }
          for (size_t i = 0; i < 7; i++)
          {
            q_goal_r1[i] = position_joint_r1(i,qi_goal_r1);
          }
        }

      // double time_cc_end = ros::Time::now().toSec();
      // double time_calcu_cc = time_cc_end - time_cc_start;
      // std::cout << "Calculation time collision check: " << time_calcu_cc << std::endl;      
      ROS_INFO("[COORD(R%d,R%d)] Collision-free trajectories exist.",pair_ir[0],pair_ir[1]);
      // new_motion = 1;
      qi_current_r1 = 0;
      qi_current_r2 = 0;

      
      // Step 3 - send initial traj (R1-R2) to centralized coordinator (service) 
      ros::ServiceClient client_initial_goal = node_handle.serviceClient<franka_coord::SendTwoTrajs>("initial_temp_goal");
      franka_coord::SendTwoTrajs srv_para;
      srv_para.request.robotid1 = robotid_r1;
      srv_para.request.robotid2 = robotid_r2;

      srv_para.request.speedfactor1 = speed_factor_r1;
      srv_para.request.speedfactor2 = speed_factor_r2;
      srv_para.request.qigoalt1 = qi_goal_r1/(double)(sample_size_r1-1);
      srv_para.request.qigoalt2 = qi_goal_r2/(double)(sample_size_r2-1);
      srv_para.request.qigoal1 = 1;
      srv_para.request.qigoal2 = 1;
      
      for (size_t i = 0; i < 7; i++)
      {
          srv_para.request.qgoalt1[i] = position_joint_r1(i,qi_goal_r1);
          srv_para.request.qgoal1[i]  = position_joint_r1(i,sample_size_r1-1);

          srv_para.request.qgoalt2[i] = position_joint_r2(i,qi_goal_r2);
          srv_para.request.qgoal2[i]  = position_joint_r2(i,sample_size_r2-1);
      }

      if (client_initial_goal.call(srv_para))
      {
        ROS_INFO("[COORD(R%d,R%d)] Initial goals sent.",pair_ir[0],pair_ir[1]);
        // std::cout << qi_goal_r2 << " "
        //     << srv_para2.request.qgoal[0] << " "
        //     << srv_para2.request.qgoal[1] << " "
        //     << srv_para2.request.qgoal[2] << " "
        //     << srv_para2.request.qgoal[3] << " "
        //     << srv_para2.request.qgoal[4] << " "
        //     << srv_para2.request.qgoal[5] << " "
        //     << srv_para2.request.qgoal[6] << " "
        //     << std::endl;

      } else
      {
        ROS_INFO("[COORD(R%d,R%d)] Failed to call service: send initial goals.",pair_ir[0],pair_ir[1]);
      }
      
      // if initial goals are both final goals, no need to check (collision matrix all 0)
      // if ((qi_goal_r1 == sample_size_r1-1) && (qi_goal_r2 == sample_size_r2-1))
      if(srv_para.request.qigoalt1 < 1)
      {
        new_motion_r1 = 1;
      }
      if(srv_para.request.qigoalt2 < 1)
      {
        new_motion_r2 = 1;
      }        
    }
    change_goal_r1 = 0;
    change_goal_r2 = 0;
    robotid_r1 = 0;
    robotid_r2 = 0;

  }
  



  return true;
}

void update_temp_goal_r2(const sensor_msgs::JointState& msg)
{
  if (new_motion_r2)  // online coordination
  {
    // R1 priority
    if (prior_r1)
    {       
      // read current state
      for (size_t i = 0; i < 7; i++)
      {
        q_current_r1[i] = msg.position[i];
      }

      // update temporary goal
      bool move_forward = 1;
      while (move_forward)
      {
        for (size_t i = 0; i < 7; i++)
        {
          if ( std::abs(position_joint_r1(i,qi_current_r1) - q_init_r1[i]) - std::abs(q_current_r1[i] - q_init_r1[i]) > 10e-6 )
          {
            move_forward *= 0;
          }
        }
        if (move_forward)
        {
          qi_current_r1 += 1;
          ROS_INFO("[COORD(R%d,R%d)] R%d move to %d", pair_ir[0],pair_ir[1],pair_ir[0],qi_current_r1);
        }
      }
      
      // if (move_forward)
      {        
        // R2 temporary goal changes?
        int qi_goal_t = collision_boundary_r1(1, qi_current_r1);
        for (size_t i = qi_current_r1; i < sample_size_r1; i++)
        {
          if (collision_boundary_r1(1, i) < qi_goal_t)
          {
            qi_goal_t = collision_boundary_r1(1, i);
          }
        }
        // R2 temporary goal changes, update goal
        if (qi_goal_t > qi_goal_r2)
        {
          qi_goal_r2 = qi_goal_t;
          if (qi_goal_r2 == sample_size_r2-1)
          {
            new_motion_r2 = 0;
          }    
        
          // send updated temp goal
          ros::NodeHandle node_handle;
          ros::ServiceClient client_update_goal = node_handle.serviceClient<franka_coord::SendTwoTrajs>("update_temp_goal");
          franka_coord::SendTwoTrajs srv_para;
          srv_para.request.robotid1 = pair_ir[0];
          srv_para.request.robotid2 = pair_ir[1];
          srv_para.request.qigoalt1 = qi_goal_r1/(double)(sample_size_r1-1);
          srv_para.request.qigoalt2 = qi_goal_r2/(double)(sample_size_r2-1);
          
          for (size_t i = 0; i < 7; i++)
          {
              srv_para.request.qgoalt1[i] = position_joint_r1(i,qi_goal_r1);
              srv_para.request.qgoalt2[i] = position_joint_r2(i,qi_goal_r2);
          }

          if (client_update_goal.call(srv_para))
          {
            ROS_INFO("[COORD(R%d,R%d)] R%d temp goal updated.",pair_ir[0],pair_ir[1],pair_ir[1]);
            // std::cout << q_goal_r2 << " "
            //     << srv_para.request.qgoalt2[0] << " "
            //     << srv_para.request.qgoalt2[1] << " "
            //     << srv_para.request.qgoalt2[2] << " "
            //     << srv_para.request.qgoalt2[3] << " "
            //     << srv_para.request.qgoalt2[4] << " "
            //     << srv_para.request.qgoalt2[5] << " "
            //     << srv_para.request.qgoalt2[6] << " "
            //     << std::endl;

          } else
          {
            ROS_INFO("[COORD(R%d,R%d)] Failed to call service: R%d temp goal update.",pair_ir[0],pair_ir[1],pair_ir[1]);
          }
        }  // R2 goal update
      }  // R1 move to next sample
    }  // R1 priority
  }  // Moving    
}

void update_temp_goal_r1(const sensor_msgs::JointState& msg)
{
  if (new_motion_r1)  // online coordination
  {
    // R1 priority
    if (prior_r2)
    {       
      // read current state
      for (size_t i = 0; i < 7; i++)
      {
        q_current_r2[i] = msg.position[i];
      }

      // update temporary goal
      bool move_forward = 1;
      while (move_forward)
      {
        for (size_t i = 0; i < 7; i++)
        {
          if ( std::abs(position_joint_r2(i,qi_current_r2) - q_init_r2[i]) - std::abs(q_current_r2[i] - q_init_r2[i]) > 10e-6 )
          {
            move_forward *= 0;
          }
        }
        if (move_forward)
        {
          qi_current_r2 += 1;
          ROS_INFO("[COORD(R%d,R%d)] R%d move to %d", pair_ir[0],pair_ir[1],pair_ir[1],qi_current_r2);
        }
      }
      
      // if (move_forward)
      {        
        // R2 temporary goal changes?
        int qi_goal_t = collision_boundary_r2(1, qi_current_r2);
        for (size_t i = qi_current_r2; i < sample_size_r2; i++)
        {
          if (collision_boundary_r2(1, i) < qi_goal_t)
          {
            qi_goal_t = collision_boundary_r2(1, i);
          }
        }
        // R2 temporary goal changes, update goal
        if (qi_goal_t > qi_goal_r1)
        {
          qi_goal_r1 = qi_goal_t;
          if (qi_goal_r1 == sample_size_r1-1)
          {
            new_motion_r1 = 0;
          }    
        
          // send updated temp goal
          ros::NodeHandle node_handle;
          ros::ServiceClient client_update_goal = node_handle.serviceClient<franka_coord::SendTwoTrajs>("update_temp_goal");
          franka_coord::SendTwoTrajs srv_para;
          srv_para.request.robotid1 = pair_ir[0];
          srv_para.request.robotid2 = pair_ir[1];
          srv_para.request.qigoalt1 = qi_goal_r1/(double)(sample_size_r1-1);
          srv_para.request.qigoalt2 = qi_goal_r2/(double)(sample_size_r2-1);
          
          for (size_t i = 0; i < 7; i++)
          {
              srv_para.request.qgoalt1[i] = position_joint_r1(i,qi_goal_r1);
              srv_para.request.qgoalt2[i] = position_joint_r2(i,qi_goal_r2);
          }

          if (client_update_goal.call(srv_para))
          {
            ROS_INFO("[COORD(R%d,R%d)] R%d temp goal updated.",pair_ir[0],pair_ir[1],pair_ir[0]);
            // std::cout << q_goal_r2 << " "
            //     << srv_para.request.qgoalt2[0] << " "
            //     << srv_para.request.qgoalt2[1] << " "
            //     << srv_para.request.qgoalt2[2] << " "
            //     << srv_para.request.qgoalt2[3] << " "
            //     << srv_para.request.qgoalt2[4] << " "
            //     << srv_para.request.qgoalt2[5] << " "
            //     << srv_para.request.qgoalt2[6] << " "
            //     << std::endl;

          } else
          {
            ROS_INFO("[COORD(R%d,R%d)] Failed to call service: R%d temp goal update.",pair_ir[0],pair_ir[1],pair_ir[0]);
          }
        }  // R2 goal update
      }  // R1 move to next sample
    }  // R1 priority
  }  // Moving    


}


void printInfo()
{
  std::cout<< "robot id: " << robotid_r1 << std::endl;
  std::cout << "Init position: " 
      << q_init_r1[0] << ", "
      << q_init_r1[1] << ", "
      << q_init_r1[2] << ", "
      << q_init_r1[3] << ", "
      << q_init_r1[4] << ", "
      << q_init_r1[5] << ", "
      << q_init_r1[6] << ", "
      << std::endl;
  std::cout << "Goal position: " 
      << q_goal_r1[0] << ", "
      << q_goal_r1[1] << ", "
      << q_goal_r1[2] << ", "
      << q_goal_r1[3] << ", "
      << q_goal_r1[4] << ", "
      << q_goal_r1[5] << ", "
      << q_goal_r1[6] << ", "
      << std::endl;
  std::cout << "Speed factor: " << speed_factor_r1 << std::endl;

  std::cout<< "robot id: " << robotid_r2 << std::endl;
  std::cout << "Init position: " 
      << q_init_r2[0] << ", "
      << q_init_r2[1] << ", "
      << q_init_r2[2] << ", "
      << q_init_r2[3] << ", "
      << q_init_r2[4] << ", "
      << q_init_r2[5] << ", "
      << q_init_r2[6] << ", "
      << std::endl;
  std::cout << "Goal position: " 
      << q_goal_r2[0] << ", "
      << q_goal_r2[1] << ", "
      << q_goal_r2[2] << ", "
      << q_goal_r2[3] << ", "
      << q_goal_r2[4] << ", "
      << q_goal_r2[5] << ", "
      << q_goal_r2[6] << ", "
      << std::endl;
  std::cout << "Speed factor: " << speed_factor_r2 << std::endl;

}

/* Collision detection of two segments */
/* Input */
/* Segment1: [(x1 y1 z1) (x2 y2 z2)] Vertice positions of segment 1 */
/* Segment2: [(x3 y3 z3) (x4 y4 z4)] Vertice positions of segment 2 */
/* width of the arm */
bool collision_detection(std::array<double, 3> segment1_v1, std::array<double, 3> segment1_v2, 
                         std::array<double, 3> segment2_v1, std::array<double, 3> segment2_v2, 
                         double collision_distance) {
  
  double x1 = segment1_v1[0];
  double y1 = segment1_v1[1];
  double z1 = segment1_v1[2];

  double x2 = segment1_v2[0];
  double y2 = segment1_v2[1];
  double z2 = segment1_v2[2];

  double x3 = segment2_v1[0];
  double y3 = segment2_v1[1];
  double z3 = segment2_v1[2];

  double x4 = segment2_v2[0];
  double y4 = segment2_v2[1];
  double z4 = segment2_v2[2];

  // sort x, y, z
  // segment 1
  double x_seg1_small = std::min(x1,x2);
  double x_seg1_large = std::max(x1,x2);
  double y_seg1_small = std::min(y1,y2);
  double y_seg1_large = std::max(y1,y2);
  double z_seg1_small = std::min(z1,z2);
  double z_seg1_large = std::max(z1,z2);
  // segment 2
  double x_seg2_small = std::min(x3,x4);
  double x_seg2_large = std::max(x3,x4);
  double y_seg2_small = std::min(y3,y4);
  double y_seg2_large = std::max(y3,y4);
  double z_seg2_small = std::min(z3,z4);
  double z_seg2_large = std::max(z3,z4);

  if((x_seg1_small - x_seg2_large > collision_distance) || (x_seg2_small - x_seg1_large > collision_distance)) {
    return 0;
  } else if ((y_seg1_small - y_seg2_large > collision_distance) || (y_seg2_small - y_seg1_large > collision_distance))
  {
    return 0;
  }  else if ((z_seg1_small - z_seg2_large > collision_distance) || (z_seg2_small - z_seg1_large > collision_distance))
  {
    return 0;
  } else {
    double A = std::pow((x2-x1), 2.0) + std::pow((y2-y1), 2.0) + std::pow((z2-z1), 2.0);
    double B = std::pow((x3-x4), 2.0) + std::pow((y3-y4), 2.0) + std::pow((z3-z4), 2.0);
    double C = 2*( (x2-x1)*(x1-x3) + (y2-y1)*(y1-y3) + (z2-z1)*(z1-z3) );
    double D = 2*( (x3-x4)*(x1-x3) + (y3-y4)*(y1-y3) + (z3-z4)*(z1-z3) );
    double E = 2*( (x2-x1)*(x3-x4) + (y2-y1)*(y3-y4) + (z2-z1)*(z3-z4) );
    double F = std::pow((x1-x3), 2.0) + std::pow((y1-y3), 2.0) + std::pow((z1-z3), 2.0);
    // std::cout << "v1: "<< x1 << ", "<<  y1 << ", " << z1 << std::endl;
    // std::cout << "v2: "<< x2 << ", "<<  y2 << ", " << z2 << std::endl;
    // std::cout << "v3: "<< x3 << ", "<<  y3 << ", " << z3 << std::endl;
    // std::cout << "v4: "<< x4 << ", "<<  y4 << ", " << z4 << std::endl;

    double coeff_seg1;  // coefficient of the point on segment 1
    double coeff_seg2;  // coefficient of the point on segment 2
    double min_distance;  // minimal distance between two segments

    if ((4*A*B-std::pow(E,2.0))==0)
    {
      coeff_seg1 = 0;
      coeff_seg2 = -C/E;
      min_distance = A*std::pow(coeff_seg1,2.0) + B*std::pow(coeff_seg2,2.0) + C*coeff_seg1 + D*coeff_seg2 + E*coeff_seg1*coeff_seg2 + F;
      
      
    } else {

      double coeff_seg1_extm = (D*E-2*B*C)/(4*A*B-std::pow(E,2.0));   // extreme point, coefficient of the point on segment 1
      double coeff_seg2_extm = (C*E-2*A*D)/(4*A*B-std::pow(E,2.0));   // extreme point, coefficient of the point on segment 2   

      if ( (coeff_seg1_extm < 0) || (coeff_seg1_extm > 1) ) {
        if ( (coeff_seg2_extm < 0) || (coeff_seg2_extm > 1) ) {
          Eigen::Matrix<double, 4, 1, Eigen::ColMajor> distance;
          coeff_seg1 = 0;
          coeff_seg2 = 0;
          distance[0] = A*std::pow(coeff_seg1,2.0) + B*std::pow(coeff_seg2,2.0) + C*coeff_seg1 + D*coeff_seg2 + E*coeff_seg1*coeff_seg2 + F;        
          coeff_seg1 = 0;
          coeff_seg2 = 1;
          distance[1] = A*std::pow(coeff_seg1,2.0) + B*std::pow(coeff_seg2,2.0) + C*coeff_seg1 + D*coeff_seg2 + E*coeff_seg1*coeff_seg2 + F;        
          coeff_seg1 = 1;
          coeff_seg2 = 0;
          distance[2] = A*std::pow(coeff_seg1,2.0) + B*std::pow(coeff_seg2,2.0) + C*coeff_seg1 + D*coeff_seg2 + E*coeff_seg1*coeff_seg2 + F;        
          coeff_seg1 = 1;
          coeff_seg2 = 1;
          distance[3] = A*std::pow(coeff_seg1,2.0) + B*std::pow(coeff_seg2,2.0) + C*coeff_seg1 + D*coeff_seg2 + E*coeff_seg1*coeff_seg2 + F;        

          min_distance = distance.minCoeff();

        } else {
          Eigen::Matrix<double, 2, 1, Eigen::ColMajor> distance;
          coeff_seg2 = coeff_seg2_extm;
          coeff_seg1 = 0;
          distance[0] = A*std::pow(coeff_seg1,2.0) + B*std::pow(coeff_seg2,2.0) + C*coeff_seg1 + D*coeff_seg2 + E*coeff_seg1*coeff_seg2 + F;        
          coeff_seg1 = 1;
          distance[1] = A*std::pow(coeff_seg1,2.0) + B*std::pow(coeff_seg2,2.0) + C*coeff_seg1 + D*coeff_seg2 + E*coeff_seg1*coeff_seg2 + F;        
          
          min_distance = distance.minCoeff();

        }
      } else {
        if ( (coeff_seg2_extm < 0) || (coeff_seg2_extm > 1) ) {
          Eigen::Matrix<double, 2, 1, Eigen::ColMajor> distance;
          coeff_seg1 = coeff_seg1_extm;
          coeff_seg2 = 0;
          distance[0] = A*std::pow(coeff_seg1,2.0) + B*std::pow(coeff_seg2,2.0) + C*coeff_seg1 + D*coeff_seg2 + E*coeff_seg1*coeff_seg2 + F;        
          coeff_seg2 = 1;
          distance[1] = A*std::pow(coeff_seg1,2.0) + B*std::pow(coeff_seg2,2.0) + C*coeff_seg1 + D*coeff_seg2 + E*coeff_seg1*coeff_seg2 + F;        

          min_distance = distance.minCoeff();
          
        } else {
          coeff_seg1 = coeff_seg1_extm;
          coeff_seg2 = coeff_seg1_extm;
          min_distance = A*std::pow(coeff_seg1,2.0) + B*std::pow(coeff_seg2,2.0) + C*coeff_seg1 + D*coeff_seg2 + E*coeff_seg1*coeff_seg2 + F;        
        }
      }
    }
  
    if (std::sqrt(min_distance) > collision_distance)
    {
      return 0;
    } else {
      return 1;
    }
    std::cout << "s1_v1: " << x1 << ", " << y1 << ", " << z1 << std::endl;
    std::cout << "s1_v2: " << x2 << ", " << y2 << ", " << z2 << std::endl;
    std::cout << "s2_v1: " << x3 << ", " << y3 << ", " << z3 << std::endl;
    std::cout << "s2_v2: " << x4 << ", " << y4 << ", " << z4 << std::endl;
    std::cout << "distance: " << std::sqrt(min_distance) << std::endl;
  }

}