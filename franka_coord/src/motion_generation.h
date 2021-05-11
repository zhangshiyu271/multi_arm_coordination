// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <sstream>
#include <iostream>
#include <fstream>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <franka/log.h>
#include <franka/exception.h>

#include <sensor_msgs/JointState.h>
#include "ros/ros.h"



//void setDefaultBehavior(franka::Robot& robot);


class MotionGeneration{

public:
  using Vector7d = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;
  using Vector7i = Eigen::Matrix<int, 7, 1, Eigen::ColMajor>;

  MotionGeneration(double speed_factor, std::array<double, 7> q_0, std::array<double, 7> q_f, std::array<double, 7> dq_0);
  franka::JointPositions operator()(const franka::RobotState& robot_state, franka::Duration period);

  void ReInitialize(double speed_factor, std::array<double, 7> q_f, double slowfactor);
  void ReInitialize2(ros::Publisher pub, double speed_factor, std::array<double, 7> q_f, double slowfactor);
  void ReInitialize3(double speed_factor, std::array<double, 7> q_0, std::array<double, 7> q_f, std::array<double, 7> dq_0);

  bool calculateTminTraj();
  bool proportionalSlowTraj(double slow_factor);
  bool calculateDesiredValues(double t, Vector7d* delta_q_d) const;
  bool generateMotion(double sample_time);
  bool generateMotionOnline(double sample_time);
  void motionInitialize();
  void publishState(franka::RobotState robot_state);

  franka::JointPositions controlOffline(const franka::RobotState& robot_state,franka::Duration period);
  franka::JointPositions controlOnline(const franka::RobotState& robot_state,franka::Duration period);



  /* Trajectory parameter: vm1,...vm7,tf */
  std::array<double, 8> traj_para;
  std::array<double, 7> q_current;
  std::array<double, 7> dq_current;

  // double sample_time = 0.01;
  double t_max = 20;
  double slow_factor = 1;
  bool motion_finished = true;

  // offline =1, online = 2 
  int control_mode = 0;



private:
  // using Vector7d = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;
  // using Vector7i = Eigen::Matrix<int, 7, 1, Eigen::ColMajor>;
  static constexpr double kDeltaQMotionFinished = 1e-6;
  
  Vector7d dq_0_;
  Vector7d q_goal_;
  Vector7d q_0_;      // used to simulate start point 
  Vector7d q_start_;  // real start point
  Vector7d q_delta_; 
  Vector7d sign_q_;
  Vector7d sign_dq_;

  Vector7d ddq_;
  Vector7d dq_m_;
  Vector7d t_1_;
  Vector7d t_2_;
  Vector7d t_f_;

  Vector7d ddq_sync_;
  Vector7d dq_m_sync_;
  Vector7d t_1_sync_;
  Vector7d t_2_sync_;
  double t_f_sync_;

  bool feasible_ = 1;
  bool feasible_q_ = 1;
  bool feasible_dq_ = 1;
  bool feasible_ddq_ = 1;

  Vector7d q_min = (Vector7d() << -2.5,   -1.5,   -2.5,   -2.7,   -2.5,   0.2,   -2.5).finished();
  Vector7d q_max = (Vector7d() << 2.5,    1.5,    2.5,   -0.2,    2.5,    3.5,    2.5).finished();
  Vector7d dq_max_con = (Vector7d() << 2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5).finished();
  Vector7d ddq_max_con = (Vector7d() << 7.5, 3.75, 5, 6.25, 7.5, 10, 10).finished();
  Vector7d dq_max;
  Vector7d ddq_max;

  double time_ = 0.0;
  double time_total_ = 0.0;

  ros::Publisher state_pub;
  sensor_msgs::JointState states;
  

};