// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "motion_generation.h"

std::ofstream motiondataq;
std::ofstream motiondatadq;
std::ofstream motiondatat;



int determineSign(double num)
  {    
    if(num > 0){
      return 1;
    } else {
      return -1;
    }
  }

MotionGeneration::MotionGeneration(double speed_factor, std::array<double, 7> q_0, std::array<double, 7> q_f, std::array<double, 7> dq_0)
{
  motiondataq.open ("/home/frankanuc03/Documents/Coordination/data_real_q1.txt");
  motiondatadq.open ("/home/frankanuc03/Documents/Coordination/data_real_dq1.txt");
  motiondatat.open ("/home/frankanuc03/Documents/Coordination/datat.txt");
  
  states.position.resize(7);
  states.velocity.resize(7);

  dq_max = speed_factor * dq_max_con;
  ddq_max = speed_factor * ddq_max_con;

  for (size_t i = 0; i < 7; i++) {
    q_current[i] = q_0[i];
    dq_current[i] = dq_0[i];

    q_0_[i] = q_0[i];
    q_goal_[i] = q_f[i];
    dq_0_[i] = dq_0[i];             //v0
  }

  ddq_.setZero();
  dq_m_.setZero();
  t_1_.setZero();
  t_2_.setZero();
  t_f_.setZero();

  ddq_sync_.setZero();
  dq_m_sync_.setZero();
  t_1_sync_.setZero();
  t_2_sync_.setZero();
  t_f_sync_ = 0;
}

void MotionGeneration::ReInitialize(double speed_factor, std::array<double, 7> q_f, double slowfactor)
{
  dq_max = speed_factor * dq_max_con;
  ddq_max = speed_factor * ddq_max_con;
  slow_factor = slowfactor;

  for (size_t i = 0; i < 7; i++) {
    q_goal_[i] = q_f[i];           //v0
  }
}

void MotionGeneration::ReInitialize2(ros::Publisher pub, double speed_factor, std::array<double, 7> q_f, double slowfactor)
{
  state_pub = pub;

  dq_max = speed_factor * dq_max_con;
  ddq_max = speed_factor * ddq_max_con;
  // slow_factor = slowfactor;

  for (size_t i = 0; i < 7; i++) {
    q_goal_[i] = q_f[i];           //v0
  }
}

void MotionGeneration::ReInitialize3(double speed_factor, std::array<double, 7> q_0, std::array<double, 7> q_f, std::array<double, 7> dq_0)
{
  states.position.resize(7);
  states.velocity.resize(7);

  dq_max = speed_factor * dq_max_con;
  ddq_max = speed_factor * ddq_max_con;

  for (size_t i = 0; i < 7; i++) {
    q_current[i] = q_0[i];
    dq_current[i] = dq_0[i];

    q_0_[i] = q_0[i];
    q_goal_[i] = q_f[i];
    dq_0_[i] = dq_0[i];             //v0
  }

  ddq_.setZero();
  dq_m_.setZero();
  t_1_.setZero();
  t_2_.setZero();
  t_f_.setZero();

  ddq_sync_.setZero();
  dq_m_sync_.setZero();
  t_1_sync_.setZero();
  t_2_sync_.setZero();
  t_f_sync_ = 0;
}



bool MotionGeneration::calculateTminTraj(){
  // traj_para: a, vm, t1, t2, tf

  /* Tmin traj of each joint */
  for (size_t i = 0; i < 7; i++) {
    ddq_[i] = ddq_max[i];
    dq_m_[i] = std::sqrt(std::pow(dq_0_[i], 2.0)/2.0 + ddq_max[i]*q_delta_[i]);
    if (dq_max[i] > dq_m_[i]){
      t_f_[i] = (2.0*dq_m_[i] - dq_0_[i]) / ddq_max[i];
    } else{
      t_f_[i] = q_delta_[i]/dq_max[i] + (dq_max[i] - dq_0_[i] + std::pow(dq_0_[i], 2.0)/(2.0*dq_max[i])) / ddq_max[i];
    }
  }

   /* Synchronized min motion time */ 
  t_f_sync_ = t_f_.maxCoeff();

  /* Adjust each traj to sync */
  std::array<double, 7> dq_m_tri = {0,0,0,0,0,0,0};
  for (size_t i = 0; i < 7; i++) {
    if(q_delta_[i] > kDeltaQMotionFinished){
      dq_m_tri[i] = q_delta_[i]/t_f_sync_ + std::sqrt(std::pow((q_delta_[i]/t_f_sync_ - dq_0_[i]/2.0),2.0) + std::pow(dq_0_[i],2.0)/4.0);     // t1=t2, triangle
      dq_m_sync_[i] = std::min(dq_m_tri[i],dq_max[i]);        // triangle or trapezoid
      ddq_sync_[i] = (dq_0_[i]*dq_m_sync_[i] - std::pow(dq_m_sync_[i],2.0) - std::pow(dq_0_[i],2.0)/2.0) / (q_delta_[i] - t_f_sync_*dq_m_sync_[i]);
      t_1_sync_[i] = (dq_m_sync_[i]-dq_0_[i])/ddq_sync_[i];
      t_2_sync_[i] = t_f_sync_-dq_m_sync_[i]/ddq_sync_[i];
    } else{
      dq_m_sync_[i] = 0;
      ddq_sync_[i] = 0;
      t_1_sync_[i] = 0;
      t_2_sync_[i] = 0;
    }
      // if vel exceeds limit?
      if(dq_max[i]-dq_m_sync_[i] < -std::pow(10,-10)){
        feasible_dq_ *= 0;
      }
      // if acc exceeds limit?
      if(ddq_max[i]-ddq_sync_[i] < -std::pow(10,-10)){
        feasible_ddq_ *= 0;
      }
  }

  /* Synchronized tmin traj para */
  traj_para[0] = dq_m_sync_[0];
  traj_para[1] = dq_m_sync_[1];
  traj_para[2] = dq_m_sync_[2];
  traj_para[3] = dq_m_sync_[3];
  traj_para[4] = dq_m_sync_[4];
  traj_para[5] = dq_m_sync_[5];
  traj_para[6] = dq_m_sync_[6];
  traj_para[7] = t_f_sync_;
  
  // std::cout << "Tmin traj para: "
  //           << traj_para[0] << ", "
  //           << traj_para[1] << ", "
  //           << traj_para[2] << ", "
  //           << traj_para[3] << ", "
  //           << traj_para[4] << ", "
  //           << traj_para[5] << ", "
  //           << traj_para[6] << ", "
  //           << traj_para[7]
  //           << std::endl;

  feasible_ = feasible_q_ * feasible_dq_ * feasible_ddq_;
  if(feasible_ == 1){
    return true;
  } else{
    return false;
  }
}


bool MotionGeneration::proportionalSlowTraj(double slow_factor){
  
    t_f_sync_ *= slow_factor;
    for (int i = 0; i < 7; ++i)
    {
      t_1_sync_[i] *= slow_factor;
      t_2_sync_[i] *= slow_factor;
      dq_m_sync_[i] /= slow_factor;
      ddq_sync_[i] = ddq_sync_[i]/std::pow(slow_factor,2.0);
    }

    traj_para[0] = dq_m_sync_[0];
    traj_para[1] = dq_m_sync_[1];
    traj_para[2] = dq_m_sync_[2];
    traj_para[3] = dq_m_sync_[3];
    traj_para[4] = dq_m_sync_[4];
    traj_para[5] = dq_m_sync_[5];
    traj_para[6] = dq_m_sync_[6];
    traj_para[7] = t_f_sync_;

      // std::cout << "Slow traj para: "
      //       << traj_para[0] << ", "
      //       << traj_para[1] << ", "
      //       << traj_para[2] << ", "
      //       << traj_para[3] << ", "
      //       << traj_para[4] << ", "
      //       << traj_para[5] << ", "
      //       << traj_para[6] << ", "
      //       << traj_para[7]
      //       << std::endl;
    
    return true;
}

bool MotionGeneration::calculateDesiredValues(double t, Vector7d* delta_q_d) const {

  std::array<bool, 7> joint_motion_finished{};

  for (size_t i = 0; i < 7; i++) {
    if (std::abs(q_delta_[i]) < kDeltaQMotionFinished) {
      (*delta_q_d)[i] = 0;
      joint_motion_finished[i] = true;
    } else {
      if (t < t_1_sync_[i]) {
        (*delta_q_d)[i] = dq_0_[i]*t + 0.5*ddq_sync_[i]*std::pow(t, 2.0);
      } else if (t >= t_1_sync_[i] && t < t_2_sync_[i]) {
        (*delta_q_d)[i] = 
        dq_0_[i]*t_1_sync_[i] + 0.5*ddq_sync_[i]*std::pow(t_1_sync_[i], 2.0) +
        (t-t_1_sync_[i])*dq_m_sync_[i];
      } else if (t >= t_2_sync_[i] && t < t_f_sync_) {
        (*delta_q_d)[i] = dq_0_[i]*t_1_sync_[i] + 0.5*ddq_sync_[i]*std::pow(t_1_sync_[i], 2.0) +
        (t_2_sync_[i]-t_1_sync_[i])*dq_m_sync_[i] +
        dq_m_sync_[i]*(t-t_2_sync_[i])-0.5*ddq_sync_[i]*std::pow(t-t_2_sync_[i], 2.0);
      } else {
        (*delta_q_d)[i] = q_delta_[i];
        joint_motion_finished[i] = true;
      }
    }
    (*delta_q_d)[i] *= sign_q_[i];
  }

  return std::all_of(joint_motion_finished.cbegin(), joint_motion_finished.cend(),
                     [](bool x) { return x; });
}

void MotionGeneration::motionInitialize(){ 
    for (size_t i = 0; i < 7; i++) {
      q_start_[i] = q_0_[i];
      q_delta_[i] = std::abs(q_goal_[i] - q_start_[i]);     //qc value
      sign_q_[i] = determineSign(q_goal_[i] - q_start_[i]); //qc sign
    }
    calculateTminTraj();
    proportionalSlowTraj(slow_factor);    
}

// offline
bool MotionGeneration::generateMotion(double sample_time) {  
  if (time_ == 0.0) {
    for (size_t i = 0; i < 7; i++) {
      q_start_[i] = q_0_[i];    // for collision check
      q_delta_[i] = std::abs(q_goal_[i] - q_start_[i]);     //qc value
      sign_q_[i] = determineSign(q_goal_[i] - q_start_[i]); //qc sign
    }   
    calculateTminTraj();
    proportionalSlowTraj(slow_factor);
  }

  Vector7d delta_q_d;
  time_ += sample_time;
  motion_finished = calculateDesiredValues(time_, &delta_q_d);

  std::array<double, 7> joint_positions;
  Eigen::VectorXd::Map(&joint_positions[0], 7) = (q_start_ + delta_q_d);
  for (size_t i = 0; i < 7; i++)
  {
    dq_current[i] = (joint_positions[i] - q_current[i]) / sample_time;
  }
  q_current = joint_positions;

  if (motion_finished) {time_ = 0;}

  return motion_finished;
}

bool MotionGeneration::generateMotionOnline(double sample_time) { 
    
  time_ = 0;
  
  if (time_ == 0.0) {
    for (size_t i = 0; i < 7; i++) {
      q_start_[i] = q_current[i];
      q_delta_[i] = std::abs(q_goal_[i] - q_start_[i]);     //qc value
      sign_q_[i] = determineSign(q_goal_[i] - q_start_[i]); //qc sign

      sign_dq_[i] = determineSign(dq_current[i]);
      dq_0_[i] = sign_dq_[i]*sign_q_[i] * std::abs(dq_current[i]);
      
    }
   
    calculateTminTraj();
  }

  Vector7d delta_q_d;
  time_ += sample_time;
  motion_finished = calculateDesiredValues(time_, &delta_q_d);

  std::array<double, 7> joint_positions;
  Eigen::VectorXd::Map(&joint_positions[0], 7) = (q_start_ + delta_q_d);
  for (size_t i = 0; i < 7; i++)
  {
    dq_current[i] = (joint_positions[i] - q_current[i]) / sample_time;
  }
  q_current = joint_positions;

  if (motion_finished) {time_ = 0;}

  return motion_finished;
}



void MotionGeneration::publishState(franka::RobotState robot_state) {
  for (size_t i = 0; i < 7; i++) {
      states.position[i] = robot_state.q_d[i];
      states.velocity[i] = robot_state.dq_d[i];
  }
  state_pub.publish(states);
}


franka::JointPositions MotionGeneration::controlOffline(const franka::RobotState& robot_state,franka::Duration period) {

  time_ += period.toSec();
  if (time_ == 0.0) 
  {
    dq_0_ = Vector7d(robot_state.dq_d.data());
    q_start_ = Vector7d(robot_state.q_d.data());
    for (size_t i = 0; i < 7; i++) {
      q_delta_[i] = std::abs(q_goal_[i] - q_start_[i]);     //qc
      sign_q_[i] = determineSign(q_goal_[i] - q_start_[i]);
    }

    calculateTminTraj();
    proportionalSlowTraj(slow_factor);  
  }

  Vector7d delta_q_d;
  motion_finished = calculateDesiredValues(time_, &delta_q_d);
  if (motion_finished) {time_ = 0;}

  std::array<double, 7> joint_positions;
  Eigen::VectorXd::Map(&joint_positions[0], 7) = (q_start_ + delta_q_d);

  // franka::JointPositions output(joint_positions);
  // output.motion_finished = motion_finished;
  
  return joint_positions;
}

franka::JointPositions MotionGeneration::controlOnline(const franka::RobotState& robot_state,franka::Duration period) {

  publishState(robot_state);
  
  // if (time_ > 5)
  {
    time_ = 0;
  }

  
  if (time_total_ == 0.0) 
  {    
    dq_0_ = Vector7d(robot_state.dq_d.data());
    q_start_ = Vector7d(robot_state.q_d.data());
    for (size_t i = 0; i < 7; i++) {
      q_delta_[i] = std::abs(q_goal_[i] - q_start_[i]);     //qc
      sign_q_[i] = determineSign(q_goal_[i] - q_start_[i]);

      sign_dq_[i] = determineSign(dq_0_[i]);
      dq_0_[i] = sign_dq_[i]*sign_q_[i] * std::abs(dq_0_[i]);
    }

    calculateTminTraj();
  }

  if ((time_total_ != 0) && (time_ == 0.0)) 
  {        
    for (size_t i = 0; i < 7; i++) {
      q_start_[i] = q_current[i];
      q_delta_[i] = std::abs(q_goal_[i] - q_start_[i]);     //qc value
      sign_q_[i] = determineSign(q_goal_[i] - q_start_[i]); //qc sign

      sign_dq_[i] = determineSign(dq_current[i]);
      dq_0_[i] = sign_dq_[i]*sign_q_[i] * std::abs(dq_current[i]);
      
    }

    calculateTminTraj();
  }

  motiondataq << robot_state.q_d[0] << " "
              << robot_state.q_d[1] << " "
              << robot_state.q_d[2] << " "
              << robot_state.q_d[3] << " "
              << robot_state.q_d[4] << " "
              << robot_state.q_d[5] << " "
              << robot_state.q_d[6] << " "
              << std::endl;
  motiondatadq <<robot_state.dq_d[0] << " "
              << robot_state.dq_d[1] << " "
              << robot_state.dq_d[2] << " "
              << robot_state.dq_d[3] << " "
              << robot_state.dq_d[4] << " "
              << robot_state.dq_d[5] << " "
              << robot_state.dq_d[6] << " "
              << std::endl;
  motiondatat << time_total_ << " " << time_ << std::endl;

  time_ += period.toSec();
  time_total_ += period.toSec();

  Vector7d delta_q_d;
  motion_finished = calculateDesiredValues(time_, &delta_q_d);
  if (motion_finished) {
    time_ = 0;
    time_total_ = 0;

    motiondataq.close();
    motiondatadq.close();
  }

  std::array<double, 7> joint_positions;
  Eigen::VectorXd::Map(&joint_positions[0], 7) = (q_start_ + delta_q_d);

  for (size_t i = 0; i < 7; i++)
  {
    dq_current[i] = (joint_positions[i] - q_current[i]) / period.toSec();
    q_current[i] = joint_positions[i];
  }  
  // motiondataq << joint_positions[0] << " "
  //             << joint_positions[1] << " "
  //             << joint_positions[2] << " "
  //             << joint_positions[3] << " "
  //             << joint_positions[4] << " "
  //             << joint_positions[5] << " "
  //             << joint_positions[6] << " "
  //             << std::endl;
  // motiondatadq <<dq_current[0] << " "
  //             << dq_current[1] << " "
  //             << dq_current[2] << " "
  //             << dq_current[3] << " "
  //             << dq_current[4] << " "
  //             << dq_current[5] << " "
  //             << dq_current[6] << " "
  //             << std::endl;
  // motiondatat << time_total_ << " " << time_ << std::endl;

  // franka::JointPositions output(joint_positions);
  // output.motion_finished = motion_finished;
  
  return joint_positions;
}

franka::JointPositions MotionGeneration::operator()(const franka::RobotState& robot_state,franka::Duration period) {
  
  // joint_positions = controlOnline(robot_state, period);
  
  if (control_mode == 1)
  {
      franka::JointPositions output(controlOffline(robot_state, period));
      output.motion_finished = motion_finished;
      return output;
  }
  if (control_mode == 2)
  {
      franka::JointPositions output(controlOnline(robot_state, period));
      output.motion_finished = motion_finished;
      return output;
  }
  
  
}