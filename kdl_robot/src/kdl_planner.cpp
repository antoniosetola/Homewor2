#include "kdl_ros_control/kdl_planner.h"
#include <cmath> 

KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
{
    velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
}

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
}

//nuovo costruttore con argomenti time duration, starting point e radius of trajectory
KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, double _trajRadius)
{
    trajDuration_ = _trajDuration;
    trajInit_ = _trajInit;
    trajRadius_= _trajRadius;
}

void KDLPlanner::CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                            double _radius, double _eqRadius
                                            )
{
    path_ = new KDL::Path_RoundedComposite(_radius,_eqRadius,new KDL::RotationalInterpolation_SingleAxis());

    for (unsigned int i = 0; i < _frames.size(); i++)
    {
        path_->Add(_frames[i]);
    }
    path_->Finish();

    velpref_->SetProfile(0,path_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_, velpref_);
}

void KDLPlanner::createCircPath(KDL::Frame &_F_start,
                                KDL::Vector &_V_centre,
                                KDL::Vector& _V_base_p,
                                KDL::Rotation& _R_base_end,
                                double alpha,
                                double eqradius
                                )
{
    KDL::RotationalInterpolation_SingleAxis* otraj;
    otraj = new KDL::RotationalInterpolation_SingleAxis();
    otraj->SetStartEnd(_F_start.M,_R_base_end);
    path_circle_ = new KDL::Path_Circle(_F_start,
                                        _V_centre,
                                        _V_base_p,
                                        _R_base_end,
                                        alpha,
                                        otraj,
                                        eqradius);
    velpref_->SetProfile(0,path_circle_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_circle_, velpref_);
}


KDL::Trajectory* KDLPlanner::getTrajectory()
{
	return traject_;
}


trajectory_point KDLPlanner::compute_trajectory(double time)
{/*
    trapezoidal velocity profile with accDuration_ acceleration time period and trajDuration_ total duration.
    time = current time
    trajDuration_  = final time
    accDuration_   = acceleration time
    trajInit_ = trajectory initial point
    trajEnd_  = trajectory final point */

  // Create trajectory point
  trajectory_point traj;

  // Calculate constant acceleration during the acceleration phase
  Eigen::Vector3d ddot_traj_c = -1.0/(std::pow(accDuration_,2)-trajDuration_*accDuration_)*(trajEnd_-trajInit_);

  // Calculate position, velocity, and acceleration based on the current time
  if(time <= accDuration_)
  {
    traj.pos = trajInit_ + 0.5*ddot_traj_c*std::pow(time,2);
    traj.vel = ddot_traj_c*time;
    traj.acc = ddot_traj_c;
  }
  else if(time <= trajDuration_-accDuration_)
  {
    traj.pos = trajInit_ + ddot_traj_c*accDuration_*(time-accDuration_/2);
    traj.vel = ddot_traj_c*accDuration_;
    traj.acc = Eigen::Vector3d::Zero();
  }
  else
  {
    traj.pos = trajEnd_ - 0.5*ddot_traj_c*std::pow(trajDuration_-time,2);
    traj.vel = ddot_traj_c*(trajDuration_-time);
    traj.acc = -ddot_traj_c;
  }

  return traj;  
}

// circular trajectory
trajectory_point KDLPlanner::compute_trajectory_circ(double time, vel_profile &vel_prof)
{
 double pi=3.14;
 /* trapezoidal velocity profile with accDuration_ acceleration time period and trajDuration_ total duration.
    time = current time
    trajDuration_  = final time
    accDuration_   = acceleration time
    trajInit_ = trajectory initial point
    trajEnd_  = trajectory final point */
 
  // Calculate trapezoidal velocity profile
  //trapezoidal_vel(time, accDuration_, vel_prof);

  // Calculate cubic polynomial velocity profile
  cubic_polinomial(time, vel_prof);

  // Calculate constant acceleration during the acceleration phase
  Eigen::Vector3d ddot_sc = -1.0/(std::pow(accDuration_,2)-trajDuration_*accDuration_)*(trajEnd_-trajInit_);

  // Create trajectory point
  trajectory_point traj;

  // Set position, velocity, and acceleration based on the current time and velocity profile
  traj.pos = Eigen::Vector3d(trajInit_(0),trajInit_(1)-trajRadius_*cos(2*pi*vel_prof.s),trajInit_(2)-trajRadius_*sin(2*pi*vel_prof.s));
  traj.vel = Eigen::Vector3d(0,trajRadius_*2*pi*vel_prof.dots*sin(2*pi*vel_prof.s),-trajRadius_*2*pi*vel_prof.dots*cos(2*pi*vel_prof.s));
  traj.acc = Eigen::Vector3d(0,trajRadius_*2*pi*(vel_prof.ddots*sin(2*pi*vel_prof.s)+2*pi*(std::pow(vel_prof.dots,2))*cos(2*pi*vel_prof.s)),trajRadius_*2*pi*(-vel_prof.ddots*cos(2*pi*vel_prof.s)+2*pi*(std::pow(vel_prof.dots,2))*sin(2*pi*vel_prof.s)));

  return traj;  
}

//linear trajectory
trajectory_point KDLPlanner::compute_trajectory_lin(double time, vel_profile &vel_prof)
{
 /* trapezoidal velocity profile with accDuration_ acceleration time period and trajDuration_ total duration.
    time = current time
    trajDuration_  = final time
    accDuration_   = acceleration time
    trajInit_ = trajectory initial point
    trajEnd_  = trajectory final point */
 
  // Calculate trapezoidal velocity profile
  //trapezoidal_vel(time, accDuration_, vel_prof);

  // Optionally, you can uncomment the line below to use cubic polynomial trajectory
  cubic_polinomial(time, vel_prof);

  // Calculate constant acceleration during the acceleration phase
  Eigen::Vector3d ddot_sc = -1.0/(std::pow(accDuration_,2)-trajDuration_*accDuration_)*(trajEnd_-trajInit_);
  
  // Create trajectory point
  trajectory_point traj;

  // Set position, velocity, and acceleration based on the current time and velocity profile
  traj.pos = Eigen::Vector3d(trajInit_(0),trajInit_(1),trajInit_(2)+vel_prof.s/5);
  traj.vel = Eigen::Vector3d(0,0,vel_prof.dots/5);
  traj.acc = Eigen::Vector3d(0,0,vel_prof.ddots/5);

  return traj;  
}


void KDLPlanner::trapezoidal_vel(double time, double tc, vel_profile &vel_prof)
{
 /*  trapezoidal velocity profile with accDuration_ acceleration time period and trajDuration_ total duration.
     time = current time
     trajDuration_  = final time
     tc   = acceleration time
     trajInit_ = trajectory initial point
     trajEnd_  = trajectory final point */

  // Calculate the constant acceleration during the acceleration phase
  double ddot_sc = -1.0/(std::pow(tc,2)-trajDuration_*tc);

  // Calculate trajectory position, velocity, and acceleration based on current time
  if(time <= accDuration_)
  {
    vel_prof.s = 0.5*ddot_sc*std::pow(time,2);
    vel_prof.dots = ddot_sc*time;
    vel_prof.ddots = ddot_sc;
  }
  else if(time <= trajDuration_-tc)
  {
    vel_prof.s = ddot_sc*tc*(time-tc/2);
    vel_prof.dots = ddot_sc*tc;
    vel_prof.ddots = 0;
  }
  else
  {
    vel_prof.s = 1 - 0.5*ddot_sc*std::pow(trajDuration_-time,2);
    vel_prof.dots = ddot_sc*(trajDuration_-time);
    vel_prof.ddots = -ddot_sc;
  }

}

void KDLPlanner::cubic_polinomial(double time, vel_profile &vel_prof){
 /*  trapezoidal velocity profile with accDuration_ acceleration time period and trajDuration_ total duration.
     time = current time
     trajDuration_  = final time
     tc   = acceleration time
     trajInit_ = trajectory initial point
     trajEnd_  = trajectory final point */

  // Coefficients of the cubic polynomial
  double a0=0;
  double a1=0;
  double a2=3/(std::pow(trajDuration_,2));
  double a3=-2/(std::pow(trajDuration_,3));

  // Calculate trajectory position, velocity, and acceleration
  vel_prof.s=a3*std::pow(time,3) + a2*std::pow(time,2) + a1* time + a0;
  vel_prof.dots=3*a3*std::pow(time,2) + 2*a2*time + a1;
  vel_prof.ddots=6*a3*time + 2*a2;

}

