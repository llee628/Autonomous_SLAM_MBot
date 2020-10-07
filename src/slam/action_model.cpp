#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>


ActionModel::ActionModel(void)
: k1_(0.01f)
, k2_(0.01f)
, initialized_(false)
{
    std::random_device rd;
    numberGenerator_ = std::mt19937(rd());
}


bool ActionModel::updateAction(const pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    if(!initialized_)
    {
        previousOdometry_ = odometry;
        initialized_      = true;
    }
    
    float deltaX     = odometry.x - previousOdometry_.x;
    float deltaY     = odometry.y - previousOdometry_.y;
    float deltaTheta = angle_diff(odometry.theta, previousOdometry_.theta);
    float direction  = 1.0;
    
    trans_ = std::sqrt(deltaY*deltaY + deltaX*deltaX);
    rot1_  = angle_diff(std::atan2(deltaY, deltaX), previousOdometry_.theta);
    
    //if(std::abs(trans_) < 0.0001)
   // {
   //     rot1_ = 0.0f;   // if translation is small, then rot1 via atan2 is unreliable, but robot is barely moving, so
   //                     // just use a simple zero-mean model for rot1
   // }
    // If rot1 is greater than pi/2, then the robot moved backwards, not made a giant initial rotation. This assumes the
    // update rate is fast enough the robot can't turn more than pi/2 in a single iteration of the localization
    if(std::abs(rot1_) > M_PI/2.0)
    {
        rot1_      = angle_diff(M_PI, rot1_);
        direction = -1.0;
    }
    
    rot2_ = angle_diff(deltaTheta, rot1_);

    moved_ = (deltaX != 0.0) || (deltaY != 0.0) || (deltaTheta != 0.0);
    
    if(moved_)
    {
        rot1Std_  = std::sqrt(k1_*std::abs(rot1_));
        transStd_ = std::sqrt(k2_*std::abs(trans_));
        rot2Std_  = std::sqrt(k1_*std::abs(rot2_));
    }
    
    previousOdometry_  = odometry;
    trans_            *= direction;
    utime_             = odometry.utime;

    return moved_;
}


particle_t ActionModel::applyAction(const particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.

    if(moved_)
    {
        particle_t newSample = sample;
       
        float sampledRot1 = std::normal_distribution<>(rot1_, rot1Std_)(numberGenerator_);
        float sampledTrans = std::normal_distribution<>(trans_, transStd_)(numberGenerator_);
        float sampledRot2 = std::normal_distribution<>(rot2_, rot2Std_)(numberGenerator_);
        
        newSample.pose.x += sampledTrans*cos(sample.pose.theta + sampledRot1);
        newSample.pose.y += sampledTrans*sin(sample.pose.theta + sampledRot1);
        newSample.pose.theta = wrap_to_pi(sample.pose.theta + sampledRot1 + sampledRot2);
        newSample.pose.utime = utime_;
        newSample.parent_pose = sample.pose;
        
        return newSample;
    }
    else
    {
        // If the robot isn't moving, then there's no need for doing any sampling. The robot is just sitting still, so make
        // sure particles don't move, but that time keeps moving forward
        particle_t newSample = sample;
        newSample.pose.utime = utime_;
        newSample.parent_pose = sample.pose;
        return newSample;
    }
}
