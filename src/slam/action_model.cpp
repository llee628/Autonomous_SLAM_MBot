#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>

#define K1      0.00000001f
#define K2      0.0001f
#define K3      0.0001f
#define K4      0.00000001f

ActionModel::ActionModel(void)
: k1_(K1)
, k2_(K2)
, k3_(K3)
, k4_(K4)
, initialized_(false)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    std::random_device rd;
    numberGenerator_ = std::mt19937(rd());
}


bool ActionModel::updateAction(const pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    if (!initialized_){
        previousOdometry_ = odometry;
        initialized_ = true;
    }

    float deltaX = odometry.x - previousOdometry_.x;
    float deltaY = odometry.y - previousOdometry_.y;
    float deltaTheta = odometry.theta - previousOdometry_.theta;
    trans_ = std::sqrt(deltaX*deltaX + deltaY*deltaY);
    rot1_ = angle_diff(std::atan2(deltaY, deltaX), previousOdometry_.theta);
    float direction = 1.0;

    if (std::abs(rot1_) > M_PI/2.0){ // to avoid huge angle rotation
        rot1_ = angle_diff(M_PI, rot1_);
        direction = -1.0;

    }

    rot2_ = angle_diff(deltaTheta, rot1_);

    //moved_ = (deltaX != 0.0) || (deltaY != 0.0) || (deltaTheta != 0.0);
    moved_ = (std::abs(deltaX - 0.0) > 1E-6) || (std::abs(deltaY - 0.0) > 1E-6) || (std::abs(deltaTheta - 0.0) > 1E-6);

    if (moved_){
        // Simple action mode
        // rot1Std_ = std::sqrt(k1_ * std::abs(rot1_));
        // transStd_ = std::sqrt(k2_ * std::abs(trans_));
        // rot2Std_ = std::sqrt(k1_ * std::abs(rot2_));

        // more complicated action mode
        rot1Std_ = std::sqrt(k1_ * rot1_ * rot1_ + k2_ * trans_ * trans_);
        transStd_ = std::sqrt(k3_ * trans_ * trans_ + k4_ * rot1_ * rot1_ + k4_ * rot2_ * rot2_);
        rot2Std_ = std::sqrt(k1_ * rot2_ * rot2_ + k2_ * trans_ * trans_);
    }

    trans_ *= direction;
    previousOdometry_ = odometry;
    utime_ = odometry.utime;

    return moved_;
}


particle_t ActionModel::applyAction(const particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.

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
