#include <mbot/mbot_channels.h>
#include <common/timestamp.h>
#include <lcmtypes/mbot_motor_command_t.hpp>
#include <lcmtypes/odometry_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <lcmtypes/robot_path_t.hpp>
#include <lcmtypes/timestamp_t.hpp>
#include <lcmtypes/message_received_t.hpp>
#include <common/angle_functions.hpp>
#include <common/pose_trace.hpp>
#include <common/lcm_config.h>
#include <slam/slam_channels.h>
#include <lcm/lcm-cpp.hpp>
#include <algorithm>
#include <iostream>
#include <cassert>
#include <signal.h>
#include "maneuver_controller.h"

#define Vmax                        0.25
#define W_STRAIGHT                  M_PI/2
#define Wmax                        M_PI/4
#define T_TARGET_THRED              0.05
#define R1_TARGET_THRED             0.03
#define R2_TARGET_THRED             0.07


/////////////////////// TODO: /////////////////////////////
/**
 * Code below is a little more than a template. You will need
 * to update the maneuver controllers to function more effectively
 * and/or add different controllers.
 * You will at least want to:
 *  - Add a form of PID to control the speed at which your
 *      robot reaches its target pose.
 *  - Add a rotation element to the StratingManeuverController
 *      to maintian a avoid deviating from the intended path.
 *  - Limit (min max) the speeds that your robot is commanded
 *      to avoid commands to slow for your bots or ones too high
 */
///////////////////////////////////////////////////////////

class StraightManeuverController : public ManeuverControllerBase
{
public:
    StraightManeuverController() = default;
    virtual mbot_motor_command_t get_command(const pose_xyt_t& pose, const pose_xyt_t& target) override
    {
        /**
        * Send the command to go straight.
        */
        const float Kv = 1.5;
        const float Kw = 1.6;
        float dx = target.x - pose.x;
        float dy = target.y - pose.y;
        float d = sqrt(pow(dx, 2) + pow(dy, 2));
        float alpha = angle_diff(atan2(dy, dx), pose.theta);
        float v = Kv*d;
        float w = Kw*alpha;

        // go slower when alpha (error) become bigger
        // for now it would contain the "go staight line" and "accurate way point" tradeoff
        
        // v *= std::cos(alpha);
        // v = std::max(0.0f, v);   // just to prevernt go backward

        if(v>=Vmax){
            v = Vmax;
        }
        
        if(w<-W_STRAIGHT){
            w = -W_STRAIGHT;
        }
        else if(w>W_STRAIGHT){
            w = W_STRAIGHT;
        }
    
        return {0, v, w};
    }

    virtual bool target_reached(const pose_xyt_t& pose, const pose_xyt_t& target)  override
    {
        return ((fabs(pose.x - target.x) < T_TARGET_THRED) && (fabs(pose.y - target.y)  < T_TARGET_THRED));
    }
};

class TurnManeuverController : public ManeuverControllerBase
{
public:
    TurnManeuverController() = default;
    virtual mbot_motor_command_t get_command(const pose_xyt_t& pose, const pose_xyt_t& target) override
    {
        /**
        * Send the command to turn.
        */
        const float Kw = 2.0;
        float beta = angle_diff(target.theta, pose.theta);
        float w = Kw * beta;
        
        if(w<-Wmax){
            w = -Wmax;
        }
        else if(w>Wmax){
            w = Wmax;
        }

        return {0, 0.0, w};
        //return {0, 0, M_PI/4};
    }

    virtual bool target_reached(const pose_xyt_t& pose, const pose_xyt_t& target)  override
    {   
        // only consider the orientation but do not care the position
        return (fabs(angle_diff(pose.theta, target.theta)) < R2_TARGET_THRED);
        

    }
};

class RotateManeuverController : public ManeuverControllerBase
{
public:
    RotateManeuverController() = default;
    virtual mbot_motor_command_t get_command(const pose_xyt_t& pose, const pose_xyt_t& target) override
    {
        /**
        * Send the command to turn.
        */
        const float Kw = 2.0;
        float dx = target.x - pose.x;
        float dy = target.y - pose.y;
        float target_heading = atan2(dy, dx);
        float alpha = angle_diff(target_heading, pose.theta);
        float w = Kw * alpha;
        
        if(w<-Wmax){
            w = -Wmax;
        }
        else if(w>Wmax){
            w = Wmax;
        }
        
        return {0, 0.0, w};
        //return {0, 0, M_PI/4};
    }

    virtual bool target_reached(const pose_xyt_t& pose, const pose_xyt_t& target)  override
    {
        float dx = target.x - pose.x;
        float dy = target.y - pose.y;
        float target_heading = atan2(dy, dx);
        return (fabs(angle_diff(target_heading, pose.theta)) < R1_TARGET_THRED);
    }
};


class MotionController
{
public:

    /**
    * Constructor for MotionController.
    */
    MotionController(lcm::LCM * instance)
            :
            lcmInstance(instance),
            odomToGlobalFrame_{0, 0, 0, 0}
    {
        subscribeToLcm();

        time_offset = 0;
        timesync_initialized_ = false;
    }

    /**
    * \brief updateCommand calculates the new motor command to send to the Mbot. This method is called after each call to
    * lcm.handle. You need to check if you have sufficient data to calculate a new command, or if the previous command
    * should just be used again until for feedback becomes available.
    *
    * \return   The motor command to send to the mbot_driver.
    */
    mbot_motor_command_t updateCommand(void)
    {
        mbot_motor_command_t cmd {now(), 0.0, 0.0};
        
        if(!targets_.empty() && !odomTrace_.empty()){
            pose_xyt_t target = targets_.back();
            pose_xyt_t pose = currentPose();

            ///////  TODO: Add different states when adding maneuver controls ///////
            if (state_ == Rotate){
                if(rotate_controller.target_reached(pose, target))
                {   
                    std::cout << "Enter DRIVE state.\n";
                    cmd.trans_v = 0.0;
                    cmd.angular_v = 0.0;
                    state_ = DRIVE;
                }
                else
                {
                    cmd = rotate_controller.get_command(pose, target);
                    cmd.utime = now();
                }
            }
            else if(state_ == TURN)
            {
                if(turn_controller.target_reached(pose, target))
                {
                    if(!assignNextTarget())
                    {
                        std::cout<<"\\rTarget Reached!";
                    }
                }
                else
                {
                    cmd = turn_controller.get_command(pose, target);
                    cmd.utime = now();
                }
            }
            else if(state_ == DRIVE)
            {
                if(straight_controller.target_reached(pose, target))
                {
                    state_ = TURN;

                    //if(!assignNextTarget())
                    //{
                    //std::cout << "\rTarget Reached!";
                    //}
                }
                else
                {
                    cmd = straight_controller.get_command(pose, target);
                    cmd.utime = now();
                }
            }
            else
            {
                std::cerr << "ERROR: MotionController: Entered unknown state: " << state_ << '\n';
            }
            //cmd = {now(), cmd.trans_v, cmd.angular_v};
            //std::cout<<"State:"<<state_<<" "<<"Pose:"<<"("<<pose.x<<","<<pose.y<<","<<pose.theta<<")"<<" "<<"Tar:"<<"("<<target.x<<","<<target.y<<","<<target.theta<<")"<<"\n";
        }
        return cmd;
    }

    bool timesync_initialized(){ return timesync_initialized_; }

    void handleTimesync(const lcm::ReceiveBuffer* buf, const std::string& channel, const timestamp_t* timesync)
    {
        timesync_initialized_ = true;
        time_offset = timesync->utime-utime_now();
    }

    void handlePath(const lcm::ReceiveBuffer* buf, const std::string& channel, const robot_path_t* path)
    {
        targets_ = path->path;
        std::reverse(targets_.begin(), targets_.end()); // store first at back to allow for easy pop_back()

        std::cout << "received new path at time: " << path->utime << "\n";
        for(auto pose : targets_)
        {
            std::cout << "(" << pose.x << "," << pose.y << "," << pose.theta << "); ";
        }
        std::cout << std::endl;

        assignNextTarget();

        //confirm that the path was received
        message_received_t confirm {now(), path->utime, channel};
        lcmInstance->publish(MESSAGE_CONFIRMATION_CHANNEL, &confirm);
    }

    void handleOdometry(const lcm::ReceiveBuffer* buf, const std::string& channel, const odometry_t* odometry)
    {
        pose_xyt_t pose {odometry->utime, odometry->x, odometry->y, odometry->theta};
        odomTrace_.addPose(pose);
    }

    void handlePose(const lcm::ReceiveBuffer* buf, const std::string& channel, const pose_xyt_t* pose)
    {
        computeOdometryOffset(*pose);
    }

private:

    enum State
    {
        Rotate,
        TURN,
        DRIVE,

    };

    pose_xyt_t odomToGlobalFrame_;      // transform to convert odometry into the global/map coordinates for navigating in a map
    PoseTrace  odomTrace_;              // trace of odometry for maintaining the offset estimate
    std::vector<pose_xyt_t> targets_;

    State state_;

    int64_t time_offset;
    bool timesync_initialized_;

    lcm::LCM * lcmInstance;

    TurnManeuverController turn_controller;
    StraightManeuverController straight_controller;
    RotateManeuverController rotate_controller;

    int64_t now()
    {
        return utime_now() + time_offset;
    }

    bool assignNextTarget(void)
    {
        if(!targets_.empty()) { targets_.pop_back(); }
        state_ = Rotate;
        return !targets_.empty();
    }

    void computeOdometryOffset(const pose_xyt_t& globalPose)
    {
        pose_xyt_t odomAtTime = odomTrace_.poseAt(globalPose.utime);
        double deltaTheta = globalPose.theta - odomAtTime.theta;
        double xOdomRotated = (odomAtTime.x * std::cos(deltaTheta)) - (odomAtTime.y * std::sin(deltaTheta));
        double yOdomRotated = (odomAtTime.x * std::sin(deltaTheta)) + (odomAtTime.y * std::cos(deltaTheta));

        odomToGlobalFrame_.x = globalPose.x - xOdomRotated;
        odomToGlobalFrame_.y = globalPose.y - yOdomRotated;
        odomToGlobalFrame_.theta = deltaTheta;
    }

    pose_xyt_t currentPose(void)
    {
        assert(!odomTrace_.empty());

        pose_xyt_t odomPose = odomTrace_.back();
        pose_xyt_t pose;
        pose.x = (odomPose.x * std::cos(odomToGlobalFrame_.theta)) - (odomPose.y * std::sin(odomToGlobalFrame_.theta))
                 + odomToGlobalFrame_.x;
        pose.y = (odomPose.x * std::sin(odomToGlobalFrame_.theta)) + (odomPose.y * std::cos(odomToGlobalFrame_.theta))
                 + odomToGlobalFrame_.y;
        pose.theta = angle_sum(odomPose.theta, odomToGlobalFrame_.theta);

        return pose;
    }

    void subscribeToLcm()
    {
        lcmInstance->subscribe(ODOMETRY_CHANNEL, &MotionController::handleOdometry, this);
        lcmInstance->subscribe(SLAM_POSE_CHANNEL, &MotionController::handlePose, this);
        lcmInstance->subscribe(CONTROLLER_PATH_CHANNEL, &MotionController::handlePath, this);
        lcmInstance->subscribe(MBOT_TIMESYNC_CHANNEL, &MotionController::handleTimesync, this);
    }
};

int main(int argc, char** argv)
{
    lcm::LCM lcmInstance(MULTICAST_URL);
    MotionController controller(&lcmInstance);

    signal(SIGINT, exit);

    while(true)
    {
        lcmInstance.handleTimeout(50);  // update at 20Hz minimum

        if(controller.timesync_initialized()){
            mbot_motor_command_t cmd = controller.updateCommand();
            lcmInstance.publish(MBOT_MOTOR_COMMAND_CHANNEL, &cmd);
        }
    }

    return 0;
}
