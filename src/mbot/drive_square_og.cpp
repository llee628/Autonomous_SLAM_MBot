#include <common/lcm_config.h>
#include <lcmtypes/odometry_t.hpp>
#include <mbot/mbot_channels.h>
#include <lcmtypes/robot_path_t.hpp>
#include <common/timestamp.h>
#include <lcmtypes/timestamp_t.hpp>
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>
#include <cmath>

int main(int argc, char** argv)
{std::cout << "here" <<std::endl;
    int numTimes = 1;

    if(argc > 1)
    {
        numTimes = std::atoi(argv[1]);
    }

    std::cout << "Commanding robot to drive around 1m square " << numTimes << " times.\n";

    robot_path_t path;
    path.path.resize(numTimes * 4);

    pose_xyt_t nextPose;

    nextPose.x = 1.0f;
    nextPose.y = 0.0f;
    nextPose.theta = M_PI_2;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[4*n] = nextPose;
    }

    nextPose.x = 1.0f;
    nextPose.y = 1.0f;
    nextPose.theta = 0;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[4*n + 1] = nextPose;
    }

    nextPose.x = 0.0f;
    nextPose.y = 1.0f;
    nextPose.theta = -M_PI;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[4*n + 2] = nextPose;
    }
    
    nextPose.x = 0.0f;
    nextPose.y = 0.0f;
    nextPose.theta = -M_PI_2;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[4*n + 3] = nextPose;
    }
    
    // Return to original heading after completing all circuits
//    nextPose.theta = 0.0f;
//    path.path.push_back(nextPose);
    
    nextPose.x = 0.0f;
    nextPose.y = 0.0f;
    nextPose.theta = 0.0f;
    path.path.insert(path.path.begin(), nextPose);

    path.path_length = path.path.size();

    // set up time stamo to send to motion controller
    timestamp_t current_time;
    current_time.utime = utime_now();

    // set up odometry struct to send to motion controller
    odometry_t current_position;
    current_position.utime = utime_now();
    current_position.x = 0.5;
    current_position.y = 0.5;
    current_position.theta = 0.0;

    lcm::LCM lcmInstance(MULTICAST_URL);
	std::cout << "publish to: " << CONTROLLER_PATH_CHANNEL << std::endl;
    lcmInstance.publish(CONTROLLER_PATH_CHANNEL, &path);
    lcmInstance.publish(ODOMETRY_CHANNEL, &current_position);
    lcmInstance.publish(MBOT_TIMESYNC_CHANNEL, &current_time);
    sleep(1);

    return 0;
}
