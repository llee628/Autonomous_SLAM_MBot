#include <common/lcm_config.h>
#include <mbot/mbot_channels.h>
#include <lcmtypes/robot_path_t.hpp>
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>
#include <cmath>

#define HALL_LENGTH 3.0f

int main(int argc, char** argv)
{std::cout << "here" <<std::endl;

    std::cout << "Commanding robot to drive event 2\n";
    
    robot_path_t path;
    path.path.resize(2);
    
    pose_xyt_t nextPose;
    
    nextPose.x = HALL_LENGTH;
    nextPose.y = 0.0f;
    nextPose.theta = M_PI;
    path.path[0] = nextPose;
    

    nextPose.x = 0.0f;
    nextPose.y = 0.0f;
    nextPose.theta = 0.0f;
    path.path[1] = nextPose;
    
    // Return to original heading after completing all circuits
//    nextPose.theta = 0.0f;
//    path.path.push_back(nextPose);
    
    nextPose.x = 0.0f;
    nextPose.y = 0.0f;
    nextPose.theta = 0.0f;
    path.path.insert(path.path.begin(), nextPose);
    
    path.path_length = path.path.size();
    
    lcm::LCM lcmInstance(MULTICAST_URL);
	std::cout << "publish to: " << CONTROLLER_PATH_CHANNEL << std::endl;
    lcmInstance.publish(CONTROLLER_PATH_CHANNEL, &path);
    sleep(1);

    return 0;
}

