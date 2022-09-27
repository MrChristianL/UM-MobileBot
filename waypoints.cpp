#include <common/lcm_config.h>
#include <mbot/mbot_channels.h>
#include <lcmtypes/robot_path_t.hpp>
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>
#include <cmath>

int main(int argc, char** argv)
{std::cout << "Gooooaaaal" <<std::endl;
    int numTimes = 1;

    if(argc > 1)
    {
        numTimes = std::atoi(argv[1]);
    }

    std::cout << "Commanding robot to drive through the set maze";

    robot_path_t path;
    path.path.resize(numTimes * 4);

    pose_xyt_t nextPose;

    //Straight 61cm x
    nextPose.x = 0.61f;
    nextPose.y = 0.0f;
    nextPose.theta = -M_PI_2; //90 deg right turn
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[n] = nextPose;
    }

    //Straight 61cm y
    nextPose.x = 0.61f;
    nextPose.y = -0.61f;
    nextPose.theta = 0; //90 deg left turn
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[n + 1] = nextPose;
    }

    nextPose.x = 1.22f;
    nextPose.y = -0.61f;
    nextPose.theta = M_PI_2; //90 deg left turn
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[n + 2] = nextPose;
    }

    nextPose.x = 1.22f;
    nextPose.y = 0.61f;
    nextPose.theta = 0;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[n + 3] = nextPose;
    }

    nextPose.x = 1.83f;
    nextPose.y = 0.61f;
    nextPose.theta = -M_PI_2;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[n + 4] = nextPose;
    }

    nextPose.x = 1.83f;
    nextPose.y = -0.61f;
    nextPose.theta = 0;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[n + 5] = nextPose;
    }

    nextPose.x = 2.44f;
    nextPose.y = -0.61f;
    nextPose.theta = M_PI_2;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[n + 6] = nextPose;
    }

    nextPose.x = 2.44f;
    nextPose.y = 0.0f;
    nextPose.theta = 0;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[n + 7] = nextPose;
    }

    nextPose.x = 3.05f;
    nextPose.y = 0.0f;
    nextPose.theta = 0;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[n + 8] = nextPose;
    }

// Return to original heading after completing all circuits
//    nextPose.theta = 0.0f;
//    path.path.push_back(nextPose);

    /*nextPose.x = 0.0f;
    nextPose.y = 0.0f;
    nextPose.theta = 0.0f; */
    path.path.insert(path.path.begin(), nextPose);

    path.path_length = path.path.size();

    lcm::LCM lcmInstance(MULTICAST_URL);
    std::cout << "publish to: " << CONTROLLER_PATH_CHANNEL << std::endl;
    lcmInstance.publish(CONTROLLER_PATH_CHANNEL, &path);
    sleep(1);

    return 0;
}
//
// Created by donep on 9/29/2021.
//
