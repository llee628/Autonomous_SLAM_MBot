#include <planning/exploration.hpp>
#include <planning/frontiers.hpp>
#include <planning/planning_channels.h>
#include <common/grid_utils.hpp>
#include <common/timestamp.h>
#include <mbot/mbot_channels.h>
#include <slam/slam_channels.h>
#include <fstream>
#include <iostream>
#include <queue>
#include <unistd.h>
#include <cassert>

#include <common/angle_functions.hpp>

const float kReachedPositionThreshold = 0.07f;  // must get within this distance of a position for it to be explored

// Define an equality operator for poses to allow direct comparison of two paths
bool operator==(const pose_xyt_t& lhs, const pose_xyt_t& rhs)
{
    return (lhs.x == rhs.x) && (lhs.y == rhs.y) && (lhs.theta == rhs.theta);
}
/*bool Exploration::isReachTarget(pose_xyt_t start, pose_xyt_t goal){
    float dx = goal.x - start.x;
    float dy = goal.y - goal.y;
    float d = std::sqrt(dx*dx+dy*dy);
    return d <= kReachedPositionThreshold;
}*/


Exploration::Exploration(int32_t teamNumber,
                         lcm::LCM* lcmInstance)
: teamNumber_(teamNumber)
, state_(exploration_status_t::STATE_INITIALIZING)
, haveNewPose_(false)
, haveNewMap_(false)
, haveHomePose_(false)
, lcmInstance_(lcmInstance)
, pathReceived_(false)
{
    assert(lcmInstance_);   // confirm a nullptr wasn't passed in
    
    lcmInstance_->subscribe(SLAM_MAP_CHANNEL, &Exploration::handleMap, this);
    lcmInstance_->subscribe(SLAM_POSE_CHANNEL, &Exploration::handlePose, this);
    lcmInstance_->subscribe(MESSAGE_CONFIRMATION_CHANNEL, &Exploration::handleConfirmation, this);
    
    // Send an initial message indicating that the exploration module is initializing. Once the first map and pose are
    // received, then it will change to the exploring map state.
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_INITIALIZING;
    status.status = exploration_status_t::STATUS_IN_PROGRESS;
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
    
    MotionPlannerParams params;
    params.robotRadius = 0.11f;
    planner_.setParams(params);
}


bool Exploration::exploreEnvironment()
{
    while((state_ != exploration_status_t::STATE_COMPLETED_EXPLORATION) 
        && (state_ != exploration_status_t::STATE_FAILED_EXPLORATION))
    {
        // If data is ready, then run an update of the exploration routine
        if(isReadyToUpdate())
        {
            runExploration();
        }
        // Otherwise wait a bit for data to arrive
        else
        {
            usleep(10000);
        }
    }
    
    // If the state is completed, then we didn't fail
    return state_ == exploration_status_t::STATE_COMPLETED_EXPLORATION;
}

void Exploration::handleMap(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const occupancy_grid_t* map)
{

    std::lock_guard<std::mutex> autoLock(dataLock_);
    incomingMap_.fromLCM(*map);
    haveNewMap_ = true;
}


void Exploration::handlePose(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const pose_xyt_t* pose)
{

    std::lock_guard<std::mutex> autoLock(dataLock_);
    incomingPose_ = *pose;
    haveNewPose_ = true;
}

void Exploration::handleConfirmation(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const message_received_t* confirm)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    if(confirm->channel == CONTROLLER_PATH_CHANNEL && confirm->creation_time == most_recent_path_time) pathReceived_ = true;
}

bool Exploration::isReadyToUpdate(void)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    return haveNewMap_ && haveNewPose_;
}


void Exploration::runExploration(void)
{
    assert(isReadyToUpdate());
    
    copyDataForUpdate();
    executeStateMachine();
}


void Exploration::copyDataForUpdate(void)
{
    std::lock_guard<std::mutex> autoLock(dataLock_);
    
    // Only copy the map if a new one has arrived because it is a costly operation
    if(haveNewMap_)
    {
        currentMap_ = incomingMap_;
        haveNewMap_ = false;
    }
    
    // Always copy the pose because it is a cheap copy
    currentPose_ = incomingPose_;
    haveNewPose_ = false;
    
    // The first pose received is considered to be the home pose
    if(!haveHomePose_)
    {
        homePose_ = incomingPose_;
        haveHomePose_ = true;
        std::cout << "INFO: Exploration: Set home pose:" << homePose_.x << ',' << homePose_.y << ',' 
            << homePose_.theta << '\n';
    }
}


void Exploration::executeStateMachine(void)
{
    bool stateChanged = false;
    int8_t nextState = state_;
    
    // Save the path from the previous iteration to determine if a new path was created and needs to be published
    robot_path_t previousPath = currentPath_;
    
    // Run the state machine until the state remains the same after an iteration of the loop
    do
    {
        switch(state_)
        {
            case exploration_status_t::STATE_INITIALIZING:
                nextState = executeInitializing();
                break;
            case exploration_status_t::STATE_EXPLORING_MAP:
                nextState = executeExploringMap(stateChanged);
                break;
                
            case exploration_status_t::STATE_RETURNING_HOME:
                nextState = executeReturningHome(stateChanged);
                break;

            case exploration_status_t::STATE_COMPLETED_EXPLORATION:
                nextState = executeCompleted(stateChanged);
                break;
                
            case exploration_status_t::STATE_FAILED_EXPLORATION:
                nextState = executeFailed(stateChanged);
                break;
        }
        
        stateChanged = nextState != state_;
        state_ = nextState;
        
    } while(stateChanged);

    //if path confirmation was not received, resend path
    if(!pathReceived_)
    {
        std::cout << "the current path was not received by motion_controller, attempting to send again:\n";

        std::cout << "timestamp: " << currentPath_.utime << "\n";

        for(auto pose : currentPath_.path){
            std::cout << "(" << pose.x << "," << pose.y << "," << pose.theta << "); ";
        }std::cout << "\n";

        lcmInstance_->publish(CONTROLLER_PATH_CHANNEL, &currentPath_);
    }

    //if path changed, send current path
    if(previousPath.path != currentPath_.path)
    { 

        std::cout << "INFO: Exploration: A new path was created on this iteration. Sending to Mbot:\n";

        std::cout << "path timestamp: " << currentPath_.utime << "\npath: ";

        for(auto pose : currentPath_.path){
            std::cout << "(" << pose.x << "," << pose.y << "," << pose.theta << "); ";
        }std::cout << "\n";

        lcmInstance_->publish(CONTROLLER_PATH_CHANNEL, &currentPath_);

        pathReceived_ = false;
        most_recent_path_time = currentPath_.utime;
    }

}


int8_t Exploration::executeInitializing(void)
{
    /////////////////////////   Create the status message    //////////////////////////
    // Immediately transition to exploring once the first bit of data has arrived
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_INITIALIZING;
    status.status = exploration_status_t::STATUS_COMPLETE;
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
    
    return exploration_status_t::STATE_EXPLORING_MAP;
}


int8_t Exploration::executeExploringMap(bool initialize)
{
    //////////////////////// TODO: Implement your method for exploring the map ///////////////////////////
    /*
    * NOTES:
    *   - At the end of each iteration, then (1) or (2) must hold, otherwise exploration is considered to have failed:
    *       (1) frontiers_.empty() == true      : all frontiers have been explored as determined by find_map_frontiers()
    *       (2) currentPath_.path_length > 1 : currently following a path to the next frontier
    * 
    *   - Use the provided function find_map_frontiers() to find all frontiers in the map.
    *   - You will need to implement logic to select which frontier to explore.
    *   - You will need to implement logic to decide when to select a new frontier to explore. Take into consideration:
    *       -- The map is evolving as you drive, so what previously looked like a safe path might not be once you have
    *           explored more of the map.
    *       -- You will likely be able to see the frontier before actually reaching the end of the path leading to it.
    */
    //std::cout<<"Current Pose:("<<currentPose_.x<<","<<currentPose_.y<<")"<<std::endl;
    //std::cout<<"Distance: "<< distance_between_points(Point<float>(currentTarget_.x, currentTarget_.y),
                                                      //Point<float>(currentPose_.x, currentPose_.y))<<std::endl;
    /////////////////////////////// End student code ///////////////////////////////
    if(not isExploringMapInit){
        planner_.setMap(currentMap_);
        frontiers_ = find_map_frontiers(currentMap_, currentPose_);
        currentPath_ = plan_path_to_frontier(frontiers_, currentPose_, currentMap_, planner_);
        currentTarget_ = currentPath_.path.back();
        // store currentPath_ to the Buffer
        pathBuffer_.insert(pathBuffer_.end(), currentPath_.path.begin(), currentPath_.path.end());

        isExploringMapInit = true;
    }

    else if(distance_between_points(Point<float>(currentTarget_.x, currentTarget_.y),
                                    Point<float>(currentPose_.x, currentPose_.y)) <= kReachedPositionThreshold){
        std::cout<<"Target reached!!!!!"<<std::endl;
        std::cout<<"Start to a new Target"<<std::endl;
        planner_.setMap(currentMap_);
        frontiers_ = find_map_frontiers(currentMap_, currentPose_);
        if(not frontiers_.empty()){
        currentPath_ = plan_path_to_frontier(frontiers_, currentPose_, currentMap_, planner_);
        currentTarget_ = currentPath_.path.back();
        pathBuffer_.insert(pathBuffer_.end(), currentPath_.path.begin()+1, currentPath_.path.end());
        }

    }
    /////////////////////////   Create the status message    //////////////////////////
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_EXPLORING_MAP;
    
    // If no frontiers remain, then exploration is complete
    if(frontiers_.empty())
    {
        status.status = exploration_status_t::STATUS_COMPLETE;
    }
    // Else if there's a path to follow, then we're still in the process of exploring
    else if(currentPath_.path.size() > 1)
    {
        status.status = exploration_status_t::STATUS_IN_PROGRESS;
    }
    // Otherwise, there are frontiers, but no valid path exists, so exploration has failed
    else
    {
        status.status = exploration_status_t::STATUS_FAILED;
    }
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
    
    ////////////////////////////   Determine the next state    ////////////////////////
    switch(status.status)
    {
        // Don't change states if we're still a work-in-progress
        case exploration_status_t::STATUS_IN_PROGRESS:
            return exploration_status_t::STATE_EXPLORING_MAP;
            
        // If exploration is completed, then head home
        case exploration_status_t::STATUS_COMPLETE:
            std::cout<<"___________________________"<<std::endl;
            std::cout<<"Exploration Finished!"<<std::endl;
            return exploration_status_t::STATE_RETURNING_HOME;
            
        // If something has gone wrong and we can't reach all frontiers, then fail the exploration.
        case exploration_status_t::STATUS_FAILED:
            return exploration_status_t::STATE_FAILED_EXPLORATION;
            
        default:
            std::cerr << "ERROR: Exploration::executeExploringMap: Set an invalid exploration status. Exploration failed!";
            return exploration_status_t::STATE_FAILED_EXPLORATION;
    }
}


int8_t Exploration::executeReturningHome(bool initialize)
{
    //////////////////////// TODO: Implement your method for returning to the home pose ///////////////////////////
    /*
    * NOTES:
    *   - At the end of each iteration, then (1) or (2) must hold, otherwise exploration is considered to have failed:
    *       (1) dist(currentPose_, targetPose_) < kReachedPositionThreshold  :  reached the home pose
    *       (2) currentPath_.path_length > 1  :  currently following a path to the home pose
    */
    std::cout<<"_________________________________"<<std::endl;
    std::cout<<"Ok, you are going to return home!"<<std::endl;
    if(not isReturningHomeInit){

        //plan 1: use astar to plan
        /*planner_.setMap(currentMap_);
        currentPath_ = planner_.planPath(currentPose_, homePose_);
        currentTarget_ = homePose_;*/

        //plan 2: store way points and replay
        std::reverse(pathBuffer_.begin(), pathBuffer_.end());
        std::vector<pose_xyt_t> returnPath;
        for(auto p: pathBuffer_){
            pose_xyt_t node;
            node.x = p.x;
            node.y = p.y;
            node.theta = angle_sum(p.theta, 3.1415926);
            returnPath.push_back(node);
        }
        /*pose_xyt_t pPrev;
        bool setPrev = false;
        for(auto p: pathBuffer_){

            if(not setPrev){
                setPrev = true;
            }
            else{
                p.theta = std::atan2(p.y - pPrev.y, p.x - pPrev.x);
            }
            //what about theta and utime?
            pPrev.x = p.x;
            pPrev.y = p.y;
            pPrev.theta = p.theta;
        }*/

        robot_path_t path;
        path.utime = currentPose_.utime;
        //path.path = pathBuffer_;
        path.path = returnPath;
        path.path_length = path.path.size();
        currentPath_ = path;
        currentTarget_ = homePose_;

        isReturningHomeInit = true;
    }

    /////////////////////////////// End student code ///////////////////////////////
    
    /////////////////////////   Create the status message    //////////////////////////
    exploration_status_t status;
    status.utime = utime_now();
    status.team_number = teamNumber_;
    status.state = exploration_status_t::STATE_RETURNING_HOME;
    
    double distToHome = distance_between_points(Point<float>(homePose_.x, homePose_.y), 
                                                Point<float>(currentPose_.x, currentPose_.y));
    // If we're within the threshold of home, then we're done.
    if(distToHome <= kReachedPositionThreshold)
    {
        status.status = exploration_status_t::STATUS_COMPLETE;
    }
    // Otherwise, if there's a path, then keep following it
    else if(currentPath_.path.size() > 1)
    {
        status.status = exploration_status_t::STATUS_IN_PROGRESS;
    }
    // Else, there's no valid path to follow and we aren't home, so we have failed.
    else
    {
        status.status = exploration_status_t::STATUS_FAILED;
    }
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &status);
    
    ////////////////////////////   Determine the next state    ////////////////////////
    if(status.status == exploration_status_t::STATUS_IN_PROGRESS)
    {
        return exploration_status_t::STATE_RETURNING_HOME;
    }
    else // if(status.status == exploration_status_t::STATUS_FAILED)
    {
        return exploration_status_t::STATE_FAILED_EXPLORATION;
    }
}

int8_t Exploration::executeCompleted(bool initialize)
{
    // Stay in the completed state forever because exploration only explores a single map.
    exploration_status_t msg;
    msg.utime = utime_now();
    msg.team_number = teamNumber_;
    msg.state = exploration_status_t::STATE_COMPLETED_EXPLORATION;
    msg.status = exploration_status_t::STATUS_COMPLETE;
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &msg);
    
    return exploration_status_t::STATE_COMPLETED_EXPLORATION;
}


int8_t Exploration::executeFailed(bool initialize)
{
    // Send the execute failed forever. There is no way to recover.
    exploration_status_t msg;
    msg.utime = utime_now();
    msg.team_number = teamNumber_;
    msg.state = exploration_status_t::STATE_FAILED_EXPLORATION;
    msg.status = exploration_status_t::STATUS_FAILED;
    
    lcmInstance_->publish(EXPLORATION_STATUS_CHANNEL, &msg);
    
    return exploration_status_t::STATE_FAILED_EXPLORATION;
}