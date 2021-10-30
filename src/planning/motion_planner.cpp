#include <planning/motion_planner.hpp>
#include <planning/astar.hpp>
#include <common/grid_utils.hpp>
#include <common/timestamp.h>
#include <lcmtypes/robot_path_t.hpp>
#include <cmath>


MotionPlanner::MotionPlanner(const MotionPlannerParams& params)
: params_(params)
{
    setParams(params);
}


MotionPlanner::MotionPlanner(const MotionPlannerParams& params, const SearchParams& searchParams)
: params_(params)
, searchParams_(searchParams)
{
}


robot_path_t MotionPlanner::planPath(const pose_xyt_t& start, 
                                     const pose_xyt_t& goal, 
                                     const SearchParams& searchParams) const
{
    // If the goal isn't valid, then no path can actually exist
    if(!isValidGoal(goal))
    {
        robot_path_t failedPath;
        failedPath.utime = utime_now();
        failedPath.path_length = 1;
        failedPath.path.push_back(start);

        std::cout << "INFO: path rejected due to invalid goal\n";        

        return failedPath;
    }
    
    // Otherwise, use A* to find the path
    return search_for_path(start, goal, distances_, searchParams);
}


robot_path_t MotionPlanner::planPath(const pose_xyt_t& start, const pose_xyt_t& goal) const
{
    return planPath(start, goal, searchParams_);
}


bool MotionPlanner::isValidGoal(const pose_xyt_t& goal) const
{
    float dx = goal.x - prev_goal.x, dy = goal.y - prev_goal.y;
    float distanceFromPrev = std::sqrt(dx * dx + dy * dy);

    //if there's more than 1 frontier, don't go to a target that is within a robot diameter of the current pose
    //std::cout<<"frontiers:"<<num_frontiers<<"distance:"<<distanceFromPrev<<"min:"<<2 * searchParams_.minDistanceToObstacle<<std::endl;

    if(num_frontiers != 1 && distanceFromPrev < 2 * searchParams_.minDistanceToObstacle) return false;
    std::cout<<"Not false in first if statement"<<std::endl;
    auto goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances_);
    std::cout<<"is Valid Target: ("<<goalCell.x<<","<<goalCell.y<<")"<<std::endl;
    std::cout<<"TargetObstacleDistance: "<<distances_(goalCell.x, goalCell.y)<<std::endl;
    std::cout<<params_.robotRadius<<std::endl;
    // A valid goal is in the grid
    if(distances_.isCellInGrid(goalCell.x, goalCell.y))
    {
        // And is far enough from obstacles that the robot can physically occupy the space
        // Add an extra cell to account for discretization error and make motion a little safer by not trying to
        // completely snuggle up against the walls in the motion plan
        return distances_(goalCell.x, goalCell.y) > params_.robotRadius;
    }
    std::cout<<"Not false in 2 if statement"<<std::endl;
    // A goal must be in the map for the robot to reach it
    return false;
}


bool MotionPlanner::isPathSafe(const robot_path_t& path) const
{

    ///////////// TODO: Implement your test for a safe path here //////////////////
    bool safeflag = true;
    for(auto p: path.path){
        auto p_cell = global_position_to_grid_cell(Point<double>(p.x, p.y), distances_);
        bool flag = distances_(p_cell.x, p_cell.y)>= 1.0f*searchParams_.minDistanceToObstacle;
        if(not flag){
            //not a safe point in the whole path;
            safeflag = false;
            break;
        }
    }
    return safeflag;
}


void MotionPlanner::setMap(const OccupancyGrid& map)
{
    distances_.setDistances(map);
}


void MotionPlanner::setParams(const MotionPlannerParams& params)
{
    searchParams_.minDistanceToObstacle = params_.robotRadius;
    searchParams_.maxDistanceWithCost = 15.0 * searchParams_.minDistanceToObstacle;
    searchParams_.distanceCostExponent = 5.0;
}
