#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>


Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
{
}


void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////
    if(!initialized_){
        previousPose_ = pose; //if we havent done this yet, just set previous pose to current
    }

    MovingLaserScan movingScan(scan, previousPose_, pose); //returns a set of rays, each w/ own origin
    
    for(auto& ray : movingScan){
        scoreEndpoint(ray, map);
        scoreRay(ray, map); 
    }

    initialized_ = true;
    previousPose_ = pose;
}


void Mapping::scoreEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map){
    if(ray.range < kMaxLaserDistance_){ //as long as we are sensing within known map
        Point<float> rayStart = global_position_to_grid_cell(ray.origin, map);
        Point<int> rayCell;

        rayCell.x = static_cast<int>((ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + rayStart.x);
        rayCell.y = static_cast<int>((ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + rayStart.y);

        if(map.isCellInGrid(rayCell.x, rayCell.y)){
            increaseCellOdds(rayCell.x, rayCell.y, map);
        }
    }
}

void Mapping::scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map){

}

void Mapping::decreaseCellOdds(int x, int y, OccupancyGrid& map){

}

void Mapping::increaseCellOdds(int x, int y, OccupancyGrid& map){
    if((std::numeric_limits<CellOdds>::max() - map(x,y)) > kHitOdds_){ //if we are not going to overflow datatype by increasing odds,
        map(x,y) += kHitOdds_;
    } else {
        map(x,y) = std::numeric_limits<CellOdds>::max();
    }
}