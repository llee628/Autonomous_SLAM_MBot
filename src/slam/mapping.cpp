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
    initialized_ = false;
}


void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////
    if(!initialized_){
        previousPose_ = pose;
    }
    initialized_ = true;
    MovingLaserScan movingScan(scan, previousPose_, pose);

    for(auto& ray: movingScan){
        scoreEndpoint(ray, map);
        scoreRay(ray, map);
    }

    previousPose_ = pose;
}

void Mapping::scoreEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map){
    if(ray.range < kMaxLaserDistance_){
        Point<float> rayStart = global_position_to_grid_position(ray.origin, map);
        Point<int> rayCell;

        rayCell.x = static_cast<int>((ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + rayStart.x);
        rayCell.y = static_cast<int>((ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + rayStart.y);

        if(map.isCellInGrid(rayCell.x, rayCell.y)){
            increaseCellOdds(rayCell.x, rayCell.y, map);
        }

    }
}
void Mapping::scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map){
    // TODO: implement the Breshenham’s line algorithm
    if(ray.range < kMaxLaserDistance_){
        Point<float> rayStart = global_position_to_grid_position(ray.origin, map);
        Point<int> rayOrigin;
        //start point of this beam
        Point<int> rayCell;
        //end point of this beam
        rayOrigin.x = static_cast<int>(rayStart.x);
        rayOrigin.y = static_cast<int>(rayStart.y);
        rayCell.x = static_cast<int>((ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + rayStart.x);
        rayCell.y = static_cast<int>((ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + rayStart.y);
        int dx = std::abs(rayCell.x - rayOrigin.x);
        int dy = std::abs(rayCell.y - rayOrigin.y);
        int sx, sy;
        int x, y, err, e2;
        // the step to take
        if(rayOrigin.x<rayCell.x){
            sx = 1;
        }
        else{
            sx = -1;
        }
        if(rayOrigin.y<rayCell.y){
            sy = 1;
        }
        else{
            sy = -1;
        }
        err = dx - dy;
        x = rayOrigin.x;
        y = rayOrigin.y;
        while(x!=rayCell.x||y!=rayCell.y){
            decreaseCellOdds(x, y, map);
            e2 = 2*err;
            if(e2>=-dy){
                err -= dy;
                x += sx;
            }
            if(e2<=dx){
                err += dx;
                y += sy;
            }
        }

    }

}
void Mapping::decreaseCellOdds(int x, int y, OccupancyGrid& map){
    if(map(x, y) - std::numeric_limits<CellOdds>::min() > kMissOdds_){
        map(x, y)-= kMissOdds_;
    }
    else{
        map(x, y) = std::numeric_limits<CellOdds>::min();
    }
}
void Mapping::increaseCellOdds(int x, int y, OccupancyGrid& map){
    //make sure we would not overflow;
    //1.check overflow
    if(std::numeric_limits<CellOdds>::max() - map(x,y) > kHitOdds_){
        map(x, y)+= kHitOdds_;
    }
    else{
        map(x, y) = std::numeric_limits<CellOdds>::max();
    }

}


