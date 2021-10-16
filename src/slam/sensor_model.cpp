#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/grid_utils.hpp>


SensorModel::SensorModel(void)
{
    ///////// TODO: Handle any initialization needed for your sensor model
}


double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map)
{
    ///////////// TODO: Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////
    // TODO: a simple one provided. probabily need to implement a more complicated one. /////

    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose);
    double scanScore = 0.0;


    ////////////////////////////////////// Simple one //////////////////////////
    // for (auto& ray : movingScan){
    //     Point<double> endpoint(ray.origin.x + ray.range * std::cos(ray.theta),
    //                            ray.origin.y + ray.range * std::sin(ray.theta));
    //     auto rayEnd = global_position_to_grid_position(endpoint, map);
    //     if (map.logOdds(rayEnd.x, rayEnd.y) > 0.0){
    //         scanScore += 1.0;
    //     }

    // }

    //////////////// More sophisticated one: /////////////////////////
    /*
        Here should check whether the range of the lidar_scan is in between the max and the min
    */

    for (auto& ray : movingScan){
        Point<double> endpoint(ray.origin.x + ray.range * std::cos(ray.theta),
                               ray.origin.y + ray.range * std::sin(ray.theta));
        
        Point<double> middlepoint(ray.origin.x + (ray.range-0.05f) * std::cos(ray.theta),
                                  ray.origin.y + (ray.range-0.05f) * std::sin(ray.theta));
    
        auto rayEnd = global_position_to_grid_position(endpoint, map);
        auto rayMiddle = global_position_to_grid_position(middlepoint, map);

        if (map.logOdds(rayEnd.x, rayEnd.y) > 0.0){
            if (map.logOdds(rayMiddle.x, rayMiddle.y) > 0.0){
                scanScore += 0.5 * map.logOdds(rayMiddle.x, rayMiddle.y) + 0.5 * map.logOdds(rayEnd.x, rayEnd.y);
            }
            else{
                scanScore += map.logOdds(rayEnd.x, rayEnd.y);
            }
        }

    }

    return scanScore;
}
