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

    //////////////// Simplified Likelihood Field Model /////////////////////////
    /*
        Look at endpoints. Take log odds of cell into scan score if positive.
        if it is not a hit, check cell before and after along ray and take fraction of
        log odds into scan score.
    */

    for (auto& ray : movingScan){
        float ref_distance = 0.05f;

        Point<double> endpoint(ray.origin.x + ray.range * std::cos(ray.theta),
                               ray.origin.y + ray.range * std::sin(ray.theta));
        
        Point<double> beforepoint(ray.origin.x + (ray.range - ref_distance) * std::cos(ray.theta),
                                  ray.origin.y + (ray.range - ref_distance) * std::sin(ray.theta));
        
        Point<double> afterpoint(ray.origin.x + (ray.range + ref_distance) * std::cos(ray.theta),
                                 ray.origin.y + (ray.range + ref_distance) * std::sin(ray.theta));
    
        auto rayEnd = global_position_to_grid_cell(endpoint, map);
        auto rayBefore = global_position_to_grid_cell(beforepoint, map);
        auto rayAfter = global_position_to_grid_cell(afterpoint, map);

        if (map.logOdds(rayEnd.x, rayEnd.y) > 0){
            scanScore += map.logOdds(rayEnd.x, rayEnd.y);
        }
        else{
            scanScore += 0.5 * map.logOdds(rayAfter.x, rayAfter.y) + 0.5 * map.logOdds(rayBefore.x, rayBefore.y);
        }

    }

    return scanScore;
}
