#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <cassert>
#include <common/angle_functions.hpp>
#include <random>


ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const pose_xyt_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    double sampleWeight = 1.0 / kNumParticles_;
    posteriorPose_ = pose;

    for(auto& p : posterior_){
        p.pose.x = posteriorPose_.x;
        p.pose.y = posteriorPose_.y;
        p.pose.theta = wrap_to_pi(posteriorPose_.theta);
        p.pose.utime = pose.utime;
        p.parent_pose = p.pose;
        p.weight = sampleWeight;
    }
}


pose_xyt_t ParticleFilter::updateFilter(const pose_xyt_t&      odometry,
                                        const lidar_t& laser,
                                        const OccupancyGrid&   map)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    
    if(hasRobotMoved)
    {
        auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(prior);
        posterior_ = computeNormalizedPosterior(proposal, laser, map);
        posteriorPose_ = estimatePosteriorPose(posterior_);
    }
    
    posteriorPose_.utime = odometry.utime;
    
    return posteriorPose_;
}

pose_xyt_t ParticleFilter::updateFilterActionOnly(const pose_xyt_t&      odometry)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    
    if(hasRobotMoved)
    {
        //auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(posterior_);
        posterior_ = proposal;
    }
    
    posteriorPose_ = odometry;
    
    return posteriorPose_;
}



pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


particles_t ParticleFilter::particles(void) const
{
    particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    
    // std::vector<particle_t> prior;
    // return prior;

    // maybe those implementations are wrong
    //////////////////////////    live code version //////////////////////////////////////
    // std::vector<particle_t> prior = posterior_;
    // double sampleWeight = 1.0/kNumParticles_;
    // std::random_device rd;
    // std::mt19937 generator(rd());
    // std::normal_distribution<> dist(0.0, 0.04);

    // for (auto& p: prior)
    // {
    //     p.pose.x = posteriorPose_.x + dist(generator);
    //     p.pose.y = posteriorPose_.y + dist(generator);
    //     p.pose.theta = posteriorPose_.theta + dist(generator);
    //     p.pose.utime = posteriorPose_.utime;
    //     p.parent_pose = posteriorPose_;
    //     p.weight = sampleWeight;
    // }
    //////////////////////        live code version end     /////////////////////////////////

    // Low Variance Resampling
    std::vector<particle_t> prior;
    double M_inv = 1.0/kNumParticles_;
    std::random_device rd;
    std::mt19937 generator(rd());

    std::uniform_real_distribution<double> dist(0.0, M_inv);

    double r = dist(generator);
    //double r = (((double) rand()) / (double)RAND_MAX) * M_inv;

    double c = posterior_[0].weight;
    double U;
    int i = 0;

    for (int m = 1; m <= kNumParticles_; m++){
        U = r + (m-1) * M_inv;
        while (c < U){
            i += 1;
            c += posterior_[i].weight;
        }

        // add p to prior
        particle_t p;
        p = posterior_[i];
        prior.push_back(p);
    }
    //////////////////////////// end of low variance resampling algorithm /////////////////////////


    return prior;

}


std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    std::vector<particle_t> proposal;
    for (auto& p : prior){
        proposal.push_back(actionModel_.applyAction(p));
    }
    return proposal;
}


std::vector<particle_t> ParticleFilter::computeNormalizedPosterior(const std::vector<particle_t>& proposal,
                                                                   const lidar_t& laser,
                                                                   const OccupancyGrid&   map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the 
    ///////////       particles in the proposal distribution
    std::vector<particle_t> posterior;
    double sumWeights = 0.0;

    for (auto& p : proposal){
        particle_t weighted = p;
        weighted.weight = sensorModel_.likelihood(weighted, laser, map);
        sumWeights += weighted.weight;
        posterior.push_back(weighted);
    }

    // normalize the weight
    for (auto& p : posterior){
        p.weight /= sumWeights;
    }

    return posterior;
}


pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    pose_xyt_t pose;

    double xMean = 0.0;
    double yMean = 0.0;
    double cosThetaMean = 0.0;
    double sinThetaMean = 0.0;

    for (auto& p : posterior){
        xMean += p.weight * p.pose.x;
        yMean += p.weight * p.pose.y;
        cosThetaMean += p.weight * std::cos(p.pose.theta);
        sinThetaMean += p.weight * std::sin(p.pose.theta);
    }

    pose.x = xMean;
    pose.y = yMean;
    pose.theta = std::atan2(sinThetaMean, cosThetaMean);

    return pose;
}
