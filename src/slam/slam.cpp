#include <slam/slam.hpp>
#include <slam/slam_channels.h>
#include <mbot/mbot_channels.h>
#include <optitrack/optitrack_channels.h>
#include <unistd.h>
#include <cassert>
#include <chrono>

OccupancyGridSLAM::OccupancyGridSLAM(int         numParticles,
                                     int8_t      hitOddsIncrease,
                                     int8_t      missOddsDecrease,
                                     lcm::LCM&   lcmComm,
                                     bool waitForOptitrack,
                                     bool mappingOnlyMode,
                                     bool actionOnlyMode,
                                     const std::string localizationOnlyMap)
: mode_(full_slam)  // default is running full SLAM, unless user specifies otherwise on the command line
, haveInitializedPoses_(false)
, waitingForOptitrack_(waitForOptitrack)
, haveMap_(false)
, numIgnoredScans_(0)
, filter_(numParticles)
, map_(10.0f, 10.0f, 0.05f) //30,30,0.1  // create a 10m x 10m grid with 0.05m cells
, mapper_(5.0f, hitOddsIncrease, missOddsDecrease)
, lcm_(lcmComm)
, mapUpdateCount_(0)
{
    // Time initialize
    startTime_ = std::chrono::steady_clock::now();
    frequency_ = 0;
    numParticles_ = numParticles;

    // Initialize data for RMS error
    square_error_x_ = 0.0f;
    square_error_y_ = 0.0f;
    square_error_theta_ = 0.0f;
    data_num_ = 0;


    // Confirm that the mode is valid -- mapping-only and localization-only are not specified
    assert(!(mappingOnlyMode && localizationOnlyMap.length() > 0));
    
    // Determine which mode to run based on the inputs
    if(mappingOnlyMode)
    {
        mode_ = mapping_only;
    }
    else if(localizationOnlyMap.length() > 0)
    {
        haveMap_ = map_.loadFromFile(localizationOnlyMap);
        assert(haveMap_);   // if there's no map, then the localization can't run!
        if(actionOnlyMode){
            mode_ = action_only;
        }
        else{
            mode_ = localization_only;
        }
    }
    
    currentOdometry_.utime = 0;
    currentScan_.utime = 0;
    
    // Laser and odometry data are always required
    lcm_.subscribe(LIDAR_CHANNEL, &OccupancyGridSLAM::handleLaser, this);
    lcm_.subscribe(ODOMETRY_CHANNEL, &OccupancyGridSLAM::handleOdometry, this);
    lcm_.subscribe(TRUE_POSE_CHANNEL, &OccupancyGridSLAM::handleOptitrack, this);
    
    // If we are only building the occupancy grid using ground-truth poses, then subscribe to the ground-truth poses.
    if(mode_ == mapping_only)
    {
        lcm_.subscribe(SLAM_POSE_CHANNEL, &OccupancyGridSLAM::handlePose, this);
    }
    
    // Zero-out all the poses to start. Either the robot will start at (0,0,0) or at the first pose received from the
    // Optitrack system.
    initialPose_.x = initialPose_.y = initialPose_.theta = 0.0f;
    previousPose_.x = previousPose_.y = previousPose_.theta = 0.0f;
    currentPose_.x  = currentPose_.y  = currentPose_.theta  = 0.0f;
}


void OccupancyGridSLAM::runSLAM(void)
{
    while(true)
    {
        // If new data has arrived
        if(isReadyToUpdate())
        {
            
            
            // std::cout << "Current duration: " << std::chrono::duration_cast<std::chrono::microseconds>(currentTime_ - startTime_).count();
            // std::cout << std::endl;
            
            // Then run an iteration of our SLAM algorithm
            runSLAMIteration();
            currentTime_ = std::chrono::steady_clock::now();
            frequency_ += 1;
            // if (std::chrono::duration_cast<std::chrono::microseconds>(currentTime_ - startTime_).count()/1000000 > 0){
                
            //     std::cout << "Num of Particles: " << numParticles_ << "  ";
            //     std::cout << "Frequency: " << frequency_ << std::endl;
            //     startTime_ = currentTime_;
            //     frequency_ = 0;
            // }
        }
        // Otherwise, do a quick spin while waiting for data rather than using more complicated condition variable.
        else
        {
            usleep(1000);
        }
    }
}


// Handlers for LCM messages
void OccupancyGridSLAM::handleLaser(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const lidar_t* scan)
{
    const int kNumIgnoredForMessage = 10;   // number of scans to ignore before printing a message about odometry
//std::cout << "laser!\n";    
    std::lock_guard<std::mutex> autoLock(dataMutex_);
    // Ignore scans until odometry data arrives -- need odometry before a scan to safely built the map
    bool haveOdom = (mode_ != mapping_only) // For full SLAM, odometry data is needed.
        && !odometryPoses_.empty() 
        && (odometryPoses_.front().utime <= scan->times.front());
    bool havePose = (mode_ == mapping_only) // For mapping-only, ground-truth poses are needed
        && !groundTruthPoses_.empty() 
        && (groundTruthPoses_.front().utime <= scan->times.front());
    
    // If there's appropriate odometry or pose data for this scan, then add it to the queue.
    if(haveOdom || havePose)
    {
        incomingScans_.push_back(*scan);
        
        // If we showed the laser error message, then provide another message indicating that laser scans are now
        // being saved
        if(numIgnoredScans_ >= kNumIgnoredForMessage)
        {
            std::cout << "INFO: OccupancyGridSLAM: Received odometry or pose data, so laser scans are now being \
                saved.\n";
            numIgnoredScans_ = 0;
        }
    }
    // Otherwise ignore it
    else
    {
        ++numIgnoredScans_;
    }
    
    if(numIgnoredScans_ == kNumIgnoredForMessage)
    {
        std::cout << "INFO: OccupancyGridSLAM: Ignoring scan because no odometry data is available. \
            Please start the odometry module or use a log with ground-truth poses.\n";
    }
}


void OccupancyGridSLAM::handleOdometry(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const odometry_t* odometry)
{
    std::lock_guard<std::mutex> autoLock(dataMutex_);

    pose_xyt_t odomPose;
    odomPose.utime = odometry->utime;
    odomPose.x = odometry->x;
    odomPose.y = odometry->y;
    odomPose.theta = odometry->theta;
    odometryPoses_.addPose(odomPose);
}


void OccupancyGridSLAM::handlePose(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const pose_xyt_t* pose)
{
    std::lock_guard<std::mutex> autoLock(dataMutex_);
    groundTruthPoses_.addPose(*pose);
}


void OccupancyGridSLAM::handleOptitrack(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const pose_xyt_t* pose)
{
    std::lock_guard<std::mutex> autoLock(dataMutex_);

    if(waitingForOptitrack_)
    {
        initialPose_ = *pose;
        waitingForOptitrack_ = false;
    }
}


bool OccupancyGridSLAM::isReadyToUpdate(void)
{
    std::lock_guard<std::mutex> autoLock(dataMutex_);
    
    bool haveData = false;
	    
    // If there's at least one scan to process, then check if odometry/pose information is available
    if(!incomingScans_.empty())
    {
        // Find if there's a scan that there is odometry data for
        const lidar_t& nextScan = incomingScans_.front();
        
        // Ensure that there's a pose that exists at or after the final laser measurement to be sure that valid
        // interpolation of robot motion during the scan can be performed.
        
        // Only care if there's odometry data if we aren't in mapping-only mode
        bool haveNewOdom = (mode_ != mapping_only) && (odometryPoses_.containsPoseAtTime(nextScan.times.front()));
        // Otherwise, only see if a new pose has arrived
        bool haveNewPose = (mode_ == mapping_only) && (groundTruthPoses_.containsPoseAtTime(nextScan.times.front()));        

        haveData = haveNewOdom || haveNewPose;
    }

    // If all SLAM data and optitrack data has arrived, then we're ready to go.
    return haveData && !waitingForOptitrack_;
}


void OccupancyGridSLAM::runSLAMIteration(void)
{
    copyDataForSLAMUpdate();
    initializePosesIfNeeded();
    
    // Sanity check the laser data to see if rplidar_driver has lost sync
    if(currentScan_.num_ranges > 100)//250)
    {
        updateLocalization();
        updateMap();
    }
    else 
    {
        std::cerr << "ERROR: OccupancyGridSLAM: Detected invalid laser scan with " << currentScan_.num_ranges 
            << " ranges.\n";
    }
}


void OccupancyGridSLAM::copyDataForSLAMUpdate(void)
{
    std::lock_guard<std::mutex> autoLock(dataMutex_);
    
    // Copy the data needed for the new SLAM update
    currentScan_ = incomingScans_.front();
    incomingScans_.pop_front();
    
    if(mode_ == mapping_only)
    {
        // No localization is performed during mapping-only mode, so the previous pose needs to be correctly adjusted
        // here.
        previousPose_ = currentPose_;
        currentPose_  = groundTruthPoses_.poseAt(currentScan_.times.back());
    }
    else
    {
        currentOdometry_ = odometryPoses_.poseAt(currentScan_.times.back());
    }
}


void OccupancyGridSLAM::initializePosesIfNeeded(void)
{
    // The initial poses need to be set with the timestamps associated with the first last scan to ensure that proper
    // interpolation of the laser scan happen in MovingLaserScan. This initialization requires the timestamp of the
    // first laser scan, so it can't be performed in the constructor.
    if(!haveInitializedPoses_)
    {
        previousPose_ = initialPose_;
        previousPose_.utime = currentScan_.times.front();
        
        currentPose_ = previousPose_;
        currentPose_.utime  = currentScan_.times.back();
        haveInitializedPoses_ = true;
        
        filter_.initializeFilterAtPose(previousPose_);
    }
    
    assert(haveInitializedPoses_);
}


void OccupancyGridSLAM::updateLocalization(void)
{
    if(haveMap_ && (mode_ != mapping_only))
    {
        previousPose_ = currentPose_;
        if(mode_ == action_only){
            currentPose_  = filter_.updateFilterActionOnly(currentOdometry_);
        }
        else{
            currentPose_  = filter_.updateFilter(currentOdometry_, currentScan_, map_);
        }

        if (!filter_.isRobotMove()){
            doRMS(currentPose_);
        }

        // if (filter_.isRobotMove()){
        //     doSquareError(initialPose_, currentPose_);
        // }
        // else {
        //     doRMS();
        // }
        auto particles = filter_.particles();

        lcm_.publish(SLAM_POSE_CHANNEL, &currentPose_);
        lcm_.publish(SLAM_PARTICLES_CHANNEL, &particles);

   }
}


void OccupancyGridSLAM::updateMap(void)
{
    if(mode_ != localization_only && mode_ != action_only)
    {
        // Process the map
        mapper_.updateMap(currentScan_, currentPose_, map_);
        haveMap_ = true;
    }

    // Publish the map even in localization-only mode to ensure the visualization is meaningful
    // Send every 5th map -- about 1Hz update rate for map output -- can change if want more or less during operation
    if(mapUpdateCount_ % 5 == 0)
    {
        if(haveMap_) {
            auto mapMessage = map_.toLCM();
            lcm_.publish(SLAM_MAP_CHANNEL, &mapMessage);
            map_.saveToFile("current.map");
        }
    }

    ++mapUpdateCount_;
}

void OccupancyGridSLAM::doSquareError(pose_xyt_t initialPose, pose_xyt_t currentPose){
    // std::cout << "initialPose x: " << initialPose.x << " ";
    // std::cout << "currentPose x: " << currentPose.x << std::endl;
    // std::cout << "initialPose y: " << initialPose.y << " ";
    // std::cout << "currentPose y: " << currentPose.y << std::endl;
    // std::cout << "initialPose theta: " << initialPose.theta << " ";
    // std::cout << "currentPose theta: " << currentPose.theta << std::endl;

    float x_error = currentPose.x - initialPose.x;
    float y_error = currentPose.y - initialPose.y;
    float theta_error = angle_diff(currentPose.theta, initialPose.theta);

    square_error_x_ += pow(x_error, 2);
    square_error_y_ += pow(y_error, 2);
    square_error_theta_ += pow(theta_error, 2);
    data_num_ += 1;
}

void OccupancyGridSLAM::doRMS(pose_xyt_t currentPose){
    float square_error_x = pow(currentPose.x, 2);
    float square_error_y = pow(currentPose.y, 2);
    float square_error_theta = pow(currentPose.theta, 2);

    float mean_x = square_error_x;
    float mean_y = square_error_y;
    float mean_theta = square_error_theta;

    float RMS_x = sqrt(mean_x);
    float RMS_y = sqrt(mean_y);
    float RMS_theta = sqrt(mean_theta);

    std::cout << "RMS of x: " << RMS_x;
    std::cout << " RMS of y: " << RMS_y;
    std::cout << " RMS of theta: " << RMS_theta;
    std::cout << std::endl;
}
