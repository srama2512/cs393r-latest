#pragma once

#include <Module.h>
#include <memory/MemoryCache.h>
#include <localization/LocalizationParams.h>
#include <math/KalmanFilter.h>
#include <math.h>
#include <chrono>
 
class ParticleFilter;
class Point2D;

class LocalizationModule : public Module {
  public:
    LocalizationModule();
    ~LocalizationModule();
    void specifyMemoryDependency();
    void specifyMemoryBlocks();
    void initSpecificModule();
    void initFromMemory();
    void initFromWorld();
    void reInit();
    void processFrame();

    void loadParams(LocalizationParams params);
    
    void moveBall(const Point2D& position);
    void movePlayer(const Point2D& position, float orientation);
  protected:
    MemoryCache cache_;
    TextLogger*& tlogger_;
    LocalizationParams params_;
    ParticleFilter* pfilter_;
    
    KalmanFilter<2, 1, 1> ball_x_kf;
    KalmanFilter<2, 1, 1> ball_y_kf;

    double prev_x, prev_y, prev_vel_x, prev_vel_y;
    std::chrono::time_point<std::chrono::system_clock> prev_time;
};
