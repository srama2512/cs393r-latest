#include <localization/LocalizationModule.h>
#include <memory/WorldObjectBlock.h>
#include <memory/LocalizationBlock.h>
#include <memory/GameStateBlock.h>
#include <memory/RobotStateBlock.h>
#include <localization/ParticleFilter.h>
#include <localization/Logging.h>

inline double signum(double val) {
  return val > 0 ? 1.0 : -1.0;
}
// Boilerplate
LocalizationModule::LocalizationModule() : tlogger_(textlogger), pfilter_(new ParticleFilter(cache_, tlogger_)) {

}

LocalizationModule::~LocalizationModule() {
  delete pfilter_;
}

// Boilerplate
void LocalizationModule::specifyMemoryDependency() {
  requiresMemoryBlock("world_objects");
  requiresMemoryBlock("localization");
  requiresMemoryBlock("vision_frame_info");
  requiresMemoryBlock("robot_state");
  requiresMemoryBlock("game_state");
  requiresMemoryBlock("vision_odometry");
}

// Boilerplate
void LocalizationModule::specifyMemoryBlocks() {
  getOrAddMemoryBlock(cache_.world_object,"world_objects");
  getOrAddMemoryBlock(cache_.localization_mem,"localization");
  getOrAddMemoryBlock(cache_.frame_info,"vision_frame_info");
  getOrAddMemoryBlock(cache_.robot_state,"robot_state");
  getOrAddMemoryBlock(cache_.game_state,"game_state");
  getOrAddMemoryBlock(cache_.odometry,"vision_odometry");
}


// Load params that are defined in cfglocalization.py
void LocalizationModule::loadParams(LocalizationParams params) {
  params_ = params;
  printf("Loaded localization params for %s\n", params_.behavior.c_str());
}

// Perform startup initialization such as allocating memory
void LocalizationModule::initSpecificModule() {
  reInit();
}

// Initialize the localization module based on data from the LocalizationBlock
void LocalizationModule::initFromMemory() {
  reInit();
}

// Initialize the localization module based on data from the WorldObjectBlock
void LocalizationModule::initFromWorld() {
  reInit();
  auto& self = cache_.world_object->objects_[cache_.robot_state->WO_SELF];
  pfilter_->init(self.loc, self.orientation);
}

// Reinitialize from scratch
void LocalizationModule::reInit() {
  pfilter_->init(Point2D(-750,0), 0.0f);
  cache_.localization_mem->player_ = Point2D(-750,0);
  cache_.localization_mem->state = decltype(cache_.localization_mem->state)::Zero();
  cache_.localization_mem->covariance = decltype(cache_.localization_mem->covariance)::Identity();
}

void LocalizationModule::moveBall(const Point2D& position) {
  // Optional: This method is called when the player is moved within the localization
  // simulator window.
}

void LocalizationModule::movePlayer(const Point2D& position, float orientation) {
  // Optional: This method is called when the player is moved within the localization
  // simulator window.
}

void LocalizationModule::processFrame() {
  auto& ball = cache_.world_object->objects_[WO_BALL];
  auto& ball_kf_object = cache_.world_object->objects_[WO_BALL_KF];
  auto& self = cache_.world_object->objects_[cache_.robot_state->WO_SELF];

  // Process the current frame and retrieve our location/orientation estimate
  // from the particle filter
  pfilter_->processFrame();
  self.loc = pfilter_->pose().translation;
  self.orientation = pfilter_->pose().rotation;
  log(40, "Localization Update: x=%2.f, y=%2.f, theta=%2.2f", self.loc.x, self.loc.y, self.orientation * RAD_T_DEG);
    
  //TODO: modify this block to use your Kalman filter implementation
  if(ball.seen) {

    double smoothed_ball_distance, smoothed_ball_bearing;
    ball_kf.processFrame(ball.visionDistance, ball.visionBearing, smoothed_ball_distance, smoothed_ball_bearing);

    double ballx = ball.visionDistance * cos(ball.visionBearing);
    double bally = ball.visionDistance * sin(ball.visionBearing);

    double ballxs = smoothed_ball_distance * cos(smoothed_ball_bearing);
    double ballys = smoothed_ball_distance * sin(smoothed_ball_bearing);

    // printf("================================================================================================\n");
    // printf("Orig/Smooth: visionDistance: %.3f, visionBearing %.3f / visionDistance: %.3f, visionBearing %.3f\n", ball.visionDistance, ball.visionBearing, smoothed_ball_distance, smoothed_ball_bearing);
    // printf("Orig/Smooth: x: %.3f y: %.3f, x: %.3f y: %.3f\n", ballx, bally, ballxs, ballys);
    // printf("================================================================================================\n");

    // Compute the relative position of the ball from vision readings
    auto relBall = Point2D::getPointFromPolar(ball.visionDistance, ball.visionBearing);

    // Compute the global position of the ball based on our assumed position and orientation
    auto globalBall = relBall.relativeToGlobal(self.loc, self.orientation);

    // Update the ball in the WorldObject block so that it can be accessed in python
    ball.loc = globalBall;
    ball.distance = ball.visionDistance;
    ball.bearing = ball.visionBearing;
    //ball.absVel = fill this in

    ball_kf_object.distance = smoothed_ball_distance;
    ball_kf_object.bearing = smoothed_ball_bearing;
    
    if(smoothed_ball_distance == -1.0 && smoothed_ball_bearing == -1.0)
      ball_kf_object.seen = false;
    else
      ball_kf_object.seen = true;

    // Update the localization memory objects with localization calculations
    // so that they are drawn in the World window
    cache_.localization_mem->state[0] = ball.loc.x;
    cache_.localization_mem->state[1] = ball.loc.y;
    cache_.localization_mem->covariance = decltype(cache_.localization_mem->covariance)::Identity() * 10000;
  } 
  //TODO: How do we handle not seeing the ball?
  else {
  }
}
