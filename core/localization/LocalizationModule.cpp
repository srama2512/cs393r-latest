#include <localization/LocalizationModule.h>
#include <memory/WorldObjectBlock.h>
#include <memory/LocalizationBlock.h>
#include <memory/GameStateBlock.h>
#include <memory/RobotStateBlock.h>
#include <localization/ParticleFilter.h>
#include <localization/Logging.h>

// Boilerplate
LocalizationModule::LocalizationModule() : tlogger_(textlogger), pfilter_(new ParticleFilter(cache_, tlogger_)) {
  ball_x_kf.set_A_matrix({{1.0, 2.0}, {0.0, 1.0}});
  ball_y_kf.set_A_matrix({{1.0, 2.0}, {0.0, 1.0}});

  ball_x_kf.set_R_matrix({{0.1, 0.0}, {0.0, 0.0}});
  ball_y_kf.set_R_matrix({{0.1, 0.0}, {0.0, 0.0}});

  ball_x_kf.set_Q_matrix({{0.1}});
  ball_y_kf.set_Q_matrix({{0.1}});

  ball_x_kf.set_sigma_matrix({{1.0, 0.0}, {0.0, 0.0}});
  ball_y_kf.set_sigma_matrix({{1.0, 0.0}, {0.0, 0.0}});

  prev_x = 0.0;
  prev_y = 0.0;
  prev_time = std::chrono::system_clock::now();
  prev_vel_x = 0;
  prev_vel_y = 0;

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
  auto& ball_kf = cache_.world_object->objects_[WO_BALL_KF];
  auto& self = cache_.world_object->objects_[cache_.robot_state->WO_SELF];

  // Retrieve the robot's current location from localization memory
  // and store it back into world objects
  auto sloc = cache_.localization_mem->player_;
  self.loc = sloc;
  

  //TODO: modify this block to use your Kalman filter implementation
  if(ball.seen) {
    double ball_x = ball.visionDistance * cos(ball.visionBearing);
    double ball_y = ball.visionDistance * sin(ball.visionBearing);
    
    auto curr_time = std::chrono::system_clock::now();
    chrono::duration<double> diff = curr_time - prev_time;
    double duration = diff.count();
    prev_time = curr_time;

    ball_x_kf.updateBelief({0.0}, {ball_x});
    ball_y_kf.updateBelief({0.0}, {ball_y});

    vector<double> smoothed_x_state = ball_x_kf.get_mu();
    double smoothed_x = smoothed_x_state[0];
    vector<double> smoothed_y_state = ball_y_kf.get_mu();
    double smoothed_y = smoothed_y_state[0];

    double vel_x = (smoothed_x - prev_x)/duration;
    double vel_y = (smoothed_y - prev_y)/duration;

    // printf("---> LM::pF x: %8.4f   y: %8.4f   vel_x: %8.4f   vel_y: %8.4f   dur: %8.4f\n", smoothed_x, smoothed_y, vel_x, vel_y, duration);

    // cout << "---> LM::pF x: " << smoothed_x << " \ty: " << smoothed_y << " \tvel_x: " << vel_x << " \tvel_y: " << vel_y << "\tdur: " << duration << endl; 

    prev_vel_x = vel_x;
    prev_vel_y = vel_y;
    prev_x = smoothed_x;
    prev_y = smoothed_y;

    vector<double> sigma_x = ball_x_kf.get_sigma();
    vector<double> sigma_y = ball_y_kf.get_sigma();

    ball_x_kf.set_A_matrix({{1.0, vel_x * duration}, {0.0, 0.0}});
    ball_y_kf.set_A_matrix({{1.0, vel_y * duration}, {0.0, 0.0}});

    double smoothed_ball_bearing = atan2(smoothed_y, smoothed_x);
    double smoothed_ball_distance = sqrt(pow(smoothed_x, 2) + pow(smoothed_y, 2));

    // Compute the relative position of the ball from vision readings
    auto relBall = Point2D::getPointFromPolar(ball.visionDistance, ball.visionBearing);

    // Compute the global position of the ball based on our assumed position and orientation
    auto globalBall = relBall.relativeToGlobal(self.loc, self.orientation);

    // Update the ball in the WorldObject block so that it can be accessed in python
    ball.loc = globalBall;
    ball.distance = ball.visionDistance;
    ball.bearing = ball.visionBearing;
    //ball.absVel = fill this in

    ball_kf.distance = smoothed_ball_distance;
    ball_kf.bearing = smoothed_ball_bearing;
    ball_kf.seen = true;

    // Update the localization memory objects with localization calculations
    // so that they are drawn in the World window
    cache_.localization_mem->state[0] = ball.loc.x;
    cache_.localization_mem->state[1] = ball.loc.y;
    cache_.localization_mem->covariance = decltype(cache_.localization_mem->covariance)::Identity() * 10000;
  } 
  //TODO: How do we handle not seeing the ball?
  else {
    auto curr_time = std::chrono::system_clock::now();
    chrono::duration<double> diff = curr_time - prev_time;
    double duration = diff.count();
    if(duration < 0.5) {
      // extrapolate based on previous velocity estimate
      double ball_x = prev_x + prev_vel_x * duration;
      double ball_y = prev_y + prev_vel_y * duration;
      //printf("---> Ball not seen. Predicting future B-) x: %8.4f   y: %8.4f   vel_x: %8.4f   vel_y: %8.4f   dur: %8.4f\n", ball_x, ball_y, prev_vel_x, prev_vel_y, duration);
    } 
  }
}
