#include <localization/ParticleFilter.h>
#include <memory/FrameInfoBlock.h>
#include <memory/OdometryBlock.h>
#include <common/Random.h>

ParticleFilter::ParticleFilter(MemoryCache& cache, TextLogger*& tlogger) 
  : cache_(cache), tlogger_(tlogger), dirty_(true) {
}

void ParticleFilter::init(Point2D loc, float orientation) {
  mean_.translation = loc;
  mean_.rotation = orientation;
  sigma_x = 50.0;
  sigma_y = 50.0;
  sigma_t = 0.1;
}

// Compute absolute positions from relative positions and object id
// Pose2D getAbsolutePosition(Pose2D relPos, int objEnum){
//   int objIdx = objEnum - WO_BEACON_BLUE_YELLOW + 1;
//   Point2D loc = landmarkLocation[objIdx];
// }

double ParticleFilter::getGaussianProb(double mu, double sigma, double x) {
  return 1.0 / (sqrt(2 * M_PI) * sigma) * exp(-(mu - x) * (mu - x) / (2 * sigma * sigma));
}

double ParticleFilter::getProbObservation(Particle p, int objEnum, double visionDistance, double visionBearing) {
  double angle = p.t + visionBearing;
  double x_pred = p.x + visionDistance * cos(angle);
  double y_pred = p.y + visionDistance * sin(angle);
  int objIdx = objEnum - WO_BEACON_BLUE_YELLOW + 1;
  Point2D loc = landmarkLocation[objIdx];
  double sigma = 50.0;
  double prob = getGaussianProb(loc.x, sigma, x_pred) * getGaussianProb(loc.y, sigma, y_pred);
  return prob;
}

void ParticleFilter::updateProbParticle(Particle& particle) {

  double prob = 1.0;
  for (int idx = WO_BEACON_BLUE_YELLOW; idx <= WO_BEACON_BLUE_YELLOW + 5; idx++) {
    auto& beacon = cache_.world_object->objects_[idx];
    if (beacon.seen) {
      prob *= getProbObservation(particle, idx, (double)beacon.visionDistance, (double)beacon.visionBearing);
    }
  }
  particle.w *= prob;
}

// void ParticleFilter::specifyMemoryDependency() {
//   //last_frame_processed_ = 0;
//   requiresMemoryBlock("world_objects");
// }

// void ParticleFilter::specifyMemoryBlocks() {
//   getOrAddMemoryBlock(cache_.world_object,"world_objects");
// }

// void ParticleFilter::initSpecificModule() {
// }
// void ParticleFilter::normalizeWeights() {
//   double weightSum = 1e-5;
//   for (auto& p : particles()) {
//     weightSum += p.w;
//   }
//   for (auto& p : particles()) {
//     p.w /= weightSum;
//   }
// }

// Particle ParticleFilter::sampleParticle(std::vector<double> cumulative_prob) {
//   assert(n_particles != 0);
//   double t = sampleU(0.0, 1.0);
//   auto it = lower_bound(cumulative_prob.begin(), cumulative_prob.end(), t);
//   int idx = it - cumulative_prob.begin();

//   return particles()[idx];
// }

// void ParticleFilter::resampleParticles() {
//   std::vector<Particle> particles_new;
//   for (int i=0; i<n_particles; i++) {
//     Particle p = sampleParticle();
//     particles_new.push_back(p);
//   }
  
//   auto& particles = particles();
//   for (int i=0; i<n_particles; i++) {
//     particles[i] = particles_new[i];
//   }
// }

void ParticleFilter::processFrame() {
  // Indicate that the cached mean needs to be updated
  dirty_ = true;

  // Retrieve odometry update - how do we integrate this into the filter?
  const auto& disp = cache_.odometry->displacement;
  log(41, "Updating particles from odometry: %2.f,%2.f @ %2.2f", disp.translation.x, disp.translation.y, disp.rotation * RAD_T_DEG);

  // Dynamics update
  for(auto& p : particles()) {
    p.x = Random::inst().sampleN() * sigma_x + p.x + disp.translation.x;
    p.y = Random::inst().sampleN() * sigma_y + p.y + disp.translation.y;
    p.t = Random::inst().sampleN() * sigma_t + p.t + disp.rotation;
  }

  // for(auto& p : particles()) {
  //   updateProbParticle(p);
  // }

  // normalizeWeights();

  // resampleParticles();  
  // normalizeWeights();
}

const Pose2D& ParticleFilter::pose() const {
  if(dirty_) {
    // Compute the mean pose estimate
    mean_ = Pose2D();
    using T = decltype(mean_.translation);
    for(const auto& p : particles()) {
      mean_.translation += T(p.x,p.y);
      mean_.rotation += p.t;
    }
    if(particles().size() > 0)
      mean_ /= static_cast<float>(particles().size());
    dirty_ = false;
  }
  return mean_;
}

void ParticleFilter::reset() {
  // Generate random particles for demonstration
  particles().resize(n_particles);
  auto frame = cache_.frame_info->frame_id;
  for(auto& p : particles()) {
    p.x = Random::inst().sampleN() * 250 + (frame * 5); //static_cast<int>(frame * 5), 250);
    p.y = Random::inst().sampleN() * 250; // 0., 250);
    p.t = Random::inst().sampleN() * M_PI / 4;  //0., M_PI / 4);
    p.w = Random::inst().sampleU();
  }
}

