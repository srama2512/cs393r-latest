#include <localization/ParticleFilter.h>
#include <memory/FrameInfoBlock.h>
#include <memory/OdometryBlock.h>
#include <common/Random.h>
#include <cmath>
#include <cstdio>

inline double loge(double val) {
  return log2(val) / log2(M_E);
}

ParticleFilter::ParticleFilter(MemoryCache& cache, TextLogger*& tlogger) 
  : cache_(cache), tlogger_(tlogger), dirty_(true) {
}

void ParticleFilter::init(Point2D loc, float orientation) {
  mean_.translation = loc;
  mean_.rotation = orientation;
  n_particles = 300;
  n_rand_particles = 1;
  sigma_x = 10.0;
  sigma_y = 10.0;
  sigma_t = 0.1;

  reset();
}

double ParticleFilter::getGaussianLogProb(double mu, double sigma, double x) {
  return -loge(sqrt(2 * M_PI) * sigma) + (-(mu - x) * (mu - x) / (2 * sigma * sigma));
}

double ParticleFilter::getLogProbObservation(Particle p, int objEnum, double visionDistance, double visionBearing) {
  double angle = p.t + visionBearing;
  double x_pred = p.x + visionDistance * cos(angle);
  double y_pred = p.y + visionDistance * sin(angle);
  int objIdx = objEnum - WO_BEACON_BLUE_YELLOW + 1;
  Point2D loc = landmarkLocation[objIdx];
  double sigma = 500.0;
  double logprob = getGaussianLogProb(loc.x, sigma, x_pred) + getGaussianLogProb(loc.y, sigma, y_pred);
  return logprob;
}

void ParticleFilter::updateLogProbParticle(Particle& particle) {

  double logprob = 0.0;
  for (int idx = WO_BEACON_BLUE_YELLOW; idx <= WO_BEACON_BLUE_YELLOW + 5; idx++) {
    auto& beacon = cache_.world_object->objects_[idx];
    if (beacon.seen) {
      double obs_prob = getLogProbObservation(particle, idx, (double)beacon.visionDistance, (double)beacon.visionBearing);
      logprob += obs_prob;
    }
  }
  particle.w += logprob;
}

bool ParticleFilter::landmarksSeen() {
  for (int idx = WO_BEACON_BLUE_YELLOW; idx <= WO_BEACON_BLUE_YELLOW + 5; idx++) {
    auto& beacon = cache_.world_object->objects_[idx];
    if (beacon.seen) {
      return true;
    }
  }
  return false;
}

void ParticleFilter::printParticles() {
  for (auto& p : particles()) {
    cout << "Particle: x: " << p.x << " y: " << p.y << " t: " << p.t << " w: " << p.w << endl;
  }
  cout << endl << endl;
}

void ParticleFilter::normalizeWeights() {
  double max_log_prob = particles()[0].w;

  for(auto& p : particles()) {
    max_log_prob = max(max_log_prob, (double) p.w);
  }

  double weightSum = 0.0;
  for (auto& p : particles()) {
    weightSum += exp(p.w - max_log_prob);
  }

  for (auto& p : particles()) {
    p.w -= max_log_prob + loge(weightSum);
  }
}

Particle ParticleFilter::sampleParticle(double t, std::vector<double> cumulative_prob) {
  assert(n_particles != 0);
  auto it = lower_bound(cumulative_prob.begin(), cumulative_prob.end(), t);
  int idx = it - cumulative_prob.begin();

  return particles()[idx];
}

void ParticleFilter::resampleParticles() {
  std::vector<Particle> particles_new;

  std::vector<double> cumulative_prob(n_particles);

  double prob = 0.0;
  for(int i = 0; i < n_particles; ++i) {
    prob += exp(particles()[i].w);
    cumulative_prob[i] = prob;
  }

  double t = Random::inst().sampleU(0.0, 1.0) / (double) n_particles;

  for (int i=0; i<n_particles; i++) {
    Particle p = sampleParticle(t, cumulative_prob);
    t += 1.0 / (double) n_particles;
    particles_new.push_back(p);
  }
  
  auto& particles_ = particles();
  for (int i=0; i<n_particles; i++) {
    particles_[i] = particles_new[i];
    particles_[i].w = -loge(n_particles);
  }
}

void ParticleFilter::addRandomParticles() {
  for(int i = 0; i < n_rand_particles; ++i) {
    int idx = rand() % n_particles;
    auto& p = particles()[idx];

    p.x = Random::inst().sampleU(-1500, 1500);
    p.y = Random::inst().sampleU(-1000, 1000);
    p.t = Random::inst().sampleU(-M_PI, M_PI);
  }
}

void ParticleFilter::processFrame() {
  // Indicate that the cached mean needs to be updated
  dirty_ = true;

  // Retrieve odometry update - how do we integrate this into the filter?
  const auto& disp = cache_.odometry->displacement;
  log(41, "Updating particles from odometry: %2.f,%2.f @ %2.2f", disp.translation.x, disp.translation.y, disp.rotation * RAD_T_DEG);
  // printf("Updating particles from odometry: %2.2f,%2.2f @ %2.2f\n", disp.translation.x, disp.translation.y, disp.rotation * RAD_T_DEG);

  bool landmarksSeenFlag = landmarksSeen();

  // add random particles only when you see beacons
  if(landmarksSeenFlag)
    addRandomParticles();

  // Dynamics update
  for(auto& p : particles()) {
    double r = sqrt(pow(disp.translation.x, 2) + pow(disp.translation.y, 2));
    double xtrans = r * cos(p.t);
    double ytrans = r * sin(p.t);

    p.x = Random::inst().sampleN() * sigma_x + p.x + xtrans;
    p.y = Random::inst().sampleN() * sigma_y + p.y + ytrans;
    p.t = Random::inst().sampleN() * sigma_t + p.t + disp.rotation;
    p.t = fmod(p.t, 2.0 * M_PI);
    if (p.t < 0.0)
      p.t += 2 * M_PI;
  }

  for(auto& p : particles()) {
    updateLogProbParticle(p);
  }

  normalizeWeights();

  // resample normalizes the probablities to uniform
  // resample only when you see beacons
  if(landmarksSeenFlag) {
    resampleParticles();
    normalizeWeights();
  }
}

const Pose2D& ParticleFilter::pose() const {
  if(dirty_) {
    // Compute the mean pose estimate
    mean_ = Pose2D();

    double sin_ = 0.0;
    double cos_ = 0.0;

    for(const auto& p : particles()) {
      mean_.translation.x += exp(p.w) * p.x;
      mean_.translation.y += exp(p.w) * p.y;
      sin_ += exp(p.w) * sin(p.t);
      cos_ += exp(p.w) * cos(p.t);
    }

    // mean_.translation.x /= (double) n_particles;
    // mean_.translation.y /= (double) n_particles;
    double angle_estimate = atan2(sin_, cos_);
    mean_.rotation = angle_estimate > 0 ? angle_estimate : 2.0 * M_PI + angle_estimate;

    // Pose2D var_;

    // for(const auto& p : particles()) {
    //   var_.translation.x += (p.x - mean_.translation.x) * (p.x - mean_.translation.x);
    //   var_.translation.y += (p.y - mean_.translation.y) * (p.y - mean_.translation.y);
    //   var_.rotation += (p.t - mean_.rotation) * (p.t - mean_.rotation);
    // }

    // var_.translation.x /= (double) n_particles;
    // var_.translation.y /= (double) n_particles;
    // var_.rotation /= (double) n_particles;

    dirty_ = false;

    // cout << "Pose mean: X: " << mean_.translation.x << " Y: " << mean_.translation.y << " theta: " << mean_.rotation << endl;
    // cout << "Pose STD : X: " << sqrt(var_.translation.x) << " Y: " << sqrt(var_.translation.y) << " theta: " << sqrt(var_.rotation) << endl;
  }
  return mean_;
}

void ParticleFilter::reset() {
  // Generate random particles for demonstration
  particles().resize(n_particles);

  for(auto& p : particles()) {
    p.x = Random::inst().sampleU(-1500, 1500);
    p.y = Random::inst().sampleU(-1000, 1000);
    p.t = Random::inst().sampleU(-M_PI, M_PI);
    p.w = loge((double) Random::inst().sampleU());
  }
  normalizeWeights();

}

