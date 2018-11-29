#include <localization/ParticleFilter.h>
#include <memory/FrameInfoBlock.h>
#include <memory/OdometryBlock.h>
#include <common/Random.h>
#include <cmath>
#include <cstdio>
#include <climits>

inline double loge(double val) {
  return log2(val) / log2(M_E);
}

inline double clip(double val, double mi, double mx) {
  return max(min(val, mx), mi);
}

ParticleFilter::ParticleFilter(MemoryCache& cache, TextLogger*& tlogger) 
  : cache_(cache), tlogger_(tlogger), dirty_(true) {
}

void ParticleFilter::init(Point2D loc, float orientation) {
  mean_.translation = loc;
  mean_.rotation = orientation;
  n_particles = 300;
  n_rand_particles = 1;
  backup_weights.resize(n_particles);
  sigma_x = 10.0;
  sigma_y = 10.0;
  sigma_t = 0.1;
  entropy_thresh = loge(n_particles) / 1.5;

  kmeans_k_ = 2;
  kmeans_iterations_ = 5;

  x_clipping_min = 350.0;
  x_clipping_max = 1000.0;
  y_clipping_min = -700.0;
  y_clipping_max = 700.0;

  reset();
}

void ParticleFilter::clipParticles() {
  for(auto& p : particles()) {
    p.x = clip(p.x, x_clipping_min, x_clipping_max);
    p.y = clip(p.y, y_clipping_min, y_clipping_max);
  }
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
  double sigma = 50.0;
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

double ParticleFilter::computeEntropy() {
  double entropy = 0.0;
  for (auto& p : particles()) {
    entropy -= (exp(p.w) * p.w);
  }
  return entropy;
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

void ParticleFilter::switchToNormalProb() {
  for(int i = 0; i < particles().size(); ++i) {
    backup_weights[i] = particles()[i].w;
    particles()[i].w = 1.0;
  }
}

void ParticleFilter::switchToLogSpaceProb() {
  for(int i = 0; i < particles().size(); ++i) {
    particles()[i].w = backup_weights[i];
  }
}

void ParticleFilter::processFrame() {
  // Indicate that the cached mean needs to be updated
  dirty_ = true;

  // Retrieve odometry update - how do we integrate this into the filter?
  const auto& disp = cache_.odometry->displacement;
  log(41, "Updating particles from odometry: %2.f,%2.f @ %2.2f", disp.translation.x, disp.translation.y, disp.rotation * RAD_T_DEG);
  // printf("Updating particles from odometry: %2.2f,%2.2f @ %2.2f\n", disp.translation.x, disp.translation.y, disp.rotation * RAD_T_DEG);

  double odometryValue = sqrt(pow(disp.translation.x , 2) + pow(disp.translation.y , 2));
  // printf("odometryValue: %2.f\n", odometryValue);
  odometryValue = min(odometryValue, 10.0);
  sigma_x = max(5.0, odometryValue);
  sigma_y = max(5.0, odometryValue);

  bool landmarksSeenFlag = landmarksSeen();

  // add random particles only when you see beacons
  if(landmarksSeenFlag) {
    // cout << "========> Beacons seen\n";
    addRandomParticles();
  }

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

  switchToLogSpaceProb();
  // clipping particles to X (min, max) Y (min, max) bounds 
  // clipParticles();

  for(auto& p : particles()) {
    updateLogProbParticle(p);
  }

  normalizeWeights();

  // resample normalizes the probablities to uniform
  // resample only when you see beacons
  if(/*landmarksSeenFlag && */computeEntropy() < entropy_thresh) {
    // printf("\n=============== Resampling particles ===============\n");
    resampleParticles();
    normalizeWeights();
  }
  else
  {
    // printf("\n=============== Not resampling particles ===============\n");
  }

  switchToNormalProb();
}

double euclid_dist(Particle a, Particle b) {
  return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

int getNearestCluster(Particle p, vector<Particle> &centroids) {
  double dist = INT_MAX;
  double idx = 0;
  for(int i = 0; i < centroids.size(); ++i) {
    double d = euclid_dist(p, centroids[i]);
    if(d < dist) {
      dist = d;
      idx = i;
    }
  }
  return idx;
}

Particle computeCentroid(vector<Particle> &cluster) {
  Particle p = {0.0, 0.0, 0.0, 0.0};

  for(int i = 0; i < cluster.size(); ++i) {
    p.x += cluster[i].x;
    p.y += cluster[i].y;
  }

  p.x /= (double) cluster.size();
  p.y /= (double) cluster.size();

  return p;
}

void recomputeCentroids(vector<Particle> &centroids, vector<vector<Particle> > &clusters) {
  for(int i = 0; i < centroids.size(); ++i) {
    centroids[i] = computeCentroid(clusters[i]);
  }
}

Particle ParticleFilter::getKmeansPose() {
  vector<Particle> centroids(kmeans_k_);

  // initialise the centroids
  for(int i = 0; i < kmeans_k_; ++i) {
    int idx = rand() % particles().size();
    centroids[i] = particles()[idx];
  }

  vector<vector<Particle> > clusters(kmeans_k_);

  for(int _ = 0; _ < kmeans_iterations_; ++_) {
    clusters.clear();
    clusters.resize(kmeans_k_);
    // run k means interation
    for(auto p : particles()) {
      int cidx = getNearestCluster(p, centroids);
      clusters[cidx].push_back(p);
    }

    // recompute centroids
    recomputeCentroids(centroids, clusters);
  }

  int size = 0;
  int cidx = 0;
  for(int i = 0; i < kmeans_k_; ++i) {
    if(size < clusters[i].size()) {
      size = clusters[i].size();
      cidx = i;
    }
  }

  Particle kmeans_pose = {0.0, 0.0, 0.0, 0.0};
  double sin_ = 0.0;
  double cos_ = 0.0;

  for(int i = 0; i < clusters[cidx].size(); ++i) {
    auto p = clusters[cidx][i];
    kmeans_pose.x += p.x;
    kmeans_pose.y += p.y;
    sin_ += sin(p.t);
    cos_ += cos(p.t);
  }

  kmeans_pose.x /= clusters[cidx].size();
  kmeans_pose.y /= clusters[cidx].size();
  double angle_estimate = atan2(sin_, cos_);
  kmeans_pose.t = angle_estimate > 0 ? angle_estimate : 2.0 * M_PI + angle_estimate;

  return kmeans_pose;
}

Pose2D& ParticleFilter::pose() {
  if(dirty_) {
    #ifndef KMEANS_ENABLED
      // Compute the mean pose estimate
      mean_ = Pose2D();

      double sin_ = 0.0;
      double cos_ = 0.0;

      int idx = 0;
      for(const auto& p : particles()) {
        mean_.translation.x += exp(backup_weights[idx]) * p.x;
        mean_.translation.y += exp(backup_weights[idx]) * p.y;
        sin_ += exp(backup_weights[idx]) * sin(p.t);
        cos_ += exp(backup_weights[idx]) * cos(p.t);
        idx++;
      }

      // mean_.translation.x /= (double) n_particles;
      // mean_.translation.y /= (double) n_particles;
      double angle_estimate = atan2(sin_, cos_);
      mean_.rotation = angle_estimate > 0 ? angle_estimate : 2.0 * M_PI + angle_estimate;
    #endif

    #ifdef KMEANS_ENABLED
      Particle kpose = getKmeansPose();
      mean_.translation.x = kpose.x;
      mean_.translation.y = kpose.y;
      mean_.rotation = kpose.t;
    #endif

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
  switchToNormalProb();

}

