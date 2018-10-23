#pragma once

#include <math/Pose2D.h>
#include <memory/WorldObjectBlock.h>
#include <memory/MemoryCache.h>
#include <memory/LocalizationBlock.h>
#include <localization/Logging.h>
#include <common/Field.h>

class ParticleFilter {
  public:
    ParticleFilter(MemoryCache& cache, TextLogger*& tlogger);
    void init(Point2D loc, float orientation);
    void processFrame();
    const Pose2D& pose() const;
    inline const std::vector<Particle>& particles() const {
      return cache_.localization_mem->particles;
    }
    void reset();

  protected:
    inline std::vector<Particle>& particles() {
      return cache_.localization_mem->particles;
    }

  private:
    MemoryCache& cache_;
    TextLogger*& tlogger_;

    mutable Pose2D mean_;
    mutable bool dirty_;
    int n_particles, n_rand_particles;
    double sigma_x, sigma_y, sigma_t;

    void updateLogProbParticle(Particle& particle);
    double getGaussianLogProb(double mu, double sigma, double x);
    double getLogProbObservation(Particle p, int objEnum, double visionDistance, double visionBearing);
    void normalizeWeights();
    Particle sampleParticle(double t, std::vector<double> cumulative_prob);
    void resampleParticles();
    void printParticles();
    void addRandomParticles();
    bool landmarksSeen();
};


