#pragma once

#include <vision/ObjectDetector.h>

#define ASPECT_RATIO_LOW_BOUND 0.8
#define ASPECT_RATIO_HIGH_BOUND 1.4
#define DENSITY_LOW_BOUND 0.6
#define AREA_SIM_LOW_BOUND 0.6
#define AREA_SIM_HIGH_BOUND 1.4
class TextLogger;

/// @ingroup vision
class BeaconDetector : public ObjectDetector {
 public:
  BeaconDetector(DETECTOR_DECLARE_ARGS);
  void init(TextLogger* tl){ textlogger = tl; }
  pair<Blob, Blob> findBeaconsOfType(const vector<Blob> &tb, const vector<Blob> &bb);
  void findBeacons(vector<Blob> &blobs);
 private:
  TextLogger* textlogger;
};
