#pragma once

#include <vision/ObjectDetector.h>

#define ASPECT_RATIO_LOW_BOUND 0.5
#define ASPECT_RATIO_HIGH_BOUND 1.5
#define DENSITY_LOW_BOUND 0.6

class TextLogger;

/// @ingroup vision
class BeaconDetector : public ObjectDetector {
 public:
  BeaconDetector(DETECTOR_DECLARE_ARGS);
  void init(TextLogger* tl){ textlogger = tl; }
  pair<Blob, Blob> findBeaconsOfType(vector<Blob> &blobs, Color tcolor, Color bcolor);
  void findBeacons(vector<Blob> &blobs);
 private:
  TextLogger* textlogger;
};
