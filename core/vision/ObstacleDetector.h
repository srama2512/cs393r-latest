#pragma once

#include <vision/ObjectDetector.h>

class TextLogger;

/// @ingroup vision
class ObstacleDetector : public ObjectDetector {
 public:
  ObstacleDetector(DETECTOR_DECLARE_ARGS);
  void findObstacles();
 private:
  TextLogger* textlogger;
};
