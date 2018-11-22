#include <vision/ObstacleDetector.h>
#include <memory/TextLogger.h>
#include <vision/Logging.h>

using namespace Eigen;

ObstacleDetector::ObstacleDetector(DETECTOR_DECLARE_ARGS) : DETECTOR_INITIALIZE {
}

void ObstacleDetector::findObstacles() {
    if(camera_ == Camera::BOTTOM) return;

    static set<WorldObjectType> obstacles = {
    	WO_BEACON_BLUE_YELLOW,
		WO_BEACON_YELLOW_BLUE,
		WO_BEACON_BLUE_PINK,
		WO_BEACON_PINK_BLUE,
		WO_BEACON_PINK_YELLOW,
		WO_BEACON_YELLOW_PINK,
    };

    int num_obstacles = 0;
    auto start_obs_enum = WO_OPPONENT1;

    for(auto& obs: obstacles) {
    	auto& object = vblocks_.world_object->objects_[obs];
    	if(object.seen == false || num_obstacles == 5)
    		continue;

    	auto& obs_obj = vblocks_.world_object->objects_[start_obs_enum + num_obstacles];

    	obs_obj.imageCenterX = object.imageCenterX;
        obs_obj.imageCenterY = object.imageCenterY;
        obs_obj.visionDistance = object.visionDistance;
        obs_obj.visionBearing = object.visionBearing;
        obs_obj.seen = object.seen;
        obs_obj.fromTopCamera = object.fromTopCamera;
        obs_obj.radius = 250;
        num_obstacles++;
    }

    for(int i = num_obstacles; i < 5; ++i) {
    	auto& object = vblocks_.world_object->objects_[start_obs_enum + i];
    	object.seen = false;
    }
}
