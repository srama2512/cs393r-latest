#ifndef MOTION_MODULE_H
#define MOTION_MODULE_H

#include <Module.h>

#include <memory/BodyModelBlock.h>
#include <memory/JointBlock.h>
#include <memory/JointCommandBlock.h>
#include <memory/SensorBlock.h>
#include <memory/WalkRequestBlock.h>
#include <common/PIDController.h>
//struct BodyModelBlock;
#define RAD_TO_DEG  180.0/ M_PI

class MotionModule: public Module {
 public:
  MotionModule();
  void specifyMemoryDependency();
  void specifyMemoryBlocks();
  void initSpecificModule();

  void processFrame();
  void processWalkFrame();

 private:
  BodyModelBlock* body_model_;
  JointCommandBlock *commands_;
  JointBlock *joint_angles_;
  SensorBlock *sensors_;
  WalkRequestBlock *walk_request_;
  PIDController xc; 
  PIDController thetac;

  // UGLY for now
  //WalkEngine walk_engine_;
  //WalkingEngineOutput walk_engine_output_;
};

#endif /* end of include guard: SENSOR_MODULE */
