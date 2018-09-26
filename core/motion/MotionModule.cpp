#include "MotionModule.h"

#include <common/RobotInfo.h>

void MotionModule::specifyMemoryDependency() {
  requiresMemoryBlock("body_model");
  requiresMemoryBlock("processed_joint_angles");
  requiresMemoryBlock("processed_joint_commands");
  requiresMemoryBlock("processed_sensors");
  requiresMemoryBlock("walk_request");
}

void MotionModule::specifyMemoryBlocks() {
  getMemoryBlock(body_model_,"body_model");
  getMemoryBlock(joint_angles_,"processed_joint_angles");
  getMemoryBlock(commands_,"processed_joint_commands");
  getMemoryBlock(sensors_,"processed_sensors");
  getMemoryBlock(walk_request_,"walk_request");
}

MotionModule::MotionModule() {
  xc = PIDController(7e-4, 0.0, 0.0, 50);
  thetac = PIDController(0.6, 0.0, 0.0, 0.174);

}
void MotionModule::initSpecificModule() {
  //walk_engine_.init(memory_);
}

void MotionModule::processFrame() {
  cout << "Reached processFrame!" << endl;
  cout << "processFrame ===> Vision distance to ball: " << walk_request_->target_point_.translation.x << endl;
  cout << "processFrame ===> Vision bearing to ball: " << walk_request_->target_point_.rotation * RAD_TO_DEG << endl;

  float vel_x = xc.update(0.0, walk_request_->target_point_.translation.x);
  float omega = thetac.update(0.0, walk_request_->target_point_.rotation);
  cout << "processFrame ===> vel_x: " << vel_x << "   omega: " << omega << endl;
  walk_request_->setWalk(vel_x, 0.0, omega);
}

void MotionModule::processWalkFrame() {
}
