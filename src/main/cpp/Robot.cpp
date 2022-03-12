// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constructor.h"
#include "ModuleBase.h"

// // All Module Includes
#include "DriveBaseModule.h"
#include "ErrorModule.h"

void Robot::RobotInit() {
    if (!Constructor::constructThreadedRobot(std::vector<ModuleBase*> {new ErrorModule, new DriveBaseModule}, this)) { // Pass a reference of this object to all modules
    // frc::DriverStation::ReportError("[Constructor] Web Construction has failed; ensure it is acyclic and constructable");
    return;
  }
}

void Robot::RobotPeriodic() {

}
 
void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
}

void Robot::TeleopPeriodic() {

  photonlib::PhotonPipelineResult result = camera.GetLatestResult();
  if (result.HasTargets()) {
        // First calculate range
        pitch_degree = units::degree_t{result.GetBestTarget().GetPitch()};
        units::meter_t range = photonlib::PhotonUtils::CalculateDistanceToTarget(
          CAMERA_HEIGHT, TARGET_HEIGHT, CAMERA_PITCH, pitch_degree);

        theta_rads = atan((2*APEX_HEIGHT*(range+sqrt(range**2-(TARGET_HEIGHT*(range**2)/APEX_HEIGHT))))/(range**2));
        theta_degs = theta_rads * (180 / pi));
        // Use this range as the measurement we give to the PID controller.
        horizontal_dist = cos(CAMERA_MOUNT_ANGLE)*range;
        velocity = sqrt(2*APEX_HEIGHT*GRAV_CONST/sin(theta_rads));
        forwardSpeed = -controller.Calculate(range.value(), GOAL_RANGE_METERS.value());

      } else {
        // If we have no targets, stay still.
        forwardSpeed = 0;
      }
  
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {
 }

void Robot::TestPeriodic() {
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif