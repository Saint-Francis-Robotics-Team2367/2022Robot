// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>


void Robot::RobotInit() {

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
        // pitch_degree = units::degree_t{result.GetBestTarget().GetPitch()};
        pitch_degree = 0;

        float range = result.GetBestTarget().GetCameraRelativePose().X().value();

        theta_rads = atan((2*APEX_HEIGHT*(range+sqrt(pow(range, 2)-(TARGET_HEIGHT*(pow(range, 2))/APEX_HEIGHT))))/(pow(range, 2)));
        theta_degs = theta_rads * (180 / pi);
        central_degs = 90 - theta_degs;
        // Use this range as the measurement we give to the PID controller.
        horizontal_dist = cos(CAMERA_MOUNT_ANGLE)*range;
        velocity = sqrt(2*APEX_HEIGHT*GRAV_CONST)/sin(theta_rads);
        // forwardSpeed = -controller.Calculate(range.value(), GOAL_RANGE_METERS.value());

      } else {
        // If we have no targets, stay still.
        // forwardSpeed = 0;
        std::cout << "no targets" << std::endl;
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
