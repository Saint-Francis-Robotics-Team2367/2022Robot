// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

// // All Module Includes
#include "DriveBaseModule.h"
#include "IntakeModule.h"
#include "ShooterModule.h"

DriveBaseModule compRobotDrive;
IntakeModule compIntake;
ShooterModule compShooter;

void Robot::RobotInit() {
  compRobotDrive.periodicInit();
  compIntake.periodicInit();
  compShooter.periodicInit();
}

void Robot::RobotPeriodic() {
  compRobotDrive.periodicRoutine();
  compIntake.periodicRoutine();
  compShooter.periodicRoutine();
}
 
void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  //Move this somewhere else later

  //Shooter Motor inits
 
}

void Robot::TeleopPeriodic() {
  
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

