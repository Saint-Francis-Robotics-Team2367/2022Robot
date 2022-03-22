// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

// // All Module Includes
#include "DriveBaseModule.h"
#include "IntakeModule.h"

DriveBaseModule compRobotDrive;
IntakeModule compIntake;

void Robot::RobotInit() {
    /*if (!Constructor::constructThreadedRobot(std::vector<ModuleBase*> {new ErrorModule, new DriveBaseModule, new AutonomousModule, new IntakeModule, new ShooterModule}, this)) { // Pass a reference of this object to all modules
     std::cout << "poopoo" << std::endl;
     return;
  }*/
  compRobotDrive.periodicInit();
  compIntake.periodicInit();
}

void Robot::RobotPeriodic() {
  compRobotDrive.periodicRoutine();
  compIntake.periodicRoutine();
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

