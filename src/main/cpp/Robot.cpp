// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit()
{
  //compRobotDrive.periodicInit();
}

void Robot::RobotPeriodic(){}

void Robot::AutonomousInit(){}

void Robot::AutonomousPeriodic(){}

void Robot::TeleopInit()
{
  std::cout << "TeleopInit: Entering Teleop" << std::endl;
   compRobotDrive.periodicInit(); // what do you need in this func.
}

void Robot::TeleopPeriodic()
{
  rightStickOutput = -1.0 * driverStick->GetRawAxis(4);

  // float rightStickOutput = -1.0 * driverStick->GetRawAxis(4);
  // //rightStickOutput = 
  //GyroPIDDrive.arcadeDrive(driverStick->GetRawAxgit bris(1),  rightStickOutput);
  
    double turnSense = compRobotDrive.getRightMotorSpeed() + compRobotDrive.getLeftMotorSpeed();
  turnSense/=2;
  
  
  //frc::SmartDashboard::PutNumber("turnSense in robotperiod", turnSense);
  //double afterCall = compRobotDrive.TurningSensitivity(turnSense);
   //frc::SmartDashboard::PutNumber("turnSense after func call in robotperiod", afterCall);

  compRobotDrive.arcadeDrive(driverStick->GetRawAxis(1),  rightStickOutput, compRobotDrive.TurningSensitivity(turnSense)); // ASK MR. P ABOUT KRISHNA'S CODE 
  //compRobotDrive.arcadeDrive(driverStick->GetRawAxis(1),  rightStickOutput);
  // It should spit out an ideal sensitivity);
  //get error instead?

}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit(){}
void Robot::TestPeriodic(){}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
