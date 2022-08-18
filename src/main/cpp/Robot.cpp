// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Solenoid.h>
#include <frc/PowerDistribution.h>
#include <frc/DoubleSolenoid.h>
#include <frc/PneumaticsControlModule.h>





void Robot::RobotInit()
{
  //compRobotDrive.periodicInit();
 
}

void Robot::RobotPeriodic()
{
  
}
void Robot::AutonomousInit()
{
}
void Robot::AutonomousPeriodic()
{

}

void Robot::TeleopInit()
{
   compRobotDrive.periodicInit();

}

void Robot::TeleopPeriodic()
{
  float rightStickOutput = -1.0 * driverStick->GetRawAxis(4);

  // float rightStickOutput = -1.0 * driverStick->GetRawAxis(4);
  // //rightStickOutput = 
  //GyroPIDDrive.arcadeDrive(driverStick->GetRawAxgit bris(1),  rightStickOutput);
  compRobotDrive.arcadeDrive(driverStick->GetRawAxis(1),  rightStickOutput, compRobotDrive.TurningSensitivity(compRobotDrive.getRightMotorSpeed() + compRobotDrive.getLeftMotorSpeed())/2); // ASK MR. P ABOUT KRISHNA'S CODE 
  // It should spit out an ideal sensitivity);
  //get error instead?

}


void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit()
{
}

void Robot::TestPeriodic()
{
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
