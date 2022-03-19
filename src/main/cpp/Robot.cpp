// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/RobotController.h>
#include <iostream>

// roboRIO-TEAM-frc.local
using namespace std;
#include "Constructor.h"
#include "ModuleBase.h"

// // All Module Includes
#include "DriveBaseModule.h"
#include "ErrorModule.h"
#include "AutonomousModule.h"

void Robot::RobotInit() {
    if (!Constructor::constructThreadedRobot(std::vector<ModuleBase*> {new ErrorModule, new DriveBaseModule, new AutonomousModule}, this)) { // Pass a reference of this object to all modules
    // frc::DriverStation::ReportError("[Constructor] Web Construction has failed; ensure it is acyclic and constructable");
    return;
  }
}

bool shooter_speed() {return false;}
bool ballOneLoaded() {return true;}
bool ballTwoLoaded() {return false;}
bool intakeExtended() {return true;}

void Robot::RobotPeriodic() {
  //frc::Shuffleboard::GetTab("Drive Train") //Done
     //.Add("Left Y", -(m_stick->GetRawAxis(1))).GetEntry().SetDouble(-(m_stick->GetRawAxis(1))); //Done
  //frc::Shuffleboard::GetTab("Drive Train") //Done
     // .Add("Right X", m_stick->GetRawAxis(4)).GetEntry().SetDouble(m_stick->GetRawAxis(4)); //Done
  //frc::Shuffleboard::GetTab("Drive Train") //Done
  //   .Add("Left Encoder", m_leftEncoder.GetPosition()).GetEntry().SetDouble(m_leftEncoder.GetPosition()); 
  //frc::Shuffleboard::GetTab("Drive Train")
  //   .Add("Right Encoder", m_rightEncoder.GetPosition()).GetEntry().SetDouble(m_rightEncoder.GetPosition());  
  frc::Shuffleboard::GetTab("Drive Train") //Done
     .Add("Battery Level", (double) frc::RobotController::GetBatteryVoltage()).GetEntry().SetDouble((double)frc::RobotController::GetBatteryVoltage()); //Done
  frc::Shuffleboard::GetTab("Drive Train")  //Done
     .Add("Distance to Goal", 0);//.GetEntry().SetDouble(m_rightEncoder.GetPosition()); //Done
  frc::Shuffleboard::GetTab("Drive Train") //Done
     .Add("Angle to Goal", 0); //Done
  frc::Shuffleboard::GetTab("Drive Train") //Done
     .AddBoolean("Shooter UTS", shooter_speed); //Done
  frc::Shuffleboard::GetTab("Drive Train")
     .AddBoolean("Intake Extended", intakeExtended);
  frc::Shuffleboard::GetTab("Drive Train")
     .AddBoolean("Ball 1 Loaded", ballOneLoaded);
  frc::Shuffleboard::GetTab("Drive Train")
     .AddBoolean("Ball 2 Loaded", ballTwoLoaded);
}
 
void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
