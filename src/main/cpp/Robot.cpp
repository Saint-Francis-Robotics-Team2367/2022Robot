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

void Robot::RobotInit() {
  // Restore factory defaults on drive motors
  m_leftLeadMotor->RestoreFactoryDefaults();
  m_rightLeadMotor->RestoreFactoryDefaults();
  m_leftFollowMotor->RestoreFactoryDefaults();
  m_rightFollowMotor->RestoreFactoryDefaults();

  // Set current limit for drive motors
  m_leftLeadMotor->SetSmartCurrentLimit(driveMotorCurrentLimit);
  m_rightLeadMotor->SetSmartCurrentLimit(driveMotorCurrentLimit);
  m_leftFollowMotor->SetSmartCurrentLimit(driveMotorCurrentLimit);
  m_rightLeadMotor->SetSmartCurrentLimit(driveMotorCurrentLimit);

  // Set drive motors to brake mode
  m_leftLeadMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rightLeadMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_leftFollowMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rightFollowMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  // Set followers and inverts for drive motors
  m_leftLeadMotor->SetInverted(true);
  m_leftFollowMotor->Follow(*m_leftLeadMotor, false);
  m_rightLeadMotor->SetInverted(false);
  m_rightFollowMotor->Follow(*m_rightLeadMotor, false);

}

bool shooter_speed() {return false;}

void Robot::RobotPeriodic() {
  frc::Shuffleboard::GetTab("Drive Train")
     .Add("Left Y", -(m_stick->GetRawAxis(1))).GetEntry().SetDouble(-(m_stick->GetRawAxis(1)));
  frc::Shuffleboard::GetTab("Drive Train")
     .Add("Right X", m_stick->GetRawAxis(4)).GetEntry().SetDouble(m_stick->GetRawAxis(4));
  frc::Shuffleboard::GetTab("Drive Train")
     .Add("Left Encoder", m_leftEncoder.GetPosition()).GetEntry().SetDouble(m_leftEncoder.GetPosition());  
  frc::Shuffleboard::GetTab("Drive Train")
     .Add("Right Encoder", m_rightEncoder.GetPosition()).GetEntry().SetDouble(m_rightEncoder.GetPosition());  
  frc::Shuffleboard::GetTab("Drive Train")
    .Add("Battery Level", (double) frc::RobotController::GetBatteryVoltage()).GetEntry().SetDouble((double)frc::RobotController::GetBatteryVoltage());
  frc::Shuffleboard::GetTab("Drive Train")
     .Add("Distance to Goal", 0);//.GetEntry().SetDouble(m_rightEncoder.GetPosition()); 
  frc::Shuffleboard::GetTab("Drive Train")  
     .Add("Angle to Goal", 0);
  frc::Shuffleboard::GetTab("Drive Train")
     .AddBoolean("Shooter UTS", shooter_speed);
}
 
void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  m_leftEncoder.SetPosition(0);
  m_rightEncoder.SetPosition(0);
  m_leftEncoder.SetPositionConversionFactor(0.168);
  m_rightEncoder.SetPositionConversionFactor(0.168);
}

void Robot::TeleopPeriodic() {
  left_y = m_stick->GetRawAxis(1);
  right_x = m_stick->GetRawAxis(4);

  m_robotDrive->ArcadeDrive(-left_y, right_x);
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {
  tested_motors = false;
}

void Robot::TestPeriodic() {
  if (tested_motors == false) {
    TestFunctions->checkMotorIDs();
    tested_motors = true;
  } else {
    exit(0);
  }
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif