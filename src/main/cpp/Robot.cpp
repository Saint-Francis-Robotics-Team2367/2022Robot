// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

// roboRIO-TEAM-frc.local

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

void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("left y: ", -(m_stick->GetRawAxis(1)));
  frc::SmartDashboard::PutNumber("right x: ", m_stick->GetRawAxis(4));
}
 
void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  // frc::Solenoid valve{0};
  m_leftLeadMotor->GetEncoder().SetPosition(0);
  m_rightLeadMotor->GetEncoder().SetPosition(0);
  // compressor = new frc::Spark(1);
  // valve.Set(false);
}

void Robot::TeleopPeriodic() {
  left_y = m_stick->GetRawAxis(1);
  right_x = m_stick->GetRawAxis(4);

  m_robotDrive->ArcadeDrive(-left_y, right_x);
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