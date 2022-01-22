// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <photonlib/PhotonUtils.h>

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
  
  // if (!result.HasTargets()) {
  //   frc::SmartDashboard::PutNumber("target distance x", -1);
  //   frc::SmartDashboard::PutNumber("target distance y", -1);
  // }
  // else {
  //   wpi::ArrayRef<photonlib::PhotonTrackedTarget> targs = result.GetTargets();
  //   float xavg;
  //   float yavg;
  //   for (int i = 0; i < targs.size(); i++) {
  //     xavg += targs[i].GetCameraRelativePose().X().value();
  //     yavg += targs[i].GetCameraRelativePose().Y().value();
  //   }
  //   xavg /= targs.size();
  //   yavg /= targs.size();
  //   frc::SmartDashboard::PutNumber("target distance x", xavg);
  //   frc::SmartDashboard::PutNumber("target distance y", yavg);
  // }
}
  
void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {
  if (cam.HasTargets()) {
    frc::SmartDashboard::PutBoolean("camera", true);
    photonlib::PhotonPipelineResult result = cam.GetLatestResult();
    float v = result.GetBestTarget().GetYaw();
    frc::SmartDashboard::PutNumber("yaw", v);
    float y = result.GetBestTarget().GetCameraRelativePose().X().value();
    frc::SmartDashboard::PutNumber("distance", y);
    if (y > 4) {
      y = 0.05;
    }
    else {
      y = 0;
    }
    if (v > abs(0.1))
      m_robotDrive->ArcadeDrive(0, v * 0.02);
    else {
      m_robotDrive->ArcadeDrive(y, 0);
    }
  }
  else {
    frc::SmartDashboard::PutBoolean("camera", false);
  }
}

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
  frc::SmartDashboard::PutNumber("left y: ", -(m_stick->GetRawAxis(1)));
  frc::SmartDashboard::PutNumber("right x: ", m_stick->GetRawAxis(4));

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