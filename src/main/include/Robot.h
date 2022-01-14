// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include "rev/CANSparkMax.h"
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Joystick.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include "SFDrive.h"


class Robot : public frc::TimedRobot {
 public:
  rev::CANSparkMax * m_leftLeadMotor = new rev::CANSparkMax(12, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax * m_rightLeadMotor = new rev::CANSparkMax(15, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax * m_leftFollowMotor = new rev::CANSparkMax(13, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax * m_rightFollowMotor = new rev::CANSparkMax(14, rev::CANSparkMax::MotorType::kBrushless);

  rev::CANEncoder m_leftEncoder = m_leftLeadMotor->GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, 42);
  rev::CANEncoder m_rightEncoder = m_rightLeadMotor->GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, 42);

  frc::Joystick *stick = new frc::Joystick(0);
  SFDrive* m_robotDrive = new SFDrive(m_leftLeadMotor, m_rightLeadMotor, m_leftFollowMotor, m_rightFollowMotor);

  double joystickY = 0.0; // negate Axis 1, not Axis 4
  double joystickX = 0.0;

  double prevTime;

  double distanceToDeccelerate;
  double currentVelocity;
  //change these
  const double maxVelocity = 21;
  const double maxAcc = 7;
  double delta = 1;
  double testBool = true;
  double currentPosition;
  double timeElapsed = 0;
  double centerToWheel = 1.23; //in feet



  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void PIDTesting();

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};

void Robot::PIDTesting() {
  //frc::SmartDashboard::PutNumber("delta", delta);
  double currentLeftLead = m_leftLeadMotor->GetOutputCurrent();
  double currentRightLead = m_rightLeadMotor->GetOutputCurrent();
  frc::SmartDashboard::PutNumber("Total Current", currentLeftLead+currentRightLead);

  //Making it so you can manually set m_p and positionTotal: m_p is essential with PID, change by an order of magnitude to start run
  double m_P = frc::SmartDashboard::GetNumber("Pd", 0.30);
  //bool isNegative;
  m_leftLeadMotor->GetPIDController().SetP(m_P);
  m_rightLeadMotor->GetPIDController().SetP(m_P);
  frc::SmartDashboard::PutNumber("Pd", m_P);

  double m_D = frc::SmartDashboard::GetNumber("D Value", 0.0);
  //bool isNegative;
  m_leftLeadMotor->GetPIDController().SetD(m_D);
  m_rightLeadMotor->GetPIDController().SetD(m_D);
  frc::SmartDashboard::PutNumber("D Value", m_D);

  double m_I = frc::SmartDashboard::GetNumber("I Value", 0.0);
  //bool isNegative;
  m_leftLeadMotor->GetPIDController().SetI(m_I);
  m_rightLeadMotor->GetPIDController().SetI(m_I);
  frc::SmartDashboard::PutNumber("I Value", m_I);

 double I_Zone = frc::SmartDashboard::GetNumber("I_Zone", 0.0);
  //bool isNegative;
  m_leftLeadMotor->GetPIDController().SetIZone(I_Zone);
  m_rightLeadMotor->GetPIDController().SetIZone(I_Zone);
  frc::SmartDashboard::PutNumber("I_Zone", I_Zone);

  m_leftLeadMotor->GetPIDController().SetIZone(I_Zone);

  double waitTime = frc::SmartDashboard::GetNumber("waitTime", 4);
  frc::SmartDashboard::PutNumber("waitTime", waitTime);
  //frc::SmartDashboard::PutNumber("waitTime", waitTime);
  // positionTotal = frc::SmartDashboard::GetNumber("positionTotal", 6);
  // frc::SmartDashboard::PutNumber("positionTotal", positionTotal);
  // positionTotal = -6;
 
  double currTime = frc::Timer::GetFPGATimestamp();
  frc::SmartDashboard::PutNumber("currTime", currTime);
  frc::SmartDashboard::PutNumber("Setpoint", delta);
  if(currTime > prevTime + waitTime) {
      m_leftLeadMotor->GetPIDController().SetReference(delta, rev::ControlType::kPosition);
      m_rightLeadMotor->GetPIDController().SetReference(delta, rev::ControlType::kPosition);
      delta = delta * -1.0;
 
      frc::SmartDashboard::PutNumber("Right Encoder", m_rightEncoder.GetPosition());
      frc::SmartDashboard::PutNumber("Left Encoder", m_leftEncoder.GetPosition());
      //frc::SmartDashboard::PutNumber("Motor Current", m_leftLeadMotor->GetOutputCurrent());

      prevTime = frc::Timer::GetFPGATimestamp();
      frc::SmartDashboard::PutNumber("prevTime", prevTime);
  }
}


