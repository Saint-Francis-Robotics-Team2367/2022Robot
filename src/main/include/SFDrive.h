// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "rev/CANSparkMax.h"
#include <frc/smartdashboard/SmartDashboard.h>

class SFDrive {
 public:
  // member variables
  const double deadband = 0.08;
  const double PI = 3.14159265;
  //an approximate, make better when real
  const double centerToWheel = 1.04;
  rev::CANSparkMax* leftLeadMotor = nullptr;
  rev::CANSparkMax* rightLeadMotor = nullptr;
  rev::CANSparkMax* leftFollowMotor = nullptr;
  rev::CANSparkMax* rightFollowMotor = nullptr;
  //works ig?
  rev::CANEncoder m_leftEncoder = leftLeadMotor->GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, 42);
  rev::CANEncoder m_rightEncoder = rightLeadMotor->GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, 42);

  //looked at the way they did it in the passed, they only have one thread variable at a time, ig that's all we need, but should I make more somehow?
  std::thread * thread = nullptr;
  bool stopThread = false;
  bool threadFinished = true;

  //Set a conversion factor
  // constructor
  SFDrive(rev::CANSparkMax* leftLeadMotor, rev::CANSparkMax* rightLeadMotor, rev::CANSparkMax* leftFollowMotor, rev::CANSparkMax* rightFollowMotor);

  
 public:
  // methods
  void ArcadeDrive(double joystickX, double joystickY);
  bool PIDDrive(float feet, float maxAcc, float maxVelocity);
  bool PIDTurn(float angle, float radius, float maxAcc, float maxVelocity);
  void graph(double currentVelocity, double currentPosition, float time, double setpoint);
  void PIDTuning(float delta);
  void setP(double value);
  void setI(double value);
  void setD(double value);


};
