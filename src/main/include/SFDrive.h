// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "rev/CANSparkMax.h"
#include "frc/Timer.h"
#include <frc/smartdashboard/SmartDashboard.h>
class SFDrive {
 public:
  // member variables
  const double deadband = 0.08;
  rev::CANSparkMax* lMotor = nullptr;
  rev::CANSparkMax* rMotor = nullptr;

  // constructor
  SFDrive(rev::CANSparkMax* lMotor, rev::CANSparkMax* rMotor);
  float prevTime;
  float prev_value_speed;
  float prev_value_turn;
  
 public:
  // methods
  void ArcadeDrive(double xSpeed, double zRotation);
  void LimitRate(float& s, float& t);
};