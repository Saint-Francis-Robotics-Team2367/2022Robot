// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "rev/CANSparkMax.h"

class SFDrive {
 public:
  // member variables
  const double deadband = 0.08;
  rev::CANSparkMax* lMotor = nullptr;
  rev::CANSparkMax* rMotor = nullptr;

  // constructor
  SFDrive(rev::CANSparkMax* lMotor, rev::CANSparkMax* rMotor);
  
 public:
  // methods
  void ArcadeDrive(double xSpeed, double zRotation);
};