// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SFDrive.h"
#include <math.h>

SFDrive::SFDrive(rev::CANSparkMax* lMotor, rev::CANSparkMax* rMotor) : lMotor{lMotor}, rMotor{rMotor} {}

void SFDrive::ArcadeDrive(double xSpeed, double zRotation) 
{
    double leftMotorOutput, rightMotorOutput;

    if (fabs(xSpeed) < deadband)
        xSpeed = 0;

    if (fabs(zRotation) < deadband)
        zRotation = 0;

    if (xSpeed >= 0.0) {
        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = xSpeed - zRotation;
    }
    else {
        leftMotorOutput = xSpeed - zRotation;
        rightMotorOutput = xSpeed + zRotation;
    }

    if (leftMotorOutput != 0)
        leftMotorOutput = std::copysign((1/(1-deadband)) * fabs(leftMotorOutput) - (deadband/(1/deadband)), leftMotorOutput);
        
    if (rightMotorOutput != 0)
        rightMotorOutput = std::copysign((1/(1-deadband)) * fabs(rightMotorOutput) - (deadband/(1/deadband)), rightMotorOutput);

    leftMotorOutput = std::copysign(pow(leftMotorOutput, 2), leftMotorOutput);
    rightMotorOutput = std::copysign(pow(rightMotorOutput, 2), rightMotorOutput);

    lMotor->Set(leftMotorOutput);
    rMotor->Set(rightMotorOutput);
}