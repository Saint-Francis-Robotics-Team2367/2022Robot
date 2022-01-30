// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SFDrive.h"
#include <math.h>

SFDrive::SFDrive(rev::CANSparkMax* lMotor, rev::CANSparkMax* rMotor) : lMotor{lMotor}, rMotor{rMotor} {}

void SFDrive::ArcadeDrive(double xSpeedi, double zRotationi) 
{
    double leftMotorOutput, rightMotorOutput;
    float xSpeed = xSpeedi;
    float zRotation = zRotationi;

    LimitRate(xSpeed, zRotation);

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

void SFDrive::LimitRate(float& s, float& t) {
    double k = 5; //1/k = rate to speed up [so 0.2 seconds]
    float currTime = frc::Timer::GetFPGATimestamp().value();
    float deltaTime = currTime - prevTime;
    float r_s = (s - prev_value_speed) / deltaTime;
    float r_t = (t - prev_value_turn) / deltaTime;

    if ((fabs(r_s) > k) && (fabs(s) > fabs(prev_value_speed))){
        s = ((k * (fabs(r_s) / r_s) * deltaTime) + prev_value_speed);
    }
    if ((fabs(r_t) > k) && (fabs(t) > fabs(prev_value_turn))){
        t = ((k * (fabs(r_t) / r_t) * deltaTime) + prev_value_turn);
    }
    frc::SmartDashboard::PutNumber("Final S", s);
    frc::SmartDashboard::PutNumber("Final T", t);

    prev_value_speed = s;
    prev_value_turn = t;
    prevTime = currTime;
}