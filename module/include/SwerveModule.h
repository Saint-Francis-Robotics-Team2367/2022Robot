#ifndef SWERVEMODULE_H
#define SWERVEMODULE_H
#include <rev/CANSparkMax.h>
#include <frc/AnalogInput.h>
#include <units/math.h>

class SwerveModule{
public:




  rev::CANSparkMax* driveMotor = nullptr;
  rev::CANSparkMax* turningMotor = nullptr;
  //if you don't include getEncoder here, it doesn't build?
  rev::SparkMaxRelativeEncoder* driveEncoder = nullptr;
  rev::SparkMaxRelativeEncoder* turningEncoder = nullptr;
  
  rev::SparkMaxPIDController* drivePID = nullptr;
  rev::SparkMaxPIDController* turnPID = nullptr;


  frc::AnalogInput* absoluteEncoder = nullptr; //not sure about this (when robot powered off, saves position to figure out where wheels are facing)

  double absoluteEncoderOffsetRadians = 0;
  bool absoluteEncoderReversed = 0;
  

  //constants, go over later
  double wheelDiameter = 4 / (39.37) ;//inches to meters
  double driveGearRatio = 0.64 ;//1/5.84
  double turnGearRatio = 1/18.0 ;//random number
  double driveEncoderRotationsToMeters = driveGearRatio * 3.14 * wheelDiameter;
  double turningEncoderRotationsToRadians = turnGearRatio * 2 * 3.14;
  double driveEncoderRPM = driveEncoderRotationsToMeters / 60; //meters/sec
  double turningEncodersRPM = turningEncoderRotationsToRadians / 60; //radians/sec
  double kPTurning = 0.5; //TUNE


  //maybe don't do constructer like this if you want
  SwerveModule(rev::CANSparkMax* driveMotor, rev::CANSparkMax* turningMotor, rev::SparkMaxRelativeEncoder* driveEncoder,
  rev::SparkMaxRelativeEncoder* turningEncoder, rev::SparkMaxPIDController* drivePID, rev::SparkMaxPIDController* turnPID, frc::AnalogInput* absoluteEncoder, double absoluteEncoderOffset, bool encoderReversed
  );
  //ANALOG INPUT (CONNECTED TO ROBORIO)
  public:

};

#endif
