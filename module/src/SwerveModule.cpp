#include "SwerveModule.h"

//BEFORE PASSING ENCODERS SETPOSITION AND VELOCITYCONVERSIONFACTOR TO RADIANS!!!!

SwerveModule::SwerveModule(rev::CANSparkMax* driveMotor, rev::CANSparkMax* turningMotor, rev::SparkMaxRelativeEncoder* driveEncoder,
  rev::SparkMaxRelativeEncoder* turningEncoder, rev::SparkMaxPIDController* drivePID, rev::SparkMaxPIDController* turnPID, frc::AnalogInput* absoluteEncoder, 
  double absoluteEncoderOffset, bool encoderReversed
  ) : driveMotor{driveMotor}, turningMotor{turningMotor}, driveEncoder{driveEncoder}, turningEncoder{turningEncoder}, drivePID{drivePID}, 
  turnPID{turnPID}, absoluteEncoder{absoluteEncoder}  {
      //does this work lol?
      this->absoluteEncoderOffsetRadians = absoluteEncoderOffsetRadians;
      this->absoluteEncoderReversed = absoluteEncoderReversed;


      //set inverts maybe and get encoders...and set to radians


      //PID Controller ENABLE CONTINUOUS OUTPUT

      
  }

