#include "DriveBaseModule.h"

bool DriveBaseModule::initDriveMotor(rev::CANSparkMax* motor, rev::CANSparkMax* follower, bool invert) {
  motor->SetInverted(invert);
  follower->Follow(*motor, false);
  return motor->GetLastError() == rev::REVLibError::kOk;
}

bool DriveBaseModule::setPowerBudget(rev::CANSparkMax* motor, float iPeak, float iRated, int limitCycles) {
  motor->SetSmartCurrentLimit(iRated);
  motor->SetSecondaryCurrentLimit(iPeak, limitCycles);
  return motor->GetLastError() == rev::REVLibError::kOk;
}

bool DriveBaseModule::setDriveCurrLimit(float iPeak, float iRated, int limitCycles) {
  bool setlFront = setPowerBudget(lMotor, iPeak, iRated, limitCycles);
  bool setrFront = setPowerBudget(rMotor, iPeak, iRated, limitCycles);
  bool setlBack = setPowerBudget(lMotorFollower, iPeak, iRated, limitCycles);
  bool setrBack = setPowerBudget(rMotorFollower, iPeak, iRated, limitCycles);

  return setlFront && setrFront && setlBack && setrBack; // Failure on false
}

void DriveBaseModule::arcadeDrive(float xSpeedi, float zRotationi) {
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

bool DriveBaseModule::PIDTurn(float angle, float radius, float maxAcc, float maxVelocity) {
<<<<<<< HEAD
  lEncoder.SetPosition(0);
  rEncoder.SetPosition(0);
  lEncoder.SetPositionConversionFactor(0.168); //check if this works!
  rEncoder.SetPositionConversionFactor(0.168); 
=======
  rEncoder.SetPosition(0);
  lEncoder.SetPosition(0);
  rEncoder.SetPositionConversionFactor(0.168); //check if this works!
  lEncoder.SetPositionConversionFactor(0.168); 
>>>>>>> temp-threading

  float currentPosition, currentVelocity, endpoint, setpoint, timeElapsed, distanceToDeccelerate = 0; //currentPosition is the set point
  float prevTime = frc::Timer::GetFPGATimestamp().value();
  endpoint = (angle / 360.0) * (radius + centerToWheel) * (2 * PI);
  // frc::SmartDashboard::PutNumber("endpoint", endpoint);


//never use while loops unless threading
  while(currentPosition < endpoint){
    timeElapsed = frc::Timer::GetFPGATimestamp().value() - prevTime;
    distanceToDeccelerate = (3 * currentVelocity * currentVelocity) / (2 * maxAcc);
    if (distanceToDeccelerate > endpoint - currentPosition) {
      currentVelocity -= (maxAcc * timeElapsed);
    }
    else //increase velocity
    {
      currentVelocity += (maxAcc * timeElapsed);
      if (currentVelocity > maxVelocity)
      {
        currentVelocity = maxVelocity;
      }
    }

    currentPosition += currentVelocity * timeElapsed;
    if(currentPosition > endpoint) {
      currentPosition = endpoint;
    }
    //same as other
   
    double outerSetpoint = (currentPosition * 12) / (PI * 6); // for now this is ticks (maybe rotations / gearRatio if not then)
    double innerSetpoint = ((radius - centerToWheel)/(radius + centerToWheel)) * outerSetpoint;
    
    // frc::SmartDashboard::PutNumber("outerSet", outerSetpoint);
    // frc::SmartDashboard::PutNumber("innerSet", innerSetpoint);

    if(currentPosition < endpoint){
      lPID.SetReference(outerSetpoint, rev::ControlType::kPosition);
      rPID.SetReference(innerSetpoint, rev::ControlType::kPosition);
    }
    prevTime = frc::Timer::GetFPGATimestamp().value();
  }
  return true;
}

bool DriveBaseModule::PIDDrive(float totalFeet, float maxAcc, float maxVelocity) {
  //forward movement only *implement backwards movement with if statement if necessary
  float currentPosition, currentVelocity, timeElapsed, distanceToDeccelerate, setpoint = 0; //currentPosition is the set point
  float prevTime = frc::Timer::GetFPGATimestamp().value();


<<<<<<< HEAD
  lEncoder.SetPosition(0);
  rEncoder.SetPosition(0);
  lEncoder.SetPositionConversionFactor(0.168); //check if this works!
  rEncoder.SetPositionConversionFactor(0.168); 
=======
  rEncoder.SetPosition(0);
  lEncoder.SetPosition(0);
  rEncoder.SetPositionConversionFactor(0.168); //check if this works!
  lEncoder.SetPositionConversionFactor(0.168); 
>>>>>>> temp-threading

  while(currentPosition < totalFeet){
    timeElapsed = frc::Timer::GetFPGATimestamp().value() - prevTime;
    distanceToDeccelerate = (3 * currentVelocity * currentVelocity) / (2 * maxAcc);
    if (distanceToDeccelerate > totalFeet - currentPosition) {
      currentVelocity -= (maxAcc * timeElapsed);
    }
    else //increase velocity
    {
      currentVelocity += (maxAcc * timeElapsed);
      if (currentVelocity > maxVelocity)
      {
        currentVelocity = maxVelocity;
      }
    }

    currentPosition += currentVelocity * timeElapsed;
    if(currentPosition > totalFeet) {
      currentPosition = totalFeet;
    }

    //converting currentPosition to ticks? for the motor: inches / (circum) * ticks * gearboxRatio, might look at this later
    setpoint = (currentPosition * 12) / (PI * 6); // for now this is ticks (maybe rotations / gearRatio if not then)
    lPID.SetReference(setpoint, rev::ControlType::kPosition);
    rPID.SetReference(setpoint, rev::ControlType::kPosition);
    prevTime = frc::Timer::GetFPGATimestamp().value();
  }
  return true;
}

void DriveBaseModule::periodicInit() {
  this->msInterval = DriveBaseModuleRunInterval;
  
  this->ErrorModulePipe = pipes[0];

  // this->BrownoutModulePipe = pipes[1];
  // this->AutonomousModulePipe = pipes[2];

  driverStick = new frc::Joystick(driverStickPort);
  operatorStick = new frc::Joystick(operatorStickPort);

  

  // if (!(initDriveMotor(lMotor, lMotorFollower, lInvert) && initDriveMotor(rMotor, rMotorFollower, rInvert))) {
  //   ErrorModulePipe->pushQueue(new Message("Could not initialize motors!", FATAL));
  //   return;
  // }

  if (!setDriveCurrLimit(motorInitMaxCurrent, motorInitRatedCurrent, motorInitLimitCycles)) {
    ErrorModulePipe->pushQueue(new Message("Failed to set motor current limit", HIGH)); // Not irrecoverable, but pretty bad
  }

  // Need to add PID Setters!!

<<<<<<< HEAD
  ErrorModulePipe->pushQueue(new Message("Ready", INFO));

  double m_P = 0.39, m_I = 0.02, m_D = 2.13, iZone = 0.03;
  //lPID = lMotor->GetPIDController();
  //rPID = rMotor->GetPIDController();
=======
  // ErrorModulePipe->pushQueue(new Message("Ready", INFO));

  double m_P = 0.39, m_I = 0.02, m_D = 2.13, iZone = 0.03;

  // lPID = (rev::SparkMaxPIDController*)malloc(sizeof(rev::SparkMaxPIDController));
  // *lPID = lMotor->GetPIDController();

  // rPID = (rev::SparkMaxPIDController*)malloc(sizeof(rev::SparkMaxPIDController));
  // *rPID = rMotor->GetPIDController();
>>>>>>> temp-threading

  lMotor->GetPIDController().SetP(m_P);
  lMotor->GetPIDController().SetI(m_I);
  lMotor->GetPIDController().SetD(m_D);
  lMotor->GetPIDController().SetIZone(iZone);

  rMotor->GetPIDController().SetP(m_P);
  rMotor->GetPIDController().SetI(m_I);
  rMotor->GetPIDController().SetD(m_D);
  rMotor->GetPIDController().SetIZone(iZone);

  //redid Encoders here, as well as h file
<<<<<<< HEAD
  lEncoder = lMotor->GetEncoder(); 
  rEncoder = rMotor->GetEncoder();
=======
  // lEncoder = (rev::SparkMaxRelativeEncoder*)malloc(sizeof(rev::SparkMaxRelativeEncoder));
  // *lEncoder = lMotor->GetEncoder(); 
>>>>>>> temp-threading

  // rEncoder =  (rev::SparkMaxRelativeEncoder*)malloc(sizeof(rev::SparkMaxRelativeEncoder));
  // *rEncoder = rMotor->GetEncoder(); 

  rEncoder.SetPosition(0);
  lEncoder.SetPosition(0);
  rEncoder.SetPositionConversionFactor(0.168); //check if this works!
  lEncoder.SetPositionConversionFactor(0.168); 
}

void DriveBaseModule::periodicRoutine() {
  // Use mode of robot to determine control source
  // Autonomous -> AutonomousPipe
  // Monitor input from BrownoutPipe
  // Command manipulators from operatorStick state

  if (!errors.empty()) { // Handle internal ModuleBase Errors
    ErrorModulePipe->pushQueue(errors.front());
    errors.pop();
  }

  if (stateRef->IsTeleop()) {
<<<<<<< HEAD
=======
    arcadeDrive(driverStick->GetRawAxis(1), driverStick->GetRawAxis(4));
    return;
  }

  if (stateRef->IsAutonomous()) {
>>>>>>> temp-threading
    if (!this->pressed && driverStick->GetRawButtonPressed(1)) {
      PIDTurn(90, 5, 1, 1);
      this->pressed = true;
    }
  }
	// Add rest of manipulator code...
}

void DriveBaseModule::LimitRate(float& s, float& t) {
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

    prev_value_speed = s;
    prev_value_turn = t;
    prevTime = currTime;
}

std::vector<uint8_t> DriveBaseModule::getConstructorArgs() { return std::vector<uint8_t> {ErrorModuleID}; }