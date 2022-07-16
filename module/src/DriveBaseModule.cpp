#include "DriveBaseModule.h"
#include <cmath>

bool DriveBaseModule::initDriveMotor(rev::CANSparkMax* motor, rev::CANSparkMax* follower, bool invert) {
  motor->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
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

float DriveBaseModule::TurningSensitivity(float speed, float rotation) {
  //return fabs(rotation) * (1 + (sliderValue - 1) * fabs(speed)); 
  //og equation, I'm changing it
  rotation = 0.514919 * cos(3.25292 * speed) + 0.506336;
  return rotation;
//returns amount of rotation
}

void DriveBaseModule::arcadeDrive(double xSpeedi, double zRotationi) {
    if (fabs(xSpeedi) < deadband)
        xSpeedi = 0;

    if (fabs(zRotationi) < deadband)
        zRotationi = 0;

    double xSpeed = std::copysign(pow(fabs(xSpeedi), 1.8), xSpeedi);
    double zRotation = std::copysign(pow(fabs(zRotationi), 2), zRotationi);

    LimitRate(xSpeed, zRotation);

    double leftMotorOutput = xSpeed + zRotation;
    double rightMotorOutput = xSpeed -  zRotation;

    if (leftMotorOutput != 0)
        leftMotorOutput = std::copysign((1/(1-deadband)) * fabs(leftMotorOutput) - (deadband/(1/deadband)), leftMotorOutput);
        
    if (rightMotorOutput != 0)
        rightMotorOutput = std::copysign((1/(1-deadband)) * fabs(rightMotorOutput) - (deadband/(1/deadband)), rightMotorOutput);

    lMotor->Set(leftMotorOutput);
    rMotor->Set(rightMotorOutput);
}


void DriveBaseModule::adjustedArcadeDrive(double xSpeed, double zRotation) {
    if (fabs(xSpeed) < deadband)
        xSpeed = 0;

    if (fabs(zRotation) < deadband)
        zRotation = 0;

    //scaling functiins

  
    //is this even right, any other adjustments I should add: have slew rate, like inertia scalar, other kinds of scalar, skim
    float angle = m_imu.GetAngle().value(); 
    float angleRate = m_imu.GetRate().value(); //rate of change of angle or just angle
    //not adjusted arcade drive?
    float rotationError = zRotation - angleRate;
    // double leftMotorOutput = xSpeed + zRotation;
    // double rightMotorOutput = xSpeed - zRotation;
    double leftMotorOutput = xSpeed + rotationError;
    double rightMotorOutput = xSpeed - rotationError;


    //right stick PID input function
    //input to PID function would be error between right stick and gyro

    
    //velocity, curvature, cheesy drive


    lPID.SetReference(leftMotorOutput, rev::CANSparkMax::ControlType::kPosition);
    rPID.SetReference(rightMotorOutput, rev::CANSparkMax::ControlType::kPosition);
}

bool DriveBaseModule::PIDTurn(float angle, float radius, float maxAcc, float maxVelocity) {
  rEncoder.SetPosition(0);
  lEncoder.SetPosition(0);
  rEncoder.SetPositionConversionFactor(0.168); //check if this works!
  lEncoder.SetPositionConversionFactor(0.168); 

  if (angle < 0) {
    maxAcc *= -1;
    maxVelocity *= -1;
  }

  frc::SmartDashboard::PutBoolean("In PIDTurn Function", true);
  float timeElapsed, distanceToDeccelerate = 0.0; //currentPosition is the set point
  double currentPosition = 0, currentVelocity = 0, endpoint = 0;
  float prevTime = frc::Timer::GetFPGATimestamp().value();
  endpoint = (angle / 360.0) * (radius + centerToWheel) * (2 * PI);
  if(fabs(endpoint) > 360) {
    //don't want this to happen
    return false;
  }
  frc::SmartDashboard::PutNumber("endpoint", endpoint);

  while(fabs(currentPosition) < fabs(endpoint)){
    frc::SmartDashboard::PutNumber("lEncoder", lEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("rEncoder", rEncoder.GetPosition());
    timeElapsed = frc::Timer::GetFPGATimestamp().value() - prevTime;
    //should be 2, * Vc^2, check this later
    distanceToDeccelerate = (3 * currentVelocity * currentVelocity) / (2 * maxAcc);
    if (fabs(distanceToDeccelerate) > fabs(endpoint - currentPosition)) {
      currentVelocity -= (maxAcc * timeElapsed);
    }
    else //increase velocity
    {
      currentVelocity += (maxAcc * timeElapsed);
      if (fabs(currentVelocity) > fabs(maxVelocity))
      {
        currentVelocity = maxVelocity;
      }
    }
    currentPosition += currentVelocity * timeElapsed;
    
    if(fabs(currentPosition) > fabs(endpoint)) {
      currentPosition = endpoint;
    }
    //same as other
   
    double outerSetpoint = (currentPosition * 12) / (PI * 6); // for now this is ticks (maybe rotations / gearRatio if not then)
    double innerSetpoint = ((radius - centerToWheel)/(radius + centerToWheel)) * outerSetpoint;
    
    frc::SmartDashboard::PutNumber("outerSet", outerSetpoint);
    frc::SmartDashboard::PutNumber("innerSet", innerSetpoint);

    if(fabs(currentPosition) < fabs(endpoint)){
      lPID.SetReference(outerSetpoint, rev::CANSparkMax::ControlType::kPosition);
      rPID.SetReference(innerSetpoint, rev::CANSparkMax::ControlType::kPosition);
    }
    prevTime = frc::Timer::GetFPGATimestamp().value();
    frc::SmartDashboard::PutNumber("prevTime", prevTime);
  }
  frc::SmartDashboard::PutBoolean("In PIDTurn Function", false);
  return true;
}

bool DriveBaseModule::PIDGyroTurn(float angle, float radius, float maxAcc, float maxVelocity) {
  rEncoder.SetPosition(0);
  lEncoder.SetPosition(0);
  rEncoder.SetPositionConversionFactor(0.64); //check if this works!
  lEncoder.SetPositionConversionFactor(0.64); 

  if (angle < 0) {
    maxAcc *= -1;
    maxVelocity *= -1;
  }

  InitGyro();
  frc::SmartDashboard::PutBoolean("In PIDTurn Function", true);
  float timeElapsed, distanceToDeccelerate = 0.0; //currentPosition is the set point
  double currentPosition = 0, currentVelocity = 0, endpoint = 0;
  float prevTime = frc::Timer::GetFPGATimestamp().value();
  endpoint = (angle / 360.0) * (radius + centerToWheel) * (2 * PI);
  
  frc::SmartDashboard::PutNumber("endpoint", endpoint);

  while(fabs(currentPosition) < fabs(endpoint)){
    frc::SmartDashboard::PutNumber("lEncoder", lEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("rEncoder", rEncoder.GetPosition());
    timeElapsed = frc::Timer::GetFPGATimestamp().value() - prevTime;
    //should be 2, * Vc^2, check this later
    distanceToDeccelerate = (3 * currentVelocity * currentVelocity) / (2 * maxAcc);
    if (fabs(distanceToDeccelerate) > fabs(endpoint - currentPosition)) {
      currentVelocity -= (maxAcc * timeElapsed);
    }
    else //increase velocity
    {
      currentVelocity += (maxAcc * timeElapsed);
      if (fabs(currentVelocity) > fabs(maxVelocity))
      {
        currentVelocity = maxVelocity;
      }
    }
    currentPosition += currentVelocity * timeElapsed;
    
    if(fabs(currentPosition) > fabs(endpoint)) {
      currentPosition = endpoint;
    }
    //same as other
   
    double outerSetpoint = (currentPosition * 12) / (PI * 4); // for now this is ticks (maybe rotations / gearRatio if not then)
    double innerSetpoint = ((radius - centerToWheel)/(radius + centerToWheel)) * outerSetpoint;
    
    frc::SmartDashboard::PutNumber("outerSet", outerSetpoint);
    frc::SmartDashboard::PutNumber("innerSet", innerSetpoint);

    if(fabs(currentPosition) < fabs(endpoint)){
      lPID.SetReference(outerSetpoint, rev::CANSparkMax::ControlType::kPosition);
      rPID.SetReference(innerSetpoint, rev::CANSparkMax::ControlType::kPosition);
    }


    prevTime = frc::Timer::GetFPGATimestamp().value();
    frc::SmartDashboard::PutNumber("prevTime", prevTime);
  }

  GyroTurn(angle);
  frc::SmartDashboard::PutBoolean("In PIDTurn Function", false);
  return true;
}

bool DriveBaseModule::PIDGyroTurnTick(float angle, float radius, float maxAcc, float maxVelocity) {
  return true;
}


bool DriveBaseModule::PIDDrive(float totalFeet, float maxAcc, float maxVelocity) {
  //forward movement only *implement backwards movement with if statement if necessary
  float timeElapsed, distanceToDeccelerate, setpoint = 0.0; //currentPosition is the set point
  double currentVelocity = 0, currentPosition = 0;
  float prevTime = frc::Timer::GetFPGATimestamp().value();

  if (totalFeet < 0) {
    maxAcc *= -1;
    maxVelocity *= -1;
  }

  rEncoder.SetPosition(0);
  lEncoder.SetPosition(0);
  rEncoder.SetPositionConversionFactor(0.64); //check if this works!
  lEncoder.SetPositionConversionFactor(0.64); 
frc::SmartDashboard::PutBoolean("inPIDDrive", true);
  while(fabs(currentPosition) < fabs(totalFeet)){
    frc::SmartDashboard::PutNumber("lEncoder", lEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("rEncoder", rEncoder.GetPosition());
    timeElapsed = frc::Timer::GetFPGATimestamp().value() - prevTime;
    distanceToDeccelerate = (3 * currentVelocity * currentVelocity) / (2 * maxAcc);
    if (fabs(distanceToDeccelerate) > fabs(totalFeet - currentPosition)) {
      currentVelocity -= (maxAcc * timeElapsed);
    }
    else //increase velocity
    {
      currentVelocity += (maxAcc * timeElapsed);
      if (fabs(currentVelocity) > fabs(maxVelocity))
      {
        currentVelocity = maxVelocity;
      }
    }

    currentPosition += currentVelocity * timeElapsed;
    if(fabs(currentPosition) > fabs(totalFeet)) {
      currentPosition = totalFeet;
    }

    setpoint = (currentPosition * 12) / (PI * 4); 
    lPID.SetReference(setpoint, rev::CANSparkMax::ControlType::kPosition);
    rPID.SetReference(setpoint, rev::CANSparkMax::ControlType::kPosition);
    prevTime = frc::Timer::GetFPGATimestamp().value();
    frc::SmartDashboard::PutNumber("prevTime", prevTime);
  }
  frc::SmartDashboard::PutBoolean("inPIDDrive", false);
  return true;
}

void DriveBaseModule::periodicInit() {
  frc::SmartDashboard::PutNumber("Sensitivity", 1);
  
  if (!(initDriveMotor(lMotor, lMotorFollower, lInvert) && initDriveMotor(rMotor, rMotorFollower, rInvert))) {
    //ErrorModulePipe->pushQueue(new Message("Could not initialize motors!", FATAL));
  }

  if (!setDriveCurrLimit(motorInitMaxCurrent, motorInitRatedCurrent, motorInitLimitCycles)) {
    ///ErrorModulePipe->pushQueue(new Message("Failed to set motor current limit", HIGH)); // Not irrecoverable, but pretty bad
  }
  // ErrorModulePipe->pushQueue(new Message("Ready", INFO));

  double m_P = 0.5, m_I = 0.00, m_D = 0.1, iZone = 0.00;

  lPID.SetP(m_P);
  lPID.SetI(m_I);
  lPID.SetD(m_D);
  lPID.SetIZone(iZone);

  rPID.SetP(m_P);
  rPID.SetI(m_I);
  rPID.SetD(m_D);
  rPID.SetIZone(iZone);

  rEncoder.SetPosition(0);
  lEncoder.SetPosition(0);
  rEncoder.SetPositionConversionFactor(0.64); //check if this works!
  lEncoder.SetPositionConversionFactor(0.64); 
}

bool DriveBaseModule::PIDDriveSimpleTick(float totalFeet) {
  if (!encoderZeroed) {
     lPID.SetOutputRange(-0.25, 0.25);
      rPID.SetOutputRange(-0.25, 0.25);
    lEncoder.SetPosition(0);
    rEncoder.SetPosition(0);
    lEncoder.SetPositionConversionFactor(1);
    rEncoder.SetPositionConversionFactor(1);
    //lEncoder.SetInverted(true);
    //rEncoder.SetInverted(true);
    encoderZeroed = true;
  }

  double setpoint = ((totalFeet * 12) / (PI * 4)) * 4.15;
  lPID.SetReference(setpoint, rev::CANSparkMax::ControlType::kPosition);
  rPID.SetReference(setpoint, rev::CANSparkMax::ControlType::kPosition);

  if (getDistanceTraversed() >= (0.95 *totalFeet)) {
    encoderZeroed = false;
  }

  frc::SmartDashboard::PutNumber("encoder", rEncoder.GetPosition());

  return (getDistanceTraversed() >= (0.95 * totalFeet));  //ADDED THIS
}


bool DriveBaseModule::PIDDriveTick(float totalFeet, float maxAcc, float maxVelocity) {
  float timeElapsed;
  float distanceToDeccelerate, setpoint = 0.0;
  double currentVelocity = 0;
  double currentPosition = 0;
  if (pidprevVelocity == 0) {
    rEncoder.SetPosition(0);
    lEncoder.SetPosition(0);
  } else {
    timeElapsed = frc::Timer::GetFPGATimestamp().value() - pidprevTime;
    distanceToDeccelerate, setpoint = 0.0; //currentPosition is the set point
    currentVelocity = pidprevVelocity;
    currentPosition = pidprevPosition; //commented this out, adds it twice
    //currentPosition = pidprevPosition + currentVelocity * timeElapsed;
  }
  //pidprevPosition = currentPosition;
  timeElapsed = frc::Timer::GetFPGATimestamp().value() - pidprevTime; //moved out


  if (totalFeet < 0) {
    maxAcc *= -1;
    maxVelocity *= -1;
  }

  rEncoder.SetPositionConversionFactor(0.64); //check if this works!
  lEncoder.SetPositionConversionFactor(0.64); 

  if(fabs(currentPosition) < fabs(totalFeet)){
    distanceToDeccelerate = (3 * currentVelocity * currentVelocity) / (2 * maxAcc);
    if (fabs(distanceToDeccelerate) > fabs(totalFeet - currentPosition)) {
      currentVelocity -= (maxAcc * timeElapsed);
    }
    else //increase velocity
    {
      currentVelocity += (maxAcc * timeElapsed);
      if (fabs(currentVelocity) > fabs(maxVelocity))
      {
        currentVelocity = maxVelocity;
      }
    }

    currentPosition += currentVelocity * timeElapsed;
    if(fabs(currentPosition) > fabs(totalFeet)) {
      currentPosition = totalFeet;
    }

    setpoint = (currentPosition * 12) / (PI * 4); 
    lPID.SetReference(setpoint, rev::CANSparkMax::ControlType::kPosition);
    rPID.SetReference(setpoint, rev::CANSparkMax::ControlType::kPosition);
    pidprevTime = frc::Timer::GetFPGATimestamp().value();
    pidprevVelocity = currentVelocity;
    pidprevPosition = currentPosition;
    return false;

  }
  pidprevVelocity = 0;
  pidprevPosition = 0;
  return true; 
}

void DriveBaseModule::periodicRoutine() {
  //sliderValue = frc::SmartDashboard::GetNumber("Sensitivity", 1);
  //left stick, right stick
  // float rightStickOutput = -1.0 * driverStick->GetRawAxis(4);
  // //rightStickOutput = 


  //arcadeDrive(driverStick->GetRawAxis(1),  rightStickOutput);
}

void DriveBaseModule::LimitRate(double& s, double& t) {
    double k = 6; //1/k = rate to speed up [so 0.2 seconds]
    double currTime = frc::Timer::GetFPGATimestamp().value();
    double deltaTime = currTime - prevTime;
    double r_s = (s - prev_value_speed) / deltaTime;
    double r_t = (t - prev_value_turn) / deltaTime;

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

float DriveBaseModule::getGyroAngle(){
  return 0.0;
}

void DriveBaseModule::InitGyro() {
  gyroInitVal = getGyroAngle() + gyroInitVal;
}

void DriveBaseModule::GyroTurn(float theta) {
  //add PID
  while (fabs(getGyroAngle() - theta) > 1) {
    frc::SmartDashboard::PutNumber("GyroTurn", getGyroAngle());
    if (getGyroAngle() < theta) {
      arcadeDrive(0, 0.2);
      //would adding a return here and below help? so it's not loopish, might be jerkish though
    }
    else {
      arcadeDrive(0, -0.2);
    
    }
  }
  arcadeDrive(0, 0); //need this to end motors
  return;
}

bool DriveBaseModule::GyroTurnTick(float theta) {
  //add PID
  float P = 0.05;
  if (fabs(getGyroAngle() - theta) > 1) {
    frc::SmartDashboard::PutNumber("GyroTurn", getGyroAngle());
    arcadeDrive(0,  (theta - getGyroAngle()) * P);
    std::cout << "ROTATIONAL VALUE " << (theta - getGyroAngle()) * P << std::endl;
    return false;
  }
  arcadeDrive(0, 0); //need this to end motors
  return true;
}

void DriveBaseModule::alignToGoal() {
 
}

float DriveBaseModule::getDistanceTraversed(){
  lEncoder.SetPositionConversionFactor(1);
  //return (lEncoder.GetPosition() / 12 * (PI * 4));
  return ((lEncoder.GetPosition() / 4.15) * (PI * 4) / 12); // 4.15 is ticks per rotation
}