#include "DriveBaseModule.h"

bool DriveBaseModule::initDriveMotor(rev::CANSparkMax* motor, rev::CANSparkMax* follower, bool invert) {
  motor->RestoreFactoryDefaults();
  follower->RestoreFactoryDefaults();
  motor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
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

    leftMotorOutput = xSpeed + zRotation;
    rightMotorOutput = xSpeed - zRotation;

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
     if(stateRef->IsDisabled()) {
      break;
    }
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
  rEncoder.SetPositionConversionFactor(0.168); //check if this works!
  lEncoder.SetPositionConversionFactor(0.168); 
frc::SmartDashboard::PutBoolean("inPIDDrive", true);
  while(fabs(currentPosition) < fabs(totalFeet)){
    if(stateRef->IsDisabled()) {
      break;
    }
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

    setpoint = (currentPosition * 12) / (PI * 6); 
    lPID.SetReference(setpoint, rev::CANSparkMax::ControlType::kPosition);
    rPID.SetReference(setpoint, rev::CANSparkMax::ControlType::kPosition);
    prevTime = frc::Timer::GetFPGATimestamp().value();
    frc::SmartDashboard::PutNumber("prevTime", prevTime);
  }
  frc::SmartDashboard::PutBoolean("inPIDDrive", false);
  return true;
}

void DriveBaseModule::periodicInit() {
  this->msInterval = DriveBaseModuleRunInterval;
  
  this->ErrorModulePipe = pipes[0];

  // this->BrownoutModulePipe = pipes[1];
  // this->AutonomousModulePipe = pipes[2];

  driverStick = new frc::Joystick(driverStickPort);
  operatorStick = new frc::Joystick(operatorStickPort);

  

  if (!(initDriveMotor(lMotor, lMotorFollower, lInvert) && initDriveMotor(rMotor, rMotorFollower, rInvert))) {
    //ErrorModulePipe->pushQueue(new Message("Could not initialize motors!", FATAL));
    return;
  }

  if (!setDriveCurrLimit(motorInitMaxCurrent, motorInitRatedCurrent, motorInitLimitCycles)) {
    ///ErrorModulePipe->pushQueue(new Message("Failed to set motor current limit", HIGH)); // Not irrecoverable, but pretty bad
  }

  // Need to add PID Setters!!

  // ErrorModulePipe->pushQueue(new Message("Ready", INFO));

  double m_P = 0.4, m_I = 0.00, m_D = 1.68, iZone = 0.00;

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
    arcadeDrive(driverStick->GetRawAxis(1), driverStick->GetRawAxis(4));
    frc::SmartDashboard::PutNumber("gyro", m_imu.GetAngle().value());
    return;
  }

	// Add rest of manipulator code...
  if(stateRef->IsAutonomousEnabled()) {
    frc::SmartDashboard::PutBoolean("InAutoEnabled1", true);
  for (int i = 0; i < pipes.size(); i++) {
    frc::SmartDashboard::PutBoolean("In Loop", true);
    GenericPipe* p = pipes[i];
    Message* m = p->popQueue();
    if (m) {
    frc::SmartDashboard::PutBoolean("Message", true);
    if (m->str == "PD") {
      frc::SmartDashboard::PutBoolean("PIDDrive Comm Succesful!", false);
      if(PIDDrive(m->vals[0], m->vals[1], m->vals[2])) {
        frc::SmartDashboard::PutBoolean("PIDDrive Comm Succesful!", true);
      }
    }
    if (m->str == "PT") {
     frc::SmartDashboard::PutBoolean("PIDTurn Comm Succesful!", false);
      if(PIDTurn(m->vals[0], m->vals[1], m->vals[2], m->vals[3])) {
        //if no here, it does this and tries to do smtng else
      frc::SmartDashboard::PutBoolean("PIDTurn Comm Succesful!", true);
    }
    }
    if (m->str == "Arcade") {
      arcadeDrive(m->vals[0], m->vals[1]);
    }
    }
  }

  }
  
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

float DriveBaseModule::getGyroAngle(){
  return(m_imu.GetAngle().value() - gyroInitVal);
}

void DriveBaseModule::InitGyro() {
  gyroInitVal = getGyroAngle();
}
void DriveBaseModule::GyroTurn(float theta) {
  //add PID
  while (fabs(getGyroAngle() - theta) < 0.1) {
    arcadeDrive(0, 0.1);
  }
}
std::vector<uint8_t> DriveBaseModule::getConstructorArgs() { return std::vector<uint8_t> {ErrorModuleID,  AutonomousModuleID}; }