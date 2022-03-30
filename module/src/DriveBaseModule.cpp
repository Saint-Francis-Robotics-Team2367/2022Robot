#include "DriveBaseModule.h"

bool DriveBaseModule::initDriveMotor(rev::CANSparkMax* motor, rev::CANSparkMax* follower, bool invert) {
  //  motor->RestoreFactoryDefaults();
  // follower->RestoreFactoryDefaults();
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
  return fabs(rotation) * (1 + (sliderValue - 1) * fabs(speed));
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

    leftMotorOutput = xSpeed + std::copysign(DriveBaseModule::TurningSensitivity(xSpeed, zRotation), zRotation);
    rightMotorOutput = xSpeed - std::copysign(DriveBaseModule::TurningSensitivity(xSpeed, zRotation), zRotation);

    if (leftMotorOutput != 0)
        leftMotorOutput = std::copysign((1/(1-deadband)) * fabs(leftMotorOutput) - (deadband/(1/deadband)), leftMotorOutput);
        
    if (rightMotorOutput != 0)
        rightMotorOutput = std::copysign((1/(1-deadband)) * fabs(rightMotorOutput) - (deadband/(1/deadband)), rightMotorOutput);

    // leftMotorOutput = std::copysign(pow(leftMotorOutput, 2), leftMotorOutput);
    // rightMotorOutput = std::copysign(pow(rightMotorOutput, 2), rightMotorOutput);

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
  float timeElapsed;
  float distanceToDeccelerate, endpoint = 0.0;
  double currentVelocity = 0;
  double currentPosition = 0;
  if (pidprevVelocity == 0) {
    rEncoder.SetPosition(0);
    lEncoder.SetPosition(0);
    InitGyro();
  }
  else {
    timeElapsed = frc::Timer::GetFPGATimestamp().value() - pidprevTime;
    distanceToDeccelerate, endpoint = 0.0; //currentPosition is the set point
    currentVelocity = pidprevVelocity;
    currentPosition = pidprevPosition;
    //currentPosition = pidprevPosition + currentVelocity * timeElapsed;
  }
  //pidprevPosition = currentPosition;
  
  rEncoder.SetPositionConversionFactor(0.64); //check if this works!
  lEncoder.SetPositionConversionFactor(0.64); 

  if (angle < 0) {
    maxAcc *= -1;
    maxVelocity *= -1;
  }

  //pidprevPosition = currentPosition;

  endpoint = (angle / 360.0) * (radius + centerToWheel) * (2 * PI);
  
  frc::SmartDashboard::PutNumber("endpoint", endpoint);

  if(fabs(currentPosition) < fabs(endpoint)){

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
    

    if(fabs(currentPosition) < fabs(endpoint)){
      lPID.SetReference(outerSetpoint, rev::CANSparkMax::ControlType::kPosition);
      rPID.SetReference(innerSetpoint, rev::CANSparkMax::ControlType::kPosition);
    }


    pidprevTime = frc::Timer::GetFPGATimestamp().value();
    pidprevVelocity = currentVelocity;
    pidprevPosition = currentPosition;
    return false;
  }
  else if (!GyroTurnTick(angle)) {
    return false;
  }
  pidprevVelocity = 0;
  pidprevPosition = 0;
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
void DriveBaseModule::periodicInit() {
  frc::SmartDashboard::PutNumber("Sensitivity", 1);
  
  if (!(initDriveMotor(lMotor, lMotorFollower, lInvert) && initDriveMotor(rMotor, rMotorFollower, rInvert))) {
    //ErrorModulePipe->pushQueue(new Message("Could not initialize motors!", FATAL));
    //return;
  }

  if (!setDriveCurrLimit(motorInitMaxCurrent, motorInitRatedCurrent, motorInitLimitCycles)) {
    ///ErrorModulePipe->pushQueue(new Message("Failed to set motor current limit", HIGH)); // Not irrecoverable, but pretty bad
  }

  // Need to add PID Setters!!

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

void DriveBaseModule::periodicRoutine() {
  // frc::SmartDashboard::PutNumber("adjustSpeed", adjustSpeed);
  // adjustSpeed = frc::SmartDashboard::GetNumber("adjustSpeed", 0.01);
  
  // Use mode of robot to determine control source
  // Autonomous -> AutonomousPipe
  // Monitor input from BrownoutPipe
  // Command manipulators from operatorStick state

  sliderValue = frc::SmartDashboard::GetNumber("Sensitivity", 1);
  frc::SmartDashboard::PutNumber("Right Encoder", rEncoder.GetPosition());
  frc::SmartDashboard::PutNumber("Left Encoder", lEncoder.GetPosition());

  arcadeDrive(driverStick->GetRawAxis(1),  -1.0 * driverStick->GetRawAxis(4));

  if(driverStick->GetRawButton(3)) {
    alignToGoal();
  }

  // if driver wants to adjust aim, hold button down to move position of robot
  // if(operatorStick->GetRawButton(3)) {
  //   arcadeDrive(0, adjustSpeed);
  // }
  // if(operatorStick->GetRawButton(2)) {
  //   arcadeDrive(0, -1*adjustSpeed);
  // }
  frc::SmartDashboard::PutNumber("gyro", getGyroAngle());
  // if (driverStick->GetRawButton(5))
  // {
  //   std::vector<float> v;
  //   if (!intakeOn)
  //   {
  //     pipes[2]->pushQueue(new Message("activate", v));
  //     intakeOn = true;
  //   }
  //   else
  //   {
  //     pipes[2]->pushQueue(new Message("disable", v));
  //     intakeOn = false;
  //   }
  // }
  // if (driverStick->GetRawButton(6))
  // {
  //   std::vector<float> v;
  //   if (!intakeOn)
  //   {
  //     pipes[2]->pushQueue(new Message("index", 1)); // put 0 and 1 for temp right now
  //     index = true;
  //   }
  //   else
  //   {
  //     pipes[2]->pushQueue(new Message("index", 0));
  //     index = false;
  //   }
  // }
  // if (driverStick->GetRawButton(1))
  // {
  //   if (!tested)
  //   {
  //     pipes[3]->pushQueue(new Message("test", 1));
  //     tested = true;
  //   }
  // }
  // else
  // {
  //   pipes[3]->pushQueue(new Message("test", 0));
  //   tested = false;
  // }
  // Add rest of manipulator code...

  // if (stateRef->IsAutonomousEnabled()) {
  //   GenericPipe *p = pipes[1];

  //   Message *m = p->popQueue();
  //   if (m)
  //   {
  //     if (m->str == "PD")
  //     {
  //       frc::SmartDashboard::PutBoolean("PIDDrive Comm Succesful!", false);
  //       if (PIDDrive(m->vals[0], m->vals[1], m->vals[2]))
  //       {
  //         frc::SmartDashboard::PutBoolean("PIDDrive Comm Succesful!", true);
  //       }
  //     }

  //     if (m->str == "PT")
  //     {
  //       frc::SmartDashboard::PutBoolean("PIDTurn Comm Succesful!", false);
  //       frc::SmartDashboard::PutNumber("InDriveBaseTheta", m->vals[0]);
  //       if (PIDGyroTurn(m->vals[0], m->vals[1], m->vals[2], m->vals[3]))
  //       {
  //         // if no here, it does this and tries to do smtng else
  //         frc::SmartDashboard::PutBoolean("PIDTurn Comm Succesful!", true);
  //       }
  //     }
  //     if (m->str == "Arcade")
  //     {
  //       arcadeDrive(m->vals[0], m->vals[1]);
  //     }
  //     pipes[1]->pushQueue(new Message("done", 0));
  //   }
  // }
}

void DriveBaseModule::LimitRate(float& s, float& t) {
    double k = 6; //1/k = rate to speed up [so 0.2 seconds]
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
  if (cam.HasTargets()) {
    frc::SmartDashboard::PutBoolean("camera", true);
    photonlib::PhotonPipelineResult result = cam.GetLatestResult();
    float v = result.GetBestTarget().GetYaw();
    frc::SmartDashboard::PutNumber("yaw", v);
    std::cout << v << std::endl;
    // float y = result.GetBestTarget().GetCameraRelativePose().X().value();
    // frc::SmartDashboard::PutNumber("distance", y);

    if (fabs(v) > 3)
      arcadeDrive(0, v * 0.2);
    // else {
    //   arcadeDrive(-y, 0);
    // }
  }
  else {
    frc::SmartDashboard::PutBoolean("camera", false);
  }
}

float DriveBaseModule::getDistanceTraversed(){
  lEncoder.SetPositionConversionFactor(1);
  //return (lEncoder.GetPosition() / 12 * (PI * 4));
  return ((lEncoder.GetPosition() / 4.15) * (PI * 4) / 12); // 4.15 is ticks per rotation
}