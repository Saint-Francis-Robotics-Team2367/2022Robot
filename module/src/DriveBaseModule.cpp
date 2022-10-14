#include "DriveBaseModule.h"

bool DriveBaseModule::initDriveMotor(rev::CANSparkMax* motor, rev::CANSparkMax* follower, bool invert) {
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

void DriveBaseModule::arcadeDrive(double xSpeedi, double zRotationi) {
    if (fabs(xSpeedi) < xDeadband)
        xSpeedi = 0;

    if (fabs(zRotationi) < yDeadband)
        zRotationi = 0;

    double xSpeed = std::copysign(pow(fabs(xSpeedi), 1.8), xSpeedi);
    double zRotation = std::copysign(pow(fabs(zRotationi), 2), zRotationi);

    LimitRate(xSpeed, zRotation);

    double leftMotorOutput = xSpeed + zRotation;
    double rightMotorOutput = xSpeed - zRotation; //removed turnSense need test

    if (leftMotorOutput != 0)
        leftMotorOutput = std::copysign((1/(1-xDeadband)) * fabs(leftMotorOutput) - (xDeadband/(1/xDeadband)), leftMotorOutput);
        
    if (rightMotorOutput != 0)
        rightMotorOutput = std::copysign((1/(1-yDeadband)) * fabs(rightMotorOutput) - (yDeadband/(1/yDeadband)), rightMotorOutput);

    lMotor->Set(leftMotorOutput);
    rMotor->Set(rightMotorOutput);
}

void DriveBaseModule::gyroDriving() {
  float rightStickOutput = -1.0 * driverStick->GetRawAxis(4);
  rightStickPID.SetSetpoint(rightStickOutput);
  arcadeDrive(driverStick->GetRawAxis(1),  GetOutput());
  frc::SmartDashboard::PutNumber("output", GetOutput());
  frc::SmartDashboard::PutNumber("gyro", gyroSource.GetRate());

}

bool DriveBaseModule::PIDDrive(float totalFeet, bool keepVelocity) {
  //forward movement only *implement backwards movement with if statement if necessary
  float timeElapsed, distanceToDeccelerate, setpoint = 0.0; //currentPosition is the set point
  double currentPosition = 0; //current velocity is a class variable
  float prevTime = frc::Timer::GetFPGATimestamp().value();

  rEncoder.SetPosition(0);
  lEncoder.SetPosition(0);
  frc::SmartDashboard::PutBoolean("inPIDDrive", true);

  if(keepVelocity) {
    while(fabs(currentPosition) < fabs(totalFeet)) {
      if(stopAuto) {
        break;
      }
      frc::SmartDashboard::PutNumber("lEncoder", lEncoder.GetPosition());
      frc::SmartDashboard::PutNumber("rEncoder", rEncoder.GetPosition());
      timeElapsed = frc::Timer::GetFPGATimestamp().value() - prevTime;

      currentVelocity += maxAcc * timeElapsed;
      if (fabs(currentVelocity) > fabs(maxVelocity)) {
        currentVelocity = maxVelocity;
      }
      currentPosition += currentVelocity * timeElapsed;
      if(fabs(currentPosition) > fabs(totalFeet)) {
          currentPosition = totalFeet;
      }


      setpoint = (currentPosition * 12);  //amt of rotations needed; / (PI * wheelDiameter) (don't need when have conversion factor)
      frc::SmartDashboard::PutNumber("setpoint", setpoint);
      lPID.SetReference(std::copysign(setpoint, totalFeet) * (-1), rev::CANSparkMax::ControlType::kPosition); //setpoint uses encoder
      rPID.SetReference(std::copysign(setpoint, totalFeet) * (-1), rev::CANSparkMax::ControlType::kPosition); //everything in abs, so will go backwards
      prevTime = frc::Timer::GetFPGATimestamp().value();
      frc::SmartDashboard::PutNumber("prevTime", prevTime);
    }
  } else {
      while(fabs(currentPosition) < fabs(totalFeet)){
        if(stopAuto) {
          break;
        }
        frc::SmartDashboard::PutNumber("lEncoder", lEncoder.GetPosition());
        frc::SmartDashboard::PutNumber("rEncoder", rEncoder.GetPosition());
        timeElapsed = frc::Timer::GetFPGATimestamp().value() - prevTime;
        distanceToDeccelerate = (3 * currentVelocity * currentVelocity) / (2 * maxAcc); //change
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

        setpoint = (currentPosition * 12);  //amt of rotations needed; / (PI * wheelDiameter) (don't need when have conversion factor)
        frc::SmartDashboard::PutNumber("setpoint", setpoint);
        lPID.SetReference(std::copysign(setpoint, totalFeet) * (-1), rev::CANSparkMax::ControlType::kPosition); //setpoint uses encoder
        rPID.SetReference(std::copysign(setpoint, totalFeet) * (-1), rev::CANSparkMax::ControlType::kPosition);
        prevTime = frc::Timer::GetFPGATimestamp().value();
        frc::SmartDashboard::PutNumber("prevTime", prevTime);
    }
  }
  frc::SmartDashboard::PutBoolean("inPIDDrive", false);
  return true;
}

bool DriveBaseModule::PIDTurn(float angle, float radius, bool keepVelocity) { //negative stuff don't work
  rEncoder.SetPosition(0);
  lEncoder.SetPosition(0);

  frc::SmartDashboard::PutBoolean("In PIDTurn Function", true);
  float timeElapsed, distanceToDeccelerate = 0.0; //currentPosition is the set point
  double currentPosition = 0, endpoint = 0; //currentVelocity in class variables
  float prevTime = frc::Timer::GetFPGATimestamp().value();

  if(radius < 0) { //need to figure out negative radius thing!!!! prolly abs thing
    angle /= 3; //for some reason with a negative radius it goes three times the length, will need some debugging, but this fixes it (prolyl smtng with absolute values)
  } //essentially don't use negative radius's, kinda weird, but we should prolly figure it out

  endpoint = (fabs(angle) / 360.0) * (fabs(radius) + centerToWheel) * (2 * PI); //fabs of angle, same for radius so can turn negative, have an if statement to change dir later
  frc::SmartDashboard::PutNumber("endpoint", endpoint);

  if(keepVelocity) {
    while(fabs(currentPosition) < fabs(endpoint)){
       if(stopAuto) {
        break;
      }
      frc::SmartDashboard::PutNumber("lEncoder", lEncoder.GetPosition());
      frc::SmartDashboard::PutNumber("rEncoder", rEncoder.GetPosition());
      timeElapsed = frc::Timer::GetFPGATimestamp().value() - prevTime;
      currentVelocity += (maxAcc * timeElapsed);
      if (fabs(currentVelocity) > fabs(maxVelocity))
      {
        currentVelocity = maxVelocity;
      }

      currentPosition += currentVelocity * timeElapsed;
      if(fabs(currentPosition) > fabs(endpoint)) {
        currentPosition = endpoint;
      }

      double outerSetpoint = (currentPosition * 12); // for now this is ticks (maybe rotations / gearRatio if not then) //change wheel diameter, might not need
      double innerSetpoint = ((fabs(radius) - centerToWheel)/(fabs(radius) + centerToWheel)) * outerSetpoint; //fabs radius for other things
      
      frc::SmartDashboard::PutNumber("outerSet", outerSetpoint);
      frc::SmartDashboard::PutNumber("innerSet", innerSetpoint);

      int multiplier = 1;
      if(radius < 0) {
        multiplier = -1;
      }

       if(angle > 0) { //angle if reversed outer and inner setpoint reverses, if radius changes change sign of thing,
          lPID.SetReference((-1)  * multiplier * outerSetpoint, rev::CANSparkMax::ControlType::kPosition);
          rPID.SetReference((-1)  * multiplier * innerSetpoint, rev::CANSparkMax::ControlType::kPosition);
        } else {
          lPID.SetReference((-1)  * multiplier * innerSetpoint, rev::CANSparkMax::ControlType::kPosition);
          rPID.SetReference((-1)  * multiplier * outerSetpoint, rev::CANSparkMax::ControlType::kPosition);
        } //figure this out


      prevTime = frc::Timer::GetFPGATimestamp().value();
      frc::SmartDashboard::PutNumber("prevTime", prevTime);
    }
  } else {
     while(fabs(currentPosition) < fabs(endpoint)){
        if(stopAuto) {
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
    
      double outerSetpoint = (currentPosition * 12); // for now this is ticks (maybe rotations / gearRatio if not then) //change wheel diameter, might not need
      double innerSetpoint = ((radius - centerToWheel)/(radius + centerToWheel)) * outerSetpoint;
      
      frc::SmartDashboard::PutNumber("outerSet", outerSetpoint);
      frc::SmartDashboard::PutNumber("innerSet", innerSetpoint);

      int multiplier = 1;
      if(radius < 0) {
        multiplier = -1;
      } //if radius is negative it goes three times the amount it's supposed to go, so divide by 3, works for pos and neg angle though

       if(angle > 0) { //angle if reversed outer and inner setpoint reverses, if radius changes change sign of thing,
          lPID.SetReference((-1)  * multiplier * outerSetpoint, rev::CANSparkMax::ControlType::kPosition);
          rPID.SetReference((-1)  * multiplier * innerSetpoint, rev::CANSparkMax::ControlType::kPosition);
        } else {
          lPID.SetReference((-1)  * multiplier * innerSetpoint, rev::CANSparkMax::ControlType::kPosition);
          rPID.SetReference((-1)  * multiplier * outerSetpoint, rev::CANSparkMax::ControlType::kPosition);
        } //figure this out


      prevTime = frc::Timer::GetFPGATimestamp().value();
      frc::SmartDashboard::PutNumber("prevTime", prevTime);
    }
  }

 
  frc::SmartDashboard::PutBoolean("In PIDTurn Function", false);
  return true;
}

void DriveBaseModule::initPath() {
  //normally a text file, for testing purposes doing this
  //we also used trig previously and just x, y, now need to incorporate radius
  robPos.x = 0;
  robPos.y = 0;

  pathPoint point1; //example
  point1.x = 10;
  point1.y = 10; 
  straightLinePoints.push_back(point1);

  radiusTurnPoint rpoint1;
  rpoint1.radius = 3; //neg
  rpoint1.angle = 30;
  radiusTurnPoints.push_back(rpoint1);
  //calculate x and y robot manually

  pathOrder.push_back(true);
  pathOrder.push_back(false);
}


void DriveBaseModule::autonomousSequence() {
  initPath();
  int index = 0;
  int lineIndex = 0;
  int curveIndex = 0;
  while(index < pathOrder.size()) {
    if(stopAuto) {
      break;
    }
    if(pathOrder.at(index)) {
      //straight line, doing turn
      pathPoint delta;
      delta.x = (straightLinePoints.at(lineIndex).x - robPos.x);
      delta.y = (straightLinePoints.at(lineIndex).y - robPos.y);

      d = sqrt(pow(delta.x, 2) + pow(delta.y, 2));  

      // pathPoint unitDir;
      // // unitDir.x = delta.x / d;
      // // unitDir.y = delta.y / d;

      // // delta.x = delta.x + unitDir.x * coordOffset; 
      // // delta.y = delta.y + unitDir.y * coordOffset;

     theta = atan2(delta.x, delta.y) * (180/(3.14159265)); 

     robPos.x += delta.x;
     robPos.y += delta.y;
     PIDTurn(theta, 0, false); //expiriment with true
     PIDDrive(d, false);
     lineIndex++;

    } else {
      robPos.x += 2; //do math later !!!!
      robPos.y += 2;  //do math later !!!!
      PIDTurn(radiusTurnPoints.at(curveIndex).angle, radiusTurnPoints.at(curveIndex).radius, false);
      curveIndex++;
    }
    index++;
    frc::SmartDashboard::PutNumber("index", index);
  }
}


void DriveBaseModule::runInit() {
  if (!(initDriveMotor(lMotor, lMotorFollower, lInvert) && initDriveMotor(rMotor, rMotorFollower, rInvert))) {
    frc::SmartDashboard::PutBoolean("Drive Motor Inits", false);
  }

  if (!setDriveCurrLimit(motorInitMaxCurrent, motorInitRatedCurrent, motorInitLimitCycles)) {
    frc::SmartDashboard::PutBoolean("Drive Curr Limits", false);
  }
  //auto drive PID controllers
  lPID.SetP(PIDProportional);
  lPID.SetI(PIDIntegral);
  lPID.SetD(PIDDerivative);
  lPID.SetIZone(PIDIZone);

  rPID.SetP(PIDProportional);
  rPID.SetI(PIDIntegral);
  rPID.SetD(PIDDerivative);
  rPID.SetIZone(PIDIZone);

  rEncoder.SetPosition(0);
  lEncoder.SetPosition(0);
  rEncoder.SetPositionConversionFactor(1.96); //check if this works! [look at other code, converts rotations to feet, might not need]
  lEncoder.SetPositionConversionFactor(1.96); //gear ratio?
}

void DriveBaseModule::run() {
  runInit();
  bool test = true;
  int counter = 0;
  while(true) { 
    auto nextRun = std::chrono::steady_clock::now() + std::chrono::milliseconds(20);
    frc::SmartDashboard::PutNumber("timesRun", ++counter);

    //need mutex to stop

    if(state == 'a') { //ik I have access to isAutonomous
      stopAuto = false;
      if(test) {
          autonomousSequence();
        test = false;
      }
      
    } else {
      //perioidic routines
      gyroDriving();
      test = true;
      stopAuto = true;
    }

    //add tuning ones
    std::this_thread::sleep_until(nextRun);
  }
}





