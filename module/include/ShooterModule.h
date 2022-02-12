#ifndef SHOOTERMODULE_H
#define SHOOTERMODULE_H

#include <math.h>

#include <string>
#include <iostream>

#include <frc/DriverStation.h>

#include "ModuleBase.h"
#include "GenericPipe.h"
#include "Macros.h"
#include "rev/CANSparkMax.h"

#define indexSetpoint 4
#define feedRate 2

#define motor0ID 0
#define motor1ID 1
#define motor2ID 2

class ShooterModule : public ModuleBase {
  GenericPipe* ErrorModulePipe;
  GenericPipe* IntakeModulePipe;

//   GenericPipe* JoystickModulePipe;

  int numBalls = 2;
  double m_P = 0.39, m_I = 0.02, m_D = 2.13, iZone = 0.03;
  double pos0 = 0, pos1 = 0, pos2 = 0;

  bool switchState = false;
  bool feedButtonState = false;
  bool intakeButtonState = false;

  bool intaking = false;
  bool feeding = false;

  rev::CANSparkMax* motor0 = new rev::CANSparkMax(motor0ID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* motor1 = new rev::CANSparkMax(motor1ID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* motor2 = new rev::CANSparkMax(motor2ID, rev::CANSparkMax::MotorType::kBrushless);

  rev::SparkMaxRelativeEncoder encoder0 = motor0->GetEncoder();
  rev::SparkMaxRelativeEncoder encoder1 = motor1->GetEncoder();
  rev::SparkMaxRelativeEncoder encoder2 = motor2->GetEncoder();

  rev::SparkMaxPIDController PID0 = motor0->GetPIDController();
  rev::SparkMaxPIDController PID1 = motor1->GetPIDController();
  rev::SparkMaxPIDController PID2 = motor2->GetPIDController();

  bool zeroEncoders();
  bool setPID();

  public:  
  std::vector<uint8_t> getConstructorArgs();
  void periodicInit();
  void periodicRoutine();

  bool feedBall();
  bool intakeBall();
};

#endif