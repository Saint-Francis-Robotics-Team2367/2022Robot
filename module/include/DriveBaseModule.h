#include "Macros.h"

#include <vector>
#include <math.h> 

#include <rev/CANSparkMax.h>
#include <frc/Joystick.h>

#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/PIDController.h>

#include <thread>
#include <chrono>
#include<mutex>
#include <atomic>

#include "gyro.h"

#define driverStickPort 0
#define operatorStickPort 1

#define PIDProportional 0.39
#define PIDIntegral 0
#define PIDDerivative 2.13
#define PIDIZone 0

#define motorInitMaxCurrent 100 // The initial max current setting
#define motorInitRatedCurrent 60 // The inital rated current settings
#define motorInitLimitCycles 2000 // The inital number of allowed ms at peak current
#define lInvert true // Inversion setings for sides (invert this if opposite side)
#define rInvert false 

#define xDeadband 0.025
#define yDeadband 0.025
#define centerToWheel 1.08333 //Center of the robot to outer side of the wheel?
#define PI 3.141592654
#define wheelDiameter 4 //inches

// #define maxAcc = 7.0
// #define maxVelocity = 21.0

class DriveBaseModule: public frc::PIDOutput{ //needed for gyroPIDDrive implementation
  double maxAcc =  7.0;
  double maxVelocity = 21.0;
  double currentVelocity = 0;
  
  
 
  frc::Joystick* driverStick = new frc::Joystick(driverStickPort);
  //frc::Joystick* operatorStick = new frc::Joystick(operatorStickPort);

  rev::CANSparkMax* lMotor = new rev::CANSparkMax(lMotorLeaderID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* lMotorFollower = new rev::CANSparkMax(lMotorFollowerID, rev::CANSparkMax::MotorType::kBrushless);

  rev::CANSparkMax* rMotor = new rev::CANSparkMax(rMotorLeaderID, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* rMotorFollower = new rev::CANSparkMax(rMotorFollowerID, rev::CANSparkMax::MotorType::kBrushless);
  //if you don't include getEncoder here, it doesn't build?
  rev::SparkMaxRelativeEncoder lEncoder = lMotor->GetEncoder();
  rev::SparkMaxRelativeEncoder rEncoder = rMotor->GetEncoder();

  rev::SparkMaxPIDController lPID = lMotor->GetPIDController();
  rev::SparkMaxPIDController rPID = rMotor->GetPIDController();


  bool initDriveMotor(rev::CANSparkMax* motor, rev::CANSparkMax* follower, bool invert); //loads initial values into motors such as current limit and phase direction
  bool setPowerBudget(rev::CANSparkMax* motor, float iPeak, float iRated, int limitCycles); //changes the current limits on the motors 
  bool setDriveCurrLimit(float iPeak, float iRated, int limitCycles);

  public: 
  std::thread driveThread;
  double stopAuto = false;
  DriveBaseModule() {
    driveThread = std::thread(&DriveBaseModule::run, this); //initializing thread so can detach in robot init
    rightStickPID.Enable();
    m_out = 0;
  }
  void LimitRate(double& s, double& t);
  void arcadeDrive(double vel, double dir); //takes two values from the joystick and converts them into motor output %
  bool PIDDrive(float totalFeet, bool keepVelocity);
  bool PIDTurn(float angle, float radius, bool keepVelocity);


  void autonomousSequence();
  void initPath();
  void run();
  void runInit();

  void gyroDriving();

  //old system doesn't work, need to fix for radius

  struct pathPoint {
    float x;
    float y;
  };

  std::vector<pathPoint> straightLinePoints;
  pathPoint robPos;

   struct radiusTurnPoint {
    float angle;
    float radius;
  };

   std::vector<radiusTurnPoint> radiusTurnPoints;
   
  std::vector<bool> pathOrder; 

  float d = 0;
  float theta = 0;

  //bool PIDGyroTurn(float angle, float radius, float maxAcc, float maxVelocity);
  float prevTime; //all for limit rate
  float prev_value_speed;
  float prev_value_turn;

  char state = 't';


  gyro gyroSource;

  //gyroPIDDrive stuff
  frc::PIDController rightStickPID{1, 0.0, 0.0, &gyroSource, this}; //maybe adjust periods here
  void PIDWrite(double output) {
    m_out = output;
  }

  double GetOutput() {
    return m_out;
  }

  private:
	    double m_out;
  
  
    
};

