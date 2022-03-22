#include "Macros.h"
#include <rev/CANSparkMax.h>
#include <frc/Joystick.h>
#include <vector>
#include <math.h>
class ShooterModule
{
public:
    void periodicInit();
    void periodicRoutine();
    void align_shooter();
private:

  frc::Joystick* driverStick = new frc::Joystick(0);
  frc::Joystick* operatorStick = new frc::Joystick(1);
    void initializePaths();

    bool setMotorPIDF(rev::CANSparkMax* motor, double P, double I, double D, double F);

  bool setShooterSetpoint(double setpoint);

  const double CAMERA_HEIGHT = 2.0;
  const double TARGET_HEIGHT = 102.0;

  // Angle between horizontal and the camera.
  const int CAMERA_PITCH = 40;

  double dist_from_apex = 3;    
  // How far from the target we want to be
  const double GOAL_RANGE_METERS = 10.0;
  const double APEX_HEIGHT = TARGET_HEIGHT + dist_from_apex;
  const double CAMERA_MOUNT_ANGLE = 40;

  double pitch_degree;
  double horizontal_dist = 0;
  
  double velocity = 0; 
  const double GRAV_CONST = 32.17;
  double theta_rads; 
  double theta_degs;
  double central_degs;
  const double pi = 3.14159;


  // PID constants should be tuned per robot
  const double P_GAIN = 0.1;
  const double D_GAIN = 0.0;

  float range = 0;

  double max_turns_neo550 = 132.0 + (2/9);

  // const int shooterMotorID = 22;
  // const int hoodMotorID = 4;
  // const int turretMotorID = 5;

  const float shooterkP = 0.000090892;
  const float shooterkI = 0.0; 
  const float shooterkD = 0.162;
  const float shooterkFF = 0.000194;
  float maxShooterOutput = 1;
  float minShooterOutput = -0.1;

  bool pressed = false;
  rev::CANSparkMax * shooterMotor = new rev::CANSparkMax(shooterIndexer, rev::CANSparkMax::MotorType::kBrushed);
  //rev::SparkMaxPIDController shooterMotorPID = shooterMotor->GetPIDController();
  //rev::SparkMaxRelativeEncoder shooterMotorEncoder = shooterMotor->GetEncoder(rev::CANEncoder::EncoderType::kNoSensor, 4000);
  //rev::SparkMaxRelativeEncoder shooterMotorEncoder = shooterMotor->GetEncoder();


  rev::CANSparkMax * shoot1 = new rev::CANSparkMax(Shooter1, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax * shoot2 = new rev::CANSparkMax(Shooter2, rev::CANSparkMax::MotorType::kBrushless); 

  rev::SparkMaxRelativeEncoder shootEncoder = shoot1->GetEncoder();
  rev::SparkMaxPIDController shootPid =       shoot1->GetPIDController();
  
  // rev::CANSparkMax * hoodMotor = new rev::CANSparkMax(hoodMotorID, rev::CANSparkMax::MotorType::kBrushless);
  // rev::SparkMaxPIDController hoodMotorPID = hoodMotor->GetPIDController();
  // rev::SparkMaxRelativeEncoder hoodMotorEncoder = hoodMotor->GetEncoder();

  // rev::CANSparkMax * turretMotor = new rev::CANSparkMax(turretMotorID, rev::CANSparkMax::MotorType::kBrushless);
  // rev::SparkMaxPIDController turretMotorPID = turretMotor->GetPIDController();
  // rev::SparkMaxRelativeEncoder turretMotorEncoder = turretMotor->GetEncoder();



  double setpoint = 0;

  // Change this to match the name of your camera
  //photonlib::PhotonCamera camera{"photonvision"};
  //photonlib::PhotonPipelineResult prevVisionResult;
  
  bool track = true;
  float turretTheta = 90;

  const float visionP  = 0.01;
  const float visionI = 0.0;
  const float visionD = 0.2;
  //frc2::PIDController* visionpid = new frc2::PIDController(visionP, visionI, visionD, ShooterModuleRunInterval);

  float turretLimitPos = 180;
  float turretLimitNeg = 0;
};
