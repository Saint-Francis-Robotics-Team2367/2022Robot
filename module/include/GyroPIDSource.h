#define GYROPIDSOURCE_H
#include <frc/ADIS16448_IMU.h>
#include <frc/PIDController.h>

class GyroPIDSource: public frc::PIDSource{ 

//class inherits Drivebasemodule and PIDOutput (virtual class)
//creates PIDOutput object which would track this
//PIDSource x frc::Adis overwrite and create instance and pass here

//include gyro PIDSource
frc::ADIS16448_IMU& m_imu;

public:
     
     GyroPIDSource(frc::ADIS16448_IMU& m_imu) : m_imu{m_imu} {};
     
     double PIDGet();
};  