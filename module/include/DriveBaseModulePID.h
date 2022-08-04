#define DRIVEBASEMODULEPID_H
#include "DriveBaseModule.h"
#include "GyroPIDSource.h"


class DriveBaseModulePID: public DriveBaseModule, frc::PIDOutput{ 

//class inherits Drivebasemodule and PIDOutput (virtual class)
//creates PIDOutput object which would track this
//PIDSource x frc::Adis overwrite and create instance and pass here

GyroPIDSource gyroSource{m_imu}; //is this fine with overides

//include gyro PIDSource∏


public:
    frc::PIDController rightStickPID{1.0, 0.0, 0.0, &gyroSource, this}; //maybe adjust periods here
    DriveBaseModulePID();
    void PIDWrite(double output);

    double GetOutput();
    private:
	    double m_out;
};  