#include "DriveBaseModule.h"
#include "navx2.h"


class GyroDrivePID: public DriveBaseModule, frc::PIDOutput{ 

//class inherits Drivebasemodule and PIDOutput (virtual class)
//creates PIDOutput object which would track thisƒƒ
//PIDSource x frc::Adis overwrite and create instance and pass here

; //is this fine with overides

//include gyro PIDSource∏


public:
    navx2 gyroSource;
    frc::PIDController rightStickPID{1, 0.0, 0.0, &gyroSource, this}; //maybe adjust periods here
    GyroDrivePID();
    void PIDWrite(double output);

    double GetOutput();
    private:
	    double m_out;
};  