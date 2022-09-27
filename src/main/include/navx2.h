#include <frc/PIDController.h>
#include "AHRS.h"

class navx2: public frc::PIDSource{ 

//class inherits Drivebasemodule and PIDOutput (virtual class)
//creates PIDOutput object which would track this
//PIDSource x frc::Adis overwrite and create instance and pass here

//include gyro PIDSource


public:
    AHRS *ahrs;
    navx2() {
        ahrs = new AHRS(frc::SerialPort::kMXP);
    }
     
     double PIDGet();

     //enter other necessary getters and setters for gyro here
};  