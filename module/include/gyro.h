#include <frc/PIDController.h>
#include "AHRS.h"

class gyro: public frc::PIDSource{ 

//class inherits Drivebasemodule and PIDOutput (virtual class)
//creates PIDOutput object which would track this
//PIDSource x frc::Adis overwrite and create instance and pass here

//include gyro PIDSource


public:
    AHRS *ahrs;
    gyro() {
        ahrs = new AHRS(frc::SerialPort::kMXP);
    }
     
    double PIDGet() {
        return ahrs->GetRate() / 150; //this work?
    };

    double GetRate() {
        return ahrs->GetRate();
    }

    double GetAngle() {
        return ahrs->GetAngle();
    }
     //enter other necessary getters and setters for gyro here
};  