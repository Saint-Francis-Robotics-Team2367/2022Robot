#pragma once

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/RobotController.h>
#include <iostream>

class Shuffleboard {
    public:
        nt::NetworkTableEntry battery = frc::Shuffleboard::GetTab("Drive Train").Add("Battery Level", (double) frc::RobotController::GetBatteryVoltage()).GetEntry();
        nt::NetworkTableEntry intake = frc::Shuffleboard::GetTab("Drive Train").Add("Intake Extended", false).GetEntry();

        //static bool intakeExtended() {return false;}

        void refreshValues()
        {
            battery.SetDouble((double) frc::RobotController::GetBatteryVoltage());
        }
};