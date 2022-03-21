#pragma once

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/RobotController.h>
#include <iostream>

class Shuffleboard {
    public:
        //Drive Train
        nt::NetworkTableEntry battery = frc::Shuffleboard::GetTab("Drive Train").Add("Battery Level", (double) frc::RobotController::GetBatteryVoltage()).GetEntry();
        nt::NetworkTableEntry distance = frc::Shuffleboard::GetTab("Drive Train").Add("Distance to Goal", 0).GetEntry();
        nt::NetworkTableEntry angle = frc::Shuffleboard::GetTab("Drive Train").Add("Angle to Goal", 0).GetEntry();
        nt::NetworkTableEntry shooter = frc::Shuffleboard::GetTab("Drive Train").Add("Shooter UTS", false).GetEntry();
        nt::NetworkTableEntry intake = frc::Shuffleboard::GetTab("Drive Train").Add("Intake Extended", false).GetEntry();
        nt::NetworkTableEntry ball1 = frc::Shuffleboard::GetTab("Drive Train").Add("Ball 1 Loaded", false).GetEntry();
        nt::NetworkTableEntry ball2 = frc::Shuffleboard::GetTab("Drive Train").Add("Ball 2 Loaded", false).GetEntry();

        static bool shooter_speed() {return false;}
        static bool ballOneLoaded() {return true;}
        static bool ballTwoLoaded() {return false;}
        static bool intakeExtended() {return true;}

        //PIDs
        nt::NetworkTableEntry pValue = frc::Shuffleboard::GetTab("PIDs").Add("P Value", 0).GetEntry();
        nt::NetworkTableEntry iValue = frc::Shuffleboard::GetTab("PIDs").Add("I Value", 0).GetEntry();
        nt::NetworkTableEntry dValue = frc::Shuffleboard::GetTab("PIDs").Add("D Value", 0).GetEntry();

        void refreshValues()
        {
            battery.SetDouble((double) frc::RobotController::GetBatteryVoltage());
            //distance.SetDouble();
            //angle.SetDouble();
            shooter.SetBoolean(shooter_speed);
            intake.SetBoolean(intakeExtended);
            ball1.SetBoolean(ballOneLoaded);
            ball2.SetBoolean(ballTwoLoaded);

            pValue.SetDouble(0);
            iValue.SetDouble(0);
            dValue.SetDouble(0);
        }
};