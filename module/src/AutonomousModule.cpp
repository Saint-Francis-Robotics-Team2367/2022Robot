#include "AutonomousModule.h"
#include <frc/SmartDashboard/SmartDashboard.h>

void AutonomousModule::periodicInit() {
    robPos.x = 0;
    robPos.y = 0;
    pathPoint p;
    p.x = 3;
    p.y = 3;
    pathPoint p1;
    p1.x = -6;
    p1.y = 6;
    path.push_back(p);
    path.push_back(p1);
    path.push_back(p);
    this->msInterval = AutonomousModuleRunInterval;
    isTrue = false;
}

void AutonomousModule::periodicRoutine() {
    if (stateRef->IsDisabled()) {frc::SmartDashboard::PutBoolean("IN auto module", false); return;}
 
    if ((stateRef->IsAutonomousEnabled()) && (pathi < path.size())) {
        frc::SmartDashboard::PutBoolean("IN auto module", true);
        float theta = atan2((path[pathi].x - robPos.x), (path[pathi].y - robPos.y)) / (3.14159265) * 180;
        frc::SmartDashboard::PutNumber("IN auto module", false);
        theta = theta - robTheta;
        robTheta += theta;

        frc::SmartDashboard::PutNumber("theta", theta);

        std::vector<float> mVec = {theta, 0, 7, 21};
        frc::SmartDashboard::PutNumber("num iterations", 15) ;

        pipes[0]->pushQueue(new Message("PT", mVec));
        frc::SmartDashboard::PutBoolean("is true", isTrue);
        isTrue = true;
        
        float d = sqrt(pow((path[pathi].x - robPos.x), 2) + pow((path[pathi].y - robPos.y), 2));
        
        mVec = {d, 7, 21};
        pipes[0]->pushQueue(new Message("PD", mVec));

        robPos.x += path[pathi].x;
        robPos.y += path[pathi].y;

        pathi++;
        return;
    }
}  
std::vector<uint8_t> AutonomousModule::getConstructorArgs() { return std::vector<uint8_t> {DriveBaseModuleID}; }