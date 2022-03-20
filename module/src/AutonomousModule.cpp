#include "AutonomousModule.h"
#include <frc/SmartDashboard/SmartDashboard.h>

void AutonomousModule::periodicInit() {
    robPos.x = 0;
    robPos.y = 0;
    pathPoint p;
    p.x = 3;
    p.y = 3;
    pathPoint p1;
    p1.x = -3;
    p1.y = 3;

    pathPoint p2;
    p2.x = 0;
    p2.y = 0;
    path.push_back(p);
    path.push_back(p1);
    path.push_back(p2);
    this->msInterval = AutonomousModuleRunInterval;
}

void AutonomousModule::periodicRoutine() {
    if (stateRef->IsDisabled()) {
        frc::SmartDashboard::PutBoolean("IN auto module", false); 
        if (!resetPath) {
        frc::SmartDashboard::PutBoolean("reset path", false);
            Message* m = pipes[0]->popQueue(); //this is nothing...?
            while (m) {
                m = pipes[0]->popQueue();
            }
            pathi = 0;
            robPos.x = 0;
            robPos.y = 0;
            robTheta = 0;
            resetPath = true;
            frc::SmartDashboard::PutBoolean("reset path", resetPath);
        }
        return;
    }
 
    if ((stateRef->IsAutonomousEnabled()) && (pathi < path.size())) {
        resetPath = false;

        pathPoint delta;
        delta.x = (path[pathi].x - robPos.x);
        delta.y = (path[pathi].y - robPos.y);

        float d = sqrt(pow(delta.x, 2) + pow(delta.y, 2));
        pathPoint unitDir;
        unitDir.x = delta.x / d;
        unitDir.y = delta.y / d;

        delta.x = delta.x + unitDir.x * coordOffset;
        delta.y = delta.y + unitDir.y * coordOffset;



        float theta = atan2((path[pathi].x - robPos.x), (path[pathi].y - robPos.y)) * (180/(3.14159265));
        theta = theta - robTheta;
        robTheta += theta;

        std::vector<float> mVec = {theta, 0, 7, 21};

        pipes[0]->pushQueue(new Message("PT", mVec));
        
        d = sqrt(pow(delta.x, 2) + pow(delta.y, 2));
        mVec = {d, 7, 21};
        pipes[0]->pushQueue(new Message("PD", mVec));

        robPos.x += delta.x;
        robPos.y += delta.y;

        pathi++;
        
    }
}  
std::vector<uint8_t> AutonomousModule::getConstructorArgs() { return std::vector<uint8_t> {DriveBaseModuleID, IntakeModuleID}; }
