// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "paths.h"
#define PATHS_FILE "paths.txt"

// // All Module Includes
#include "DriveBaseModule.h"
#include "IntakeModule.h"
#include "ShooterModule.h"
#include "AutonomousModule.h"


DriveBaseModule compRobotDrive;
IntakeModule compIntake;
ShooterModule compShooter;
//AutonomousModule automod;

void Robot::RobotInit() {
  compRobotDrive.periodicInit();
  compIntake.periodicInit();
  compShooter.periodicInit();
}

void Robot::RobotPeriodic() {
  compRobotDrive.periodicRoutine();
  compIntake.periodicRoutine();
  compShooter.periodicRoutine();
}
 
void Robot::AutonomousInit() { 
  // int sindex = 0;
  // pathi = 0;
  // paths.clear();

  // while(sindex < paths.length()){ 
  //     if (paths[sindex] == '*') {
  //         pathi++;
  //         sindex++;
  //     }
  //     else if ((pathi == currpath) && (paths[sindex] == 's')) {
  //         pathPoint finalp = path[path.size() - 1];
  //         float dToGoal = sqrt(pow(finalp.x, 2) + pow(finalp.y, 2));

  //         std::cout << "dtogoal " << dToGoal << std::endl;
          
  //         pathPoint shootingp;
  //         shootingp.x = (-1 * finalp.x / dToGoal) * (dToGoal - shootingDistance) + finalp.x;
  //         shootingp.y = (-1 * finalp.y / dToGoal) * (dToGoal - shootingDistance) + finalp.y;
          
  //         path.push_back(shootingp);
  //         shootingPoints[path.size() - 1] = true;
  //         sindex += 2;
  //     }
  //     else if ((pathi == currpath) && (paths[sindex] == 'r')) {
  //         pathPoint finalp = path[path.size() - 1];
          
  //         path.push_back(finalp);
  //         shootingPoints[path.size() - 1] = true;
  //         sindex += 2;
  //     }
  //     else if (pathi == currpath) {
  //         pathPoint p;
  //         p.x = atof(paths.substr(sindex, 6).c_str());
  //         p.y = atof(paths.substr(sindex + 7, 6).c_str());
  //         path.push_back(p);
  //         sindex += 14;
  //     }
  // }

  // robTheta = startingThetas[currpath];
  // pathi = 1;
  // robPos = path[0];

}
void Robot::AutonomousPeriodic() {
  // //testing PID's during comp
  if ((!tested) && (compRobotDrive.PIDDriveSimpleTick(2))) {
    tested = true;
  }


  // //***HAVE INDEXEING BE OPEN (maybe just two) AND ON ALL THE TIME (would this ruin ball?), so all 3 indexing motors on, then activate shooter when needed
  // if(pathi < path.size()) {
  //   if(turnFlag) {
  //     if(compRobotDrive.GyroTurnTick(theta)) {
  //       turnFlag = false;
  //     }

  //   } else if (moveFlag) { //does this logic work?, NOT RETURNING ANYTHIGN IN PIDDRIVESIMPLETICK
  //     if(compRobotDrive.PIDDriveSimpleTick(d)) {
  //       moveFlag = false;
  //       //which intakes, all 3 right because doing 1 ball at a time, all 3 for some time, then when shoot you shoot\
  //      compIntake.disable(); 
  //     }
  //     else if (compRobotDrive.getDistanceTraversed() < (0.85 * d)) {
  //      compIntake.enable();
  //     }
  //   } 
  //   else if(shootFlag) {
  //     //I added a timestamp for half a second of waiting time in the shoot() method, check if needed !!!!!, added because shooterIndexer needs a bit of time
  //     if(compShooter.shoot()) {
  //       //maybe add turning here if need be...
  //       compShooter.stopShooting();
  //       shootFlag = false;
  //     }
  //   } else {
  //     //do we disable INTAKE HERE TOO !!!!!!!!!
  //     pathPoint delta;
  //     delta.x = (path[pathi].x - robPos.x);
  //     delta.y = (path[pathi].y - robPos.y);

  //     d = sqrt(pow(delta.x, 2) + pow(delta.y, 2));
  //     pathPoint unitDir;
  //     unitDir.x = delta.x / d;
  //     unitDir.y = delta.y / d;

  //     delta.x = delta.x + unitDir.x * coordOffset; //should be plus right?
  //     delta.y = delta.y + unitDir.y * coordOffset;

  //     theta = atan2((path[pathi].x - robPos.x), (path[pathi].y - robPos.y)) * (180/(3.14159265));
  //     theta = theta - robTheta;
  //     robTheta += theta;
  //     turnFlag = true;
  //     compRobotDrive.InitGyro();

  //       //turn theta amount !!

  //     d = sqrt(pow(delta.x, 2) + pow(delta.y, 2));
  //     moveFlag = true;
        
  //       //move d amount


  //     robPos.x += delta.x;
  //     robPos.y += delta.y;
  //     pathi++;

  //     if (shootingPoints[pathi]) { 
  //       shootFlag == true;
  //     } 
  //   }
  // }
}



void Robot::TeleopInit() {
  //Move this somewhere else later

  //Shooter Motor inits
 
}

void Robot::TeleopPeriodic() {
  
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {
 }

void Robot::TestPeriodic() {
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

