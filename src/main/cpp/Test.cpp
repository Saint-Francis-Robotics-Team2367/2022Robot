// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
#include "Test.h"
 
void Test::checkMotorIDs(){
   int count = 0;
   const int maxNumIDs = 16;
   std::string workingMotorIDString = "Working Motor IDs: ";
   std::list<int> motorList;
   std::list<int>::iterator currentID;
 
   for (int i = 0; i < maxNumIDs; i++) {
   motorList.push_back(i);
   std::cout << "TestInit: Motor ID: " << i << std::endl;     
   }
 
   for (currentID = motorList.begin(); currentID != motorList.end(); currentID++) {
   std::cout << "TestPeriodic: Testing Motor ID: " << *currentID << std::endl;
 
   rev::CANSparkMax* motor = new rev::CANSparkMax(*currentID, rev::CANSparkMax::MotorType::kBrushless);
 
   motor->Set(0.5);
 
   // for some reason GetFault() is needed for GetLastError() to catch the error - need to investigate
  //  motor->GetFault(rev::REVLibError::kHALError);
   motor->GetFault(rev::CANSparkMax::FaultID::kMotorFault);
   if ((motor->GetLastError() == rev::REVLibError::kHALError)){
       std::cout << "Deleting motor with motor ID of " << *currentID << std::endl;
       currentID = motorList.erase(currentID);
       currentID--;  
      
   } else {
       std::cout << "Working motor ID " << *currentID << " is kept in list" << std::endl;
       motor->Set(0.0);
   }
 
   std::cout << "Deleting motor" << std::endl;
   delete motor;
 
   }
   std::cout << "Done iterating through list" << std::endl;
 
   std::cout << "Working motor ID ";
 
   for (auto &j : motorList) {
   std::cout << j << " ";
   workingMotorIDString.append(std::to_string(j) + " ");
   }
   std::cout << std::endl;
   std::cout << workingMotorIDString << std::endl;
   // frc::FRC_ReportError() status=-1 format = workingMotorIDString;

   // this line needs to be fixed
   // frc::FRC_ReportError(-1, workingMotorIDString);
   //frc::ReportError(workingMotorIDString, "Test.cpp", 1, "test")
   //frc::ReportError(workingMotorIDString);
   //frc::ReportWarning(workingMotorIDString);
   std::cout << std::endl;
 
   std::cout << "Done printing working motorID list" << std::endl;
   std::cout << "Deleting list..." << std::endl;
   // motorList.~list<int>();
   motorList.clear();
}

