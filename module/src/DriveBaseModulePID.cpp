#include "DriveBaseModulePID.h"


DriveBaseModulePID::DriveBaseModulePID()
{
	m_out=0;
	rightStickPID.Enable();
}

void DriveBaseModulePID::PIDWrite(double output)
{
	m_out = output;
}

double DriveBaseModulePID::GetOutput()
{
	return m_out;
}
double DriveBaseModulePID::CalculatePID() 
{
	
	return rightStickPID.GetSetpoint();
}

