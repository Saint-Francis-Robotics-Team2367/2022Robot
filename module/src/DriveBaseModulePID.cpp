#include "DriveBaseModulePID.h"


DriveBaseModulePID::DriveBaseModulePID()
{
	m_out=0;
}

void DriveBaseModulePID::PIDWrite(double output)
{
	m_out = output;
}

double DriveBaseModulePID::GetOutput()
{
	return m_out;
}

