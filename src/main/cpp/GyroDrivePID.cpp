#include "GyroDrivePID.h"


GyroDrivePID::GyroDrivePID()
{
	m_out=0;
	rightStickPID.Enable();
}

void GyroDrivePID::PIDWrite(double output)
{
	m_out = output;
}

double GyroDrivePID::GetOutput()
{
	return m_out;
}


