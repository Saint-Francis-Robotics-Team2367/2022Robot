#include "GyroPIDSource.h"


double GyroPIDSource::PIDGet() {
    return m_imu.GetRate().value() / 550; //can change later
}