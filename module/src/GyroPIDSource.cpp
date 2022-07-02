#include "GyroPIDSource.h"


double GyroPIDSource::PIDGet() {
    return m_imu.GetRate().value() / 250;
}