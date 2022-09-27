#include "navx2.h"


double navx2::PIDGet() {
    return ahrs->GetRate() / 550; //can change later
}

