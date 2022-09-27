#include "navx2.h"


double navx2::PIDGet() {
    return ahrs->GetRate() / 1000; //can change later, ask how this works too...
}

