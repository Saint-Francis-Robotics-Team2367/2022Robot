#include <string>

std::string paths = "0.0000,0.0000 r *\
                     19.202,99.203 r 6.7220,51.498 s *\
                     70.714,68.917 r 54.232,19.336 s *\
                     83.286,55.205 r 43.858,25.393 s *\
                     25.782,73.619 r 0.0000,52.884 s *\
                     68.838,64.201 r -56.11,24.102 s *\
                     85.017,50.400 r -44.38,31.243 s *\
                    "; //first coordinate is the robot position

float startingThetas[7] = {
    0, 190.8, 225.6, -56.5, 14.6, 46.9, -59.5
};
int currpath = 0;