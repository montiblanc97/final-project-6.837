#ifndef A3_SPRING_H
#define A3_SPRING_H

#include <vecmath.h>
#include <vector>

class Spring {
public:
    Vector3f* end1;
    Vector3f* end2;

    float rest_length;
    float stiffness;
};


#endif //A3_SPRING_H
