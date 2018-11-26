#ifndef A3_HIT_H
#define A3_HIT_H

#include <vecmath.h>

class Hit {
public:
    Vector3f resolveDirection = Vector3f(-1, -1, -1);
    float resolveDist = -1;
};


#endif //A3_HIT_H
