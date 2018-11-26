#ifndef A3_SPHERE_H
#define A3_SPHERE_H

#include "wall.h"
#include "hit.h"

class Sphere {
public:
    Sphere(Vector3f center, float radius);

    bool intersectsWall(Wall wall, Hit& hit);
    bool intersectsSphere(Sphere other, Hit& hit);

    void updateCenter(Vector3f center);

private:
    Vector3f _center;
    float _radius;
};


#endif //A3_SPHERE_H
