#ifndef A3_PLANE_H
#define A3_PLANE_H

#include <vecmath.h>

class Wall{
public:
    Wall(Vector3f bottom_left_corner, Vector3f top_left_corner, Vector3f top_right_corner);

public:
    Vector3f _normal;
    float _d;
    float _xmin;
    float _xmax;
    float _ymin;
    float _ymax;
    float _zmin;
    float _zmax;
};


#endif //A3_PLANE_H
