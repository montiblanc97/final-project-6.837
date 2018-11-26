#include "wall.h"


Wall::Wall(Vector3f bottom_left_corner, Vector3f top_left_corner, Vector3f top_right_corner) {
    Vector3f u = top_left_corner - bottom_left_corner;
    Vector3f v = top_right_corner - bottom_left_corner;
    _normal = Vector3f::cross(u, v).normalized();
    _d = -(Vector3f::dot(_normal, bottom_left_corner));  // plug in any point on plane

    _xmin = bottom_left_corner[0];
    _xmax = top_right_corner[0];
    _ymin = bottom_left_corner[1];
    _ymax = top_right_corner[1];
    _zmin = bottom_left_corner[2];
    _zmax = top_right_corner[2];
}
