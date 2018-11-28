#include "sphere.h"

Sphere::Sphere(Vector3f center, float radius) {
    _center = center;
    _radius = radius;
}



/**
 *
 * @param wall to check intersection
 * @param hit to be modified on intersection
 * @return if intersecting or not
 */
bool Sphere::intersectsWall(Wall wall, Hit& hit) {
    // find closest point on plane to center of sphere, calculate if distance greater than radius
    Vector3f sphere_to_any_point = _center - Vector3f(wall._xmin, wall._ymin, wall._zmax);
    float dist = Vector3f::dot(sphere_to_any_point, wall._normal);  // dot to normal gives shortest distance

    // intersection
    if (dist <= _radius) {
        // go away from the wall by however much it intersected
        hit.resolveDirection = wall._normal;
        hit.resolveDist = dist;
        return true;
    }

    return false;
}


/**
 *
 * @param other sphere to check intersection
 * @param hit to be modified on intersection
 * @return if intersecting or not
 */
bool Sphere::intersectsSphere(Sphere other, Hit& hit) {
    Vector3f to_other = other._center - _center;
    float dist_to_other = to_other.abs();
    float radii_dist = _radius + other._radius;

    // no intersection or point intersection, centers distance farther than radii
    if (dist_to_other >= radii_dist + 0.001) {
        return false;
    }

    // resolve intersection info
    // go away from the other sphere
    hit.resolveDirection = -to_other;

    // https://gamedev.stackexchange.com/questions/75756/sphere-sphere-intersection-and-circle-sphere-intersection
    float h = 1.0f/2 + (_radius * _radius - other._radius * other._radius)/(2 * dist_to_other * dist_to_other);
    Vector3f intersecting_circle_center = _center + h * (other._center - _center);
    float intersecting_this_dist = (_center - intersecting_circle_center).abs();

    // go away the dist from this sphere's center to that of the circle from intersection
    hit.resolveDist = abs(_radius - intersecting_this_dist);

    return true;
}

/**
 *
 * @param other sphere to check intersection
 * @return if intersecting or not
 */
bool Sphere::intersectsSphere(Sphere other) {
    Vector3f to_other = other._center - _center;
    float dist_to_other = to_other.abs();
    float radii_dist = _radius + other._radius;
    
    // no intersection or point intersection, centers distance farther than radii
    if (dist_to_other >= radii_dist + 0.001) {
        return false;
    }
    
    return true;
}

void Sphere::updateCenter(Vector3f center) {
    _center = center;
}
