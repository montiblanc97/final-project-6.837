#ifndef PENDULUMSYSTEM_H
#define PENDULUMSYSTEM_H

#include <vector>

#include "particlesystem.h"
#include "wall.h"
#include "sphere.h"

class Spring {
public:
    Spring(int end1, int end2, float rest_length, float stiffness);

    int end1;
    int end2;

    float rest_length;
    float stiffness;
};

class BallSystem : public ParticleSystem
{
public:
    BallSystem(float stepsize);

    std::vector<Vector3f> evalF(std::vector<Vector3f>& state) override;
    void draw(GLProgram&);

    // inherits 
    // std::vector<Vector3f> m_vVecState;

    float _stepsize;

    std::vector<Wall> _walls;
    std::vector<Sphere> _spheres;  // note indexed for each particle

    std::vector<int> _collided;
    std::vector<Vector3f> _colors;
};

#endif
