#ifndef PENDULUMSYSTEM_H
#define PENDULUMSYSTEM_H

#include <vector>

#include "particlesystem.h"

class Spring {
public:
    Spring(int end1, int end2, float rest_length, float stiffness);

    int end1;
    int end2;

    float rest_length;
    float stiffness;
};

class PendulumSystem : public ParticleSystem
{
public:
    PendulumSystem();

    std::vector<Vector3f> evalF(std::vector<Vector3f> state) override;
    void draw(GLProgram&);

    // inherits 
    // std::vector<Vector3f> m_vVecState;
    std::vector<Spring> springs;
};

#endif
