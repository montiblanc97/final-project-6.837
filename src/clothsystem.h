#ifndef CLOTHSYSTEM_H
#define CLOTHSYSTEM_H

#include <vector>

#include "particlesystem.h"
#include "pendulumsystem.h"

class ClothSystem : public ParticleSystem
{
    ///ADD MORE FUNCTION AND FIELDS HERE
public:
    ClothSystem();

    // evalF is called by the integrator at least once per time step
    std::vector<Vector3f> evalF(std::vector<Vector3f> state) override;

    // draw is called once per frame
    void draw(GLProgram& ctx);

    // inherits
    // std::vector<Vector3f> m_vVecState;

    std::vector<Spring> structural;
    std::vector<Spring> shear;
    std::vector<Spring> flexion;

    bool wind = false;
    void toggleWind();
};

int positionIndexOf(int i, int j);
std::vector<Vector3f> springHelper(std::vector<Vector3f> f, std::vector<Vector3f> state, std::vector<Spring> springs);


#endif
