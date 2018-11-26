#include "timestepper.h"

#include <cstdio>

void ForwardEuler::takeStep(ParticleSystem *particleSystem, float stepSize) {
    //TODO: See handout 3.1
    std::vector<Vector3f> current = particleSystem->getState();
    std::vector<Vector3f> derivatives = particleSystem->evalF(current);

    std::vector<Vector3f> updated;
    for (int i = 0; i < current.size(); i++) {
        Vector3f particle = current[i];
        Vector3f deriv = derivatives[i];
        float x = particle[0];
        float x_h = stepSize * deriv[0];
        float y = particle[1];
        float y_h = stepSize * deriv[1];
        float z = particle[2];
        updated.emplace_back(x + x_h, y + y_h, z);
    }
    particleSystem->setState(updated);
}

void Trapezoidal::takeStep(ParticleSystem *particleSystem, float stepSize) {
    //TODO: See handout 3.1
    std::vector<Vector3f> current = particleSystem->getState();
    std::vector<Vector3f> f0 = particleSystem->evalF(current);

    std::vector<Vector3f> stepped;
    for (int i = 0; i < current.size(); i++) {
        Vector3f particle = current[i];
        Vector3f deriv = f0[i];
        float x = particle[0];
        float x_h = stepSize * deriv[0];
        float y = particle[1];
        float y_h = stepSize * deriv[1];
        float z = particle[2];

        stepped.emplace_back(x + x_h, y + y_h, z);
    }
    std::vector<Vector3f> f1 = particleSystem->evalF(stepped);

    std::vector<Vector3f> updated;
    for (int i = 0; i < current.size(); i++) {
            Vector3f particle = current[i];
            Vector3f change = f0[i] + f1[0];
            change = change * stepSize / 2;
            updated.push_back(particle + change);
    }
    particleSystem->setState(updated);
}

std::vector<Vector3f> rangeKuttaHelper(std::vector<Vector3f> pos, std::vector<Vector3f> prev_k, ParticleSystem *particleSystem, float stepSize) {
    std::vector<Vector3f> out;
    out.reserve(pos.size());
    for (int i=0;i<pos.size();i++) {
        out.push_back(pos[i] + stepSize * prev_k[i]);
    }
    return particleSystem->evalF(out);
}

void RK4::takeStep(ParticleSystem *particleSystem, float stepSize) {
    std::vector<Vector3f> current = particleSystem->getState();

    std::vector<Vector3f> k1 = particleSystem->evalF(current);
    std::vector<Vector3f> k2 = rangeKuttaHelper(current, k1, particleSystem, stepSize/2);
    std::vector<Vector3f> k3 = rangeKuttaHelper(current, k2, particleSystem, stepSize/2);
    std::vector<Vector3f> k4 = rangeKuttaHelper(current, k3, particleSystem, stepSize);

    std::vector<Vector3f> updated;
    for (int i=0; i<current.size(); i++) {
        Vector3f change = k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i];
        change = change * stepSize / 6;
        updated.push_back(current[i] + change);
    }

    particleSystem->setState(updated);
}

