#include "pendulumsystem.h"

#include <cassert>
#include "camera.h"
#include "vertexrecorder.h"

// TODO adjust to number of particles.
const int NUM_PARTICLES = 4;
const float mass = 1;
const float drag_constant = 0.5;
const float rest_length = 0.01;
const float stiffness = 40;


PendulumSystem::PendulumSystem()
{

    // TODO 4.2 Add particles for simple pendulum
    // TODO 4.3 Extend to multiple particles

    // To add a bit of randomness, use e.g.
    // float f = rand_uniform(-0.5f, 0.5f);
    // in your initial conditions.

    // big vector of 2n with position at even indices, velocity at odd

    // fixed point
    m_vVecState.emplace_back(0, 0, 0);  // position
    m_vVecState.emplace_back(0, 0, 0);  // velocity

    for (int i=0; i<NUM_PARTICLES; i++) {
        m_vVecState.emplace_back(rand_uniform(-0.5f, 0.5f), rand_uniform(-0.5f, 0.5f), rand_uniform(-0.5f, 0.5f));  // position
        m_vVecState.emplace_back(rand_uniform(-0.5f, 0.5f), rand_uniform(-0.5f, 0.5f), rand_uniform(-0.5f, 0.5f));  // velocity

        int index = (i + 1) * 2;
        springs.emplace_back(index-2, index, rest_length, stiffness);
    }
}


std::vector<Vector3f> PendulumSystem::evalF(std::vector<Vector3f> state)
{
    std::vector<Vector3f> f;
    // TODO 4.1: implement evalF

    // ignore first one (fixed point)
    f.emplace_back(0, 0, 0);
    f.emplace_back(0, 0, 0);

    for (int i=2; i<state.size(); i+=2) {  //position and velocity stored in same
        Vector3f vel = state[i + 1];

        f.push_back(vel);  // derivative of position is velocity

        Vector3f net_force = Vector3f(0, 0, 0);
        //  - gravity
        net_force[1] = net_force[1] - 9.8 * mass;

        //  - viscous drag
        net_force = net_force - drag_constant * vel;

        f.push_back(net_force/mass);
    }

    //  - springs
    for (int i=0; i<springs.size(); i++) {
        Spring spr = springs[i];

        Vector3f pos1 = state[spr.end1];
        Vector3f pos2 = state[spr.end2];
        Vector3f d = pos1 - pos2;

        // i to j
        Vector3f spring_force = -spr.stiffness * (d.abs() - spr.rest_length) * d / d.abs() / mass;
        f[spr.end1 + 1] += spring_force;
        // j to i
        f[spr.end2 + 1] -= spring_force;
    }

    // spring affects fixed point, reset it here
    f[0] = Vector3f(0, 0, 0);
    f[1] = Vector3f(0, 0, 0);

    return f;
}

// render the system (ie draw the particles)
void PendulumSystem::draw(GLProgram& gl)
{
    std::vector<Vector3f> current = getState();

    const Vector3f PENDULUM_COLOR(0.73f, 0.0f, 0.83f);
    gl.updateMaterial(PENDULUM_COLOR);

    // TODO 4.2, 4.3

    // example code. Replace with your own drawing  code
    for (int i=0; i<current.size(); i+=2) {
        Vector3f pos = current[i];

        gl.updateModelMatrix(Matrix4f::translation(pos));
        drawSphere(0.075f, 10, 10);
    }

    gl.disableLighting();
    gl.updateModelMatrix(Matrix4f::identity()); // update uniforms after mode change
    VertexRecorder rec;

    for (int i=0; i<this->springs.size(); i++) {
        Spring spr = this->springs[i];
        Vector3f pos1 = current[spr.end1];
        Vector3f pos2 = current[spr.end2];

        Vector3f diff = pos2 - pos1;

        rec.record(pos1, PENDULUM_COLOR);
        rec.record(pos1 + diff, PENDULUM_COLOR);
    }
    glLineWidth(3.0f);
    rec.draw(GL_LINES);
}

Spring::Spring(int end1, int end2, float rest_length, float stiffness) {
    this->end1 = end1;
    this->end2 = end2;
    this->rest_length = rest_length;
    this->stiffness = stiffness;
}
