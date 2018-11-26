#include "ballsystem.h"

#include <cassert>
#include "camera.h"
#include "vertexrecorder.h"

// TODO adjust to number of particles.
const int NUM_PARTICLES = 4;
const float mass = 1;
const float drag_constant = 0.5;

const float sphere_radius = 0.075f;


BallSystem::BallSystem()
{
    // make walls
    _walls.emplace_back(Vector3f(-1, -1, -1), Vector3f(-1, -1, 1), Vector3f(1, -1, 1));  // floor
    _walls.emplace_back(Vector3f(-1, -1, -1), Vector3f(-1, 1, -1), Vector3f(1, 1, -1));  // front
    _walls.emplace_back(Vector3f(-1, -1, 1), Vector3f(-1, 1, 1), Vector3f(1, 1, 1));  // back
    _walls.emplace_back(Vector3f(-1, -1, -1), Vector3f(-1, 1, -1), Vector3f(1, 1, 1));  // left
    _walls.emplace_back(Vector3f(1, -1, -1), Vector3f(1, 1, -1), Vector3f(1, 1, 1));  // right


    // big vector of 2n with position at even indices, velocity at odd

    for (int i=0; i<NUM_PARTICLES; i++) {
        Vector3f position = Vector3f(rand_uniform(-0.5f, 0.5f), rand_uniform(-0.5f, 0.5f), rand_uniform(-0.5f, 0.5f));
        m_vVecState.push_back(position);  // position
        m_vVecState.emplace_back(rand_uniform(-0.5f, 0.5f), rand_uniform(-0.5f, 0.5f), rand_uniform(-0.5f, 0.5f));  // velocity

        // add sphere rep for each
        _spheres.emplace_back(position, sphere_radius);
    }
}


std::vector<Vector3f> BallSystem::evalF(std::vector<Vector3f> state)
{
    // need to first update sphere positions to the particles (not handled during time step)
    for (int i=0; i<_spheres.size(); i+=1) {
        Vector3f current_position = state[i*2];  // get position in combined vector
        _spheres[i].updateCenter(current_position);
    }


    std::vector<Vector3f> f;

    for (int i=0; i<state.size(); i+=2) {  //position and velocity stored in same
        Vector3f vel = state[i + 1];

        f.push_back(vel);  // derivative of position is velocity

        Vector3f net_force = Vector3f(0, 0, 0);
        //  - gravity
        net_force[1] = net_force[1] - 9.8 * mass;

        //  - viscous drag
        net_force = net_force - drag_constant * vel;

        //TODO: collision detection/resolution

        f.push_back(net_force/mass);
    }

    return f;
}

// render the system (ie draw the particles)
void BallSystem::draw(GLProgram& gl)
{
    std::vector<Vector3f> current = getState();

    const Vector3f PENDULUM_COLOR(0.73f, 0.0f, 0.83f);
    gl.updateMaterial(PENDULUM_COLOR);

    // TODO 4.2, 4.3

    // example code. Replace with your own drawing  code
    for (int i=0; i<current.size(); i+=2) {
        Vector3f pos = current[i];

        gl.updateModelMatrix(Matrix4f::translation(pos));
        drawSphere(sphere_radius, 10, 10);
    }

    gl.disableLighting();
    gl.updateModelMatrix(Matrix4f::identity()); // update uniforms after mode change
    VertexRecorder rec;

    glLineWidth(3.0f);
    rec.draw(GL_LINES);
}
