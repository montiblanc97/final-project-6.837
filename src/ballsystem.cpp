#include "ballsystem.h"

#include <cassert>
#include <cmath>

#include "camera.h"
#include <iostream>
#include "vertexrecorder.h"

// TODO adjust to number of particles.
const int NUM_PARTICLES = 50;
const float mass = 1;
const float drag_constant = 1;

const float sphere_radius = 0.75f;
const Vector3f FLOOR_COLOR(1.0f, 1.0f, 1.0f);

BallSystem::BallSystem(float stepsize)
{
    // make walls
    _walls.emplace_back(Vector3f(-1, -3, -1), Vector3f(-1, -3, 1), Vector3f(1, -3, 1));  // floor
    _walls.emplace_back(Vector3f(1, 1, 3.f), Vector3f(1, -3, 3.f), Vector3f(-1, 1, 5.f));  // front
    _walls.emplace_back(Vector3f(-1, 3, 0.75f), Vector3f(-1, -3, 0.75f), Vector3f(1, 3, 0.f));  // back
    _walls.emplace_back(Vector3f(-3, 1, 1), Vector3f(-3, -3, 1), Vector3f(-3, 1, -1));  // left
    _walls.emplace_back(Vector3f(3, 1, -1), Vector3f(3, -1, -1), Vector3f(3, 1, 1));  // right


    // big vector of 2n with position at even indices, velocity at odd

    for (int i=0; i<NUM_PARTICLES; i++) {
        Vector3f position = Vector3f((i%3)-1, (i+1) * 1, 4);
        m_vVecState.push_back(position);  // position
        m_vVecState.emplace_back(rand_uniform(0, 1), rand_uniform(0, 1), rand_uniform(0, 1));  // velocity

        // add sphere rep for each
        _spheres.emplace_back(position, sphere_radius);
    }

    _collided = std::vector<int>(NUM_PARTICLES, 0);
    _stepsize = stepsize;

    for (int i=0; i<NUM_PARTICLES; i++) {
        _colors.emplace_back(rand_uniform(0, 1), rand_uniform(0, 1), rand_uniform(0, 1));
    }
}


std::vector<Vector3f> BallSystem::evalF(std::vector<Vector3f>& state)
{
    // need to first update sphere positions to the particles (not handled during time step)
    for (int i=0; i<_spheres.size(); i+=1) {
        Vector3f current_position = state[i*2];  // get position in combined vector
        _spheres[i].updateCenter(current_position);
    }


    // even position - velocity; odd position - acceleration
    std::vector<Vector3f> f(state.size(), Vector3f(0, 0, 0));

    for (int i=0; i<_spheres.size(); i+=1) {  //position and velocity stored in same
        // VELOCITY
        Vector3f vel = state[2*i+1];
        f[i*2] += vel; // derivative of position is velocity

        // ACCELERATION
        Vector3f net_force = Vector3f(0, 0, 0);
        net_force[1] = net_force[1] - 9.8 * mass;  // gravity
        net_force = net_force - drag_constant * vel;  // drag

        net_force = (net_force/mass);

        // Collision detection -- stop ball movement as collision detected
        //TODO: collision resolution
        Vector3f collision_force = Vector3f(0, 0, 0);

        for (int j=0; j<_spheres.size(); j+=1){
            if (i == j) {
                continue;
            }
            Hit hit = Hit();
            if (_spheres[i].intersectsSphere(_spheres[j], hit)) {
                collision_force += hit.resolveDirection * hit.resolveDist * 1/_stepsize * 10;
            }
        }

        for (int j=0; j<_walls.size(); j+=1) {
            Hit hit = Hit();
            if (_spheres[i].intersectsWall(_walls[j], hit)) {
                _collided[i] += 1;

                collision_force += _walls[j]._normal * fmax(0.5, abs(Vector3f::dot(_walls[j]._normal, vel))) * 0.1/_stepsize;
                if (j==0) {  // floor needs more power to counteract gravity
                    collision_force += _walls[j]._normal * 30/_stepsize;

                    if (_collided[i] >= 50) {
                        if (vel.absSquared() + collision_force.absSquared() - net_force.absSquared() < 2500) {
                            collision_force = Vector3f(0);
                            net_force = Vector3f(0);
                            f[i*2] = Vector3f(0);
                        } else {
                            _collided[i] = 0;
                        }
                    }
                }
            }
        }

        f[i*2+1] = net_force + collision_force;
    }

    return f;
}

// render the system (ie draw the particles)
void BallSystem::draw(GLProgram& gl)
{
    std::vector<Vector3f> current = getState();

    // TODO 4.2, 4.3

    // example code. Replace with your own drawing  code
    for (int i=0; i<current.size(); i+=2) {
        gl.updateMaterial(_colors[i/2]);
        Vector3f pos = current[i];

        gl.updateModelMatrix(Matrix4f::translation(pos));
        drawSphere(sphere_radius, 10, 10);
    }

    // set uniforms for floor
    gl.updateMaterial(FLOOR_COLOR);
    gl.updateModelMatrix(Matrix4f::translation(0, -3, 0));
    // draw floor
    drawQuad(50.0f);

    gl.updateModelMatrix(Matrix4f::rotateX(1.57) * Matrix4f::translation(0, -15, 0));
    drawQuad(50.0f);

    gl.disableLighting();
    gl.updateModelMatrix(Matrix4f::identity()); // update uniforms after mode change
    VertexRecorder rec;

    glLineWidth(3.0f);
    rec.draw(GL_LINES);
}
