#include "ballsystem.h"

#include <cassert>
#include <cmath>

#include "camera.h"
#include <iostream>
#include "vertexrecorder.h"

// TODO adjust to number of particles.
const int NUM_PARTICLES = 4;
const float mass = 1;
const float drag_constant = 0.5;

const float sphere_radius = 0.3f;
const Vector3f COLLISION_COLOR(0.3f, 0.5f, 0.5f);
const Vector3f FLOOR_COLOR(1.0f, 1.0f, 1.0f);


BallSystem::BallSystem()
{
    // make walls
    _walls.emplace_back(Vector3f(-1, -3, -1), Vector3f(-1, -3, 1), Vector3f(1, -3, 1));  // floor
//    _walls.emplace_back(Vector3f(-1, -1, -1), Vector3f(-1, 1, -1), Vector3f(1, 1, -1));  // front
//    _walls.emplace_back(Vector3f(-1, -1, 1), Vector3f(-1, 1, 1), Vector3f(1, 1, 1));  // back
//    _walls.emplace_back(Vector3f(-1, -1, -1), Vector3f(-1, 1, -1), Vector3f(1, 1, 1));  // left
//    _walls.emplace_back(Vector3f(1, -1, -1), Vector3f(1, 1, -1), Vector3f(1, 1, 1));  // right


    // big vector of 2n with position at even indices, velocity at odd

    for (int i=0; i<NUM_PARTICLES; i++) {
        Vector3f position = Vector3f((i%3)-1, (i+1) * 3, 0);
        m_vVecState.push_back(position);  // position
        m_vVecState.emplace_back(0, 0, 0);  // velocity

        // add sphere rep for each
        _spheres.emplace_back(position, sphere_radius);
    }

    _collided = std::vector<bool>(NUM_PARTICLES, false);
}


std::vector<Vector3f> BallSystem::evalF(std::vector<Vector3f> state)
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
        for (int j=i+1; j<_spheres.size(); j+=1){
            Hit hit = Hit();
            if (_spheres[i].intersectsSphere(_spheres[j], hit)) {
                _collided[i] = true;
                _collided[j] = true;
//                std::cout << hit.resolveDist << std::endl;
                f[i*2] += hit.resolveDirection * fmax(0.1, fmin(hit.resolveDist, 0.01)) * 0.00001;
//                net_force += hit.resolveDirection * hit.resolveDist * fmin(fmax(4, abs(vel[1])), 5) * 200;
            }
        }

        for (int j=0; j<_walls.size(); j+=1) {
            Hit hit = Hit();
            if (_spheres[i].intersectsWall(_walls[j], hit)) {
                _collided[i] = true;
//                std::cout << hit.resolveDirection[0] << hit.resolveDirection[1] << hit.resolveDirection[2] << std::endl;
//                std::cout << Vector3f::dot(_walls[j]._normal, vel) << std::endl;
                net_force += hit.resolveDirection * hit.resolveDist * fmax(0.5, abs(Vector3f::dot(_walls[j]._normal, vel))) * 1500;

                if (abs(Vector3f::dot(_walls[j]._normal, vel)) + abs(Vector3f::dot(_walls[j]._normal, net_force)) < 2.f) {
                    std::cout << "hit" << std::endl;
                    net_force = Vector3f(0);
                    f[i*2] = Vector3f(0);
                }
            }
        }

//        // Stop movement on collision
//        if (_collided[i]) {
//            f[i*2] = Vector3f(0, 1, 0);
//            f[i*2+1] = Vector3f(0, 1, 0);
//            continue;
//        }

        f[i*2+1] = net_force;
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
        if (_collided[int(i/2)]){
            gl.updateMaterial(COLLISION_COLOR);
        }
        drawSphere(sphere_radius, 10, 10);
        gl.updateMaterial(PENDULUM_COLOR);
        
    }

    // set uniforms for floor
    gl.updateMaterial(FLOOR_COLOR);
    gl.updateModelMatrix(Matrix4f::translation(0, -3, 0));
    // draw floor
    drawQuad(50.0f);

    gl.disableLighting();
    gl.updateModelMatrix(Matrix4f::identity()); // update uniforms after mode change
    VertexRecorder rec;

    glLineWidth(3.0f);
    rec.draw(GL_LINES);
}
