#include "ballsystem.h"

#include <cassert>
#include "camera.h"
#include "vertexrecorder.h"

// TODO adjust to number of particles.
const int NUM_PARTICLES = 4;
const float mass = 1;
const float drag_constant = 0.5;

const float sphere_radius = 0.5f;
const Vector3f COLLISION_COLOR(0.3f, 0.5f, 0.5f);
std::vector<bool> collided(NUM_PARTICLES, false);

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
        Vector3f position = Vector3f(rand_uniform(-1.0f, 1.0f), rand_uniform(-1.0f, 1.0f), rand_uniform(-1.0f, 1.0f));
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


    // even position - velocity; odd position - acceleration
    std::vector<Vector3f> f(state.size(), Vector3f(0, 0, 0));

    for (int i=0; i<_spheres.size(); i+=1) {  //position and velocity stored in same
        Vector3f vel = state[2*i+1];

        f[i*2] += vel; // derivative of position is velocity

        Vector3f net_force = Vector3f(0, 0, 0);
        //  - gravity
        net_force[1] = net_force[1] - 9.8 * mass;

        //  - viscous drag
        net_force = net_force - drag_constant * vel;

        // Collision detection -- stop ball movement as collision detected
        //TODO: collision resolution
        for (int j=i+1; j<_spheres.size(); j+=1){
            if (_spheres[i].intersectsSphere(_spheres[j])){
                // TODO: resolve collision
                // change color if collided
                collided[i] = true;
                collided[j] = true;
            }
        }

        f[i*2+1] = (net_force/mass);
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
        if (collided[int(i/2)]){
            gl.updateMaterial(COLLISION_COLOR);
        }
        drawSphere(sphere_radius, 10, 10);
        gl.updateMaterial(PENDULUM_COLOR);
        
    }

    gl.disableLighting();
    gl.updateModelMatrix(Matrix4f::identity()); // update uniforms after mode change
    VertexRecorder rec;

    glLineWidth(3.0f);
    rec.draw(GL_LINES);
}
