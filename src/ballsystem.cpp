#include "ballsystem.h"

#include <cassert>
#include "camera.h"
#include "vertexrecorder.h"

// TODO adjust to number of particles.
const int NUM_PARTICLES = 5;
const float mass = 1;
const float drag_constant = 0.5;

const float sphere_radius = 0.3f;
const Vector3f COLLISION_COLOR(0.3f, 0.5f, 0.5f);
const Vector3f FLOOR_COLOR(1.0f, 1.0f, 1.0f);
const float rest = 1.0f;
const float k = 30;


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
    std::vector<Vector3f> pos = getPos(state);
    std::vector<Vector3f> velocity = getVelocity(state);
    
    // need to first update sphere positions to the particles (not handled during time step)
    for (int i=0; i<_spheres.size(); i+=1) {
        Vector3f current_position = pos[i];
        _spheres[i].updateCenter(current_position);
    }


    // even position - velocity; odd position - acceleration
    std::vector<Vector3f> f(state.size(), Vector3f(0, 0, 0));
    // modeling collision resolution as spring between two colliding balls
    // collided[i] - vector of balls colliding with ball i
    std::vector<std::vector<int>> collided;
    collided.resize(_spheres.size());
    

    // detect which balls might collide
    // if two ball collides, insert a spring in between
    // TODO: resolve wall collision
    for (int i=0; i<_spheres.size(); i+=1) {  //position and velocity stored in same
        // Collision detection
        for (int j=i+1; j<_spheres.size(); j+=1){
            if (_spheres[i].intersectsSphere(_spheres[j])){
                // TODO: resolve collision
                // change color if collided
//                _collided[i] = true;
//                _collided[j] = true;
                collided[i].push_back(j);
                collided[j].push_back(i);
            }
        }

        for (int j=0; j<_walls.size(); j+=1) {
            Hit hit = Hit();
            if (_spheres[i].intersectsWall(_walls[j], hit)) {
                _collided[i] = true;
                
            }
        }

        // Stop movement on floor collision
        if (_collided[i]) {
            f[i*2] = Vector3f(0, 0, 0);
            f[i*2+1] = Vector3f(0, 0, 0);
            continue;
        }


        Vector3f vel = velocity[i];

        f[i*2] += vel; // derivative of position is velocity

        Vector3f net_force = Vector3f(0, 0, 0);
        //  - gravity
        net_force[1] = net_force[1] - 9.8 * mass;
//        // - collision spring
//        Vector3f spring = Vector3f(0);
//        for (int m=0; m < collided[i].size(); m += 1){
//            int n = collided[i][m];
//            Vector3f diff = pos[i]-pos[n];
//            spring += -k * (diff.abs()-rest) * diff.normalized();
//        }
        
        // resolve collision
        for (int m=0; m < collided[i].size(); m += 1){
            int n = collided[i][m];
            Vector3f diff = (pos[i]-pos[n]).normalized();
            
            float x1 = Vector3f::dot(diff, vel);
            Vector3f thisVel = velocity[i];
            Vector3f v1x = diff * x1;
            Vector3f v1y = thisVel - v1x;
            
            diff = -1 * diff;
            Vector3f otherVel = velocity[n];
            float x2 = Vector3f::dot(diff, otherVel);
            Vector3f v2x = diff * x2;
            Vector3f v2y = otherVel - v2x;
            
            float m1 = mass;
            float m2 = mass;
            float combinedMass = m1 + m2;
            Vector3f newThisVel = (v1x * ((m1 - m2) / combinedMass)) + (v2x * ((2.0f * m2) / combinedMass)) + v1y;
            Vector3f newOtherVel = (v1x * ((2.0f * m1) / combinedMass)) + (v2x * ((m2 - m1) / combinedMass)) + v2y;
            
            f[i*2] = newThisVel;
            velocity[i] = newThisVel;
            f[n*2] = newOtherVel;
            velocity[n] = newOtherVel; 
        }

        //  - viscous drag
        net_force = net_force - drag_constant * vel;

        f[i*2+1] = (net_force/mass);
    }

    return f;
}

// returns vector of position
std::vector<Vector3f> BallSystem::getPos(std::vector<Vector3f> state)
{
    std::vector<Vector3f> pos;
    for (unsigned i=0; i<state.size(); i+=2){
        pos.push_back(state[i]);
        //        std::cout<< state[i][0]<< "," << state[i][1] << "," << state[i][2] << std::endl;
    }
    return pos;
}

// returns vector of velocity
std::vector<Vector3f> BallSystem::getVelocity(std::vector<Vector3f> state)
{
    std::vector<Vector3f> velocity;
    for (unsigned i=1; i<state.size(); i+=2){
        velocity.push_back(state[i]);
    }
    return velocity;
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
