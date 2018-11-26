#include "clothsystem.h"
#include "camera.h"
#include "vertexrecorder.h"
#include <stdio.h>
#include <stdlib.h>

 // your system should at least contain 8x8 particles.
const int W = 8;
const int H = 8;

float x_min = -2;
float x_max = 2;
float y_min = -1;
float y_max = 1;

float delta_x = abs(x_max - x_min) / W;
float delta_y = abs(y_max - y_min) / H;

const float mass = 1;
const float drag_constant = 5;

const float structural_rest_length = 0.05;
const float structural_stiffness = 300;
const float shear_rest_length = 1;
const float shear_stiffness = 10;
const float flexion_rest_length = 0.5;
const float flexion_stiffness = 10;

const float wind_max = 200;
const float wind_min = -180;

ClothSystem::ClothSystem()
{
    // TODO 5. Initialize m_vVecState with cloth particles. 
    // You can again use rand_uniform(lo, hi) to make things a bit more interesting

    // left to right, up to down iteration
    for (int i=0; i<W; i++) {
        float y = y_max - i * delta_y;
        for (int u=0; u<H; u++) {
            float x = x_min + u * delta_x;
            float z = rand_uniform(-0.05, 0.05);

            // position and velocity
            m_vVecState.emplace_back(x, y, z);
            m_vVecState.emplace_back(rand_uniform(-0.05, 0.05), rand_uniform(-0.05, 0.05), rand_uniform(-0.05, 0.05));
        }
    }

    // structural springs
    for (int x=0; x<W; x++) {
        for (int y=0; y<H; y++) {
            int index = positionIndexOf(x, y);
            if (x != W-1) {
                structural.emplace_back(index, positionIndexOf(x + 1, y), structural_rest_length, structural_stiffness);
            }
            if (y != H-1) {
                structural.emplace_back(index, positionIndexOf(x, y + 1), structural_rest_length, structural_stiffness);
            }
        }
    }

    // shear springs
    for (int x=0; x<W; x++) {
        for (int y=0; y<H-1; y++) {
            int index = positionIndexOf(x, y);
            if (x != W-1) {
                shear.emplace_back(index, positionIndexOf(x + 1, y + 1), shear_rest_length, shear_stiffness);
            }
            if (x != 0) {
                shear.emplace_back(index, positionIndexOf(x - 1, y + 1), shear_rest_length, shear_stiffness);
            }
        }
    }

    // flexion springs
    for (int x=0; x<W; x++) {
        for (int y=0; y<H; y++) {
            int index = positionIndexOf(x, y);
            if (x < W-2) {
                flexion.emplace_back(index, positionIndexOf(x + 2, y), flexion_rest_length, flexion_stiffness);
            }
            if (y < H-2) {
                flexion.emplace_back(index, positionIndexOf(x, y + 2), flexion_rest_length, flexion_stiffness);
            }
        }
    }
}

int positionIndexOf(int i, int j) {
    return (i + W * j)*2;  // * 2 to account for velocity
}

std::vector<Vector3f> ClothSystem::evalF(std::vector<Vector3f> state)
{
    std::vector<Vector3f> f;
    // TODO 5. implement evalF

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

        //  - wind
        if (wind) {
            net_force[2] += rand_uniform(wind_min, wind_max);
        }

        f.push_back(net_force/mass);
    }

    // - structural springs
    f = springHelper(f, state, structural);
    // - shear springs
    f = springHelper(f, state, shear);
    // - flexion springs
    f = springHelper(f, state, flexion);


    // hold first and last of first row from moving
    f[0] = Vector3f(0, 0, 0);
    f[W*2-1] = Vector3f(0, 0, 0);

    return f;
}

std::vector<Vector3f> springHelper(std::vector<Vector3f> f, std::vector<Vector3f> state, std::vector<Spring> springs) {
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
    return f;
}


void ClothSystem::draw(GLProgram& gl)
{
    //TODO 5: render the system 
    //         - ie draw the particles as little spheres
    //         - or draw the springs as little lines or cylinders
    //         - or draw wireframe mesh
    std::vector<Vector3f> current = getState();

    const Vector3f CLOTH_COLOR(0.9f, 0.9f, 0.9f);
    gl.updateMaterial(CLOTH_COLOR);

    for (int i=0; i<current.size(); i+= 2) {
        Vector3f pos = current[i];

        gl.updateModelMatrix(Matrix4f::translation(pos));
        drawSphere(0.04f, 8, 8);
    }

    // EXAMPLE for how to render cloth particles.
    //  - you should replace this code.

    // EXAMPLE: This shows you how to render lines to debug the spring system.
    //
    //          You should replace this code.
    //
    //          Since lines don't have a clearly defined normal, we can't use
    //          a regular lighting model.
    //          GLprogram has a "color only" mode, where illumination
    //          is disabled, and you specify color directly as vertex attribute.
    //          Note: enableLighting/disableLighting invalidates uniforms,
    //          so you'll have to update the transformation/material parameters
    //          after a mode change.
    gl.disableLighting();
    gl.updateModelMatrix(Matrix4f::identity()); // update uniforms after mode change
    VertexRecorder rec;

    for (int i=0; i<this->structural.size(); i++) {
        Spring spr = this->structural[i];
        Vector3f pos1 = current[spr.end1];
        Vector3f pos2 = current[spr.end2];

        Vector3f diff = pos2 - pos1;

        rec.record(pos1, CLOTH_COLOR);
        rec.record(pos1 + diff, CLOTH_COLOR);
    }
    glLineWidth(3.0f);
    rec.draw(GL_LINES);

    gl.enableLighting(); // reset to default lighting model
    // EXAMPLE END
}

void ClothSystem::toggleWind() {
    wind = !wind;
}
