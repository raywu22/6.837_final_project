#include "watersystem.h"

#include <cassert>
#include "camera.h"
#include "vertexrecorder.h"
#include <iostream>

using namespace std;

const float c_pi = 3.14159265358979323846f;

const float TANK_WIDTH = 10;
const float TANK_LENGTH = 10;
const float TANK_HEIGHT = 5;

const float GRAVITY = -9.8;
const float MASS = 0.1;
const float H_KERNEL = 10.0;
const float K_GAS_CONSTANT = 10.0;
const float MU = 10.0;
const float REST_DENSITY = 10.0;
const float K_DRAG = 0.1;

WaterSystem::WaterSystem()
{
  // single particle that is dropped
  vector<Vector3f> initialState;

  Vector3f position(0.0, 1.0, 0.0);
  Vector3f velocity(0, 0, 0);

  initialState.push_back(position);
  initialState.push_back(velocity);

  // particles that make up the water into which particle falls
  for (int i=-TANK_WIDTH/2; i<TANK_WIDTH/2; ++i) {
    for (int j=0; j>-TANK_HEIGHT; --j) {
      for (int k=-TANK_LENGTH/2; k<TANK_LENGTH/2; ++k) {
	position = Vector3f(i/5.0, j/5.0, k/5.0);
	initialState.push_back(position);
	initialState.push_back(velocity);
      }
    }
  }

  setState(initialState);
}


std::vector<Vector3f> WaterSystem::evalF(std::vector<Vector3f> state)
{
  std::vector<Vector3f> f;
  Vector3f fGravity = Vector3f(0.0, MASS * GRAVITY, 0.0);
  std::vector<std::vector<int>> particleNeighbors;
  std::vector<float> particleDensity;

  // first pass: calculate density of all particles
  for (int i=0; i<(int) state.size()/2; ++i) {
    std::vector<int> nearestParticles = neighborsOfParticle(i, state);
    float density = calculateDensityOfParticle(i, state, nearestParticles);
    
    particleNeighbors.push_back(nearestParticles);
    particleDensity.push_back(density);
  }
    
  // second pass: calculate forces
  for (int i=0; i<(int) state.size()/2; ++i) {
    Vector3f velocity = getVelocityAt(state, i);
    std::vector<int> nearestParticles = particleNeighbors.at(i);

    Vector3f fPressure = calculatePressureForceOnParticle(i, state, nearestParticles, particleDensity);
    Vector3f fViscosity = calculateViscosityForceOnParticle(i, state, nearestParticles, particleDensity);
    Vector3f fExternal = calculateExternalForceOnParticle();

    Vector3f totalForce = fPressure + fViscosity + fGravity + fExternal;
    Vector3f acceleration = totalForce / MASS;

    f.push_back(velocity);
    f.push_back(acceleration);
  }

  return f;
}

// render the system (ie draw the particles)
void WaterSystem::draw(GLProgram& gl)
{
    const Vector3f PENDULUM_COLOR(0.73f, 0.0f, 0.83f);
    gl.updateMaterial(PENDULUM_COLOR);

    // TODO 4.2, 4.3

    // example code. Replace with your own drawing  code
    //gl.updateModelMatrix(Matrix4f::translation(Vector3f(-0.5, 1.0, 0)));
    vector<Vector3f> currentState = getState();
   
    for (int i=0; i<(int) currentState.size()/2; ++i) {
      gl.updateModelMatrix(Matrix4f::translation(getPositionAt(currentState, i)));
      drawSphere(0.075f, 10, 10);
    }
}

std::vector<int> WaterSystem::neighborsOfParticle(int i, std::vector<Vector3f> state) {
  std::vector<int> foo;
  return foo;
}


float WaterSystem::calculateKernel(WaterSystem::KernelType type, float r) {
  if (r < 0 || r > H_KERNEL) {
    return 0;
  }

  float numerator = 0;
  float denominator = 1;

  if (type == Poly6) {
    numerator = 315 * pow( pow(H_KERNEL, 2) - pow(r, 2), 3);
    denominator = 64 * c_pi * pow(H_KERNEL, 9);
  } else if (type == Spiky) {
    numerator = 15 * pow(H_KERNEL - r, 3);
    denominator = c_pi * pow(H_KERNEL, 6);
  } else if (type == Viscosity) {
    float term1 = -pow(r, 3) / (2 * pow(H_KERNEL, 3));
    float term2 = pow(r, 2) / pow(H_KERNEL, 2);
    float term3 = H_KERNEL / (2 * r);

    numerator = 15 * (term1 + term2 + term3 - 1);
    denominator = 2 * c_pi * pow(H_KERNEL, 3);
  }

  return numerator / denominator;
}

float WaterSystem::calculateDensityOfParticle(int i, std::vector<Vector3f> state, std::vector<int> nearestParticles) {
    float density = 0;
    Vector3f x_i = getPositionAt(state, i);

    for (int j = 0; j<(int) nearestParticles.size(); ++j) {
      float index = nearestParticles.at(j);
      Vector3f x_j = getPositionAt(state, index);
      float r = (x_i - x_j).abs();
      float W = calculateKernel(Poly6, r);

      density += MASS * W;
  }

  return density;
}

Vector3f WaterSystem::calculatePressureForceOnParticle(int i, std::vector<Vector3f> state, std::vector<int> nearestParticles, std::vector<float> particleDensity) {
  Vector3f force = Vector3f();
  Vector3f x_i = getPositionAt(state, i);
  float density_i = particleDensity.at(i);

  for (int j=0; j<(int) nearestParticles.size(); ++j) {
    Vector3f x_j = getPositionAt(state, nearestParticles.at(j));
    float density_j = particleDensity.at(j);
    Vector3f r_ij = x_i - x_j;
    float q_ij = r_ij.abs() / H_KERNEL;

    float numerator1 = density_i + density_j - 2 * REST_DENSITY;
    float numerator2 = pow((1 - q_ij), 2);
    force += MASS * numerator1 * numerator2 * r_ij / (density_j * q_ij);
  }

  force = force * 15 * K_GAS_CONSTANT / (c_pi * pow(H_KERNEL, 4));
  return force;
}

Vector3f WaterSystem::calculateViscosityForceOnParticle(int i, std::vector<Vector3f> state, std::vector<int> nearestParticles, std::vector<float> particleDensity) {
  Vector3f force = Vector3f();
  Vector3f x_i = getPositionAt(state, i);
  Vector3f v_i = getVelocityAt(state, i);

  for (int j=0; j<(int) nearestParticles.size(); ++j) {
    float index = nearestParticles.at(j);
    Vector3f x_j = getPositionAt(state, index);
    Vector3f v_j = getVelocityAt(state, index);
    float density_j = particleDensity.at(index);
    Vector3f r_ij = x_i - x_j;
    float q_ij = r_ij.abs() / H_KERNEL;

    force += MASS * (v_i - v_j) * (1 - q_ij) / density_j;
  }

  force = force * 40 * MU / (c_pi * pow(H_KERNEL, 4));
  return force;
}

Vector3f WaterSystem::calculateExternalForceOnParticle() {
  return Vector3f();
}
