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
const float K_GAS_CONSTANT = 10.0;
const float ENVIRONMENTAL_PRESSURE = 10.0;
const float K_DRAG = 0.1;

enum KernelType {
  Poly6,
  Spiky,
  Viscosity
};

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
    // TODO: implement evalF
    //  - gravity
    //  - viscous drag

    for (int i=0; i< 1; ++i) {
	Vector3f velocity = getVelocityAt(state, i);
	
	Vector3f fGravity(0.0, MASS * GRAVITY, 0.0);
	Vector3f fDrag = -K_DRAG * velocity;

	Vector3f totalForce;
	if (state[0].y() < 0)
	    totalForce = -fGravity + fDrag;
	else
	    totalForce = fGravity + fDrag;

	Vector3f acceleration = totalForce / MASS;
	
	f.push_back(velocity);
	f.push_back(acceleration);
    }

    for (int i=1; i<(int) state.size()/2; ++i) {
      f.push_back(Vector3f());
      f.push_back(Vector3f());
    }
    //cout << f.size() << endl;

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

float WaterSystem::calculateKernel(float r) {
  // poly6 kernel
  float d = 10.0;

  if (r < 0 || r > d) {
    return 0;
  }

  return 315 / (64 * c_pi * pow(d, 9)) * pow((pow(d, 2) - pow(r, 2)), 3);
}

float WaterSystem::calculateDensityOfParticle(float i, std::vector<Vector3f> state) {
    float density = 0;

    Vector3f x_i = state.at(i);

    for (int j = 0; j<(int) state.size(); ++j) {
    Vector3f x_j = state.at(j);
    float r = (x_i - x_j).abs();
    float W = calculateKernel(r);
    density += MASS * W;
  }

  return density;
}

float WaterSystem::calculatePressureOfParticle(float i, std::vector<Vector3f> state) {
  float density = calculateDensityOfParticle(i, state);
  return K_GAS_CONSTANT * (density - ENVIRONMENTAL_PRESSURE);
}
