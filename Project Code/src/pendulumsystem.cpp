#include "pendulumsystem.h"

#include <cassert>
#include "camera.h"
#include "vertexrecorder.h"
#include <iostream>

using namespace std;

// TODO adjust to number of particles.
const int NUM_PARTICLES = 4;
const float MASS = 1.0;
const float K_DRAG = 0.5;
const float K_SPRING = 30.0;
const float REST_LENGTH = 0.1;
const float GRAVITY = -9.8;

PendulumSystem::PendulumSystem()
{

    // TODO 4.2 Add particles for simple pendulum
    // TODO 4.3 Extend to multiple particles

    // To add a bit of randomness, use e.g.
    // float f = rand_uniform(-0.5f, 0.5f);
    // in your initial conditions.
  
  vector<Vector3f> initialState;

  Vector3f position(-0.5, 1.0, 0.0);
  Vector3f velocity(0, 0, 0);

  int counter = 0;

  do {
    initialState.push_back(position);
    initialState.push_back(velocity);

    position += Vector3f(rand_uniform(-0.5f, 0.5f), rand_uniform(-0.5f, 0.5f), rand_uniform(-0.5f, 0.5f));
    ++counter;
  } while(counter <= NUM_PARTICLES);

  setState(initialState);
}


std::vector<Vector3f> PendulumSystem::evalF(std::vector<Vector3f> state)
{
    std::vector<Vector3f> f;
    // TODO 4.1: implement evalF
    //  - gravity
    //  - viscous drag
    //  - springs

    for (int i=0; i<(int) state.size()/2; ++i) {
      if (i==0) {
	f.push_back(Vector3f());
	f.push_back(Vector3f());
      } else {
	Vector3f pointi = getPositionAt(state, i-1);
	Vector3f pointj = getPositionAt(state, i);
	Vector3f distance = pointj - pointi;
	Vector3f velocity = getVelocityAt(state, i);

	Vector3f fGravity(0.0, MASS * GRAVITY, 0.0);
	Vector3f fDrag = -K_DRAG * velocity;
	Vector3f fSpringDown = -K_SPRING * (distance.abs() - REST_LENGTH) * (distance / distance.abs());
	Vector3f fSpringUp = Vector3f();
	
	if (i < (int) state.size()/2 - 1) {
	  Vector3f pointk = getPositionAt(state, i+1);
	  Vector3f distance2 = pointj - pointk;

	  fSpringUp = -K_SPRING * (distance2.abs() - REST_LENGTH) * (distance2 / distance2.abs());
	}

	Vector3f totalForce = fGravity + fDrag + fSpringUp + fSpringDown;
	Vector3f acceleration = totalForce / MASS;

	f.push_back(velocity);
	f.push_back(acceleration);
      }
    }

    return f;
}

// render the system (ie draw the particles)
void PendulumSystem::draw(GLProgram& gl)
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
