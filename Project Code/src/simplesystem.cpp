#include "simplesystem.h"

#include "camera.h"
#include "vertexrecorder.h"

using namespace std;

SimpleSystem::SimpleSystem()
{
    // TODO 3.2 initialize the simple system
  vector<Vector3f> initialState;

  Vector3f initialPosition(0.5f, 0.0f, 0.0f);
  Vector3f initialVelocity(0.0f, -0.5f, 0.0f);

  initialState.push_back(initialPosition);
  initialState.push_back(initialVelocity);
  
  setState(initialState);
}

std::vector<Vector3f> SimpleSystem::evalF(std::vector<Vector3f> state)
{
    std::vector<Vector3f> f;

    // TODO 3.2: implement evalF
    // for a given state, evaluate f(X,t)
    for (int i=0; i<(int) state.size()/2; ++i) {
      f.push_back(-1.0*getVelocityAt(state, i));
      f.push_back(getPositionAt(state, i));
    }

    return f;
}

// render the system (ie draw the particles)
void SimpleSystem::draw(GLProgram& gl)
{

    // TODO 3.2: draw the particle. 
    //           we provide code that draws a static sphere.
    //           you should replace it with your own
    //           drawing code.
    //           In this assignment, you must manage two
    //           kinds of uniforms before you draw
    //            1. Update material uniforms (color)
    //            2. Update transform uniforms
    //           GLProgram is a helper object that has
    //           methods to set the uniform state.

    const Vector3f PARTICLE_COLOR(0.4f, 0.7f, 1.0f);
    gl.updateMaterial(PARTICLE_COLOR);
    Vector3f pos(getPositionAt(getState(), 0)); //YOUR PARTICLE POSITION
    gl.updateModelMatrix(Matrix4f::translation(pos));
    drawSphere(0.075f, 10, 10);
}
