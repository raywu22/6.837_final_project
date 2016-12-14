#include "timestepper.h"

#include <cstdio>
#include <iostream>

using namespace std;

void ForwardEuler::takeStep(ParticleSystem* particleSystem, float stepSize)
{
   //TODO: See handout 3.1 
  vector<Vector3f> newState;
  vector<Vector3f> oldState = particleSystem->getState();
  vector<Vector3f> f0derivative = particleSystem->evalF(oldState);

  for (int i=0; i<(int) f0derivative.size(); ++i) {
    newState.push_back(oldState.at(i) + stepSize*f0derivative.at(i));
  }

  particleSystem->setState(newState);
}

void Trapezoidal::takeStep(ParticleSystem* particleSystem, float stepSize)
{
   //TODO: See handout 3.1 
  vector<Vector3f> newState;
  vector<Vector3f> oldState = particleSystem->getState();
  vector<Vector3f> f0derivative = particleSystem->evalF(oldState);
  vector<Vector3f> intermediateState;

  for (int i=0; i<(int) f0derivative.size(); ++i) {
    intermediateState.push_back(oldState.at(i) + stepSize*f0derivative.at(i));
  }

  vector<Vector3f> f1derivative = particleSystem->evalF(intermediateState);

  for (int i=0; i<(int) f1derivative.size(); ++i) {
    newState.push_back(oldState.at(i) + stepSize/2.0*(f0derivative.at(i)+f1derivative.at(i)));
  }

  particleSystem->setState(newState);
}


void RK4::takeStep(ParticleSystem* particleSystem, float stepSize)
{
   //TODO: See handout 4.4
  vector<Vector3f> newState;
  vector<Vector3f> oldState = particleSystem->getState();
  vector<Vector3f> k1 = particleSystem->evalF(oldState);
  vector<Vector3f> intermediateState;

  for (int i=0; i<(int) k1.size(); ++i) {
    intermediateState.push_back(oldState.at(i) + stepSize/2.0*k1.at(i));
  }

  vector<Vector3f> k2 = particleSystem->evalF(intermediateState);

  intermediateState.clear();
  
  for (int i=0; i<(int) k2.size(); ++i) {
    intermediateState.push_back(oldState.at(i) + stepSize/2.0*k2.at(i));
  }

  vector<Vector3f> k3 = particleSystem->evalF(intermediateState);

  intermediateState.clear();

  for (int i=0; i<(int) k3.size(); ++i) {
    intermediateState.push_back(oldState.at(i) + stepSize*k3.at(i));
  }

  vector<Vector3f> k4 = particleSystem->evalF(intermediateState);

  for (int i=0; i<(int) oldState.size(); ++i) {
    newState.push_back(oldState.at(i) + stepSize/6.0*(k1.at(i) + 2*k2.at(i) + 2*k3.at(i) + k4.at(i)));
  }

  particleSystem->setState(newState);
}
