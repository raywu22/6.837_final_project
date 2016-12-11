#include "watersystem.h"

#include <cassert>
#include "camera.h"
#include "vertexrecorder.h"
#include <iostream>

using namespace std;

const float c_pi = 3.14159265358979323846f;

const float TANK_STANDARD_MINUS = -1.0f;
const float TANK_STANDARD_PLUS = 1.0f;

const float TANK_START_X = TANK_STANDARD_MINUS;
const float TANK_END_X = TANK_STANDARD_PLUS;
const float TANK_START_Y = TANK_STANDARD_MINUS;
const float TANK_END_Y = 0.0f;
const float TANK_START_Z = TANK_STANDARD_MINUS;
const float TANK_END_Z = TANK_STANDARD_PLUS;

const float GRID_START_X = TANK_STANDARD_MINUS;
const float GRID_END_X = TANK_STANDARD_PLUS;
const float GRID_START_Y = TANK_STANDARD_MINUS;
const float GRID_END_Y = 1.0f;
const float GRID_START_Z = TANK_STANDARD_MINUS;
const float GRID_END_Z = TANK_STANDARD_PLUS;

const float PARTICLE_SPACING = 0.25f;
const float CELL_SPACING = 0.25f;
const float NUM_X_INDICES = (GRID_END_X - GRID_START_X) / CELL_SPACING;
const float NUM_Y_INDICES = (GRID_END_Y - GRID_START_Y) / CELL_SPACING;
const float NUM_Z_INDICES = (GRID_END_Z - GRID_START_Z) / CELL_SPACING;
const float NUM_TOTAL_INDICES = NUM_X_INDICES * NUM_Y_INDICES * NUM_Z_INDICES;

const float NEIGHBOR_RADIUS = 0.25f;

const float GRAVITY = 0.1;
const float MASS = 0.1;
const float H_KERNEL = 1.0;
const float K_GAS_CONSTANT = 1.0;
const float MU = 1.0;
const float REST_DENSITY = 1.0;

void WaterSystem::printGrid() {
  for (int i=0; i<(int) systemGrid.size(); ++i) {
    vector<int> cellElements = systemGrid.at(i);
    cout << "cell " << i << ": ";

    for (int j=0; j<(int) cellElements.size(); ++j) {
      cout << cellElements.at(j) << " ";
    }

    cout << endl;
  }
}

int WaterSystem::posToGridIndex(float x, float y, float z) {
    int xIndex = (x - GRID_START_X) / CELL_SPACING;
    int yIndex = (y - GRID_START_Y) / CELL_SPACING;
    int zIndex = (z - GRID_START_Z) / CELL_SPACING;
    return xIndex * NUM_Y_INDICES * NUM_Z_INDICES + yIndex * NUM_Z_INDICES + zIndex;
}

void WaterSystem::clearGrid() {
  for (int i = 0; i <(int) systemGrid.size(); i++)
    systemGrid[i] = vector<int>();
}

WaterSystem::WaterSystem()
{
    // single particle that is dropped
    vector<Vector3f> initialState;
    vector<vector<int>> initialGrid;
    
    for (float x = GRID_START_X; x < GRID_END_X; x += CELL_SPACING)
    for (float y = GRID_START_Y; y < GRID_END_Y; y += CELL_SPACING)
    for (float z = GRID_START_Z; z < GRID_END_Z; z += CELL_SPACING)
      initialGrid.push_back(vector<int>());
    
    int curStateIndex = 0;
    // particles that make up the water into which particle falls
    for (float x = TANK_START_X; x < TANK_END_X; x += PARTICLE_SPACING)
    for (float y = TANK_START_Y; y < TANK_END_Y; y += PARTICLE_SPACING)
    for (float z = TANK_START_Z; z < TANK_END_Z; z += PARTICLE_SPACING) {
       //populate the initial grid with current index
        int gridIndex = WaterSystem::posToGridIndex(x, y, z);
        Vector3f position = Vector3f(x, y, z);
        initialState.push_back(position);
        initialState.push_back(Vector3f(0, 0, 0));
        initialGrid[gridIndex].push_back(curStateIndex);
        curStateIndex++;
    }
    setState(initialState);
    setGrid(initialGrid);
}

void WaterSystem::updateGrid(std::vector<Vector3f> state){
    clearGrid();
    for (int stateIndex = 0; stateIndex <(int) state.size(); stateIndex += 2) {
        Vector3f pos = state[stateIndex];
        int gridIndex = WaterSystem::posToGridIndex(pos.x(), pos.y(), pos.z());
        if (gridIndex < 0 || gridIndex >= NUM_TOTAL_INDICES)
	         cout << "not in grid" << endl;
        else
	        systemGrid[gridIndex].push_back(stateIndex/2);
    }
}

std::vector<int> WaterSystem::getNeighbors(int i, std::vector<Vector3f> state) {
    Vector3f iPos = state[2 * i];
    std::vector<int> neighboringIndices = vector<int>();
   
    for (float x = iPos.x() - CELL_SPACING; x < iPos.x() + CELL_SPACING*2; x += CELL_SPACING)
    for (float y = iPos.y() - CELL_SPACING; y < iPos.y() + CELL_SPACING*2; y += CELL_SPACING)
    for (float z = iPos.z() - CELL_SPACING; z < iPos.z() + CELL_SPACING*2; z += CELL_SPACING) {
        int gridIndex = posToGridIndex(x, y, z);
        if (gridIndex >= 0 && gridIndex < NUM_TOTAL_INDICES) {
            for (int neighborIndex : systemGrid[gridIndex]) {
                Vector3f neighborDistance = state[2*neighborIndex] - iPos;
                if (neighborDistance.abs() <= NEIGHBOR_RADIUS)
                    neighboringIndices.push_back(neighborIndex);
            }
        }
	}
    return neighboringIndices;
}

std::vector<Vector3f> WaterSystem::evalF(std::vector<Vector3f> state)
{
    WaterSystem::updateGrid(state);
  
    std::vector<Vector3f> f;
    Vector3f fGravity = Vector3f(0.0, MASS * GRAVITY, 0.0);
    std::vector<std::vector<int>> particleNeighbors;
    std::vector<float> particleDensity;
    
    // first pass: calculate density of all particles
    for (int i=0; i<(int) state.size()/2; ++i) {
        std::vector<int> nearestParticles = WaterSystem::getNeighbors(i, state);
        float density = calculateDensityOfParticle(i, state, nearestParticles);
        
        particleNeighbors.push_back(nearestParticles);
        particleDensity.push_back(density);
    }
  //  cout << "done first pass" << endl;
  //  */
  //  // second pass: calculate forces
  //  for (int i=0; i<(int) state.size()/2; ++i) {
  //    Vector3f velocity = getVelocityAt(state, i);
  //    /*
  //    std::vector<int> nearestParticles = particleNeighbors.at(i);
  //    Vector3f fPressure = calculatePressureForceOnParticle(i, state, nearestParticles, particleDensity);
  //    Vector3f fViscosity = calculateViscosityForceOnParticle(i, state, nearestParticles, particleDensity);
  //    Vector3f fExternal = calculateExternalForceOnParticle();
  //
  //    Vector3f totalForce = fPressure + fViscosity + fGravity + fExternal;
  //    Vector3f acceleration = totalForce / MASS;*/
  //    Vector3f acceleration = fGravity / MASS;
  //
  //    f.push_back(velocity);
  //    f.push_back(acceleration);
  //  }
  //
  //  return f;
    vector<Vector3f> zeros;
    for (int i = 0; i < state.size() / 2; i++) {
        zeros.push_back(Vector3f(0, i / 1000.0f,0));
        zeros.push_back(Vector3f(0,0,0));
    }
    return zeros;
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
