#include "clothsystem.h"
#include "camera.h"
#include "vertexrecorder.h"
#include <iostream>

using namespace std;

 // your system should at least contain 8x8 particles.
const int W = 8;
const int H = 8;
const float MASS = 0.1;
const float K_DRAG = 0.5;
const float K_STRUCTURAL_SPRING = 50.0;
const float K_SHEAR_SPRING = 50.0;
const float K_FLEXION_SPRING = 50.0;
const float STRUCTURAL_REST_LENGTH = 0.2;
const float SHEAR_REST_LENGTH = sqrt(pow(0.2,2)*2);
const float FLEXION_REST_LENGTH = 0.4;
const float GRAVITY = -9.8;

ClothSystem::ClothSystem()
{
    // TODO 5. Initialize m_vVecState with cloth particles. 
    // You can again use rand_uniform(lo, hi) to make things a bit more interesting
  vector<Vector3f> initialState;

  Vector3f position(0.4, 1, 0);
  Vector3f velocity(0, 0, 0);

  for (int i=0; i<W*H; ++i) {
    if (i%W == 0) {
      position = Vector3f(0.4, 1, 0) - (i/W)*Vector3f(0, 0.2, 0);
    } else {
      position += Vector3f(0.2, 0, 0);
    }

    initialState.push_back(position);
    initialState.push_back(velocity);
  }

  setState(initialState);
}


std::vector<Vector3f> ClothSystem::evalF(std::vector<Vector3f> state)
{
  //cerr << "eval f start" << endl;
    std::vector<Vector3f> f;
    // TODO 5. implement evalF
    // - gravity
    // - viscous drag
    // - structural springs
    // - shear springs
    // - flexion springs
    springs.clear();

    for (int i=0; i<(int) state.size()/2; ++i) {
      Vector3f pointi = getPositionAt(state, i);
      Vector3f velocity = getVelocityAt(state, i);

      Vector3f pointj;
      Vector3f distance;

      Vector3f fGravity(0.0, MASS * GRAVITY, 0.0);
      Vector3f fDrag = -K_DRAG * velocity;
      Vector3f fStructuralLeft = Vector3f();
      Vector3f fStructuralRight = Vector3f();
      Vector3f fStructuralUp = Vector3f();
      Vector3f fStructuralDown = Vector3f();
      Vector3f fShearUpLeft = Vector3f();
      Vector3f fShearUpRight = Vector3f();
      Vector3f fShearDownLeft = Vector3f();
      Vector3f fShearDownRight = Vector3f();
      Vector3f fFlexionLeft = Vector3f();
      Vector3f fFlexionRight = Vector3f();
      Vector3f fFlexionUp = Vector3f();
      Vector3f fFlexionDown = Vector3f();
     
      // if particle isn't in the first column, there's a structural spring to the left
      if (i%W != 0) {
	pointj = getPositionAt(state, i-1);
	distance = pointi - pointj;
	fStructuralLeft = -K_STRUCTURAL_SPRING * (distance.abs() - STRUCTURAL_REST_LENGTH) * (distance / distance.abs());
	springs.push_back(Vector2f(i, i-1));

	// if particle isn't in the first column or top row, there's a shear spring up and left
	if (i >= W) {
	  pointj = getPositionAt(state, i-W-1);
	  distance = pointi - pointj;
	  fShearUpLeft = -K_SHEAR_SPRING * (distance.abs() - SHEAR_REST_LENGTH) * (distance / distance.abs());
	  springs.push_back(Vector2f(i, i-W-1));
	}

	// if particle isn't in the first column or bottom row, there's a shear spring down and left
	if (i < W*(H-1)) {
	  pointj = getPositionAt(state, i+W-1);
	  distance = pointi - pointj;
	  fShearDownLeft = -K_SHEAR_SPRING * (distance.abs() - SHEAR_REST_LENGTH) * (distance / distance.abs());
	  springs.push_back(Vector2f(i, i+W-1));
	}

	// if particle isn't in first or second column, there's a horizontal flexion spring to the left
	if (i%W != 1) {
	  pointj = getPositionAt(state, i-2);
	  distance = pointi - pointj;
	  fFlexionLeft = -K_FLEXION_SPRING * (distance.abs() - FLEXION_REST_LENGTH) * (distance / distance.abs());
	  springs.push_back(Vector2f(i, i-2));
	}
      }

      // if particle isn't in the last column, there's a structural spring to the right
      if (i%W != W-1) {
	pointj = getPositionAt(state, i+1);
	distance = pointi - pointj;
	fStructuralRight = -K_STRUCTURAL_SPRING * (distance.abs() - STRUCTURAL_REST_LENGTH) * (distance / distance.abs());
	
	// if particle isn't in the last column or top row, there's a shear spring up and right
	if (i >= W) {
	  pointj = getPositionAt(state, i-W+1);
	  distance = pointi - pointj;
	  fShearUpRight = -K_SHEAR_SPRING * (distance.abs() - SHEAR_REST_LENGTH) * (distance / distance.abs());
	}

	// if particle isn't in the last column or bottom row, there's a shear spring down and right
	if (i < W*(H-1)) {
	  pointj = getPositionAt(state, i+W+1);
	  distance = pointi - pointj;
	  fShearDownRight = -K_SHEAR_SPRING * (distance.abs() - SHEAR_REST_LENGTH) * (distance / distance.abs());
	}
	
	// if particle isn't in last or next-to-last column, there's a horizontal flexion spring to the right
	if (i%W != W-2) {
	  pointj = getPositionAt(state, i+2);
	  distance = pointi - pointj;
	  fFlexionRight = -K_FLEXION_SPRING * (distance.abs() - FLEXION_REST_LENGTH) * (distance / distance.abs());
	}
      }

      // if particle isn't in the top row, there's a structural spring up
      if (i >= W) {
	pointj = getPositionAt(state, i-W);
	distance = pointi - pointj;
	fStructuralUp = -K_STRUCTURAL_SPRING * (distance.abs() - STRUCTURAL_REST_LENGTH) * (distance / distance.abs());
	springs.push_back(Vector2f(i, i-W));
	
	// if particle isn't in top or second row, there's a vertical flexion spring up
	if (i >= 2*W) {
	  pointj = getPositionAt(state, i-2*W);
	  distance = pointi - pointj;
	  fFlexionUp = -K_FLEXION_SPRING * (distance.abs() - FLEXION_REST_LENGTH) * (distance / distance.abs());
	  springs.push_back(Vector2f(i, i-2*W));
        }
      }
      
      // if particle isn't in the bottom row, there's a structural spring down
      if (i < W*(H-1)) {
	pointj = getPositionAt(state, i+W);
	distance = pointi - pointj;
	fStructuralDown = -K_STRUCTURAL_SPRING * (distance.abs() - STRUCTURAL_REST_LENGTH) * (distance / distance.abs());
	
	// if particle isn't in last or next-to-last row, there's a vertical flexion spring down
	if (i < W*(H-2)) {
	  pointj = getPositionAt(state, i+2*W);
	  distance = pointi - pointj;
	  fFlexionDown = -K_FLEXION_SPRING * (distance.abs() - FLEXION_REST_LENGTH) * (distance / distance.abs());
	}
      }
      
      Vector3f totalFStructural = fStructuralLeft + fStructuralRight + fStructuralUp + fStructuralDown;
      Vector3f totalFShear = fShearUpLeft + fShearUpRight + fShearDownLeft + fShearDownRight;
      Vector3f totalFFlexion = fFlexionLeft + fFlexionRight + fFlexionUp + fFlexionDown;
      Vector3f totalForce = fGravity + fDrag + totalFStructural + totalFShear + totalFFlexion;
      Vector3f acceleration = totalForce / MASS;
      
      if (i == 0 || i == W-1) {
	velocity = Vector3f();
	acceleration = Vector3f();
      }

      f.push_back(velocity);
      f.push_back(acceleration);
    }
    //cerr << "eval f done" << endl;
    return f;
}


void ClothSystem::draw(GLProgram& gl)
{
    //TODO 5: render the system 
    //         - ie draw the particles as little spheres
    //         - or draw the springs as little lines or cylinders
    //         - or draw wireframe mesh

    const Vector3f CLOTH_COLOR(0.9f, 0.9f, 0.9f);
    gl.updateMaterial(CLOTH_COLOR);
    /*
    // EXAMPLE for how to render cloth particles.
    //  - you should replace this code.
    float w = 0.2f;
    Vector3f O(0.4f, 1, 0);
    gl.updateModelMatrix(Matrix4f::translation(O));
    drawSphere(0.04f, 8, 8);
    gl.updateModelMatrix(Matrix4f::translation(O + Vector3f(w, 0, 0)));
    drawSphere(0.04f, 8, 8);
    gl.updateModelMatrix(Matrix4f::translation(O + Vector3f(w, -w, 0)));
    drawSphere(0.04f, 8, 8);
    gl.updateModelMatrix(Matrix4f::translation(O + Vector3f(0, -w, 0)));
    drawSphere(0.04f, 8, 8);

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
    rec.record(O, CLOTH_COLOR);
    rec.record(O + Vector3f(w, 0, 0), CLOTH_COLOR);
    rec.record(O, CLOTH_COLOR);
    rec.record(O + Vector3f(0, -w, 0), CLOTH_COLOR);

    rec.record(O + Vector3f(w, 0, 0), CLOTH_COLOR);
    rec.record(O + Vector3f(w, -w, 0), CLOTH_COLOR);

    rec.record(O + Vector3f(0, -w, 0), CLOTH_COLOR);
    rec.record(O + Vector3f(w, -w, 0), CLOTH_COLOR);
    glLineWidth(3.0f);
    rec.draw(GL_LINES);

    gl.enableLighting(); // reset to default lighting model
    // EXAMPLE END*/

    vector<Vector3f> currentState = getState();

    for (int i=0; i<(int) currentState.size()/2; ++i) {
      gl.updateModelMatrix(Matrix4f::translation(getPositionAt(currentState, i)));
      drawSphere(0.04f, 8, 8);
    }

    gl.disableLighting();
    gl.updateModelMatrix(Matrix4f::identity());
    VertexRecorder rec;
    Vector3f O(0.4f, 1, 0);
    vector<Vector2f> springs = getSprings();

    for (int i=0; i<(int) springs.size(); ++i) {
      rec.record(getPositionAt(currentState, springs.at(i).x()), CLOTH_COLOR);
      rec.record(getPositionAt(currentState, springs.at(i).y()), CLOTH_COLOR);
    }
    
    glLineWidth(3.0f);
    rec.draw(GL_LINES);
    
    gl.enableLighting();
}

