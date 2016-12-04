#ifndef WATERSYSTEM_H
#define WATERSYSTEM_H

#include <vector>

#include "particlesystem.h"

class WaterSystem : public ParticleSystem
{
public:
    WaterSystem();

    std::vector<Vector3f> evalF(std::vector<Vector3f> state) override;
    void draw(GLProgram&);
	float calculateKernel(float r);
	float calculateDensityOfParticle(float i, std::vector<Vector3f> state);
	float calculatePressureOfParticle(float i, std::vector<Vector3f> state);

	enum KernelType {
	  Poly6,
	  Spiky,
	  Viscosity
	};
    // inherits 
    // std::vector<Vector3f> m_vVecState;
private:
    std::vector<Vector3f> systemGrid};
#endif
