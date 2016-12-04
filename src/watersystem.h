#ifndef WATERSYSTEM_H
#define WATERSYSTEM_H

#include <vector>

#include "particlesystem.h"

class WaterSystem : public ParticleSystem
{
public:
	enum KernelType {
	  Poly6,
	  Spiky,
	  Viscosity
	};

    WaterSystem();

    std::vector<Vector3f> evalF(std::vector<Vector3f> state) override;
    void draw(GLProgram&);
	
    // inherits 
    // std::vector<Vector3f> m_vVecState;
private:
    std::vector<Vector3f> systemGrid};

	float calculateKernel(KernelType type, float r);
	float calculateDensityOfParticle(int i, std::vector<Vector3f> state, std::vector<int> nearestParticles);
	Vector3f calculatePressureForceOnParticle(int i, std::vector<Vector3f> state, std::vector<int> nearestParticles, std::vector<float> particleDensity);
	Vector3f calculateViscosityForceOnParticle(int i, std::vector<Vector3f> state, std::vector<int> nearestParticles, std::vector<float> particleDensity);
	Vector3f calculateExternalForceOnParticle();
#endif
