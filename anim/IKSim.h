#pragma once

#include "BaseSimulator.h"
#include "BaseSystem.h"
//#include "Hermite.h"
#include "HermiteSystem.h"
#include "Bob.h"
#include <Eigen/Dense>

#define SPEED 3

class IKSim : public BaseSimulator
{
protected:

	double delta_t;
	double prev_t;

	BaseSystem* m_object;
	double param[2]; // pass parameters to the system

	HermiteSystem* hermite;
	double hermite_t;

	Vector pTargetP;
	Vector velocity;
	boolean transition = true;

	double initial;
	double increment;

	boolean fileLoaded;

public:

	IKSim(const std::string& name, BaseSystem* target);

	int step(double time);
	int init(double time);

	int command(int argc, myCONST_SPEC char** argv);

	void setHermite(HermiteSystem* target);

};
