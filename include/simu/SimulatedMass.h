#pragma once

#include "simu/SimuCommon.h"
#include "argus_utils/geometry/PoseSE3.h"
#include "argus_utils/random/MultivariateGaussian.hpp"

namespace argus
{

class SimulatedMass 
: public SimulatedBody
{
public:

	SimulatedMass( ros::NodeHandle& ph );

	// Initialize state and set the simulation time
	virtual void Initialize( const ros::Time& now );

	// Run the simulation up to the specified time
	virtual void Tic( const ros::Time& now );

	virtual PoseSE3 GetPose() const;

	virtual PoseSE3::TangentVector GetVelocity() const;

private:

	PoseSE3 _initialPose;
	PoseSE3::TangentVector _initialVelocity;

	bool _enableNoise;
	MultivariateGaussian<> _noiseGenerator;

	ros::Time _simTime;
	PoseSE3 _pose;
	PoseSE3::TangentVector _velocity;
};

}