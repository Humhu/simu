#pragma once

#include <ros/ros.h>
#include "argus_utils/random/MultivariateGaussian.hpp"
#include "simu/SetNoiseProperties.h"

namespace argus
{

class NoiseGenerator
{
public:

	NoiseGenerator();
	void Initialize( ros::NodeHandle& ph, unsigned int dim );

	VectorType Sample();
	MultivariateGaussian<>& Distribution();
	const MultivariateGaussian<>& Distribution() const;

private:

	MultivariateGaussian<> _generator;
	bool _enabled;
	ros::ServiceServer _server;

	bool NoiseCallback( simu::SetNoiseProperties::Request& req,
	                    simu::SetNoiseProperties::Response& res );
};

}