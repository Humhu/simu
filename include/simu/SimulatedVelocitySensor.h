#include "simu/SimuCommon.h"
#include "simu/NoiseGenerator.h"

#include "extrinsics_array/ExtrinsicsInterface.h"
#include "argus_utils/random/MultivariateGaussian.hpp"
#include "argus_utils/synchronization/SynchronizationTypes.h"

namespace argus
{

class SimulatedVelocitySensor
: public SimulatedSensor
{
public:

	SimulatedVelocitySensor( ros::NodeHandle& nh,
	                         ros::NodeHandle& ph,
	                         SimulatedBody& parent );

protected:

	virtual void Sample( const ros::Time& now );

private:

	SimulatedBody& _parent;

	ExtrinsicsInterface _extrinsicsManager;
	ros::Publisher _velPub;
	NoiseGenerator _noise;

	bool _enableNoise;
	std::string _frameID;
};

}