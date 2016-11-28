#include "simu/SimuCommon.h"
#include "argus_utils/utils/ParamUtils.h"

namespace argus
{

SimulatedObject::SimulatedObject() {}

SimulatedBody::SimulatedBody( ros::NodeHandle& ph )
{
	GetParamRequired( ph, "frame_id", _frameID );
}

const std::string& SimulatedBody::GetFrameID() const
{
	return _frameID;
}

SimulatedSensor::SimulatedSensor( ros::NodeHandle& nh,
                                  ros::NodeHandle& ph )
{
	double initFreq;
	GetParamRequired( ph, "frequency", initFreq );
	_frequency.Initialize( nh, initFreq, "frequency",
	                       "Sensor sampling frequency" );
}

void SimulatedSensor::Initialize( const ros::Time& now )
{
	_lastUpdate = now;
}

void SimulatedSensor::Tic( const ros::Time& now )
{
	if( now < _lastUpdate + ros::Duration( 1.0/_frequency ) )
	{
		return;
	}
	Sample( now );
}

}