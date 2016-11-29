#include "simu/SimuCommon.h"
#include "argus_utils/utils/ParamUtils.h"

namespace argus
{

SimulatedObject::SimulatedObject() {}

SimulatedBody::SimulatedBody( ros::NodeHandle& ph,
                              const std::string& refFrame )
: _referenceFrame( refFrame )
{
	GetParamRequired( ph, "frame_id", _bodyFrame );
}

const std::string& SimulatedBody::GetBodyFrame() const
{
	return _bodyFrame;
}

const std::string& SimulatedBody::GetReferenceFrame() const
{
	return _referenceFrame;
}

SimulatedSensor::SimulatedSensor( ros::NodeHandle& ph )
{
	double initFreq;
	GetParamRequired( ph, "frequency", initFreq );
	_frequency.Initialize( ph, initFreq, "frequency",
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
	_lastUpdate = now;
	Sample( now );
}

}