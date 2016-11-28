#include "simu/SimulationCore.h"
#include "argus_utils/utils/ParamUtils.h"
#include <boost/foreach.hpp>

namespace argus
{

SimulationCore::SimulationCore( ros::NodeHandle& ph )
: _inited( false )
{
	GetParam( ph, "time_scale", _timeScale, 1.0 );
	double ticFreq;
	GetParamRequired( ph, "tic_frequency", ticFreq );
	_timer = ph.createTimer( ros::Duration( 1.0/ticFreq ),
	                         &SimulationCore::TimerCallback,
	                         this,
	                         false, // Not one-shot
	                         true ); // No autostart
}

void SimulationCore::RegisterObject( const SimulatedObject::Ptr& obj )
{
	_objects.emplace_back( obj );
}

void SimulationCore::Start( const ros::Time& now )
{
	BOOST_FOREACH( const SimulatedObject::Ptr& obj, _objects )
	{
		obj->Initialize( now );
	}
	_timer.start();
}

void SimulationCore::TimerCallback( const ros::TimerEvent& event )
{
	if( !_inited )
	{
		_startTime = event.current_real;
		_inited = true;
	}

	double dt = _timeScale * ( event.current_real - _startTime ).toSec();
	ros::Time simNow = _startTime + ros::Duration( dt );

	BOOST_FOREACH( const SimulatedObject::Ptr& obj, _objects )
	{
		obj->Tic( simNow );
	}
}

}