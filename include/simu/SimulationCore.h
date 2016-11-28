#pragma once

#include "simu/SimuCommon.h"
#include <vector>

namespace argus
{

// Primary spinner for all simulation classes
class SimulationCore
{
public:

	SimulationCore( ros::NodeHandle& ph );

	void RegisterObject( const SimulatedObject::Ptr& obj );

	void Start( const ros::Time& now );

private:

	ros::Timer _timer;

	bool _inited;
	ros::Time _startTime;
	double _timeScale;

	std::vector<SimulatedObject::Ptr> _objects;

	void TimerCallback( const ros::TimerEvent& event );

};

}