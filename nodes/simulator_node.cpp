#include <ros/ros.h>

#include "simu/SimulatedMass.h"
#include "simu/SimulatedVelocitySensor.h"
#include "simu/SimulationCore.h"

#include "simu/InitializeSimulation.h"

#include "argus_utils/utils/ParamUtils.h"

#include <unordered_map>

using namespace argus;

class SimulatorNode
{
public:

	SimulatorNode( ros::NodeHandle& nh, ros::NodeHandle& ph )
	{
		YAML::Node bodies;
		GetParam( ph, "bodies", bodies );
		YAML::Node::const_iterator iter;
		for( iter = bodies.begin(); iter!= bodies.end(); ++iter )
		{
			const std::string& name = iter->first.as<std::string>();
			YAML::Node info = iter->second;
			std::string type;
			GetParamRequired( info, "type", type );
			if( type == "mass" )
			{
				ros::NodeHandle mh( ph.resolveName( "bodies/" + name ) );
				_bodyRegistry[ name ] = std::make_shared<SimulatedMass>( mh );
			}
			else
			{
				throw std::invalid_argument( "Unknown body type: " + type );
			}
		}

		YAML::Node sensors;
		GetParam( ph, "sensors", sensors );
		for( iter = sensors.begin(); iter!= sensors.end(); ++iter )
		{
			const std::string& name = iter->first.as<std::string>();
			YAML::Node info = iter->second;
			std::string type, parent;
			GetParamRequired( info, "type", type );
			GetParamRequired( info, "parent", parent );
			if( type == "velocity_sensor" )
			{
				ros::NodeHandle mh( ph.resolveName( "sensors/" + name ) );
				_sensorRegistry[ name ] = std::make_shared<SimulatedVelocitySensor>
				    ( nh, mh, *_bodyRegistry.at(parent) );
			}
			else
			{
				throw std::invalid_argument( "Unknown sensor type: " + type );
			}
		}
	}

private:

	typedef std::unordered_map<std::string, SimulatedBody::Ptr> BodyRegistry;
	typedef std::unordered_map<std::string, SimulatedSensor::Ptr> SensorRegistry;
	BodyRegistry _bodyRegistry;
	SensorRegistry _sensorRegistry;

	bool InitializeCallback( simu::InitializeSimulation::Request& req,
	                         simu::InitializeSimulation::Response& res )
	{
		BodyRegistry::iterator biter;
		for( biter = _bodyRegistry.begin(); biter != _bodyRegistry.end(); ++biter )
		{
			biter->second->Initialize( req.sim_time );
		}

		SensorRegistry::iterator siter;
		for( siter = _sensorRegistry.begin(); siter != _sensorRegistry.end(); ++siter )
		{
			siter->second->Initialize( req.sim_time );
		}
		return true;
	}
};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "simulator_node" );
	ros::NodeHandle nh, ph( "~" );
	SimulatorNode node( nh, ph );
	ros::spin();
	return 0;
}