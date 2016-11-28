#pragma once

#include <ros/ros.h>
#include <memory>

#include <Eigen/StdVector>

#include "paraset/ParameterManager.hpp"

namespace argus
{

class SimulatedObject
{
public:

	typedef std::shared_ptr<SimulatedObject> Ptr;

	SimulatedObject();

	// Initialize state and set the first time
	virtual void Initialize( const ros::Time& now ) = 0;

	// Simulate until the specified time
	virtual void Tic( const ros::Time& now ) = 0;

};

class SimulatedBody
: public SimulatedObject
{
public:

	typedef std::shared_ptr<SimulatedBody> Ptr;

	SimulatedBody( ros::NodeHandle& ph );

	const std::string& GetFrameID() const;

	virtual PoseSE3 GetPose() const = 0;
	virtual PoseSE3::TangentVector GetVelocity() const = 0;

private:

	std::string _frameID;
};

class SimulatedSensor
: public SimulatedObject
{
public:

	typedef std::shared_ptr<SimulatedSensor> Ptr;
	
	SimulatedSensor( ros::NodeHandle& nh, ros::NodeHandle& ph );

	virtual void Initialize( const ros::Time& now );
	virtual void Tic( const ros::Time& now );

protected:

	// Take a reading and publish it
	virtual void Sample( const ros::Time& now ) = 0;

private:

	NumericParam _frequency;
	ros::Time _lastUpdate;
};

}