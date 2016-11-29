#pragma once

#include "simu/SimuCommon.h"
#include "simu/NoiseGenerator.h"

#include "argus_utils/geometry/PoseSE3.h"
#include "argus_utils/random/MultivariateGaussian.hpp"

#include "simu/SetBodyPose.h"
#include "simu/SetBodyVelocity.h"
#include "geometry_msgs/Twist.h"

namespace argus
{

// Simulates a point mass moving in free space
// Publishes ground truth as a nav_msgs::Odometry topic
class SimulatedMass 
: public SimulatedBody
{
public:

	SimulatedMass( ros::NodeHandle& nh,
	               ros::NodeHandle& ph, 
	               const std::string& referenceFrame );

	// Initialize state and set the simulation time
	virtual void Initialize( const ros::Time& now );

	// Run the simulation up to the specified time
	virtual void Tic( const ros::Time& now );

	virtual PoseSE3 GetPose() const;

	virtual PoseSE3::TangentVector GetVelocity() const;

private:

	PoseSE3 _initialPose;
	PoseSE3::TangentVector _initialVelocity;

	ros::Subscriber _cmdSub;
	ros::Publisher _odomPub;

	bool _enableNoise;
	NoiseGenerator _poseNoise;
	NoiseGenerator _velocityNoise;
	ros::ServiceServer _poseServer;
	ros::ServiceServer _velocityServer;

	ros::Time _simTime;
	PoseSE3 _pose;
	PoseSE3::TangentVector _velocity;


	void CommandCallback( const geometry_msgs::Twist::ConstPtr& msg );
	bool PoseCallback( simu::SetBodyPose::Request& req,
	                   simu::SetBodyPose::Response& res );
	bool VelocityCallback( simu::SetBodyVelocity::Request& req,
	                       simu::SetBodyVelocity::Response& res );

};

}