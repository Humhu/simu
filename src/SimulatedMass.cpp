#include "simu/SimulatedMass.h"
#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/geometry/GeometryUtils.h"
#include "argus_utils/utils/MathUtils.h"

#include "nav_msgs/Odometry.h"

#include <sstream>

#define POSE_DIM (PoseSE3::TangentDimension)

namespace argus
{

SimulatedMass::SimulatedMass( ros::NodeHandle& nh,
                              ros::NodeHandle& ph,
                              const std::string& referenceFrame )
: SimulatedBody( ph, referenceFrame )
{
	ros::NodeHandle pnh( ph.resolveName( "pose_noise" ) );
	_poseNoise.Initialize( pnh, POSE_DIM );
	ros::NodeHandle vnh( ph.resolveName( "velocity_noise" ) );
	_velocityNoise.Initialize( vnh, POSE_DIM );

	GetParamRequired( ph, "pose", _initialPose );
	GetParamRequired( ph, "velocity", _initialVelocity );

	unsigned int odomBuffSize;
	GetParam( ph, "odom_buffer_size", odomBuffSize, (unsigned int) 10 );
	_odomPub = ph.advertise<nav_msgs::Odometry>( "odom", odomBuffSize );

	unsigned int cmdBuffSize;
	GetParam( ph, "cmd_buffer_size", cmdBuffSize, (unsigned int) 10 );
	_cmdSub = ph.subscribe( "cmd_vel", 
	                        cmdBuffSize,
	                        &SimulatedMass::CommandCallback,
	                        this );
	_poseServer = ph.advertiseService( "set_pose",
	                                   &SimulatedMass::PoseCallback,
	                                   this );
	_velocityServer = ph.advertiseService( "set_velocity",
	                                       &SimulatedMass::VelocityCallback,
	                                       this );
}

void SimulatedMass::Initialize( const ros::Time& now )
{
	_pose = _initialPose;
	_velocity = _initialVelocity;
	_simTime = now;
}

void SimulatedMass::Tic( const ros::Time& now )
{
	VectorType x = VectorType::Zero( 2*POSE_DIM );
	x.tail<POSE_DIM>()  = _velocity;
	double dt = ( now - _simTime ).toSec();
	
	MatrixType A = IntegralMatrix<double>( dt, 1 );
	VectorType xNext = A * x;
	VectorType poseNoise = _poseNoise.Sample();
	_pose = _pose * PoseSE3::Exp( xNext.head<POSE_DIM>() + 
	                              poseNoise );
	_velocity = xNext.tail<POSE_DIM>() + _velocityNoise.Sample();
	_simTime = now;

	nav_msgs::Odometry msg;
	msg.header.frame_id = GetReferenceFrame();
	msg.header.stamp = now;
	msg.child_frame_id = GetBodyFrame();
	msg.pose.pose = PoseToMsg( GetPose() );
	msg.twist.twist = TangentToMsg( GetVelocity() );
	_odomPub.publish( msg );
}

PoseSE3 SimulatedMass::GetPose() const
{
	return _pose;
}

PoseSE3::TangentVector SimulatedMass::GetVelocity() const
{
	return _velocity;
}

void SimulatedMass::CommandCallback( const geometry_msgs::Twist::ConstPtr& msg )
{
	_velocity = MsgToTangent( *msg );
}

bool SimulatedMass::PoseCallback( simu::SetBodyPose::Request& req,
                                  simu::SetBodyPose::Response& res )
{
	_pose = MsgToPose( req.pose );
	res.set_pose = req.pose;
	return true;
}

bool SimulatedMass::VelocityCallback( simu::SetBodyVelocity::Request& req,
                                      simu::SetBodyVelocity::Response& res )
{
	_velocity = MsgToTangent( req.velocity );
	res.set_velocity = req.velocity;
	return true;
}

}