#include "simu/SimulatedVelocitySensor.h"
#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/geometry/GeometryUtils.h"
#include "geometry_msgs/TwistStamped.h"

#define POSE_DIM (PoseSE3::TangentDimension)

namespace argus
{

SimulatedVelocitySensor::SimulatedVelocitySensor( ros::NodeHandle& nh,
                                                  ros::NodeHandle& ph,
                                                  SimulatedBody& parent )
: SimulatedSensor( nh, ph ),
  _parent( parent ),
  _extrinsicsManager( nh, ph ), 
  _noiseGenerator( POSE_DIM )
{
	unsigned int buffSize;
	GetParam( ph, "buffer_size", buffSize, (unsigned int) 10 );
	_velPub = ph.advertise<geometry_msgs::TwistStamped>( "velocity",
	                                                     buffSize );
	GetParamRequired( ph, "frame_id", _frameID );
	GetParam( ph, "enable_noise", _enableNoise, false );
	if( _enableNoise )
	{
		PoseSE3::CovarianceMatrix cov;
		GetParamRequired( ph, "noise_covariance", cov );
		_noiseGenerator.SetCovariance( cov );
	}
}

void SimulatedVelocitySensor::Sample( const ros::Time& now )
{
	PoseSE3::TangentVector velocity = _parent.GetVelocity();
	PoseSE3 extrinsics = _extrinsicsManager.GetExtrinsics( _parent.GetFrameID(),
	                                                       _frameID,
	                                                       now );
	PoseSE3::TangentVector transformedVelocity = TransformTangent( velocity, 
	                                                               extrinsics );
	if( _enableNoise )
	{
		transformedVelocity += _noiseGenerator.Sample();
	}

	geometry_msgs::TwistStamped msg;
	msg.header.frame_id = _frameID;
	msg.header.stamp = now;
	msg.twist = TangentToMsg( transformedVelocity );
	_velPub.publish( msg );
}

}