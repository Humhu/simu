#include "simu/SimulatedMass.h"
#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/geometry/GeometryUtils.h"
#include "argus_utils/utils/MathUtils.h"
#include <sstream>

#define POSE_DIM (PoseSE3::TangentDimension)

namespace argus
{

SimulatedMass::SimulatedMass( ros::NodeHandle& ph )
: SimulatedBody( ph )
{
	GetParamRequired( ph, "pose", _initialPose );
	GetParamRequired( ph, "velocity", _initialVelocity );

	GetParam( ph, "enable_noise", _enableNoise, false );
	if( _enableNoise )
	{
		MatrixType cov( 2*POSE_DIM ,2*POSE_DIM );
		GetParamRequired( ph, "noise_covariance", cov );
		_noiseGenerator = MultivariateGaussian<>( 2*POSE_DIM );
		_noiseGenerator.SetCovariance( cov );
	}
}

void SimulatedMass::Initialize( const ros::Time& now )
{
	_pose = _initialPose;
	_velocity = _initialVelocity;
	_simTime = now;
}

void SimulatedMass::Tic( const ros::Time& now )
{
	VectorType x( 2*POSE_DIM );
	x.tail<POSE_DIM>()  = _velocity;
	double dt = ( now - _simTime ).toSec();
	
	MatrixType A = IntegralMatrix<double>( dt, 1 );
	VectorType xNext = A * x;
	if( _enableNoise )
	{
		xNext += _noiseGenerator.Sample();
	}

	_pose = _pose * PoseSE3::Exp( xNext.head<POSE_DIM>() );
	_velocity = xNext.tail<POSE_DIM>();
	_simTime = now;
}

PoseSE3 SimulatedMass::GetPose() const
{
	return _pose;
}

PoseSE3::TangentVector SimulatedMass::GetVelocity() const
{
	return _velocity;
}

}