#include "argus_utils/utils/ParamUtils.h"
#include "simu/NoiseGenerator.h"

namespace argus
{

NoiseGenerator::NoiseGenerator() {}

void NoiseGenerator::Initialize( ros::NodeHandle& ph, 
                                 unsigned int dim )
{
	_generator = MultivariateGaussian<>( dim );
	GetParam( ph, "enable_noise", _enabled, false );
	if( _enabled )
	{
		MatrixType cov( dim, dim );
		GetParamRequired( ph, "covariance", cov );
		_generator.SetCovariance( cov );
	}

	_server = ph.advertiseService( "set_noise",
	                               &NoiseGenerator::NoiseCallback,
	                               this );
}

VectorType NoiseGenerator::Sample()
{
	if( _enabled )
	{
		return _generator.Sample();
	}
	else
	{
		return VectorType::Zero( _generator.GetDimension() );
	}
}

MultivariateGaussian<>& NoiseGenerator::Distribution()
{
	return _generator;
}

const MultivariateGaussian<>& NoiseGenerator::Distribution() const
{
	return _generator;
}

bool NoiseGenerator::NoiseCallback( simu::SetNoiseProperties::Request& req,
                                    simu::SetNoiseProperties::Response& res )
{
	_enabled = req.enable_noise;
	if( _enabled )
	{
		VectorType mean = GetVectorView( req.noise_mean );
		MatrixType cov( mean.size(), mean.size() );
		if( !ParseMatrix( req.noise_cov, cov ) )
		{
			ROS_ERROR_STREAM( "Error parsing noise covariance." );
			return false;
		}
		_generator.SetMean( mean );
		_generator.SetCovariance( cov );
	}
	return true;
}



}