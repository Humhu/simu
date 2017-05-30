#!/usr/bin/env python

import rospy, math
import numpy as np
from percepto_msgs.srv import SetParameters
from simu.srv import SetNoiseProperties, SetNoisePropertiesRequest

# TODO Move this into argus_utils
def wait_for_service( srv ):
    rospy.loginfo( 'Waiting for service %s', srv )
    rospy.wait_for_service( srv )
    rospy.loginfo( 'Service now available %s', srv )

def parse_matrix( mat, dim ):
    if type(mat[0]) == str:
        mat = [ float(d) for d in mat ]
    if len(mat) == dim:
        return np.diag( mat )
    elif len(mat) == dim*dim:
        return np.reshape( (dim,dim), mat )
    else:
        raise RuntimeError( 'Cannot parse %d values as %d-dim matrix'
                             % (len(mat), dim) )

class ParameterDependentNoise:
    """Simulates noise that varies with parameters."""

    def __init__( self ):
        noise_dim = rospy.get_param( '~noise_dim' )

        self.noise_mean = np.zeros( noise_dim )
        self.optimal_params = np.array( rospy.get_param( '~optimal_parameters' ) )

        self.min_noise_cov = parse_matrix( rospy.get_param( '~min_noise_cov' ),
                                           noise_dim )
        self.noise_cov_slope = parse_matrix( rospy.get_param( '~noise_cov_slope' ),
                                             noise_dim )
        self.noise_rate = rospy.get_param( '~noise_growth_rate' )

        noise_topic = rospy.get_param( '~noise_set_service' )
        wait_for_service( noise_topic )
        self.noise_proxy = rospy.ServiceProxy( noise_topic, 
                                               SetNoiseProperties )
        self.param_server = rospy.Service( '~set_parameters',
                                           SetParameters,
                                           self.ParamCallback )

    def ComputeNoise( self, p ):
        err = p - self.optimal_params
        scale = err.dot( err )
        cov = self.min_noise_cov + \
              self.noise_cov_slope * (math.exp( self.noise_rate * scale ) - 1.0)
        return self.noise_mean, cov

    def ParamCallback( self, req ):
        input_params = np.array( req.parameters )
        n_mean, n_cov = self.ComputeNoise( input_params )

        # Compute noise properties
        req = SetNoisePropertiesRequest()
        req.enable_noise = True
        req.noise_mean = n_mean
        req.noise_cov = n_cov.flat

        rospy.loginfo( 'Setting noise to cov: \n%s', np.array_str( n_cov ) )
        try:
            self.noise_proxy.call( req )
        except rospy.ServiceException:
            rospy.logerr( 'Could not call noise setting service.' )
            return None
        return []

if __name__=='__main__':
    rospy.init_node( 'parameter_dependent_noise' )
    try:
        pdn = ParameterDependentNoise()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
