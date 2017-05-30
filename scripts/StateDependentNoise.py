#!/usr/bin/env python

import rospy, math
import numpy as np
from nav_msgs.msg import Odometry
from percepto_msgs.srv import SetParameters
from simu.srv import SetNoiseProperties, SetNoisePropertiesRequest
from paraset.msg import RuntimeParameter
from paraset.srv import SetRuntimeParameter, SetRuntimeParameterRequest

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

def twist_to_vector( msg ):
    return np.array( [ msg.linear.x,
                       msg.linear.y,
                       msg.linear.z,
                       msg.angular.x,
                       msg.angular.y,
                       msg.angular.z ] )

class StateDependentNoise:
    """Simulates noise that varies with state."""

    def __init__( self ):
        noise_dim = rospy.get_param( '~noise_dim' )
        self.noise_mean = np.zeros( noise_dim )
        self.optimal_params = np.array( rospy.get_param( '~optimal_parameters' ) )

        self.min_noise_cov = parse_matrix( rospy.get_param( '~min_noise_cov' ),
                                           noise_dim )
        self.noise_cov_slope = parse_matrix( rospy.get_param( '~noise_cov_slope' ),
                                             noise_dim )
        self.noise_cov_decay = rospy.get_param( '~noise_cov_decay' )

        self.min_sensor_rate = rospy.get_param( '~min_sensor_rate' )
        self.max_sensor_rate = rospy.get_param( '~max_sensor_rate' )
        self.sensor_rate_decay = rospy.get_param( '~sensor_rate_decay' )

        self.cutoff_level = rospy.get_param( '~cutoff_level' )

        noise_topic = rospy.get_param( '~noise_set_service' )
        wait_for_service( noise_topic )
        self.noise_proxy = rospy.ServiceProxy( noise_topic, 
                                               SetNoiseProperties )

        rate_topic = rospy.get_param( '~rate_set_service' )
        wait_for_service( rate_topic )
        self.rate_proxy = rospy.ServiceProxy( rate_topic, 
                                               SetRuntimeParameter,
                                               True )

        self.last_odom = None
        self.odom_sub = rospy.Subscriber( 'odom', Odometry, self.OdomCallback )

        self.param_server = rospy.Service( '~set_parameters',
                                           SetParameters,
                                           self.ParamCallback )
        self.last_params = self.optimal_params

        update_rate = rospy.get_param( '~update_rate' )
        self.update_timer = rospy.Timer( rospy.Duration( 1.0/update_rate ),
                                         self.UpdateCallback )

    def UpdateCallback( self, event ):
        if self.last_odom is None:
            rospy.logwarn( 'No odom messages received.' )
            return

        # Compute noise properties
        n_mean, n_cov, rate = self.ComputeProperties()
        self.SetNoise( n_mean, n_cov )
        self.SetRate( rate )

    def OdomCallback( self, msg ):
        self.last_odom = msg

    def ParamCallback( self, req ):
        self.last_params = np.array( req.parameters )
        return []

    def ComputeProperties( self ):
        """As velocity increases, covariance increases and rate decreases.
        err_factor = base_factor * speed
        """
        base_factor = np.linalg.norm( self.last_params - self.optimal_params )

        vel = twist_to_vector( self.last_odom.twist.twist )
        speed = np.linalg.norm( vel )

        err_factor = base_factor * speed
        rospy.loginfo( 'Base factor: %f speed: %f err_factor: %f',
                       base_factor, speed, err_factor )

        cov = (self.min_noise_cov - self.noise_cov_slope) + \
            self.noise_cov_slope * math.exp( err_factor * self.noise_cov_decay )
        
        if err_factor > self.cutoff_level:
            rate = 0
        else:
            del_rate = self.max_sensor_rate - self.min_sensor_rate
            rate = self.min_sensor_rate + \
                del_rate * math.exp( err_factor * -self.sensor_rate_decay )

        return self.noise_mean, cov, rate

    def SetNoise( self, mean, cov ):
        req = SetNoisePropertiesRequest()
        req.enable_noise = True
        req.noise_mean = mean
        req.noise_cov = cov.flat

        rospy.loginfo( 'Setting noise to cov: \n%s', np.array_str( cov ) )
        try:
            self.noise_proxy.call( req )
        except rospy.ServiceException:
            rospy.logerr( 'Could not call noise setting service.' )

    def SetRate( self, rate ):
        req = SetRuntimeParameterRequest()
        req.param.type = RuntimeParameter.PARAM_NUMERIC
        req.param.numeric_value = rate

        rospy.loginfo( 'Setting rate to: %f Hz', rate )
        try:
            self.rate_proxy.call( req )
        except rospy.ServiceException:
            rospy.logerr( 'Could not call rate setting service.' )

if __name__=='__main__':
    rospy.init_node( 'state_dependent_noise' )
    try:
        pdn = StateDependentNoise()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
