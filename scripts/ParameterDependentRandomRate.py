#!/usr/bin/env python

import rospy, math
import numpy as np
from percepto_msgs.srv import SetParameters
from paraset.msg import RuntimeParameter
from paraset.srv import SetRuntimeParameter, SetRuntimeParameterRequest

# TODO Move this into argus_utils
def wait_for_service( srv ):
    rospy.loginfo( 'Waiting for service %s', srv )
    rospy.wait_for_service( srv )
    rospy.loginfo( 'Service now available %s', srv )

class ParameterDependentRandomRate:
    """Simulates sensing rate that varies with parameters.

    Sensing frequency is computed as exponential decay from max rate.
    """

    def __init__( self ):
        self.optimal_params = np.array( rospy.get_param( '~optimal_parameters' ) )

        self.max_rate = rospy.get_param( '~max_sensing_rate' )
        self.rate_decay = rospy.get_param( '~rate_decay_constant' )

        rate_topic = rospy.get_param( '~rate_set_service' )
        wait_for_service( rate_topic )
        self.rate_proxy = rospy.ServiceProxy( rate_topic, 
                                               SetRuntimeParameter )
        self.param_server = rospy.Service( '~set_parameters',
                                           SetParameters,
                                           self.ParamCallback )

        self.last_params = None
        sample_rate = rospy.get_param( '~sample_rate' )
        self.set_timer = rospy.Timer( rospy.Duraiton( 1.0/sample_rate ),
                                      self.TimerCallback )

    def TimerCallback( self ):
        if self.last_params is None:
            return

        

    def ComputeRate( self, p ):
        err = p - self.optimal_params
        scale = err.dot( err )
        rate = self.max_rate * math.exp( -self.rate_decay * scale )
        return rate

    def ParamCallback( self, req ):
        input_params = np.array( req.parameters )
        rate = self.ComputeRate( input_params )

        # Compute noise properties
        req = SetRuntimeParameterRequest()
        req.param.type = RuntimeParameter.PARAM_NUMERIC
        req.param.numeric_value = rate

        rospy.loginfo( 'Setting rate to: %f Hz', rate )
        try:
            self.rate_proxy.call( req )
        except rospy.ServiceException:
            rospy.logerr( 'Could not call rate setting service.' )
            return None
        return []

if __name__=='__main__':
    rospy.init_node( 'parameter_dependent_rate' )
    try:
        pdn = ParameterDependentRandomRate()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
