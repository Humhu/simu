#!/usr/bin/env python

import rospy, math
import numpy as np
from percepto_msgs.srv import SetParameters

# TODO Move this into argus_utils
def wait_for_service( srv ):
    rospy.loginfo( 'Waiting for service %s', srv )
    rospy.wait_for_service( srv )
    rospy.loginfo( 'Service now available %s', srv )

class ParameterSplitter:

    def __init__( self ):
        topics = rospy.get_param( '~topics' )
        self.proxies = []
        for topic in topics:
            wait_for_service( topic )
            self.proxies.append( rospy.ServiceProxy( topic , SetParameters ) )
        self.param_server = rospy.Service( '~set_parameters',
                                           SetParameters,
                                           self.ParamCallback )

    def ParamCallback( self, req ):
        for proxy in self.proxies:
            try:
                proxy.call( req )
            except rospy.ServiceException:
                rospy.logerr( 'Could not call sub-proxy!' )
                return None
        return []

if __name__=='__main__':
    rospy.init_node( 'parameter_splitter' )
    try:
        pdn = ParameterSplitter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
