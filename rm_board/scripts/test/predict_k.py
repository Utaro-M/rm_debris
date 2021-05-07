#!/usr/bin/env python

import rospy
import message_filters
import numpy as np
import numpy.linalg as LA
import math
from geometry_msgs.msg import Polygon
from rm_debris_vision.srv import PredK
from std_msgs.msg import Float32MultiArray
def get_trigger (req):
    mat_tmp=np.zeros([1,3])
    for v in req.mat.points:
        np.apend(mat,np.array([v.x,v.y,v.z]),axis=0)
    mat=mat_tmp[1,:]
    eign_list=LA.eig(mat)
    eign=tolist(eign_list[0])
    eignvector=tolist(eign_list[1])
    # eign_service=Polygon()
    eign_service.points
    res=PredK()
    res.eign.data=eign
    res.eignvector.points=eignvector
    # for v in eignvector:        
    #     res.eignvector.points
    # print ("\n\nset centroid {}\n".format(centroid))
    return PredKResponse(res)

if __name__ == '__main__':
    rospy.init_node('predict_k')

    s = rospy.Service('predict_k', PredK, get_trigger)
    rospy.spin()    
    
