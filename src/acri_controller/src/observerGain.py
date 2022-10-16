#!/usr/bin/env python
# license removed for brevity

from __future__ import print_function

from scipy.signal import place_poles
import numpy as np
import rospy
from acri_controller.srv import ObserverInit, ObserverInitResponse

def calculateObserverGain(req):
    print("here")
    G = np.array([[req.G_00, req.G_01, req.G_02], [req.G_10, req.G_11, req.G_12], [req.G_20, req.G_21, req.G_22]])
    C = np.array([[req.C_0, req.C_1, req.C_2]])
    print("here")
    print(G)
    print(C)
    poles = np.array([req.pole1, req.pole2, req.pole3])
    K = place_poles(G.T, C.T, poles).gain_matrix
    Lc = K.T
    response = ObserverInitResponse()
    response.Lc_0 = Lc[0]
    response.Lc_1 = Lc[1]
    response.Lc_2 = Lc[2]
    return response

def calculateObserverGainServer():
    rospy.init_node("calculateObserverGainServer")
    s = rospy.Service('calculateObserverGain', ObserverInit, calculateObserverGain)
    print("node is running")
    rospy.spin()

if __name__=="__main__":
    calculateObserverGainServer()