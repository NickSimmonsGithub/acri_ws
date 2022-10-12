#!/usr/bin/env python
# license removed for brevity

import rospy
import numpy as np
import matplotlib.pyplot as plt
from acri_controller.msg import State
from acri_controller.srv import System
from acri_controller.srv import Control

def updateState(state, v):
    rospy.wait_for_service('update_state')
    try:
        update_state = rospy.ServiceProxy('update_state', System)
        out = update_state(state, v)
        return out.x_out
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def calculateControl(state):
    rospy.wait_for_service('calculate_control')
    try:
        calculate_control = rospy.ServiceProxy('calculate_control', Control)
        out = calculate_control(state)
        return out.v
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":

    # Define the state object.
    state = State()
    state.theta   = -0.2
    state.P_theta = 0
    state.z       = 0.2

    # Define the time array.
    simTime = 10
    dt = 0.01
    time = np.linspace(0, simTime, num=simTime/dt+1)

    # Define the state arrays.
    theta   = np.zeros(np.size(time))
    P_theta = np.zeros(np.size(time))
    z       = np.zeros(np.size(time))
    u       = np.zeros(np.size(time))

    theta[0]   = state.theta
    P_theta[0] = state.P_theta
    z[0]       = state.z
    u[0]       = 0

    # Run the simulation.
    for i in range(np.size(time) - 1):
        v = calculateControl(state)
        x_out = updateState(state, v)
        theta[i + 1] = x_out.theta
        P_theta[i + 1] = x_out.P_theta
        z[i + 1] = x_out.z
        u[i + 1] = v
        state = x_out

    print("Final state vector: [" + str((180/np.pi)*theta[np.size(theta) - 1]) + ", " + str(P_theta[np.size(P_theta) - 1]) + ", " + str(z[np.size(z) - 1]) + "]")

    # Plot the data.
    plt.figure()
    plt.subplot(4, 1, 1)
    plt.plot(time, theta * 180.0/np.pi)
    plt.title("$x_1$")
    plt.ylabel("$/theta$ (degrees)")
    plt.xlabel("time (s)")

    plt.subplot(4, 1, 2)
    plt.plot(time, P_theta)
    plt.title("$x_2$")
    plt.ylabel("$P_/theta$")
    plt.xlabel("time (s)")

    plt.subplot(4, 1, 3)
    plt.plot(time, z)
    plt.title("$x_3$")
    plt.ylabel("z (m)")
    plt.xlabel("time (s)")

    plt.subplot(4, 1, 4)
    plt.plot(time, u)
    plt.title("$u$")
    plt.ylabel("u (m/s)")
    plt.xlabel("time (s)")


    plt.show()