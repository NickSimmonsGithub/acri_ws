#!/usr/bin/env python
# license removed for brevity

import rospy
import numpy as np
import matplotlib.pyplot as plt
from acri_controller.msg import State
from sensor_msgs.msg import Imu
from acri_controller.srv import System
from acri_controller.srv import Control
from acri_controller.msg import from_NUC
from acri_controller.srv import SystemObs
from tf.transformations import euler_from_quaternion

class Controller():

    # System state.
    stateObs = State()

    # d parameter.
    d = 0.45

    # Constant which relates the velocity to the tread velocity reference.
    a = 2.3

    # fromNUC message.
    _fromNUC = from_NUC()

    # Flag to enable controller.
    controlEnabled = False

    def __init__(self):

        # Initialise the node.
        rospy.init_node("Controller", anonymous=False)

        # Subscribe to the /an_device/Imu topic.
        self.ImuSub = rospy.Subscriber("/an_device/Imu", Imu, self.IMUCallback)

        # Initialise the from_NUC publisher.
        self.fromNUCPub = rospy.Publisher('acri_hw/from_NUC', from_NUC, queue_size=1)

        # spin the node.
        rospy.spin()

    def updateObserver(self, state, y, v):
        rospy.wait_for_service('update_observer')
        try:
            update_state = rospy.ServiceProxy('update_observer', SystemObs)
            out = update_state(state, y, v)
            return out.x_out
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def calculateControl(self, state):
        rospy.wait_for_service('calculate_control')
        try:
            calculate_control = rospy.ServiceProxy('calculate_control', Control)
            out = calculate_control(state)
            return out.v
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def IMUCallback(self, data):

        # Convert angles from quaternions to euler.
        orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

        # Correct for the orientation of the sensor.
        roll = roll - np.pi

        # Correct for lack of squareness in the sensor mount.
        roll = roll + 0.02

        # Correct for rollover.
        if((roll < -np.pi) & (roll >= -2*np.pi)):
            roll = roll + 2*np.pi

        # Print the relevant measurements.
        rateOfRoll = data.angular_velocity.x
        print_string = "roll: " + "{:.5f}".format(roll) + ", rate of roll: " + "{:.5f}".format(rateOfRoll)
        print(print_string)

        # Check whether observer should be enabled or not.
        if((rateOfRoll  >= 0.4)):

            # Set the controller initial conditions.
            self.stateObs.theta   = roll
            self.stateObs.P_theta = 0
            self.stateObs.z       = -self.d * np.tan(roll)

            # Begin the control algorithm.
            self.controlEnabled = True
        
        if(self.controlEnabled == True):
            v             = self.calculateControl(self.stateObs)
            self.stateObs = self.updateObserver(self.stateObs, roll, v)
            ref           = v / self.a
            self._fromNUC.isROSControlEnabled = 1
            self._fromNUC.isROSEnabled = 1
            self._fromNUC.left_tread = ref
            self._fromNUC.right_tread = ref
            self.fromNUCPub.publish(self._fromNUC)

        else:
            self._fromNUC.isROSControlEnabled = 0
            self._fromNUC.isROSEnabled = 1
            self._fromNUC.left_tread = 0
            self._fromNUC.right_tread = 0
            self.fromNUCPub.publish(self._fromNUC)


if __name__ == "__main__":
    try:
        Application = Controller()
    except rospy.ROSInterruptException:
        pass
