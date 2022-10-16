#!/usr/bin/env python
# license removed for brevity

import tkinter as tk
import threading
import rospy
from acri_controller.msg import from_NUC
from sensor_msgs.msg import Imu
import csv
import numpy as np
from tf.transformations import euler_from_quaternion

class MomentOfInertiaSysID():

    # main tkinter window.
    root = tk.Tk()

    # from_NUC publisher.
    fromNUCPub = rospy.Publisher('acri_hw/from_NUC', from_NUC, queue_size=1)

    # from_NUC ROS msg.
    fromNUC = from_NUC()

    # flag which indicates if the estop has been pressed.
    estopPressed = True

    # flag which indicates if the experiment is running.
    experimentIsRunning = True

    def __init__(self):

        # Initialise the ROS node.
        rospy.init_node("momentOfInertiaSysID")

        # Initialise all UI elements.
        self.estop = tk.Button(
            text="TANK IS INACTIVE",
            width=45,
            height=10,
            bg="red",
            fg="black",
            command=self.estopUpdate,
        )

        # Initialise all UI elements.
        self.experiment = tk.Button(
            text="EXPERIMENT IS INACTIVE",
            width=45,
            height=10,
            bg="red",
            fg="black",
            command=self.experimentUpdate,
        )

        # Position UI elements.
        self.estop.grid(row=0, column=0)
        self.experiment.grid(row=1, column=0)

        # Bind the user's keys to their respective events.
        self.root.bind('<Return>', self.estopEventCallback)
        self.root.bind('j', self.experimentEventCallback)

        # Imu topic subscriber.
        ImuSUB = rospy.Subscriber('/an_device/Imu', Imu, self.ImuSubCallback)

        # Initialise UI.
        self.root.mainloop()

    # Handle the IMU subscriber callback.
    def ImuSubCallback(self, data):

        # If the experiment is enabled, enable the logic of the Imu Callback.
        if((self.estopPressed == False) & (self.experimentIsRunning == True)):

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



    # Handles estop logic.
    def estopUpdate(self):
        if(self.estopPressed == True):
            self.estop.config(
                bg='green',
                text='TANK IS ACTIVE. STAY CLEAR!'
            )
            self.estopPressed = False
        else:
            self.estop.config(
                bg='red',
                text='TANK IS INACTIVE'
            )
            self.estopPressed = True
            self.fromNUC.isROSControlEnabled = 0
            self.fromNUC.isROSEnabled = 0
            self.fromNUC.left_tread = 0
            self.fromNUC.right_tread = 0
            self.fromNUCPub.publish(self.fromNUC)

    # Define the event callbacks for each button's corresponding keyboard command.
    def estopEventCallback(self, event):
        self.estopUpdate()

    def experimentEventCallback(self, event):
        self.experimentUpdate()

    # Handles experiment starting logic.
    def experimentUpdate(self):
        if(self.experimentIsRunning == True):
            self.experiment.config(
                bg='green',
                text='EXPERIMENT IS ACTIVE. STAY CLEAR!'
            )
            self.experimentIsRunning = False
        else:
            self.experiment.config(
                bg='red',
                text='EXPERIMENT IS INACTIVE'
            )
            self.experimentIsRunning = True

if __name__ == '__main__':
    try:
        Application = MomentOfInertiaSysID()
    except rospy.ROSInterruptException:
        pass

