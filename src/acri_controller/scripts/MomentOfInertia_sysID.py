#!/usr/bin/env python
# license removed for brevity

import tkinter as tk
import threading
import rospy
from acri_controller.msg import from_NUC
from std_msgs.msg import String
import csv

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

        # Initialise UI.
        self.root.mainloop()


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

