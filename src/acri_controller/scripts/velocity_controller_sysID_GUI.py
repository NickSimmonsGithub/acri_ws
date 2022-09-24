#!/usr/bin/env python
# license removed for brevity

from cgitb import text
from concurrent.futures import thread
from readline import insert_text
import rospy
import tkinter as tk
import threading
from acri_gazebo.msg import track_velocity
from std_msgs.msg import String

def isfloat(string) -> bool:
    try:
        float(string)
        return True
    except ValueError:
        return False

class VelocitySysIDGUI():

    # main tkinter window.
    root = tk.Tk()

    # healthy status ROS publisher.
    statusPub = rospy.Publisher('/acri_hw/status', String, queue_size=1)

    # velocity ROS publisher.
    velocityPub = rospy.Publisher('/acri_hw/tread_velocity_reference', track_velocity, queue_size=1)

    # velocity ROS msg.
    trackVelocity = track_velocity()
    experimentVelocity = 0.0

    # velocity publisher operating frequency.
    velocityPubTimerFreq = 25.0       # Hz

    # GUI Variables.
    maxVelReference = ""
    time = ""
    distance = ""
    constant = ""

    velTimerTime = 0.0
    velTimerMaxVelRef = 0.0

    velPublisherEnabled = False

    # flag which indicates if the estop has been pressed.
    estopPressed = True

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

        self.start = tk.Button(
            text="START",
            width=20,
            height=3,
            bg="cyan",
            fg="black",
            command=self.startButtonCallback,
        )

        self.calculateConstant = tk.Button(
            text="CALCULATE CONSTANT",
            width=20,
            height=1,
            bg="cyan",
            fg="black",
            command=self.calculateConst,
        )

        self.vrefLabel = tk.Label(
            width=15,
            height=1,
            text="max vref:"
        )

        self.timeLabel = tk.Label(
            width=15,
            height=1,
            text="experiment time: "
        )

        self.distanceLabel = tk.Label(
            width=15,
            height=1,
            text="distance: "
        )

        self.constLabel = tk.Label(
            width=15,
            height=1,
            text="constant: "
        )

        self.vref = tk.Text(
            width=10,
            height=1,
        )

        self.time = tk.Text(
            width=10,
            height=1,
        )

        self.distance = tk.Text(
            width=10,
            height=1,
        )

        self.const = tk.Text(
            width=10,
            height=1,
        )

        # Position all UI elements.
        self.estop.grid(row=0, rowspan=3, columnspan=4)
        self.start.grid(row=4, column=2, rowspan=3, columnspan=2)
        self.calculateConstant.grid(row=7, column=2, columnspan=2)
        self.vrefLabel.grid(row=4, column=0)
        self.vref.grid(row=4, column=1)
        self.timeLabel.grid(row=5, column=0)
        self.time.grid(row=5, column=1)
        self.distanceLabel.grid(row=6, column=0)
        self.distance.grid(row=6, column=1)
        self.constLabel.grid(row=7, column=0)
        self.const.grid(row=7, column=1)

        # Bind the user's keys to their respective events.
        self.root.bind('<Return>', self.estopEventCallback)

        # Initalise the HEALTHY timer.
        self.timer = threading.Timer(0.1, self.timerCallback)

        # Initialise the velocity publisher timer.
        self.velocityTimer = threading.Timer(1.0/self.velocityPubTimerFreq, self.velocityTimerCallback)

        # Initialise the ROS node.
        rospy.init_node('velocity_sysID', anonymous=False)

        # Initialise UI.
        self.root.mainloop()

    # Calculates the constant.
    def calculateConst(self):
        self.const.delete(1.0, "end")            
        # Check that all relevant text boxes have valid numerical inputs.
        if(isfloat(self.vref.get("1.0", "end-1c")) & isfloat(self.time.get("1.0", "end-1c")) & isfloat(self.distance.get("1.0", "end-1c"))):
            # Extract all values from their text box.
            vrefmax = float(self.vref.get("1.0", "end-1c"))
            time = float(self.time.get("1.0", "end-1c"))
            distance = float(self.distance.get("1.0", "end-1c"))
            # Check all values are greater than zero.
            if((vrefmax > 0.0) & (time > 0.0) & (distance > 0.0)):
                constant = (2*distance) / (vrefmax * time)
                self.const.insert(1.0, str(constant))

    # Updates the tread velocity and publishes it to the tank.
    def updateTreadVel(self, velRef):
        if(velRef > 1.0):
            velRef = 1.0
        elif(velRef < -1.0):
            velRef = -1.0
        self.trackVelocity.left = velRef
        self.trackVelocity.right = velRef
        self.velocityPub.publish(self.trackVelocity)

    # Define the event callbacks for each button's corresponding keyboard command.
    def estopEventCallback(self, event):
        self.estopUpdate()

    # Handles estop logic.
    def estopUpdate(self):
        if(self.estopPressed == True):
            self.estop.config(
                bg='green',
                text='TANK IS ACTIVE. STAY CLEAR!'
            )
            self.estopPressed = False
            if(self.timer.is_alive() == False):
                self.timerCallback()
        else:
            self.estop.config(
                bg='red',
                text='TANK IS INACTIVE'
            )
            self.updateTreadVel(0.0)
            self.estopPressed = True
            if(self.timer.is_alive() == True):
                self.timer.cancel()
            if(self.velocityTimer.is_alive() == True):
                self.velocityTimer.cancel()

    # Callback function for "START" button.
    def startButtonCallback(self):
        # Check the timer hasn't already been enabled and the ROS-STM32 connection is enabled.
        if(self.velPublisherEnabled == False):
            self.experimentVelocity = 0
            # Check that all relevant text boxes have valid numerical inputs.
            if(isfloat(self.vref.get("1.0", "end-1c")) & isfloat(self.time.get("1.0", "end-1c"))):
                # Extract all values from their text box.
                vrefmax = float(self.vref.get("1.0", "end-1c"))
                time = float(self.time.get("1.0", "end-1c"))
                # Check all values are greater than zero.
                if((vrefmax > 0.0) & (time > 0.0)):
                    self.velPublisherEnabled = True
                    self.velTimerTime = time
                    self.velTimerMaxVelRef = vrefmax
                    self.velocityTimerCallback()

    # Callback function for velocity publisher.
    def velocityTimerCallback(self):        
        # Calculate the next velocity and publish it.
        self.experimentVelocity += self.velTimerMaxVelRef / (self.velTimerTime * self.velocityPubTimerFreq)
        self.updateTreadVel(self.experimentVelocity)
        # If the velocity is not at its largest value, restart the timer.
        if(self.experimentVelocity <= self.velTimerMaxVelRef):
            self.velocityTimer = threading.Timer(1.0/self.velocityPubTimerFreq, self.velocityTimerCallback)
            self.velocityTimer.start()
        else:
            self.updateTreadVel(0.0)
            self.velPublisherEnabled = False
            if(self.velocityTimer.is_alive() == True):
                self.velocityTimer.cancel()

    # Callback function for estop publisher.
    def timerCallback(self):
        # publish the message "HEALTHY" to the status topic.
        self.statusPub.publish("HEALTHY")
        # Restart the timer.
        self.timer = threading.Timer(0.1, self.timerCallback)
        self.timer.start()


if __name__ == '__main__':
    try:
        Application = VelocitySysIDGUI()
    except rospy.ROSInterruptException:
        pass



    