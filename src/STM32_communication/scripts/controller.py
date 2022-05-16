#!/usr/bin/env python
# license removed for brevity

from concurrent.futures import thread
import rospy
import tkinter as tk
import threading
from acri_gazebo.msg import track_velocity
from std_msgs.msg import String

class Controller():

    # main tkinter window.
    root = tk.Tk()

    # healthy status ROS publisher.
    statusPub = rospy.Publisher('/acri_hw/status', String, queue_size=1)

    # velocity ROS publisher.
    velocityPub = rospy.Publisher('/acri_hw/track_velocity', track_velocity, queue_size=1)

    # variables which store the current linear and angular velocity reference.
    linearVel = 0.0
    angularVel = 0.0

    # each click of the button will increment its corresponding variable by this amount.
    linearVelIncrementVal = 0.1
    angularVelIncrementVal = 0.1

    # velocity ROS msg.
    trackVelocity = track_velocity()

    # flag which indicates if the estop has been pressed.
    estopPressed = True

    def __init__(self):        

        # Initialise all UI elements.
        self.estop = tk.Button(
            text="TANK IS INACTIVE",
            width=50,
            height=10,
            bg="red",
            fg="black",
            command=self.estopUpdate,
        )

        self.left = tk.Button(
            text='<',
            width=10,
            height=5,
            bg="gray",
            fg="black",
            command=self.decreaseAngularVel,
        )

        self.right = tk.Button(
            text='>',
            width=10,
            height=5,
            bg="gray",
            fg="black",
            command=self.increaseAngularVel,
        )

        self.forward = tk.Button(
            text='^',
            width=10,
            height=5,
            bg="gray",
            fg="black",
            command=self.increaseLinearVel,
        )

        self.backward = tk.Button(
            text='v',
            width=10,
            height=5,
            bg="gray",
            fg="black",
            command=self.decreaseLinearVel,
        )

        # Position all UI elements.
        self.estop.grid(row=0, rowspan=3, columnspan=3)
        self.left.grid(row=4, column=0)
        self.right.grid(row=4, column=2)
        self.forward.grid(row=3, columnspan=3)
        self.backward.grid(row=4, column=1)

        # Bind the user's keys to their respective events.
        self.root.bind('<Left>', self.leftEventCallback)
        self.root.bind('<Right>', self.rightEventCallback)
        self.root.bind('<Up>', self.forwardEventCallback)
        self.root.bind('<Down>', self.backwardEventCallback)
        self.root.bind('<Return>', self.estopEventCallback)

        # Initalise the HEALTHY timer.
        self.timer = threading.Timer(0.1, self.timerCallback)

        # Initialise the ROS node.
        rospy.init_node('controller', anonymous=False)
        
        # Initialise UI.
        self.root.mainloop()


    # Calculate the velocity reference for each track from the linear and angular velocity reference, then publish it to ROS.
    def updateTreadVel(self):

        # Calculate left tread velocity. Clip between +-1.0.
        if((self.linearVel + self.angularVel < 1.0) & (self.linearVel + self.angularVel > -1.0)):
            self.trackVelocity.left = self.linearVel + self.angularVel
        elif(self.linearVel + self.angularVel >= 1.0):
            self.trackVelocity.left = 1.0
        elif(self.linearVel + self.angularVel <= -1.0):
            self.trackVelocity.left = -1.0

        # Calculate right tread velocity. Clip between +-1.0.
        if((self.linearVel - self.angularVel < 1.0) & (self.linearVel - self.angularVel > -1.0)):
            self.trackVelocity.right = self.linearVel - self.angularVel
        elif(self.linearVel - self.angularVel >= 1.0):
            self.trackVelocity.right = 1.0
        elif(self.linearVel - self.angularVel <= -1.0):
            self.trackVelocity.right = -1.0

        # Publish the velocity reference.
        self.velocityPub.publish(self.trackVelocity)

    # Define 5 event callbacks for each button's corresponding keyboard command.
    def estopEventCallback(self, event):
        self.estopUpdate()

    def leftEventCallback(self, event):
        self.decreaseAngularVel()

    def rightEventCallback(self, event):
        self.increaseAngularVel()

    def forwardEventCallback(self, event):
        self.increaseLinearVel()

    def backwardEventCallback(self, event):
        self.decreaseLinearVel()

    # Increase the linear velocity by the increment value.
    def increaseLinearVel(self):
        self.linearVel = self.linearVel + self.linearVelIncrementVal
        self.updateTreadVel()

    # Decrease the linear velocity by the increment value.
    def decreaseLinearVel(self):
        self.linearVel = self.linearVel - self.linearVelIncrementVal
        self.updateTreadVel()

    # Increase the angular velocity by the increment value.
    def increaseAngularVel(self):
        self.angularVel = self.angularVel + self.angularVelIncrementVal
        self.updateTreadVel()

    # Decrease the angular velocity by the increment value.
    def decreaseAngularVel(self):
        self.angularVel = self.angularVel - self.angularVelIncrementVal
        self.updateTreadVel()

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
            self.angularVel = 0
            self.linearVel = 0
            self.updateTreadVel()
            self.estopPressed = True
            if(self.timer.is_alive() == True):
                self.timer.cancel()

    # Callback function for estop publisher.
    def timerCallback(self):
        # publish the message "HEALTHY" to the status topic.
        self.statusPub.publish("HEALTHY")
        # Restart the timer.
        self.timer = threading.Timer(0.1, self.timerCallback)
        self.timer.start()

            


if __name__ == '__main__':
    try:
        Application = Controller()
    except rospy.ROSInterruptException:
        pass
    
