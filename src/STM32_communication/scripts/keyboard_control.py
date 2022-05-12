#!/usr/bin/env python
# license removed for brevity

import rospy
import tkinter as tk
from acri_gazebo.msg import track_velocity

class KeyboardControl():

    # main tkinter window.
    root = tk.Tk()

    # the ROS publisher this node uses.
    pub = rospy.Publisher('/acri_hw/track_velocity', track_velocity, queue_size=1)

    # track_velocity custom message type whose contents are published to the /acri_sim/track_velocity topic.
    trackVelocity = track_velocity()

    # value to increment the track velocities by for each press of a key.
    incrementVal = 0.2


    # Class constructor.
    def __init__(self):

        # Set the left and right track velocity to zero.
        self.trackVelocity.left = 0
        self.trackVelocity.right = 0

        # initialise the keyboard_control GUI command.
        Title = tk.Label(self.root, text="Left Tread: W + S. Right Tread: Up and Down Arrow Keys. Estop: enter.", font=('Arial', 14)).pack()

        # Bind W and S keys to LeftTreadForward and LeftTreadBackward events. 
        self.root.bind('w', self.LeftTreadForward)
        self.root.bind('s', self.LeftTreadBackward)

        # Bind the <Up> and <Down> keys to RightTreadForward and RightTreadBackward events.
        self.root.bind('<Up>', self.RightTreadForward)
        self.root.bind('<Down>', self.RightTreadBackward)

        # Bind the <Enter> key to the Estop event.
        self.root.bind('<Return>', self.Estop)

        # Initialise the publisher node. 
        rospy.init_node('keyboard_control', anonymous=False)

        # Start the GUI.
        self.root.mainloop()

    def Estop(self, event):
        self.trackVelocity.left = 0
        self.trackVelocity.right = 0
        self.pub.publish(self.trackVelocity)

    def PublishTreadVelocity(self):
        rospy.loginfo(self.trackVelocity)
        self.pub.publish(self.trackVelocity)

    def LeftTreadForward(self, event):
        self.trackVelocity.left = self.trackVelocity.left + self.incrementVal
        self.PublishTreadVelocity()

    def LeftTreadBackward(self, event):
        self.trackVelocity.left = self.trackVelocity.left - self.incrementVal
        self.PublishTreadVelocity()

    def RightTreadForward(self, event):
        self.trackVelocity.right = self.trackVelocity.right + self.incrementVal
        self.PublishTreadVelocity()

    def RightTreadBackward(self, event):
        self.trackVelocity.right = self.trackVelocity.right - self.incrementVal
        self.PublishTreadVelocity()

if __name__ == '__main__':
    try:
        Application = KeyboardControl()
    except rospy.ROSInterruptException:
        pass