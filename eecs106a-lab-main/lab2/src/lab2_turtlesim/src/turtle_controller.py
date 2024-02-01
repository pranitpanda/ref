#!/usr/bin/env python
# The line above tells Linux that this file is a Python script, and that the OS
# should use the Python interpreter in /usr/bin/env to run it. Don't forget to
# use "chmod +x [filename]" to make this script executable.

# Import the rospy package. For an import to work, it must be specified
# in both the package manifest AND the Python file in which it is used.
import rospy
import sys

# Import the String message type from the /msg directory of the std_msgs package.
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

# from my_chatter.msg import TimestampString

# Define the method which contains the node's main functionality
def talker():

    # Create an instance of the rospy.Publisher object which we can  use to
    # publish messages to a topic. This publisher publishes messages of type
    # std_msgs/String to the topic /chatter_talk
    turtle_name = sys.argv[1]
    print("TURTLE NAME: ", turtle_name)
    pub = rospy.Publisher(f'{turtle_name}/cmd_vel', Twist, queue_size=10)
    
    # Create a timer object that will sleep long enough to result in a 10Hz
    # publishing rate
    r = rospy.Rate(10) # 10hz

    # Loop until the node is killed with Ctrl-C
    while not rospy.is_shutdown():
        # get user input
        pub_input = input("Please enter a line of text and press <Enter>: ")
         
        lx = ly = lz = 0
        ax = ay = az = 0
        for let in pub_input:
            if let == "w":
                lx += 2
            if let == "a":
                az += 2 
            if let == "s":
                lx -= 2
            if let == "d":
                az -= 2

        vec = Twist(Vector3(lx,ly,lz), Vector3(ax,ay,az))
        
        # pub_time = rospy.get_time()
        
        # Publish our string to the 'chatter_talk' topic
        pub.publish(vec)
        print(rospy.get_name() + ": I sent \"%s\"" % vec)
        
        # Use our rate object to sleep until it is time to publish again
        r.sleep()
            
# This is Python's syntax for a main() method, which is run by default when
# exectued in the shell
if __name__ == '__main__':

    # Run this program as a new node in the ROS computation graph called /talker.
    rospy.init_node('talker', anonymous=True)

    # Check if the node has received a signal to shut down. If not, run the
    # talker method.
    try:
        talker()
    except rospy.ROSInterruptException: pass
