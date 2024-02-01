#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
from intera_interface import gripper as robot_gripper

import numpy as np
from numpy import linalg
import sys

def main():
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    right_gripper = robot_gripper.Gripper('right_gripper')
    right_gripper.open()

    while not rospy.is_shutdown():
        command = input('Press 1 for pick up position, 2 for drop off position, 0 for reset position, c for close, o for open')
        # grippy = input("open, close, or nothing")
        
        # Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        link = "right_gripper_tip"

        request.ik_request.ik_link_name = link
        # request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        if command=="1":
            # Set the desired orientation for the end effector HERE
            request.ik_request.pose_stamped.pose.position.x = 0.881
            request.ik_request.pose_stamped.pose.position.y = 0.114
            request.ik_request.pose_stamped.pose.position.z = -0.154      
            request.ik_request.pose_stamped.pose.orientation.x = 0.0
            request.ik_request.pose_stamped.pose.orientation.y = 1.0
            request.ik_request.pose_stamped.pose.orientation.z = 0.0
            request.ik_request.pose_stamped.pose.orientation.w = 0.0
        elif command == "0":
            request.ik_request.pose_stamped.pose.position.x = 0.871
            request.ik_request.pose_stamped.pose.position.y = 0.031
            request.ik_request.pose_stamped.pose.position.z = 0.205    
            request.ik_request.pose_stamped.pose.orientation.x = 0.0
            request.ik_request.pose_stamped.pose.orientation.y = 1.0
            request.ik_request.pose_stamped.pose.orientation.z = 0.0
            request.ik_request.pose_stamped.pose.orientation.w = 0.0
        elif command == "2":
            request.ik_request.pose_stamped.pose.position.x = 0.853
            request.ik_request.pose_stamped.pose.position.y = -0.144
            request.ik_request.pose_stamped.pose.position.z = -0.152      
            request.ik_request.pose_stamped.pose.orientation.x = 0.0
            request.ik_request.pose_stamped.pose.orientation.y = 1.0
            request.ik_request.pose_stamped.pose.orientation.z = 0.0
            request.ik_request.pose_stamped.pose.orientation.w = 0.0
        elif command == "c":
            right_gripper.close()
            continue

        elif command == "o":
            right_gripper.open()
            continue
        

        try:
            # Send the request to the service
            response = compute_ik(request)
            
            # Print the response HERE
            print(response)
            group = MoveGroupCommander("right_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # TRY THIS
            # Setting just the position without specifying the orientation
            #group.set_position_target([0.5, 0, 0.0])

            # Plan IK
            plan = group.plan()
            user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
            
            # Execute IK if safe
            if user_input == 'y':
                group.execute(plan[1])

            
            
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

# Python's syntax for a main() method
if __name__ == '__main__':
    main()
