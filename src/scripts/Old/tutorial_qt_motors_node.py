#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

right_arm_pitch_pos = 0
rospy.init_node('my_tutorial_node')
rospy.loginfo("my_tutorial_node started!")

right_arm_pub = rospy.Publisher('/qt_robot/right_arm_position/command', Float64MultiArray, queue_size=1)
rospy.sleep(3.0)

def state_callback(msg):
    global right_arm_pitch_pos
    right_arm_pitch_pos = msg.position[msg.name.index("RightShoulderPitch")]

rospy.Subscriber('/qt_robot/joints/state', JointState, state_callback)

if __name__ == '__main__':
    right_arm_pitch_ref = 5
    while not rospy.is_shutdown():
        try:
            r_a_ref = Float64MultiArray()
            r_a_ref.data = [right_arm_pitch_ref, 0]
            right_arm_pub.publish(r_a_ref)
            rospy.sleep(4)
            rospy.loginfo("Current position : %.2f" ,right_arm_pitch_pos)
            right_arm_pitch_ref = -5 if right_arm_pitch_ref == 5 else 5
        except KeyboardInterrupt:
            pass
    rospy.loginfo("finsihed!")
