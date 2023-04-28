#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

right_arm_pitch_pos = 0
left_arm_pitch_pos = 0

rospy.init_node('my_tutorial_node')
rospy.loginfo("my_tutorial_node started!")

right_arm_pub = rospy.Publisher('/qt_robot/right_arm_position/command', Float64MultiArray, queue_size=1)
rospy.sleep(3.0)

def state_callback(msg):
    global right_arm_pitch_pos
    right_arm_pitch_pos = msg.position[msg.name.index("RightShoulderPitch")]

rospy.Subscriber('/qt_robot/joints/state', JointState, state_callback)

if __name__ == '__main__':
    right_arm_pitch_ref = 30
    while not rospy.is_shutdown():
        try:
            r_a_ref = Float64MultiArray()
            r_a_ref.data = [right_arm_pitch_ref, -40, -80]
            right_arm_pub.publish(r_a_ref)
            rospy.sleep(4)
            rospy.loginfo("Current position : %.2f" ,right_arm_pitch_pos)
            right_arm_pitch_ref = -30 if right_arm_pitch_ref == 30 else 30
        except KeyboardInterrupt:
            pass
    rospy.loginfo("finsihed!")

############## NEW CHUNK #############
'''   
def FormatSymmetricArmPos(shoulder_pitch_raw, shoulder_roll_raw, elbow_roll_raw):
    #Initialise data types
    Right_ref = Float64MultiArray()
    Left_ref = Float64MultiArray()
    
    #This stage is pointless, but a precaution none the less i suppose
    shoulder_pitch = int(shoulder_pitch_raw)
    shoulder_roll = int(shoulder_roll_raw)
    elbow_roll = int(elbow_roll_raw)
    
    #Add arm positions into messages
    Right_ref.data = [shoulder_pitch, shoulder_roll, elbow_roll]
    Left_ref.data = [-shoulder_pitch, shoulder_roll, elbow_roll] #Left arm's shoulder pitch is reverse from right
    
    return Right_ref, Left_ref
    
try: 
    #Move arms into "double front bicep" flexing position
    right_arm_pos, left_arm_pos = FormatSymmetricArmPos(90,0,-70) 
    #Publish arm positions
    LetArmMovementFinish()
    right_pub.publish(right_arm_pos)
    left_pub.publish(left_arm_pos)  
    
    #Replace this with system to check if action completed
    rospy.sleep(4) #Give the robot some seconds to move to it's position before moving back to default
    
    #Move back to default position
    right_arm_pos, left_arm_pos = FormatSymmetricArmPos(-90,-70,0)
    #Publish arm positions
    LetArmMovementFinish()
    right_pub.publish(right_arm_pos)
    left_pub.publish(left_arm_pos)
'''
