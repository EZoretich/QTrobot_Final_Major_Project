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

# ---------- Function to play note ~ offset included ~ ---------------------
def play_note(offset, key):
    # --------------------- FIRST GO TO OFFSET
    r_a_ref = Float64MultiArray()
    #print(Float64MultiArray())
    r_a_ref.data = offset
    #print(r_a_ref)
    pub = right_arm_pub.publish(r_a_ref)
    check_angle(r_a_ref.data, key)
    #rospy.sleep(2)
    #print("Key Pos:     ", key)
    #print("Array:   ", Float64MultiArray)
    #print("Offset Of:    ", offset)
    
    # --------------------- HIT NOTE
    
    r_a_ref = Float64MultiArray()
    #print(Float64MultiArray())
    r_a_ref.data = key
    #print(r_a_ref)
    pub = right_arm_pub.publish(r_a_ref)
    check_angle(r_a_ref.data, key)
    #rospy.sleep(0.5)
    #print("Note Played:    ", key)
    
    # --------------------- BACK TO OFFSET
    
    r_a_ref = Float64MultiArray()
    #print(Float64MultiArray())
    r_a_ref.data = offset
    #print(r_a_ref)
    pub = right_arm_pub.publish(r_a_ref)
    check_angle(r_a_ref.data, key)
    #rospy.sleep(2)
    #print("Offset Of:    ", offset)

# ---------------------------------------------------------------
# ---------- Function to check if position has been reached ---------------------

def check_angle(data, target):
    target_reached = False
    while not target_reached:
        if data == target:
            target_reached = True
            print("Target angle Reached")



if __name__ == '__main__':
    #right_arm_pitch_ref = 30
    # LIST OF JOINT ANGLES READING, OBTAINED FROM READ_JOINT_ANGLES
    Home = [0, 0, 0]#[-87.0, -86.30000305175781, -7.400000095367432]
    right_to_pose = [34.5, -50.20000076293945, -36.099998474121094]
    offset_C_FI = [28.399999618530273, -50.5, -54.099998474121094] 
    C_FI = [8.399999618530273, -50.5, -54.099998474121094]
    offset_B = [27.400000095367432, -62.20000076293945, -46.599998474121094]
    B = [7.400000095367432, -62.20000076293945, -46.599998474121094]
    offset_A = [27.800000190734863, -72.0, -42.0]
    A = [7.800000190734863, -72.0, -42.0]
    offset_G = [26.099999904632568, -85.0, -31.200000762939453]
    G = [6.099999904632568, -85.0, -31.200000762939453]
    
    '''r_a_ref = Float64MultiArray()
    #print(Float64MultiArray())
    r_a_ref.data = right_to_pose
    #print(r_a_ref)
    right_arm_pub.publish(r_a_ref)
    rospy.sleep(3)'''
    while not rospy.is_shutdown():
        try:
            
            '''r_a_ref = Float64MultiArray()
            #print(Float64MultiArray())
            r_a_ref.data = right_to_pose
            #print(r_a_ref)
            right_arm_pub.publish(r_a_ref)'''
            '''rospy.sleep(3)
            r_a_ref = Float64MultiArray()
            #print(Float64MultiArray())
            r_a_ref.data = home
            #print(r_a_ref)
            right_arm_pub.publish(r_a_ref)
            rospy.sleep(3)
            r_a_ref = Float64MultiArray()
            #print(Float64MultiArray())
            r_a_ref.data = offset_C_FI
            #print(r_a_ref)
            right_arm_pub.publish(r_a_ref)
            rospy.sleep(3)'''
            # -------------------- START HITTING KEYS
            play_note(offset_C_FI, C_FI)
            #play_note(offset_B, B)
            #play_note(offset_A, A)
            #play_note(offset_G, G)
            
            '''rospy.loginfo("Current position : %.2f" ,right_arm_pitch_pos)
            r_a_ref.data = right_to_pos
            #print(r_a_ref)
            right_arm_pub.publish(r_a_ref)
            rospy.sleep(2)
            rospy.loginfo("Current position : %.2f" ,right_arm_pitch_pos)
            #right_arm_pitch_ref = -30 if right_arm_pitch_ref == 30 else 30'''
        except KeyboardInterrupt:
            pass
    rospy.loginfo("finsihed!")
