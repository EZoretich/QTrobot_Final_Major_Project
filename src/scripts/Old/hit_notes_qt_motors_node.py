#!/usr/bin/env python3
import sys
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

#right_arm_pitch_pos = 0
right_shoulder_pitch_pos = 0
right_shoulder_roll_pos = 0
right_elbow_roll_pos = 0
#mot_pos = 0
###
left_arm_pitch_pos = 0
rospy.init_node('my_tutorial_node')
rospy.loginfo("my_tutorial_node started!")

right_arm_pub = rospy.Publisher('/qt_robot/right_arm_position/command', Float64MultiArray, queue_size=1)
left_arm_pub = rospy.Publisher('/qt_robot/left_arm_position/command', Float64MultiArray, queue_size=1)
rospy.sleep(3.0)

def state_callback(msg):
    global right_shoulder_pitch_pos
    global right_shoulder_roll_pos
    global right_elbow_roll_pos
    global left_arm_pitch_pos
    #
    #global mot_pos
    right_shoulder_pitch_pos = msg.position[msg.name.index("RightShoulderPitch")]
    #added for all joints
    right_shoulder_roll_pos = msg.position[msg.name.index("RightShoulderRoll")]
    right_elbow_roll_pos = msg.position[msg.name.index("RightElbowRoll")]
    
    left_arm_pitch_pos = msg.position[msg.name.index("LeftShoulderPitch")]
    #
    #mot_pos = msg.position[]

rospy.Subscriber('/qt_robot/joints/state', JointState, state_callback)

# ------------------------ CHECK MOTORS POSITION

'''def check_pos():
my pose = get pose
ur_con.tar_check(my_pose)
    def tar_check(self,tar_pos):
        # check if we have reached target pos, ignore orientation
        i = 0
        tar_reached = False
        while not tar_reached:
            current_pos = self.get_pose()
            tar_reached = (round(current_pos.position.x, 3) == round(tar_pos.position.x, 3)) and (round(current_pos.position.y, 3) == round(tar_pos.position.y, 3)) and (round(current_pos.position.z, 3) == round(tar_pos.position.z, 3))
            i+= 1
        return True'''




# ---------- Function to play RIGHT ARM notes ~ offset included ~ ---------------------
def play_note_r(offset, key):
    # --------------------- FIRST GO TO OFFSET
    r_a_ref = Float64MultiArray()
    #print(Float64MultiArray())
    r_a_ref.data = offset
    #print(r_a_ref)
    right_arm_pub.publish(r_a_ref)
    rospy.sleep(2)
    #print("Offset Of:    ", offset)
    
    # --------------------- HIT NOTE
    
    r_a_ref = Float64MultiArray()
    #print(Float64MultiArray())
    r_a_ref.data = key
    #print(r_a_ref)
    right_arm_pub.publish(r_a_ref)
    rospy.sleep(0.5)
    #print("Note Played:    ", key)
    
    # --------------------- BACK TO OFFSET
    
    r_a_ref = Float64MultiArray()
    #print(Float64MultiArray())
    r_a_ref.data = offset
    #print(r_a_ref)
    right_arm_pub.publish(r_a_ref)
    rospy.sleep(2)
    #print("Offset Of:    ", offset)

# ---------------------------------------------------------------
# ---------- Function to play LEFT ARM notes ~ offset included ~ ---------------------
def play_note_l(offset, key):
    # --------------------- FIRST GO TO OFFSET
    l_a_ref = Float64MultiArray()
    #print(Float64MultiArray())
    l_a_ref.data = offset
    #print(l_a_ref)
    left_arm_pub.publish(l_a_ref)
    rospy.sleep(2)
    #print("Offset Of:    ", offset)
    
    # --------------------- HIT NOTE
    
    l_a_ref = Float64MultiArray()
    #print(Float64MultiArray())
    l_a_ref.data = key
    #print(l_a_ref)
    left_arm_pub.publish(l_a_ref)
    rospy.sleep(0.5)
    #print("Note Played:    ", key)
    
    # --------------------- BACK TO OFFSET
    
    l_a_ref = Float64MultiArray()
    #print(Float64MultiArray())
    l_a_ref.data = offset
    #print(l_a_ref)
    left_arm_pub.publish(l_a_ref)
    rospy.sleep(2)
    #print("Offset Of:    ", offset)


if __name__ == '__main__':
    #right_arm_pitch_ref = 30
    # LIST OF JOINT ANGLES READING, OBTAINED FROM READ_JOINT_ANGLES
    #test = [0.30000001192092896, -41.400001525878906, -9.100000381469727]
    #test2 = [60.30000001192092896, -41.400001525878906, -9.100000381469727]
    
    Home = [0, 0, 0]#[-87.0, -86.30000305175781, -7.400000095367432]
    #---------------------------------------- RIGHT ARM ----------------------------------------
    right_to_pose1 = [-40.0, -27.0, -2.9000000953674316]#[34.5, -50.20000076293945, -36.099998474121094]
    right_to_pose2 = [33.20000076293945, -25.100000381469727, -56.70000076293945]
    #right_to_pose2 = [28.399999618530273, -32.900001525878906, -3.200000047683716]
    offset_C_FI = [28.399999618530273, -50.5, -54.099998474121094] 
    C_FI = [8.399999618530273, -50.5, -54.099998474121094]
    offset_B = [27.400000095367432, -62.20000076293945, -46.599998474121094]
    B = [7.400000095367432, -62.20000076293945, -46.599998474121094]
    offset_A = [27.800000190734863, -72.0, -42.0]
    A = [7.800000190734863, -72.0, -42.0]
    offset_G = [26.099999904632568, -85.0, -31.200000762939453]
    G = [6.099999904632568, -85.0, -31.200000762939453]
    # ---------------------------------------- LEFT ARM ----------------------------------------
    #left_to_pose = [-9.399999618530273, -43.29999923706055, -13.0]
    left_to_pose1 = [40.0, -27.0, -2.9000000953674316]
    left_to_pose2 = [-33.20000076293945, -25.100000381469727, -56.70000076293945]
    offset_C_R = [-31.699999809265137, -41.0, -57.0]
    C_R = [-11.699999809265137, -41.0, -57.0]
    offset_D = [-31.0, -51.79999923706055, -51.79999923706055]
    D = [-11.0, -51.79999923706055, -51.79999923706055]
    offset_E = [-31.0, -64.80000305175781, -44.0]
    E = [-11.0, -64.80000305175781, -44.0]
    offset_F = [-31.0, -75.9000015258789, -37.099998474121094]
    F = [-11.0, -75.9000015258789, -37.099998474121094]
    
    
    # TESTING CHECK MOTOR POSITION
    pose_1 = [0.0, 0.0, 0.0]
    pose_2 = [33.0, -25.0, -56.0]
    
    #print(right_shoulder_pitch_pos)
    
    '''r_a_ref = Float64MultiArray()
    r_a_ref.data = pose_1
    right_arm_pub.publish(r_a_ref)
    rospy.sleep(2)
    print("Sh_pitch, Sh_roll, El_roll:	", right_shoulder_pitch_pos, right_shoulder_roll_pos, right_elbow_roll_pos)
    #print(JointState)
    #print(mot_pos)
    
    r_a_ref = Float64MultiArray()
    r_a_ref.data = pose_2
    right_arm_pub.publish(r_a_ref)
    rospy.sleep(5)
    print("Sh_pitch, Sh_roll, El_roll:	", right_shoulder_pitch_pos, right_shoulder_roll_pos, right_elbow_roll_pos)'''
    
    #right_shoulder_pitch_pos, right_shoulder_roll_pos, right_elbow_roll_pos
    l_a_ref = Float64MultiArray()
    #print(Float64MultiArray())
    l_a_ref.data = Home
    #print(r_a_ref)
    left_arm_pub.publish(l_a_ref)
    rospy.sleep(2)
    r_a_ref = Float64MultiArray()
    #print(Float64MultiArray())
    r_a_ref.data = Home
    #print(r_a_ref)
    right_arm_pub.publish(r_a_ref)
    rospy.sleep(2)
  
    # ---------- RIGHT TO POSE
    r_a_ref = Float64MultiArray()
    #print(Float64MultiArray())
    r_a_ref.data = right_to_pose1
    #print(r_a_ref)
    right_arm_pub.publish(r_a_ref)
    rospy.sleep(2)
    
    r_a_ref = Float64MultiArray()
    #print(Float64MultiArray())
    r_a_ref.data = right_to_pose2
    #print(r_a_ref)
    right_arm_pub.publish(r_a_ref)
    rospy.sleep(2)
    
    # ---------- LEFT TO POSE
    
    l_a_ref = Float64MultiArray()
    #print(Float64MultiArray())
    l_a_ref.data = left_to_pose1
    #print(r_a_ref)
    left_arm_pub.publish(l_a_ref)
    rospy.sleep(2)
    
    l_a_ref = Float64MultiArray()
    #print(Float64MultiArray())
    l_a_ref.data = left_to_pose2
    #print(r_a_ref)
    left_arm_pub.publish(l_a_ref)
    rospy.sleep(2)
    
    '''r_a_ref = Float64MultiArray()
    #print(Float64MultiArray())
    r_a_ref.data = right_to_pose
    #print(r_a_ref)
    right_arm_pub.publish(r_a_ref)
    rospy.sleep(3)'''
    while not rospy.is_shutdown():
        try:

            # -------------------- ARM get ready POSITION

            # -------------------- START HITTING KEYS --> RIGHT ARM
            #play_note_r(offset_C_FI, C_FI)
            
            play_note_r(offset_B, B)
            play_note_r(offset_A, A)
            play_note_r(offset_G, G)
            
            # -------------------- START HITTING KEYS --> LEFT ARM
            play_note_l(offset_C_R, C_R)
            play_note_l(offset_D, D)
            play_note_l(offset_E, E)
            play_note_l(offset_F, F)
            '''
            rospy.loginfo("Current position : %.2f" ,right_arm_pitch_pos)
            r_a_ref.data = G
            #print(r_a_ref)
            right_arm_pub.publish(r_a_ref)
            rospy.sleep(4)
            rospy.loginfo("Current position : %.2f" ,right_arm_pitch_pos)
            #right_arm_pitch_ref = -30 if right_arm_pitch_ref == 30 else 30'''
        except KeyboardInterrupt:
            break #todo replace me
    rospy.loginfo("finsihed!")
    
    
