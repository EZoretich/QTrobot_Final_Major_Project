#!/usr/bin/env python

import rospy
import time
from musi_care.srv import qt_command
from qt_motors_controller.srv import *
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
import librosa
import librosa.display
import pyaudio
import wave
import numpy as np
from matplotlib import pyplot as plt
from glob import glob
import soundfile as sf
import joblib
from sklearn.neighbors import KNeighborsClassifier
import csv


class QT_Controller:

    def __init__(self):
        rospy.init_node('QT_Xylophone')

        self.right_shoulder_pitch_pos = 0
        self.right_shoulder_roll_pos = 0
        self.right_elbow_roll_pos = 0
        self.left_shoulder_pitch_pos = 0
        self.left_shoulder_roll_pos = 0
        self.left_elbow_roll_pos = 0

        rospy.Subscriber('/qt_robot/joints/state', JointState, self.state_callback)

    def state_callback(self, msg):
        self.right_shoulder_pitch_pos = msg.position[msg.name.index("RightShoulderPitch")]
        self.right_shoulder_roll_pos = msg.position[msg.name.index("RightShoulderRoll")]
        self.right_elbow_roll_pos = msg.position[msg.name.index("RightElbowRoll")]
        self.left_shoulder_pitch_pos = msg.position[msg.name.index("LeftShoulderPitch")]
        self.left_shoulder_roll_pos = msg.position[msg.name.index("LeftShoulderRoll")]
        self.left_elbow_roll_pos = msg.position[msg.name.index("LeftElbowRoll")]

    def change_mot_speed(speed_percent):
        action_type = "velocity"
        action_content = speed_percent
        command_blocking = False

        rospy.wait_for_service('/qt_command_service')
        command_controller = rospy.ServiceProxy('/qt_command_service', qt_command)
        #command_complete = command_controller(action_type, action_content, command_blocking)

        #print(command_complete)

    def get_ready(self, arm, move_1, move_2, move_3):
        print("----- GET READY -----")
        start = rospy.get_time()  # ---- Start Timing
        action_type = arm
        action_content = move_1
        command_blocking = False

        command_complete = self.command_controller(action_type, action_content, command_blocking)
        rospy.sleep(0.1)
        print(command_complete)
        finish = rospy.get_time()  # ----- Stop Timing
        elapsed = round((finish - start), 3)  # ----- Calculate Time Elapsed
        print(elapsed, "   Sec.")
        print("----------", action_type, "----------")
        print("DESIRED POS:	", action_content)
        if action_type == "right_arm":
            actual_pos = [self.r_s_pitch_pos, self.r_s_roll_pos, self.r_e_roll_pos]
            print("REAL POS:	", actual_pos)
        else:
            actual_pos = [self.l_s_pitch_pos, self.l_s_roll_pos, self.l_e_roll_pos]
            print("REAL POS:	", actual_pos)

        # rospy.sleep(0.1)

        # ------------------------------------------------------------------------
        start = rospy.get_time()  # ----- Start Timing
        action_type = arm
        action_content = move_2
        command_blocking = False

        command_complete = self.command_controller(action_type, action_content, command_blocking)
        rospy.sleep(0.1)
        print(command_complete)
        finish = rospy.get_time()  # ----- Stop Timing
        elapsed = round((finish - start), 3)  # ----- Calculate Time Elapsed
        print(elapsed, "   Sec.")
        print("----------", action_type, "----------")
        print("DESIRED POS:	", action_content)
        if action_type == "right_arm":
            actual_pos = [self.r_s_pitch_pos, self.r_s_roll_pos, self.r_e_roll_pos]
            print("REAL POS:	", actual_pos)
        else:
            actual_pos = [self.l_s_pitch_pos, self.l_s_roll_pos, self.l_e_roll_pos]
            print("REAL POS:	", actual_pos)

        # rospy.sleep(0.1)

        # ------------------------------------------------------------------------
        start = rospy.get_time()  # ----- Start Timing
        action_type = arm
        action_content = move_3
        command_blocking = False

        command_complete = self.command_controller(action_type, action_content, command_blocking)
        rospy.sleep(0.1)
        print(command_complete)
        finish = rospy.get_time()  # ----- Stop Timing
        elapsed = round((finish - start), 3)  # ----- Calculate Time Elapsed
        print(elapsed, "   Sec.")
        print("----------", action_type, "----------")
        print("DESIRED POS:	", action_content)
        if action_type == "right_arm":
            actual_pos = [self.r_s_pitch_pos, self.r_s_roll_pos, self.r_e_roll_pos]
            print("REAL POS:	", actual_pos)
        else:
            actual_pos = [self.l_s_pitch_pos, self.l_s_roll_pos, self.l_e_roll_pos]
            print("REAL POS:	", actual_pos)

        # rospy.sleep(0.1)



    def play_note(self, arm, key, offset):
        print("----- PLAY NOTE -----")
        actual_pos = [self.right_shoulder_pitch_pos, self.right_shoulder_roll_pos, self.right_elbow_roll_pos]
        # ---------- OFFSET
        start = rospy.get_time() # ----- Start Timing
        action_type = arm
        action_content = offset
        command_blocking = False
        #print(key)

        rospy.wait_for_service('/qt_command_service')
        command_controller = rospy.ServiceProxy('/qt_command_service', qt_command)
        command_complete = command_controller(action_type, action_content, command_blocking)
        rospy.sleep(0.1)
        #check_pos(actual_pos,  action_content)
        print(command_complete)
        finish = rospy.get_time() # ----- Stop Timing
        elapsed = round((finish - start), 3) # ----- Calculate Time Elapsed
        print(elapsed, "   Sec.")
        print("----------", action_type, "----------")
        print("DESIRED POS:	", action_content)
        if action_type == "right_arm":
            actual_pos = [self.r_s_pitch_pos, self.r_s_roll_pos, self.r_e_roll_pos]
            print("REAL POS:	", actual_pos)
        else:
            actual_pos = [self.l_s_pitch_pos, self.l_s_roll_pos, self.l_e_roll_pos]
            print("REAL POS:	", actual_pos)

        #rospy.sleep(2)

        # ---------- NOTE
        start = rospy.get_time() # ----- Start Timing
        action_type = arm
        action_content = key
        command_blocking = False

        rospy.wait_for_service('/qt_command_service')
        command_controller = rospy.ServiceProxy('/qt_command_service', qt_command)
        command_complete = command_controller(action_type, action_content, command_blocking)
        rospy.sleep(0.1)
        #check_pos(actual_pos,  action_content)
        print(command_complete)
        finish = rospy.get_time() # ----- Stop Timing
        elapsed = round((finish - start), 3) # ----- Calculate Time Elapsed
        print(elapsed, "   Sec.")
        print("----------", action_type, "----------")
        print("DESIRED POS:	", action_content)
        if action_type == "right_arm":
            actual_pos = [self.r_s_pitch_pos, self.r_s_roll_pos, self.r_e_roll_pos]
            print("REAL POS:	", actual_pos)
        else:
            actual_pos = [self.l_s_pitch_pos, self.l_s_roll_pos, self.l_e_roll_pos]
            print("REAL POS:	", actual_pos)

        #rospy.sleep(2)
        # ---------- OFFSET 
        start = rospy.get_time() # ----- Start Timing
        action_type = arm
        action_content = offset
        command_blocking = False

        rospy.wait_for_service('/qt_command_service')
        command_controller = rospy.ServiceProxy('/qt_command_service', qt_command)
        command_complete = command_controller(action_type, action_content, command_blocking)
        rospy.sleep(0.1)
        #check_pos(actual_pos,  action_content)
        print(command_complete)
        finish = rospy.get_time() # ----- Stop Timing
        elapsed = round((finish - start), 3) # ----- Calculate Time Elapsed
        print(elapsed, "   Sec.")
        print("----------", action_type, "----------")
        print("DESIRED POS:	", action_content)
        if action_type == "right_arm":
            actual_pos = [self.r_s_pitch_pos, self.r_s_roll_pos, self.r_e_roll_pos]
            print("REAL POS:	", actual_pos)
        else:
            actual_pos = [self.l_s_pitch_pos, self.l_s_roll_pos, self.l_e_roll_pos]
            print("REAL POS:	", actual_pos)

        #rospy.sleep(2)


    def go_home(self, arm, pose):
        start = rospy.get_time() # ----- Start Timing
        action_type = arm
        action_content = pose
        command_blocking = False

        rospy.wait_for_service('/qt_command_service')
        command_controller = rospy.ServiceProxy('/qt_command_service', qt_command)
        command_complete = command_controller(action_type, action_content, command_blocking)
        rospy.sleep(0.1)
        print(command_complete)
        finish = rospy.get_time() # ----- Stop Timing
        elapsed = round((finish - start), 3) # ----- Calculate Time Elapsed
        print(elapsed, "   Sec.")
        print("----------", action_type, "----------")
        print("DESIRED POS:	", action_content)
        if action_type == "right_arm":
            actual_pos = [self.r_s_pitch_pos, self.r_s_roll_pos, self.r_e_roll_pos]
            print("REAL POS:	", actual_pos)
        else:
            actual_pos = [self.l_s_pitch_pos, self.l_s_roll_pos, self.l_e_roll_pos]
            print("REAL POS:	", actual_pos)


    def record_melody(rec_length):
        CHUNK = 1024
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 22050 #44100

        p =pyaudio.PyAudio()

        stream = p.open(format=FORMAT,
		        channels=CHANNELS,
		        rate=RATE,
		        input=True,
		        frames_per_buffer=CHUNK)

        print("Start Recording ...")
        frames = []
        seconds = rec_length
        for i in range(0, int(RATE/ CHUNK * seconds)):
            data = stream.read(CHUNK)
            frames.append(data)

        print("Recording Stopped.")
        stream.stop_stream()
        stream.close()
        p.terminate()


        wf = wave.open("User_Melody.wav", 'wb')
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(p.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(frames))
        wf.close()

    # --------------------------- SLICE MELODY BY NOTE DETECTED

    def trim_song_by_notes(raw, note_db, trim_samples):
        onset_env = librosa.onset.onset_strength(y = raw, sr=sr, S = note_db)
        peaks = librosa.util.peak_pick(onset_env,pre_max=2, post_max=2, pre_avg=3, post_avg=5, delta=3.0, wait=10)


        #print(len(raw)) #340452
        #print(peaks) #[ 45 119 192 264 337 406 478 554]
        #print(len(onset_env)) #665
        peaks_raw = []

        env_len = len(onset_env)
        raw_len = len(raw)
        for p in peaks:
            prop = round((p*raw_len)/env_len) - 1000 # - 1000 because without, it trims out part of the next note/begin of same note
            peaks_raw.append(prop)
        print(peaks_raw)

        for i in range(len(peaks_raw) - 1):
            start = peaks_raw[i]
            end = peaks_raw[i+1]
            print(start, end)
            segment = raw[start:end]
            #plot_wave(segment)
            trim_samples.append(segment)

        # Add the last segment (from the last peak to the end of the array)
        trim_samples.append(raw[peaks_raw[-1]:])
        #plot_wave(raw)
        return trim_samples
        

