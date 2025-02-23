#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from tf.transformations import quaternion_from_euler
from stockfish import Stockfish
import queue
import sounddevice as sd
import vosk
import json
import random
import dev.FilePathFinder as FilePathFinder
#from sensor_msgs.msg import JointState


FLOAT_TOLERANCE_PRECISE = 1e-9
FLOAT_TOLERANCE_LOOSE = 1e-4

JOINT_MAX = []
JOINT_MIN = []


def initRobot(velocityScale = 1.0, accelerationScale = 1.0, maxPlanTime = 100):
    # Initialize MoveIt and ROS
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("raise_arm", anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("manipulator")
    
    if velocityScale >= 0 :
        if velocityScale - 3.14 >= FLOAT_TOLERANCE_PRECISE:
            velocityScale = 3.14
        group.set_max_velocity_scaling_factor(velocityScale)
    if accelerationScale >= 0:
        group.set_max_acceleration_scaling_factor(accelerationScale)
    if maxPlanTime - 0.0 >= 0:
        group.set_planning_time(maxPlanTime)

    return group, robot, scene



def getVoiceModelandQ(model_path = FilePathFinder.getPath("voiceModel")):
    model = vosk.Model(model_path)
    q = queue.Queue()
    return model, q

    

def setChessEngine(depth,level,enginePath=FilePathFinder.getPath("chessEngine")):
    if level > 20:
        level = 20
    stockfish = Stockfish(enginePath)
    stockfish.set_depth(20)
    stockfish.set_skill_level(20)
    return stockfish

def setFenPos(stockfish,fen_position = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"):
    stockfish.set_fen_position(fen_position)

def getMove(stockfish):
    bm = stockfish.get_best_move()
    print(f"AI's bestMove: {bm}")
    return bm

#the angles are in radians 1 rad * pi= 180 deg
def rotateJointsAbs(group,posList = [0, -1.57, 1.57, -1.57, -1.57, 0]):
    group.go(posList,wait=True)
    group.stop()
    group.clear_pose_targets()
    

def audio_callback(indata, frames, time, status):
    """Callback function to receive audio data from the microphone."""
    if status:
        print(status)
    q.put(bytes(indata))

def recognize_speech(model,q):
    """Captures speech from the microphone and returns it as a string."""
    recognized_text = ""

    # Configure the microphone settings
    with sd.RawInputStream(samplerate=16000, blocksize=8000, dtype="int16",
                           channels=1, callback=audio_callback):
        print("Speak into the microphone...")

        recognizer = vosk.KaldiRecognizer(model, 16000)

        while True:
            data = q.get()
            if recognizer.AcceptWaveform(data):
                result = json.loads(recognizer.Result())
                recognized_text = result["text"]
                print("Recognized:", recognized_text)
                return recognized_text  # Return the recognized text immediately

def getRandomJointConfig(): #all joints support +-360 deg, but it raises an error for some joints
    ls = []
    ls.append(random.randint(-628,628)/100.0)
    ls.append(random.randint(-314,0)/100.0)
    ls.append(random.randint(-314,314)/100.0)
    ls.append(random.randint(-314,314)/100.0)
    ls.append(random.randint(-314,314)/100.0) 
    ls.append(random.randint(-314,314)/100.0) #last writst angle max and min???
    print(ls)
    return ls

def moveArmWithText(text):
    if text == "rotate base":
        rotateJointsAbs(group,posList=[6.28,0,0,0,0,0])
    elif text == "reverse rotate":
        rotateJointsAbs(group,posList=[-6.28,0,0,0,0,0])
    elif text == "initialize":
        rotateJointsAbs(group)
    elif text == "random random random":
        rotateJointsAbs(group,getRandomJointConfig())
        
    elif text == "quit":
        return 'q'
    return 'c'


group, robot, scene = initRobot()
model, q = getVoiceModelandQ()



while True:
    ret = moveArmWithText(recognize_speech(model,q))
    if ret == 'q':
        break


# # Get current pose
# pose_goal = group.get_current_pose().pose
# # Raise the arm by increasing the Z position
# pose_goal.position.z += 0.05  # Raise by 40 cm

# # Convert Euler angles (roll, pitch, yaw) to quaternion
# roll = -1.5  # No roll
# pitch = 1.5  # No pitch
# yaw = -1.57  # Rotate 90 degrees around Z-axis (in radians)

# q = quaternion_from_euler(roll, pitch, yaw)

# # Apply the new orientation
# pose_goal.orientation.x = q[0]
# pose_goal.orientation.y = q[1]
# pose_goal.orientation.z = q[2]
# pose_goal.orientation.w = q[3]

# # Set target pose and execute movement
# group.set_pose_target(pose_goal)
# plan = group.go(wait=True)

# # Stop movement and clear targets
# group.stop()
# group.clear_pose_targets()

# Shutdown MoveIt
moveit_commander.roscpp_shutdown()
rospy.loginfo("Arm Raised Successfully!")
