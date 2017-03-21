#!/usr/bin/env python
import os
import math, time
import rospy
import subprocess, shlex
from datetime import datetime
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy
from kobuki_msgs.msg import Sound
from actionlib_msgs.msg import GoalID
from kobuki_msgs.msg import SensorState
from sensor_msgs.msg import BatteryState
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionGoal
from motion_detection_sensor_status_publisher.msg import SensorStatusMsg

running_motion_analysis_human = False
running_motion_analysis_obj = False
running_ros_visual = False
movement_sensor_sub = None
pc_needs_to_charge = False
kobuki_battery_sub = None
kobuki_max_charge = 164 #Validated fully charged battery value
check_batteries = False
nav_status_sub = None
pc_battery_sub = None
running_hpr = False
navigating = False
charging = False
sound_pub = None
instruction_pub = None
goal_point = []
int_sub = None
joy_pub = None

#first_detect becomes false the first time we see 'ok'
#from the motion detection sensor, to ensure that
#the first 'detect' values are not from the human's
#movements approaching and lying to bed.
first_detect = True


def init():
    global movement_sensor_sub, nav_status_sub, pub_stop, pc_battery_sub, kobuki_battery_sub
    global check_batteries, joy_sub, sound_pub, int_pub, instruction_pub
    rospy.init_node('radio_node_manager_main_controller')
    check_batteries = rospy.get_param("~check_batteries", False)
    instruction_topic = rospy.get_param("~instruction_topic", "radio_node_manager_main_controller/instruction")
    movement_sensor_sub = rospy.Subscriber('motion_detection_sensor_status_publisher/status', SensorStatusMsg, motionSensorStatus)
    nav_status_sub = rospy.Subscriber('move_base/status', GoalStatusArray, currentNavStatus)
    joy_sub = rospy.Subscriber('joy', Joy, joyCallback)
    sound_pub = rospy.Publisher('mobile_base/commands/sound', Sound)
    goal_subscriber = rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, getGoalPoint)
    pub_stop = rospy.Publisher('move_base/cancel', GoalID, queue_size=10)
    int_pub = rospy.Publisher('radio_generate_report', Int32, queue_size=1)
    instruction_pub = rospy.Publisher(instruction_topic, Int32, queue_size=1)
    if check_batteries:
        #pc_battery_sub = rospy.Subscriber('placeholder', PlaceHolderMsg, pcBatteryCallback)
        kobuki_battery_sub = rospy.Subscriber('mobile_base/sensors/core', SensorState, kobukiBatteryCallback)

    while not rospy.is_shutdown():  
        rospy.spin()

'''
0   # The goal has yet to be processed by the action server
1   # The goal is currently being processed by the action server
2   # The goal received a cancel request after it started executing
    # and has since completed its execution (Terminal State)
3   # The goal was achieved successfully by the action server (Terminal State)
4   # The goal was aborted during execution by the action server due
    # to some failure (Terminal State)
5   # The goal was rejected by the action server without being processed,
    # because the goal was unattainable or invalid (Terminal State)
6   # The goal received a cancel request after it started executing
    # and has not yet completed execution
7   # The goal received a cancel request before it started executing,
    # but the action server has not yet confirmed that the goal is canceled
8   # The goal received a cancel request before it started executing
    # and was successfully cancelled (Terminal State)
9   # An action client can determine that a goal is LOST. This should not be
    # sent over the wire by an action server
'''
def currentNavStatus(current_status_msg):
    global goal_point, goal_reached, navigating
    if len(current_status_msg.status_list) > 0:
        status =  current_status_msg.status_list[0].status
        if navigating:
            if status == 3:
                navigating = False
                #print 'Starting motion_analysis'
                #print 'Starting HPR'
                print 'Reached destination!'

            if status > 3:
                navigating = False
                print 'Send navigation error to the user'
        else:
            if status == 1:
                HPR(False)
                rosVisual(False)
                motionAnalysisHuman(False)
                motionAnalysisObject(False)
                navigating = True

def getGoalPoint(goal_msg):
    global gym_x, gym_y
    print goal_msg

def motionSensorStatus(ssm):
    global running_motion_analysis_human, first_detect, running_hpr, sound_pub
    global movement_sensor_sub
    cur_st = ssm.status
    print curr_st

#this method only changes the pc_needs_to_charge value. The rest are left for
#the kobukiBatteryCallback method.
def pcBatteryCallback(msg):
    global pc_needs_to_charge
    print msg
    if msg.percentage*100 < 0.05:
        pc_needs_to_charge = True
    else:
        pc_needs_to_charge = False


def kobukiBatteryCallback(msg):
    global kobuki_max_charge, charging, pc_needs_to_charge, navigating
    #print msg
    if (msg.battery/kobuki_max_charge*100) < 0.05 or pc_needs_to_charge: #less that 5% battery on kobuki
        if msg.charger == 0:
            charging = False
            print 'I am not charging, and I definitely need to!'
            if navigating:
                cancelNavigationGoal()
                print 'Message the user about my current battery state'
                print 'Now I need to navigate back to my base. Publish such message!'
        else:
            print 'Charging'
            charging = True

def cancelNavigationGoal():
    global pub_stop, navigating, sound_pub
    print 'Cancelling navigation goal'
    pub_stop.publish(GoalID())
    navigating = False
    sound_msg = Sound()
    sound_msg.value = 0
    sound_pub.publish(sound_msg)


def createReport():
    global int_pub
    int_pub.publish(0)




def joyCallback(msg):
    global ost_pub, instruction_pub
    #X starts HPR
    #A starts ros_visual
    #B starts motion_analysis for human
    #Y starts motion_analysis for object
    #Up/Forward on cross-pad initializes robot position
    #Back/Select to cancel navigation goal
    #Start sends a report based on today's date.
    #R1 pills are placed to the correct position
    #RightStick[Pressed] enables auto-docking
    #R2+X stops HPR
    #R2+A stops ros_visual
    #R2+B stops motion_analysis for human
    #R2+Y stops motion_analysis for object
    #Combinations of the A-B-X-Y buttons are disabled.
    #You always have to press one of the buttons.
    if msg.buttons[0] == 0 and msg.buttons[1] == 0 and msg.buttons[2] == 1 and msg.buttons[3] == 0 and (msg.axes[5] ==0 or msg.axes[5] == 1):
        HPR(True)
    elif msg.buttons[0] == 1 and msg.buttons[1] == 0 and msg.buttons[2] == 0 and msg.buttons[3] == 0 and (msg.axes[5] ==0 or msg.axes[5] == 1):
        rosVisual(True)
    elif msg.buttons[0] == 0 and msg.buttons[1] == 1 and msg.buttons[2] == 0 and msg.buttons[3] == 0 and (msg.axes[5] ==0 or msg.axes[5] == 1):
        motionAnalysisHuman(True)
    elif msg.buttons[0] == 0 and msg.buttons[1] == 0 and msg.buttons[2] == 0 and msg.buttons[3] == 1 and (msg.axes[5] ==0 or msg.axes[5] == 1):
        motionAnalysisObject(True)
    elif msg.axes[7] == 1:
        initial_pose()
    if msg.buttons[6] == 1:
        cancelNavigationGoal()
    if msg.buttons[7] == 1:
        createReport()
    if msg.buttons[5] == 1:
        ost_pub.publish(2)
        sound_msg = Sound()
        sound_msg.value = 0
        sound_pub.publish(sound_msg)
    if msg.buttons[10] == 1:
        instruction_pub.publish(4)
    if msg.buttons[2] == 1 and msg.axes[5] != 0 and msg.axes[5] != 1:
        HPR(False)
    if msg.buttons[0] == 1 and msg.axes[5] != 0 and msg.axes[5] != 1:
        rosVisual(False)
    if msg.buttons[1] == 1 and msg.axes[5] != 0 and msg.axes[5] != 1:
        motionAnalysisHuman(False)
    if msg.buttons[3] == 1 and msg.axes[5] != 0 and msg.axes[5] != 1:
        motionAnalysisObject(False)

def HPR(start):
    global running_hpr, sound_pub, instruction_pub
    if start:
        if not running_hpr:
            instruction_pub.publish(0)
            command = "roslaunch hpr_wrapper  wrapper.launch"
            command = shlex.split(command)
            subprocess.Popen(command)
            running_hpr = True
    else:
        if running_hpr:
            command = "rosnode kill laser_analysis"
            command = shlex.split(command)
            subprocess.Popen(command)
            command = "rosnode kill laser_overlap_trace"
            command = shlex.split(command)
            subprocess.Popen(command)
            command = "rosnode kill laser_clustering"
            command = shlex.split(command)
            subprocess.Popen(command)
            command = "rosnode kill laser_wall_extraction"
            command = shlex.split(command)
            subprocess.Popen(command)
            subprocess.Popen(command)
            command = "rosnode kill hpr_wrapper"
            command = shlex.split(command)
            subprocess.Popen(command)
            running_hpr = False
            sound_msg = Sound()
            sound_msg.value = 1
            sound_pub.publish(sound_msg)

def motionAnalysisHuman(start):
    global running_motion_analysis_human, sound_pub, instruction_pub
    if start:
        if not running_motion_analysis_human:
            instruction_pub.publish(1)
            command = "roslaunch motion_analysis_wrapper wrapper.launch"
            command = shlex.split(command)
            subprocess.Popen(command)
            running_motion_analysis_human = True
    else:
        if running_motion_analysis_human:
            command = "rosnode kill motion_analysis"
            command = shlex.split(command)
            subprocess.Popen(command)
            command = "rosnode kill motion_analysis_wrapper"
            command = shlex.split(command)
            subprocess.Popen(command)
            running_motion_analysis_human = False
            sound_msg = Sound()
            sound_msg.value = 1
            sound_pub.publish(sound_msg)

def motionAnalysisObject(start):
    global running_motion_analysis_obj, sound_pub, instruction_pub
    if start:
        if not running_motion_analysis_obj:
            instruction_pub.publish(2)
            command = "roslaunch motion_analysis_wrapper wrapper.launch"
            command = shlex.split(command)
            subprocess.Popen(command)
            running_motion_analysis_obj = True
            sound_msg = Sound()
            sound_msg.value = 0
            sound_pub.publish(sound_msg)
    else:
        if running_motion_analysis_obj:
            command = "rosnode kill motion_analysis"
            command = shlex.split(command)
            subprocess.Popen(command)
            command = "rosnode kill motion_analysis_wrapper"
            command = shlex.split(command)
            subprocess.Popen(command)
            running_motion_analysis_obj = False
            sound_msg = Sound()
            sound_msg.value = 1
            sound_pub.publish(sound_msg)

def rosVisual(start):
    global running_ros_visual, sound_pub, instruction_pub
    if start:
        if not running_ros_visual:
            instruction_pub.publish(3)
            command = "roslaunch ros_visual_wrapper wrapper.launch"
            command = shlex.split(command)
            subprocess.Popen(command)
            running_ros_visual = True
            sound_msg = Sound()
            sound_msg.value = 0
            sound_pub.publish(sound_msg)
    else:
        if running_ros_visual:
            command = "rosnode kill decision_making"
            command = shlex.split(command)
            subprocess.Popen(command)
            command = "rosnode kill fusion"
            command = shlex.split(command)
            subprocess.Popen(command)
            command = "rosnode kill depth"
            command = shlex.split(command)
            subprocess.Popen(command)
            command = "rosnode kill chroma"
            command = shlex.split(command)
            subprocess.Popen(command)
	    command = "rosnode kill classifier"
            command = shlex.split(command)
            subprocess.Popen(command)
            command = "rosnode kill ros_visual_wrapper"
            command = shlex.split(command)
            subprocess.Popen(command)
            running_ros_visual = False
            sound_msg = Sound()
            sound_msg.value = 1
            sound_pub.publish(sound_msg)

if __name__ == '__main__':
    init() 

