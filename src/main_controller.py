#!/usr/bin/env python
import os
import threading
import math, time
import rospy, rospkg
import subprocess, shlex
from datetime import datetime
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy
from kobuki_msgs.msg import Sound
from actionlib_msgs.msg import GoalID
from kobuki_msgs.msg import SensorState
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
#from move_base_msgs.msg import MoveBaseActionGoal
#from motion_detection_sensor_status_publisher.msg import SensorStatusMsg

running_motion_analysis_human = False
running_motion_analysis_obj = False
timeBasedEventsThread = None
chargingCheckThread = None
running_ros_visual = False
pc_needs_to_charge = False
kobuki_max_charge = 164 #Validated fully charged battery value
check_batteries = False
instruction_pub = None
pill_intake_mode = 1
running_hpr = False
navigating = False
charging = False
curr_battery = kobuki_max_charge
prev_battery = kobuki_max_charge
sound_pub = None
state_file = ''
next_state = 0 # States are 0=new breakfast, 1=lunch, 2=dinner, 3=breakfast
joy_pub = None

#first_detect becomes false the first time we see 'ok'
#from the motion detection sensor, to ensure that
#the first 'detect' values are not from the human's
#movements approaching and lying to bed.
first_detect = True


def init():
    global pub_stop, check_batteries, sound_pub, int_pub, instruction_pub, state_file
    rospy.init_node('radio_node_manager_main_controller')
    check_batteries = rospy.get_param("~check_batteries", True)
    instruction_topic = rospy.get_param("~instruction_topic", "radio_node_manager_main_controller/instruction")
    #rospy.Subscriber('motion_detection_sensor_status_publisher/status', SensorStatusMsg, motionSensorStatus)
    rospy.Subscriber('move_base/status', GoalStatusArray, currentNavStatus)
    rospy.Subscriber('joy', Joy, joyCallback)
    sound_pub = rospy.Publisher('mobile_base/commands/sound', Sound)
    #rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, getGoalPoint)
    pub_stop = rospy.Publisher('move_base/cancel', GoalID, queue_size=10)
    int_pub = rospy.Publisher('radio_generate_report', Int32, queue_size=1)
    instruction_pub = rospy.Publisher(instruction_topic, Int32, queue_size=1)
    rospy.Subscriber("android_app/goal", PoseStamped, androidGoal)
    rospy.Subscriber("android_app/other", Int32, androidOther)

    rospack = rospkg.RosPack()
    state_file = rospack.get_path('radio_node_manager_main_controller')+'/state/saved.state'

    timeBasedEvents()

    if os.path.isfile(state_file):
        with open(state_file) as f:
            next_state = int(f.read())

    if check_batteries:
        #rospy.Subscriber('placeholder', PlaceHolderMsg, pcBatteryCallback)
        rospy.Subscriber('mobile_base/sensors/core', SensorState, kobukiBatteryCallback)

    while not rospy.is_shutdown():
        rospy.spin()
    timeBasedEventsThread.cancel()

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
    global navigating
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

def androidGoal(goal_msg):
    global goal_publisher
    # Here we can check anything we need before publishing
    # the message coming from the user's android app
    # For now, just send the robot the goal message
    goal_publisher.publish(goal_msg)

def androidOther(msg):
    # Map:
    # -1 = Cancel navigation goal
    # More to come (?)
    if msg.data == -1:
        # Here we can check anything we need before publishing
        # the message coming from the user's android app
        # For now, just send the robot the cancel goal message
        cancelNavigationGoal()

def getGoalPoint(goal_msg):
    print goal_msg

'''
def motionSensorStatus(ssm):
    global running_motion_analysis_human, first_detect, running_hpr, sound_pub
    cur_st = ssm.status
    print curr_st
'''

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
    global curr_battery
    '''
    print msg
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
    '''
    curr_battery = msg.battery

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

def initial_pose():
    global instruction_pub
    instruction_pub.publish(5)

def saveState():
    global next_state, state_file
    with open(state_file,'w') as f:
        f.write(next_state)

def updateState():
    global next_state
    next_state += 1
    if next_state > 3:
        next_state = 0

def checkIfCharging():
    global charging, chargingCheckThread, prev_battery, curr_battery
    chargingCheckThread = threading.Timer(300, checkIfCharging)
    chargingCheckThread.start()
    if prev_battery < curr_battery:
        charging = True
        chargingCheckThread.cancel()
    elif prev_battery > curr_battery:
        charging = False
        chargingCheckThread.cancel()

def goChargeNow():
    global goal_publisher
    # TODO before docking position
    goal_msg = PoseStamped()
    goal_msg.header.stamp = rospy.Time.now()
    goal_msg.header.frame_id = 'map'
    #goal_msg.pose.position.x = 123
    #goal_msg.pose.position.y = 321
    #goal_msg.pose.orientation.z = 1
    #goal_msg.pose.orientation.z = 2
    goal_publisher.publish(goal_msg)
    # TODO Check if we have arrived in the docking position
    # and then enable auto-docking
    #dock()
    #checkIfCharging()

def dock():
    global instruction_pub
    instruction_pub.publish(4)

def goTo(x,y,z,w):
    global goal_publisher
    goal_msg = PoseStamped()
    goal_msg.header.stamp = rospy.Time.now()
    goal_msg.header.frame_id = 'map'
    goal_msg.pose.position.x = x
    goal_msg.pose.position.y = y
    goal_msg.pose.orientation.z = z
    goal_msg.pose.orientation.w = w
    goal_publisher.publish(goal_msg)

def timeBasedEvents():
    global timeBasedEventsThread, charging
    global prev_battery, curr_battery, chargingCheckThread
    # Check time every 10 minutes
    timeBasedEventsThread = threading.Timer(600, timeBasedEvents)
    timeBasedEventsThread.start()
    timestamp = time.strftime('%H:%M:%S')
    # Do something based on timestamp
    h = timestamp.split(':')[0]
    m = timestamp.split(':')[1]
    if h >= 21 and not charging:
        charging = True
        if chargingCheckThread is not None:
            if not chargingCheckThread.isAlive():
                goChargeNow()
                prev_battery = curr_battery
        else:
            goChargeNow()
            prev_battery = curr_battery
    elif h>=18 and m >=30 and next_state == 2:
        charging = False
        updateState()
        # TODO ADL position 2
        #goTo(123,321,2.1,3.2)
    elif h >= 15 and m >=30 and not charging:
        charging = True
        if chargingCheckThread is not None:
            if not chargingCheckThread.isAlive():
                goChargeNow()
                prev_battery = curr_battery
        else:
            goChargeNow()
            prev_battery = curr_battery
    elif h >= 13 and m >=15 and next_state == 1:
        charging = False
        updateState()
        # TODO ADL position 1
        # TODO Check if the robot is in the correct position
        # (it should be in the position) that was published at around 10:45
        #goTo(123,321,2.1,3.2)
    elif h >= 10 and m >= 45 and next_state == 0:
        charging = False
        updateState()
        # TODO ADL position 1
        #goTo(123,321,2.1,3.2)
    elif h >= 9 and m >= 45 and not charging:
        charging = True
        if chargingCheckThread is not None:
            if not chargingCheckThread.isAlive():
                goChargeNow()
                prev_battery = curr_battery
        else:
            goChargeNow()
            prev_battery = curr_battery
    elif h >= 7 and m >= 45 and next_state == 3:
        # TODO ADL position 1
        #goTo(123,321,2.1,3.2)

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
    '''
    if msg.buttons[5] == 1:
        #ost_pub.publish(2)
        #sound_msg = Sound()
        #sound_msg.value = 0
        #sound_pub.publish(sound_msg)
    '''
    if msg.buttons[10] == 1:
        dock()
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
            if pill_intake_mode == 1:
                instruction_pub.publish(2)
            else:
                instruction_pub.publish(22)
            command = "roslaunch motion_analysis_wrapper wrapper.launch"
            command = shlex.split(command)
            subprocess.Popen(command)
            running_motion_analysis_obj = True
            time.sleep(10)
            ost_pub.publish(2)
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