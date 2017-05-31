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
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID
from kobuki_msgs.msg import SensorState
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from radio_services.srv import InstructionWithAnswer
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
goal_publisher = None
pill_intake_mode = 1
goal_reached = False
running_hpr = False
docking_pos = None
navigating = False
curr_battery = 0
prev_battery = 0
charging = False
sound_pub = None
twist_pub = None
adl_pos1 = None
adl_pos2 = None
state_file = ''
next_state = 0 # States are 0 = new breakfast (day 1), 1=lunch (day 2), 2 = dinner (day 2), 3 = breakfast (day 3)
joy_pub = None


def init():
    global pub_stop, check_batteries, sound_pub
    global state_file, goal_publisher, twist_pub
    rospy.init_node('radio_node_manager_main_controller')
    check_batteries = rospy.get_param("~check_batteries", True)
    #rospy.Subscriber('motion_detection_sensor_status_publisher/status', SensorStatusMsg, motionSensorStatus)
    rospy.Subscriber('move_base/status', GoalStatusArray, currentNavStatus)
    rospy.Subscriber('joy', Joy, joyCallback)
    sound_pub = rospy.Publisher('mobile_base/commands/sound', Sound, queue_size=1)
    #rospy.Subscriber("/move_base/goal", PoseStamped, getGoalPoint)
    pub_stop = rospy.Publisher('move_base/cancel', GoalID, queue_size=10)
    twist_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)
    rospy.Subscriber("android_app/goal", PoseStamped, androidGoal)
    rospy.Subscriber("android_app/other", Int32, androidOther)
    goal_publisher = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)

    rospack = rospkg.RosPack()
    state_file = rospack.get_path('radio_node_manager_main_controller')+'/state/saved.state'

    if os.path.isfile(state_file):
        with open(state_file, 'r+') as f:
            next_state = int(f.read())

    if check_batteries:
        #rospy.Subscriber('placeholder', PlaceHolderMsg, pcBatteryCallback)
        rospy.Subscriber('mobile_base/sensors/core', SensorState, kobukiBatteryCallback)

    time.sleep(20)
    initGoalPoints()
    initial_pose()
    clear_costmap()
    #timeBasedEvents()

    while not rospy.is_shutdown():
        rospy.spin()
    if timeBasedEventsThread is not None:
        timeBasedEventsThread.cancel()
    if chargingCheckThread is not None:
        chargingCheckThread.cancel()

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
    global navigating, goal_reached
    if len(current_status_msg.status_list) > 0:
        status =  current_status_msg.status_list[0].status
        if navigating:
            if status == 3:
                navigating = False
                goal_reached = True
                #print 'Starting motion_analysis'
                #print 'Starting HPR'
                print 'Reached destination!'

            if status > 3:
                navigating = False
                print 'Send navigation error to the user'
        else:
            if status == 1:
                goal_reached = False
                HPR(False)
                rosVisual(False)
                motionAnalysisHuman(False)
                motionAnalysisObject(False)
                navigating = True

def androidGoal(goal_msg):
    global goal_publisher, docking_pos
    # Here we can check anything we need before publishing
    # the message coming from the user's android app
    # For now, just send the robot the goal message
    print goal_msg
    clear_costmap()
    goal_publisher.publish(goal_msg)
    # Check if the goal point is the "My room" position
    # and if it is, go back to the charging position after 3 minutes
    if goal_msg.pose.position.x == -2.798 and goal_msg.pose.position.y == 7.444 and goal_msg.pose.orientation.z == -0.538 and goal_msg.pose.orientation.w == 0.843:
        time.sleep(300)
        goTo(docking_pos)

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
    if msg.percentage*100 < 0.05:
        pc_needs_to_charge = True
    else:
        pc_needs_to_charge = False


def kobukiBatteryCallback(msg):
    global kobuki_max_charge, charging, pc_needs_to_charge, navigating
    global curr_battery
    curr_battery = msg.battery

def cancelNavigationGoal():
    global pub_stop, navigating, sound_pub
    pub_stop.publish(GoalID())
    navigating = False
    sound_msg = Sound()
    sound_msg.value = 0
    sound_pub.publish(sound_msg)

def createReport():
    rospy.wait_for_service('radio_report_generation')
    try:
        service = rospy.ServiceProxy('radio_report_generation', InstructionWithAnswer)
        answer = service(0)
        if answer:
            sound_msg = Sound()
            sound_msg.value = 0
            sound_pub.publish(sound_msg)
    except rospy.ServiceException, e:
        print e
def initial_pose():
    rospy.wait_for_service('robot_instruction_receiver')
    try:
        service = rospy.ServiceProxy('robot_instruction_receiver', InstructionWithAnswer)
        answer = service(5)
    except rospy.ServiceException, e:
        print e
    time.sleep(10)

def clear_costmap():
    command = "rosservice call /move_base/clear_costmaps"
    command = shlex.split(command)
    subprocess.Popen(command)
    time.sleep(5)

def saveState():
    global next_state, state_file
    with open(state_file,'a+') as f:
        f.write(str(next_state))

def updateState():
    global next_state
    next_state += 1
    if next_state > 3:
        next_state = 0
    saveState()

def setState(s):
    global next_state
    next_state = s
    saveState()

def blindBack():
    global twist_pub
    cmd_vel = Twist()
    cmd_vel.linear.x = -0.5
    dt = datetime.now()
    start = dt.minute*60000000 + dt.second*1000000 + dt.microsecond
    finish = start
    while finish - start < 300000:
        twist_pub.publish(cmd_vel)
        dt = datetime.now()
        finish = dt.minute*60000000 + dt.second*1000000 + dt.microsecond
    time.sleep(2)

def checkIfCharging():
    global charging, chargingCheckThread, prev_battery, curr_battery
    chargingCheckThread = threading.Timer(120, checkIfCharging)
    chargingCheckThread.start()
    if prev_battery < curr_battery:
        charging = True
        chargingCheckThread.cancel()
    elif prev_battery > curr_battery:
        charging = False
        chargingCheckThread.cancel()
        blindBack()
        goChargeNow()

def goChargeNow():
    global docking_pos, goal_reached
    goTo(docking_pos)
    time.sleep(10)
    while not goal_reached:
        time.sleep(2)
    dock()
    #checkIfCharging()

def dock():
    rospy.wait_for_service('robot_instruction_receiver')
    try:
        service = rospy.ServiceProxy('robot_instruction_receiver', InstructionWithAnswer)
        answer = service(4)
    except rospy.ServiceException, e:
        print e

def goTo(x,y,z,w):
    global goal_publisher, goal_reached, navigating
    clear_costmap()
    goal_msg = PoseStamped()
    goal_msg.header.stamp = rospy.Time.now()
    goal_msg.header.frame_id = 'map'
    goal_msg.pose.position.x = x
    goal_msg.pose.position.y = y
    goal_msg.pose.orientation.z = z
    goal_msg.pose.orientation.w = w
    goal_publisher.publish(goal_msg)
    goal_reached = False
    navigating = True

def goTo(goal_msg):
    global goal_publisher, goal_reached, navigating
    clear_costmap()
    goal_msg.header.stamp = rospy.Time.now()
    goal_publisher.publish(goal_msg)
    goal_reached = False
    navigating = True

def initGoalPoints():
    global adl_pos1, adl_pos2, docking_pos
    adl_pos1 = PoseStamped()
    adl_pos1.header.stamp = rospy.Time.now()
    adl_pos1.header.frame_id = 'map'
    adl_pos1.pose.position.x = -4.66805749793
    adl_pos1.pose.position.y = 2.97347905327
    adl_pos1.pose.orientation.z = 0.296646442416
    adl_pos1.pose.orientation.w = 0.954987375939

    adl_pos2 = PoseStamped()
    adl_pos2.header.stamp = rospy.Time.now()
    adl_pos2.header.frame_id = 'map'
    adl_pos2.pose.position.x = -4.44401648886
    adl_pos2.pose.position.y = 7.49628868363
    adl_pos2.pose.orientation.z = -0.689436075633
    adl_pos2.pose.orientation.w = 0.724346531445

    docking_pos = PoseStamped()
    docking_pos.header.stamp = rospy.Time.now()
    docking_pos.header.frame_id = 'map'
    docking_pos.pose.position.x = -4.1083759801
    docking_pos.pose.position.y = 3.57356253611
    docking_pos.pose.orientation.z = 0.972631230579
    docking_pos.pose.orientation.w = -0.231643692

def timeBasedEvents():
    global timeBasedEventsThread, charging
    global prev_battery, curr_battery, chargingCheckThread
    global adl_pos1, adl_pos1
    # Check time every 5 minutes
    timeBasedEventsThread = threading.Timer(300, timeBasedEvents)
    timeBasedEventsThread.start()
    timestamp = time.strftime('%H:%M:%S')
    now = datetime.now()
    if not ("Sunday" in now.strftime("%A") or "Saturday" in now.strftime("%A")):
        if "Monday" in now.strftime("%A"):
            setState(0)
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
        elif h >= 18 and m >= 30 and next_state == 2:
            charging = False
            updateState()
            blindBack()
            initial_pose()
            goTo(adl_pos2)
        elif h >= 15 and m >= 30 and not charging:
            charging = True
            if chargingCheckThread is not None:
                if not chargingCheckThread.isAlive():
                    goChargeNow()
                    prev_battery = curr_battery
            else:
                goChargeNow()
                prev_battery = curr_battery
        elif h >= 13 and m >= 15 and next_state == 1:
            charging = False
            updateState()
            blindBack()
            initial_pose()
            goTo(adl_pos1)
        elif h >= 12 and m >= 45 and not charging:
            charging = True
            if chargingCheckThread is not None:
                if not chargingCheckThread.isAlive():
                    goChargeNow()
                    prev_battery = curr_battery
            else:
                goChargeNow()
                prev_battery = curr_battery
        elif h >= 11 and m >= 25 and  next_state == 0:
            charging = False
            blindBack()
            initial_pose()
            goTo(adl_pos1)
            updateState()
        elif h >= 10 and m >= 45 and  next_state == 1:
            charging = False
            blindBack()
            initial_pose()
            goTo(adl_pos2)
        elif h >= 10 and not charging:
            charging = True
            if chargingCheckThread is not None:
                if not chargingCheckThread.isAlive():
                    goChargeNow()
                    prev_battery = curr_battery
            else:
                goChargeNow()
                prev_battery = curr_battery
        elif h >= 7 and m >= 45 and next_state == 3:
            blindBack()
            initial_pose()
            goTo(adl_pos1)

def joyCallback(msg):
    global ost_pub, sound_pub
    # X starts HPR
    # A starts ros_visual
    # B starts motion_analysis for human
    # Y starts motion_analysis for object
    # Up/Forward on cross-pad initializes robot position
    # Back/Select to cancel navigation goal
    # Start sends a report based on today's date.
    # R1 pills are placed to the correct position
    # RightStick[Pressed] enables auto-docking
    # R2+X stops HPR
    # R2+A stops ros_visual
    # R2+B stops motion_analysis for human
    # R2+Y stops motion_analysis for object
    # Combinations of the A-B-X-Y buttons are disabled.
    # You always have to press one of the buttons.
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
    elif msg.axes[7] == -1:
        clear_costmap()
        sound_msg = Sound()
        sound_msg.value = 0
        sound_pub.publish(sound_msg)

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
    global running_hpr, sound_pub 
    print 'here!'
    if start:
        rospy.wait_for_service('robot_instruction_receiver')
        try:
            service = rospy.ServiceProxy('robot_instruction_receiver', InstructionWithAnswer)
            answer = service(0)
        except rospy.ServiceException, e:
            print e
        command = "roslaunch hpr_wrapper  wrapper.launch"
        command = shlex.split(command)
        subprocess.Popen(command)
        running_hpr = True
    else:
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
    global running_motion_analysis_human, sound_pub
    if start:
        rospy.wait_for_service('robot_instruction_receiver')
        try:
            service = rospy.ServiceProxy('robot_instruction_receiver', InstructionWithAnswer)
            answer = service(1)
        except rospy.ServiceException, e:
            print e
        time.sleep(11)
        command = "roslaunch motion_analysis_wrapper wrapper.launch"
        command = shlex.split(command)
        subprocess.Popen(command)
        running_motion_analysis_human = True
    else:
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
    global running_motion_analysis_obj, sound_pub 
    if start:
        mode = 0
        if pill_intake_mode == 1:
            mode = 2
        else:
            mode = 22
        rospy.wait_for_service('robot_instruction_receiver')
        try:
            service = rospy.ServiceProxy('robot_instruction_receiver', InstructionWithAnswer)
            answer = service(mode)
        except rospy.ServiceException, e:
            print e
        time.sleep(11)
        command = "roslaunch motion_analysis_wrapper wrapper.launch"
        command = shlex.split(command)
        subprocess.Popen(command)
        running_motion_analysis_obj = True
    else:
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
    global running_ros_visual, sound_pub
    if start:
        rospy.wait_for_service('robot_instruction_receiver')
        try:
            service = rospy.ServiceProxy('robot_instruction_receiver', InstructionWithAnswer)
            answer = service(3)
        except rospy.ServiceException, e:
            print e
        command = "roslaunch ros_visual_wrapper wrapper.launch"
        command = shlex.split(command)
        subprocess.Popen(command)
        running_ros_visual = True
    else:
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
