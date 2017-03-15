#!/usr/bin/env python
import os
import rospkg
import smtplib
import math, time
import roslib, rospy
import subprocess, shlex
from datetime import datetime
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy
from kobuki_msgs.msg import Sound
from email.MIMEText import MIMEText
from actionlib_msgs.msg import GoalID
from kobuki_msgs.msg import SensorState
from sensor_msgs.msg import BatteryState
from email.MIMEMultipart import MIMEMultipart
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from motion_detection_sensor_status_publisher.msg import SensorStatusMsg
from tf.transformations import quaternion_from_euler, euler_from_quaternion

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
goal_point = []
joy_sub = None
gym_x = -13.982
gym_y = 18.833

#first_detect becomes false the first time we see 'ok'
#from the motion detection sensor, to ensure that
#the first 'detect' values are not from the human's
#movements approaching and lying to bed.
first_detect = True


def init():
    global movement_sensor_sub, nav_status_sub, pub_stop, pc_battery_sub, kobuki_battery_sub
    global check_batteries, joy_sub, sound_pub
    rospy.init_node('radio_node_manager')
    rospy.get_param("~check_batteries", False)
    movement_sensor_sub = rospy.Subscriber('motion_detection_sensor_status_publisher/status', SensorStatusMsg, motionSensorStatus)
    nav_status_sub = rospy.Subscriber('move_base/status', GoalStatusArray, currentNavStatus)
    joy_sub = rospy.Subscriber('joy', Joy, joyCallback)
    sound_pub = rospy.Publisher('mobile_base/commands/sound', Sound)
    goal_subscriber = rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, getGoalPoint)
    pub_stop = rospy.Publisher('move_base/cancel', GoalID, queue_size=10)
    if check_batteries:
        #pc_battery_sub = rospy.Subscriber('placeholder', PlaceHolderMsg, pcBatteryCallback)
        kobuki_battery_sub = rospy.Subscriber('mobile_base/sensors/core', SensorState, kobukiBatteryCallback)
    rospy.Subscriber('emergency_stop', Int32, emergencyShutdown)
    #TODO ask the main server about what the initial state should be
    #Set initial pose on launch based on the main server's last saved position.
    #Maybe the robot itself could do this too. A save_current_state node would be useful.

    #Run here all the initial nodes
    command = "roslaunch turtlebot_radio_bringup radio_bringup.launch"
    command = shlex.split(command)
    subprocess.Popen(command)
    time.sleep(30)
    command = "roslaunch turtlebot_navigation radio_move_base.launch"
    command = shlex.split(command)
    subprocess.Popen(command)
    time.sleep(2)
    command = "roslaunch turtlebot_navigation radio_amcl_demo.launch"
    command = shlex.split(command)
    subprocess.Popen(command)
    time.sleep(2)
    command = "rosrun map_server map_server /home/turtlebot/test1.yaml"
    command = shlex.split(command)
    subprocess.Popen(command)
    time.sleep(15)
    
    pub_start = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
    start_point = PoseWithCovarianceStamped()
    #start point position x
    start_point.pose.pose.position.x = -6.329
    #start point position y     
    start_point.pose.pose.position.y = 10.087
    start_point.header.stamp = rospy.Time.now()
    #start_point.pose.pose.orientation.z = -2.122
    quat = quaternion_from_euler(0.0, 0.0, -2.122) # roll, pitch, yaw
    start_point.pose.pose.orientation = Quaternion(*quat.tolist())
    #start_point.pose.pose.orientation.w = 0
    start_point.header.frame_id = 'map'
    rospy.sleep(1)
    pub_start.publish(start_point)
    sound_msg = Sound()
    sound_msg.value = 6
    sound_pub.publish(sound_msg)

    while not rospy.is_shutdown():  
        rospy.spin()
    #we reach this point only after Ctrl-C
    startStopHPR(False, True)
    startStopRosVisual(False, True)
    startStopMotionAnalysisHuman(False, True)
    startStopMotionAnalysisObject(False, True)

    command = "rosnode kill amcl"
    command = shlex.split(command) 
    subprocess.Popen(command) 
    command = "rosnode kill app_manager"
    command = shlex.split(command) 
    subprocess.Popen(command) 
    command = "rosnode kill bumper2pointcloud"
    command = shlex.split(command) 
    subprocess.Popen(command) 
    command = "rosnode kill capability_server"
    command = shlex.split(command) 
    subprocess.Popen(command) 
    command = "rosnode kill capability_server_nodelet_manager"
    command = shlex.split(command) 
    subprocess.Popen(command) 
    command = "rosnode kill chroma"
    command = shlex.split(command) 
    subprocess.Popen(command) 
    command = "rosnode kill cmd_vel_mux"
    command = shlex.split(command) 
    subprocess.Popen(command) 
    command = "rosnode kill diagnostic_aggregator"
    command = shlex.split(command) 
    subprocess.Popen(command) 
    command = "rosnode kill hokuyo"
    command = shlex.split(command) 
    subprocess.Popen(command)
    command = "rosnode kill interactions"
    command = shlex.split(command) 
    subprocess.Popen(command) 
    command = "rosnode kill joystick"
    command = shlex.split(command) 
    subprocess.Popen(command) 
    command = "rosnode kill kobuki_safety_controller"
    command = shlex.split(command) 
    subprocess.Popen(command)  
    command = "rosnode kill map_server"
    command = shlex.split(command) 
    subprocess.Popen(command) 
    command = "rosnode kill mobile_base"
    command = shlex.split(command) 
    subprocess.Popen(command) 
    command = "rosnode kill mobile_base_nodelet_manager"
    command = shlex.split(command) 
    subprocess.Popen(command) 
    command = "rosnode kill move_base"
    command = shlex.split(command) 
    subprocess.Popen(command) 
    command = "rosnode kill navigation_velocity_smoother"
    command = shlex.split(command) 
    subprocess.Popen(command) 
    command = "rosnode kill radio_node_manager"
    command = shlex.split(command) 
    subprocess.Popen(command) 
    command = "rosnode kill teleop_velocity_smoother"
    command = shlex.split(command) 
    subprocess.Popen(command) 
    command = "rosnode kill turtlebot_teleop_joystick"
    command = shlex.split(command) 
    subprocess.Popen(command) 
    command = "rosnode kill xtion_pro_cam"
    command = shlex.split(command) 
    subprocess.Popen(command)  

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
                startStopHPR(False, True)
                startStopRosVisual(False, True)
                startStopMotionAnalysisHuman(False, True)
                startStopMotionAnalysisObject(False, True)
                navigating = True


def motionSensorStatus(ssm):
    global running_motion_analysis_human, first_detect, running_hpr, sound_pub
    global movement_sensor_sub
    cur_st = ssm.status
    if cur_st == 'ok':
        print 'Got an ok!'
        if first_detect:
            first_detect = False
            print 'Waiting for an actual detection now'
    elif not first_detect and cur_st == 'detect':
        print 'Unsubscribing from the movement sensor...'
        movement_sensor_sub.unregister()
        startStopHPR(True, False)
        startStopMotionAnalysisHuman(True, False)

#Check if the point equals the Gym point so that we can count how many times
#the gym button was clicked.
def getGoalPoint(goal_msg):
    global gym_x, gym_y
    if goal_msg.goal.target_pose.pose.position.x == gym_x and goal_msg.goal.target_pose.pose.position.y == gym_y:
        rospack = rospkg.RosPack()
        path = rospack.get_path("radio_node_manager")+'/logs/'
        dt = datetime.now()
        with open(path+'official_log_'+datetime.today().strftime("%d-%m-%Y")+'.log','ab+') as f:
            f.write("Went to the gym at "+dt.strftime("%H:%M:%S")+"\n")

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
                #TODO
                print 'Stopping motion_analysis'
                print 'Stopping HPR'
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
    global sound_pub
    print 'report'
    files_to_remove = []
    fromaddr = "roboskelncsr@gmail.com"
    toaddr = ["gstavrinos@iit.demokritos.gr", "gs.juggle@gmail.com"]
    subject = "Medical Report as of "+datetime.now().strftime("%d/%m/%Y %H:%M:%S")
    msg = MIMEMultipart()
    msg['From'] = fromaddr
    msg['To'] = ", ".join(toaddr)
    msg['Subject'] = subject
     
    body = subject+"\n\n\n"

    body += "Getting out of bed and pill intake:\n"

    rospack = rospkg.RosPack()
    path = rospack.get_path('motion_analysis_wrapper')+'/logs/'
    files = []
    found_file = False
    for i in os.listdir(path):
        if os.path.isfile(os.path.join(path,i)) and 'official_log_'+datetime.today().strftime("%d-%m-%Y") in i:
        found_file = True
                files.append(i)
        files_to_remove.append(path+i)

    if found_file:
            for f in files:
                with open(path+f, 'r') as myfile:
                        body += myfile.read()
    files = []
    found_file = False

    body += "---\n\n\n"

    body += "Walking 4 meters:\n"

    path = rospack.get_path('hpr_wrapper')+'/logs/'
    for i in os.listdir(path):
        if os.path.isfile(os.path.join(path,i)) and 'official_log_'+datetime.today().strftime("%d-%m-%Y") in i:
                found_file = True
        files.append(i)
        files_to_remove.append(path+i)

    if found_file:
            for f in files:
                with open(path+f, 'r') as myfile:
                        body += myfile.read()

    files = []
    found_file = False

    body += "---\n\n\n"

    body += "Standing from a chair:\n"

    path = rospack.get_path('ros_visual_wrapper')+'/logs/'
    for i in os.listdir(path):
        if os.path.isfile(os.path.join(path,i)) and 'official_log_'+datetime.today().strftime("%d-%m-%Y") in i:
                found_file = True
        files.append(i)
        files_to_remove.append(path+i)

    if found_file:
        for f in files:
            with open(path+f, 'r') as myfile:
                    body += myfile.read()

    body += "---\n\n\n"

    msg.attach(MIMEText(body, 'plain'))

    files = []
    found_file = False

    body += "---\n\n\n"

    path = rospack.get_path('radio_node_manager')+'/logs/'
    for i in os.listdir(path):
        if os.path.isfile(os.path.join(path,i)) and 'official_log_'+datetime.today().strftime("%d-%m-%Y") in i:
                found_file = True
        files.append(i)
        files_to_remove.append(path+i)

    if found_file:
        for f in files:
            with open(path+f, 'r') as myfile:
                    body += myfile.read()

    body += "---\n\n\n"

    msg.attach(MIMEText(body, 'plain'))
 
    server = smtplib.SMTP('smtp.gmail.com', 587)
    server.starttls()
    server.login(fromaddr, "reportt0d0cs")
    text = msg.as_string()
    server.sendmail(fromaddr, toaddr, text)
    server.quit()
    for f in files_to_remove:
        os.remove(f)
        print 'Deleted',f
    sound_msg = Sound()
    sound_msg.value = 0
    sound_pub.publish(sound_msg)


def joyCallback(msg):
    #X starts/stops HPR
    #A starts/stops ros_visual
    #B starts/stops motion_analysis for human
    #Y starts/stops motion_analysis for object
    #Back/Select to cancel navigation goal
    #Start sends a report based on today's date.
    #Combinations of the A-B-X-Y buttons are disabled.
    #You always have to press one of the buttons.
    if msg.buttons[0] == 0 and msg.buttons[1] == 0 and msg.buttons[2] == 1 and msg.buttons[3] == 0:
        startStopHPR(True, True)
    elif msg.buttons[0] == 1 and msg.buttons[1] == 0 and msg.buttons[2] == 0 and msg.buttons[3] == 0:
        startStopRosVisual(True, True)
    elif msg.buttons[0] == 0 and msg.buttons[1] == 1 and msg.buttons[2] == 0 and msg.buttons[3] == 0:
        startStopMotionAnalysisHuman(True, True)
    elif msg.buttons[0] == 0 and msg.buttons[1] == 0 and msg.buttons[2] == 0 and msg.buttons[3] == 1:
        startStopMotionAnalysisObject(True, True)
    if msg.buttons[6] == 1:
        cancelNavigationGoal()
    if msg.buttons[7] == 1:
        createReport()
    if msg.buttons[5] == 1:
        command = "rostopic pub /motion_analysis/object_state std_msgs/Int32 2"
        command = shlex.split(command)
        subprocess.Popen(command)
        sound_msg = Sound()
        sound_msg.value = 0
        sound_pub.publish(sound_msg)
    if msg.buttons[10] == 1:
        command = "roslaunch kobuki_auto_docking minimal.launch"
        command = shlex.split(command)
        subprocess.Popen(command)
        command = "roslaunch kobuki_auto_docking activate.launch"
        command = shlex.split(command)
        subprocess.Popen(command)

        sound_msg = Sound()
        sound_msg.value = 0
        sound_pub.publish(sound_msg)


    print msg

def startStopHPR(start, stop):
    global running_hpr, sound_pub
    if not running_hpr:
        if start:
            print 'Starting HPR'
            command = "roslaunch human_pattern_recognition  hpr.launch"
            command = shlex.split(command)
            subprocess.Popen(command)
            command = "roslaunch hpr_wrapper  wrapper.launch"
            command = shlex.split(command)
            subprocess.Popen(command)
            running_hpr = True
            sound_msg = Sound()
            sound_msg.value = 0
            sound_pub.publish(sound_msg)
    else:
        if stop:
            print 'Stopping HPR'
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

def startStopMotionAnalysisHuman(start, stop):
    global running_motion_analysis_human, sound_pub
    if not running_motion_analysis_human:
        if start:
            print 'Starting motion_analysis'
            command = "roslaunch motion_analysis human_event_detection.launch"
            command = shlex.split(command)
            subprocess.Popen(command)
            command = "roslaunch motion_analysis_wrapper wrapper.launch"
            command = shlex.split(command)
            subprocess.Popen(command)
            running_motion_analysis_human = True
            sound_msg = Sound()
            sound_msg.value = 0
            sound_pub.publish(sound_msg)
    else:
        if stop:
            print 'Stopping motion_analysis'
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

def startStopMotionAnalysisObject(start, stop):
    global running_motion_analysis_obj, sound_pub
    if not running_motion_analysis_obj:
        if start:
            print 'Starting motion_analysis'
            command = "roslaunch motion_analysis object_event_detection.launch"
            command = shlex.split(command)
            subprocess.Popen(command)
            command = "roslaunch motion_analysis_wrapper wrapper.launch"
            command = shlex.split(command)
            subprocess.Popen(command)
            running_motion_analysis_obj = True
            sound_msg = Sound()
            sound_msg.value = 0
            sound_pub.publish(sound_msg)
    else:
        if stop:
            print 'Stopping motion_analysis'
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

def startStopRosVisual(start, stop):
    global running_ros_visual, sound_pub
    if not running_ros_visual:
        if start:
            print 'Starting ros_visual'
            command = "roslaunch ros_visual ros_visual.launch"
            command = shlex.split(command)
            subprocess.Popen(command)
            command = "roslaunch ros_visual_wrapper wrapper.launch"
            command = shlex.split(command)
            subprocess.Popen(command)
            running_ros_visual = True
            sound_msg = Sound()
            sound_msg.value = 0
            sound_pub.publish(sound_msg)
    else:
        if stop:
            print 'Stopping ros_visual'
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
            command = "rosnode kill ros_visual_wrapper"
            command = shlex.split(command)
            subprocess.Popen(command)
            running_ros_visual = False
            sound_msg = Sound()
            sound_msg.value = 1
            sound_pub.publish(sound_msg)

def emergencyShutdown(msg):
    #command = '/usr/bin/dbus-send --system --print-reply --dest="org.freedesktop.ConsoleKit" /org/freedesktop/ConsoleKit/Manager org.freedesktop.ConsoleKit.Manager.Stop'
    command = "rosnode kill mobile_base_nodelet_manager"
    command = shlex.split(command)
    subprocess.Popen(command)

    command = "rosnode kill -a"
    command = shlex.split(command)
    subprocess.Popen(command)

if __name__ == '__main__':
    init() 

