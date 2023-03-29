#!/usr/bin/env python

"""
This is a lib of Tasks that can be used by the task_manager.
Each task is implemented following the Behaviour Tree philosophy as a tree.
Upon instantation, these sub-trees will be appended to the ROOT tree handled by the manager.
"""
import rclpy
import py_trees
import py_trees_ros

import rclpy.action
from rclpy.action import GoalResponse
from transforms3d.euler import euler2quat as quaternion_from_euler
from transforms3d.euler import quat2euler as euler_from_quaternion
#from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import pi
from collections import OrderedDict
import numpy as np
import ast
import math

try:
    from termcolor import colored
except ImportError:
    raise ImportError('termcolor pkg not found in your python installation. Please run pip install termcolor')

from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32, String
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion, Twist
#from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from visualization_msgs.msg import Marker
import ast


# Check installed packages to offer sub-trees (tasks)
# If related pkgs are not installed, we wont allow instantiations of tasks
# Yet we will need to ensure that the SRV are running (i.e. nodes are launched)

try:
    # Giraff robot
    from giraff_interfaces.msg import StalkRef
    from giraff_interfaces.srv import GiraffSrvMsg, GiraffSrvMsgRequest
    giraff_robot = True
except ImportError:
    giraff_robot = False

try:
    # Google Speech SRV
    from speech.srv import say
    google_speech_node_found = True
except ImportError:
    google_speech_node_found = False

try:
    # Acapela Speech SRV (movecare)
    from movecare_interface.srv import Speech
    acapela_speech_node_found = True
except ImportError:
    acapela_speech_node_found = False

try:
    # User Recognizer
    from user_recognizer.msg import DoRecognizeAction, DoRecognizeGoal, DoRecognizeFeedback
    face_detector_node_found = True
except ImportError:
    face_detector_node_found = False

try:
    # OpenPose
    from openpose_pkg.srv import StartDetectionHuman, ChangeFrecDetection, StartDetectionHumanRequest
    openpose_node_found = True
except ImportError:
    openpose_node_found = False

try:
    # Find and approach user
    from appro_user.msg import appro_user_action_msgAction, appro_user_action_msgGoal, appro_user_action_msgFeedback
    appro_user_node_found = True
except ImportError:
    appro_user_node_found = False

try:
    # Autodocking
    from autodocking.msg import DoDockingAction, DoDockingGoal, DoDockingFeedback
    autodocking_node_found = True
except ImportError:
    autodocking_node_found = False

try:
    # HRI (movecare)
    from movecare_interface.msg import TaskAction, TaskGoal, TaskFeedback
    hri_node_found = True
except ImportError:
    hri_node_found = False

try:
    # Teleoperation
    from teleoperation.msg import teleop_requestAction, teleop_requestGoal, teleop_requestFeedback
    teleop_node_found = True
except ImportError:
    teleop_node_found = False

try:
    # Object Search
    from rfid_actionserver.msg import rfidAction, rfidGoal, rfidFeedback, rfidResult
    object_search_node_found = True
except ImportError:
    object_search_node_found = False


# OLFACTION
try:
    # Olfaction msgs
    from olfaction_msgs.msg import gas_sensor_array
    olfaction_node_found = True
except ImportError:
    olfaction_node_found = False

try:
    # Gas Classification
    from scikit_bridge.srv import classify_volatile
    scikit_bridge_node_found = True
except ImportError:
    scikit_bridge_node_found = False

try:
    # Ontology - Semantics
    from ontology_bridge.srv import semantic_request, semantic_requestRequest
    ontology_bridge_node_found = True
except ImportError:
    ontology_bridge_node_found = False

try:
    # Gas Source Localization
    from gsl.msg import gsl_action_msgAction, gsl_action_msgGoal, gsl_action_msgFeedback
    gsl_node_found = True
except ImportError:
    gsl_node_found = False


# TO REMOVE
try:
    # Fake Battery
    from fake_battery.srv import simulate_battery_recharge, simulate_battery_discharge
    fake_battery_node_found = True
except ImportError:
    fake_battery_node_found = False


# =============================================================
# ====================== SUBTREE ==============================
# =============================================================
class subtree():
    """
    A base Subtree class defining the basic components to work with behaiur trees
    """
    def __init__(self):
        self.ROOT = None                # The BT parent node (see pi_trees_lib)
        self.task_created = False       # Boolean to set if the subtree has been created successfully or some components are not available


# =============================================================
# ==================== GO TO POINT ============================
# =============================================================
class subtree_go_to_point(subtree):
    """
    A subtree to implement the simple_action_client for move_base
    """

    def __init__(self, name, pose):
        self.name = name
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = '/map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose
        self.ROOT = SimpleActionTask(name, "move_base", MoveBaseAction, goal, done_cb=self.done_cb, result_timeout=120, reset_after=False, announce=True)
        self.task_created = True

    def done_cb(self, result_state, result):
        # Report result
        if result_state == 3:
            # Navigation sucessfully completed
            self.ROOT.trace = '"' + self.name + '" : "ok"'
        else:
            self.ROOT.trace = '"' + self.name + '" : "error"'


# =============================================================
# ==================== CHECK CHARGING =========================
# =============================================================
class subtree_check_charging(subtree):
    """
        A Subtree to check if the robot is charging batteries (docked)
        It is designed to work in conjunction with:
        - A node that publishes a sensor_msgs::BatteryState msg with info about the battery voltage/current and charging state
    """

    def __init__(self, battery_topic):
        self.battery_topic = battery_topic

        self.ROOT = MonitorTask("CHECK_CHARGING", self.battery_topic, BatteryState, self.monitor_battery_cb)

        # Always return True. since the task can be created!
        self.task_created = True


    # CHECK CHARGING
    def monitor_battery_cb(self, msg):
        # msg is type BatteryState
        if msg.power_supply_status == 1 or msg.power_supply_status == 4:
            return TaskStatus.SUCCESS
        elif msg.power_supply_status == 2 or msg.power_supply_status == 3:
            return TaskStatus.FAILURE
        else:
            return TaskStatus.RUNNING


# =============================================================
# ==================== CHECK NOT CHARGING =====================
# =============================================================
class subtree_check_not_charging(subtree):
    """
        A Subtree to check if the robot is not charging batteries (un-docked)
        It is designed to work in conjunction with:
        - A node that publishes a sensor_msgs::BatteryState msg with info about the battery voltage/current and charging state
    """

    def __init__(self, battery_topic):
        self.battery_topic = battery_topic

        self.ROOT = MonitorTask("CHECK_NOT_CHARGING", self.battery_topic, BatteryState, self.monitor_battery_cb)

        # Always return True. since the task can be created!
        self.task_created = True


    # CHECK CHARGING
    def monitor_battery_cb(self, msg):
        # msg is type BatteryState
        if msg.power_supply_status == 1 or msg.power_supply_status == 4:
            return TaskStatus.FAILURE
        elif msg.power_supply_status == 2 or msg.power_supply_status == 3:
            return TaskStatus.SUCCESS
        else:
            return TaskStatus.RUNNING


# =============================================================
# ======================== SAY ================================
# =============================================================
class subtree_say(subtree):
    """
    A subtree to implement a simple talk via service call
    """
    def __init__(self, dialogue):
        try:
            if acapela_speech_node_found:
                self.ROOT = ServiceTask("SAY", "speech", Speech, dialogue, result_cb=None, wait_for_service=True, timeout=5, announce=True)
                self.task_created = True
            else:
                rospy.logwarn("[bt_manager] ROS Acapela Speech service NOT FOUND! Trying with GOOGLE API.")

                # Is google available?
                if google_speech_node_found:
                    self.ROOT = ServiceTask("SAY", "speech/say_text", say, dialogue, result_cb=None, wait_for_service=True, timeout=5, announce=True)
                    self.task_created = True
                else:
                    rospy.logwarn("[bt_manager] ROS Google Speech module not found! I Cannot execute a SAY task.")
                    self.task_created = False

        except rospy.ROSException as excp:
            print (colored("[bt_manager] " + str(excp) + ". Skipping task", "red"))
            self.task_created = False


# =============================================================
# ======================== WAIT ===============================
# =============================================================
class subtree_wait(subtree):
    """
    A subtree to implement a non-blocking wait_Sec
    """

    def __init__(self, time_sec, announce=True):
        #rospy.loginfo("[bt_manager] New Task Wait for %.2f seconds", time_sec)
        self.ROOT = WaitSec("WAIT_" + str(time_sec) + "_SEC", time_sec, reset_after=False, announce=announce)
        self.task_created = True


# =============================================================
# ==================== CMD_VEL_TIME ===========================
# =============================================================
class subtree_cmd_vel_time(subtree):
    """
    A subtree to implement a simple CMD_VEL for a given time (useful for testing)
    """

    def __init__(self, lin_speed, ang_speed, time_sec):
        #rospy.loginfo("[bt_manager] New cmd_vel_time Task")
        self.pub_twist = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.ROOT = Sequence("CMD_VEL_TIME", reset_after=False, announce=False)

        MOVE = GenericTask("MOVE", self.move_cb, (lin_speed, ang_speed), reset_after=True)
        self.ROOT.add_child(MOVE)

        WAIT_TIME = WaitSec("WAIT_SEC", time_sec, reset_after=False)
        self.ROOT.add_child(WAIT_TIME)

        STOP = GenericTask("STOP", self.stop_cb, reset_after=False)
        self.ROOT.add_child(STOP)

        self.task_created = True

    def move_cb(self, msg):
        twist = Twist()
        twist.linear.x = float(msg[0])  # m/s
        twist.angular.z = float(msg[1])  # rad
        self.pub_twist.publish(twist)
        return TaskStatus.SUCCESS

    def stop_cb(self, msg):
        twist = Twist()
        twist.linear.x = 0.0  # m/s
        twist.angular.z = 0.0  # rad
        self.pub_twist.publish(twist)
        return TaskStatus.SUCCESS


# =============================================================
# =====================    DOCKING     ========================
# =============================================================
class subtree_dock(subtree):

    def __init__(self, docking_station_pose, speech):
        if autodocking_node_found:
            self.docking_station_pose = docking_station_pose
            self.speech = speech
            self.docking_result_status = ""

            # Populate CHARGE_MANEUVER -> NAV_DOCK + DOCK
            # ------------------------------------------------
            self.ROOT = Sequence("DOCK", reset_after=False, announce=True)

            # NAV_DOCK: Navigate close to Docking location
            nav_dock = subtree_go_to_point(name="GO_DOCK", pose=self.docking_station_pose)
            if nav_dock.task_created:
                self.ROOT.add_child(nav_dock.ROOT)

            # SAY DOCK (Service)
            if self.speech:
                say_dock = subtree_say("Executing Docking...please wait")
                if say_dock.task_created:
                    self.ROOT.add_child(say_dock.ROOT)

            # DO DOK (Action)
            goal = DoDockingGoal()
            goal.docking_method = "camera"
            do_dock = SimpleActionTask("DOCK_ACTION", "autodocking", DoDockingAction, goal, result_timeout=120,
                                       reset_after=False, done_cb=self.dock_action_done_cb)
            self.ROOT.add_child(do_dock)

            # SAY DOCK (Service)
            if google_speech_node_found and self.speech:
                try:
                    say_dock_end = ServiceTaskDynamic("SAY", "speech/say_text", say,
                                                      request_cb=self.say_dock_result, result_cb=None)
                    self.ROOT.add_child(say_dock_end)
                except rospy.ROSException as excp:
                    print colored("[bt_manager] " + str(excp) + ". Disabling Speech", "red")

            if openpose_node_found:
                # Stop openpose
                self.srv_req = StartDetectionHumanRequest("off","off")
                openpose = ServiceTask("OpenPose-OFF", "openpose/start_detection_humans_service", StartDetectionHuman, self.srv_req ,result_cb=None, wait_for_service=True, timeout=5, announce=False)
                self.ROOT.add_child(openpose)
                
            # BT Created.
            self.task_created = True

        else:
            rospy.logwarn("[bt_manager] ROS autodocking module not found! I Cannot execute a DOCKING task.")
            self.task_created = False

    def dock_action_done_cb(self, result_state, result):
        if result.success == 1:
            self.docking_result_status = "success"
        else:
            self.docking_result_status = "failure"
            self.ROOT.trace = '"docking":"failed to perform the dock"'

    def say_dock_result(self):
        if self.docking_result_status == "success":
            return "Docking completed! Waiting to fully recharge the battery"
        else:
            return "Sorry!. There has been a problem while Docking."


# =============================================================
# =====================    UNDOCKING     ======================
# =============================================================
class subtree_undock(subtree):
    """
    A BT to implement an easy but functional Un-dock procedure.
    It first check that the robot is charging, else it does nothing.
    """

    def __init__(self, speech):
        self.speech = speech
        self.pub_twist = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.ROOT = Sequence("UNDOCK_MANEUVER", reset_after=False, announce=True)

        # SAY DOCK (Service)
        if self.speech:
            say_dock = subtree_say("Executing Un-Docking...please wait")
            if say_dock.task_created:
                self.ROOT.add_child(say_dock.ROOT)

        # Reset Giraff base
        if giraff_robot:
            # RESTART BASE (Service)
            try:
                self.srv_req = giraff_srv_msgRequest("restart",0)
                restart_base = ServiceTask("RESET_BASE", "giraff_ros_driver/command", giraff_srv_msg, self.srv_req, result_cb=None)
                self.ROOT.add_child(restart_base)
            except rospy.ROSException as excp:
                print colored("[bt_manager] " + str(excp), "red")

            wait_task = WaitSec("WAIT_RESTART", 5)
            self.ROOT.add_child(wait_task)

        # UNDOCK
        undock = GenericTask("UNDOCK", self.undock_cb, reset_after=False)
        self.ROOT.add_child(undock)
        self.task_created = True

    # FUNCTIONS AND CALLBACKS
    def undock_cb(self, msg):
        twist = Twist()
        twist.angular.z = 0.0  # rad
        twist.linear.x = -0.1  # m/s
        t_start = rospy.get_time()
        while rospy.get_time() - t_start < 2.0:
            self.pub_twist.publish(twist)
            rospy.sleep(0.1)  # seconds

        turn_time = 4  # sec
        twist.angular.z = 3.14159 / turn_time  # rad
        twist.linear.x = 0.0  # m/s
        t_start = rospy.get_time()
        while rospy.get_time() - t_start < turn_time:
            self.pub_twist.publish(twist)
            rospy.sleep(0.1)  # seconds

        # stop
        twist.angular.z = 0.0  # rad
        twist.linear.x = 0.0  # m/s
        t_start = rospy.get_time()
        while rospy.get_time() - t_start < 1.0:
            self.pub_twist.publish(twist)
            rospy.sleep(0.1)  # seconds
        return TaskStatus.SUCCESS


# =============================================================
# ===================== RECHARGE TILL FULL ====================
# =============================================================
class subtree_recharge_till_full(subtree):
    """
    A BT that first check if the robot is already charging, and if not will command a Dock task
    It first check that the robot is charging, else it does nothing.
    Then I'll wait till a full charge is achieved
    An important aspect is that the priority of this task is dynamic with the battery %, so high priority task can interrupt
    the charging cycle (but not cancel it)
    """
    current_battery_msg = None
    def __init__(self, battery_topic, docking_station_pose, speech):
        if autodocking_node_found:

            # Sequence to Recharge full with dynamic priority
            self.ROOT = Sequence("RECHARGE_FULL", reset_after=False, announce=True)

            # Update Task Priority (Dynamic priority base on Battery)
            update_priority = MonitorTask("UPDATE_RIORITY", battery_topic, BatteryState, self.update_priority_cb)
            self.ROOT.add_child(update_priority)

            # Subtree for Ensure Charging
            ensure_ch = Selector("ENSURE_CHARGING", reset_after=True, announce=False)
            self.ROOT.add_child(ensure_ch)

            # CHECK_CHARGING
            check_charging = subtree_check_charging(battery_topic)
            if check_charging.task_created:
                ensure_ch.add_child(check_charging.ROOT)

            # DO DOK (Action)
            dock = subtree_dock(docking_station_pose, speech)
            if dock.task_created:
                ensure_ch.add_child(dock.ROOT)

            # Finally, the wait_full task
            wait_full = GenericTask("WAIT_FULL", self.wait_full_cb, reset_after=False)
            self.ROOT.add_child(wait_full)

            # BT Created.
            self.task_created = True

        else:
            rospy.logwarn("[bt_manager] ROS autodocking module not found! I Cannot execute an ENSURE_CHARGING task.")
            self.task_created = False

    # UPDATE PRIORITY
    def update_priority_cb(self, msg):
        # msg is type BatteryState
        self.current_battery_msg = msg
        # Work with percentages (requires intelligent battery manager)
        new_priority = int(math.ceil( (0.9 - msg.percentage)*10 ))
        # rospy.logwarn("new priority is:" + str(new_priority) + " --- current priority is:" + str(self.ROOT.priority) )
        # Allow decrease priority according to charge state
        if new_priority < self.ROOT.priority:
            self.ROOT.priority = new_priority

        return TaskStatus.SUCCESS   # Success becasue we are in a Sequence

    # WAIT FULL: Returns SUCCESS only when battery is completely charged!. RUNNING otherwise
    def wait_full_cb(self, msg):
        if self.current_battery_msg is None:
            return TaskStatus.RUNNING

        if self.current_battery_msg.power_supply_status == 4:
            # rospy.loginfo("[task_manager] BATTERY FULL - level: " + str(float(self.current_battery.voltage)))
            return TaskStatus.SUCCESS
        else:
            # rospy.loginfo("[patrol_pkg] CHARGING BATTERY - level: " + str(float(self.current_battery.voltage)))
            return TaskStatus.RUNNING


# =============================================================
# ===================== IDLE TO DOCK ==========================
# =============================================================
class subtree_idle_to_dock(subtree):
    """
    A BT aimed to be always with minimum priority and Permanent.
    After a time-out, it will command the robot to go docking and wait there
    Usefull as a security task in case the robot is left unnatended out of the charging station
    """
    def __init__(self, battery_topic, docking_station_pose, idle_timeout, speech):
        if autodocking_node_found:

            self.ROOT = Sequence("IDLE_TO_DOCK", reset_after=False, announce=False)

            # Wait seconds
            wait = subtree_wait(idle_timeout, announce=False)
            if wait.task_created:
                self.ROOT.add_child(wait.ROOT)

            # Subtree for Ensure Charging
            ensure_ch = Selector("ENSURE_CHARGING", reset_after=False, announce=False)
            self.ROOT.add_child(ensure_ch)

            # CHECK_CHARGING
            check_charging = subtree_check_charging(battery_topic)
            if check_charging.task_created:
                ensure_ch.add_child(check_charging.ROOT)

            # DO DOK (Action)
            dock = subtree_dock(docking_station_pose, speech)
            if dock.task_created:
                ensure_ch.add_child(dock.ROOT)

            # BT Created.
            self.task_created = True

        else:
            rospy.logwarn("[bt_manager] ROS autodocking module not found! I Cannot execute an ENSURE_CHARGING task.")
            self.task_created = False

    # UPDATE PRIORITY
    def update_priority_cb(self, msg):
        # msg is type BatteryState
        new_priority = int(math.ceil( (0.9 - msg.percentage)*10 ))
        # Allo decrease priority according to charge state
        if new_priority < self.ROOT.priority:
            self.ROOT.priority = new_priority
        # Check if task completed
        if self.ROOT.priority == -1 and msg.power_supply_status == 4:
            self.ROOT.permanence = False

        return TaskStatus.FAILURE   #Failure becasue we are in a Selector


# =============================================================
# =================== ENSURE NOT CHARGING =====================
# =============================================================
class subtree_ensure_not_charging(subtree):
    """
    A BT that first check if the robot is charging, and if yes will command an Un-Dock task
    It first check that the robot is not-charging, else it does nothing.
    """
    def __init__(self, battery_topic, speech):

        self.ROOT = Selector("ENSURE_DISCHARGING", reset_after=False, announce=False)

        # CHECH_DSICHARGING
        check_discharging = subtree_check_not_charging(battery_topic)
        if check_discharging.task_created:
            self.ROOT.add_child(check_discharging.ROOT)

        # DO UN-DOK (Action)
        undock = subtree_undock(speech)
        if undock.task_created:
            self.ROOT.add_child(undock.ROOT)

        # BT Created.
        self.task_created = True




# =============================================================
# ===================== FACE_DETECTION ========================
# =============================================================
class subtree_face_detection(subtree):

    def __init__(self, action_type, tout, just_once, speech):
        if face_detector_node_found:
            try:
                self.ROOT = Sequence ("FACE_DETECTION", reset_after=False, announce=True)
                
                # DETECT_FACES Task                
                goal = DoRecognizeGoal()
                goal.type = action_type     # 0=face_detection / 1=recognition
                goal.timeout = tout         # secs
                goal.just_one = just_once   # return just the biggest detected face
                do_recog = SimpleActionTask("USER_RECOG", "user_recognizer", DoRecognizeAction, goal, result_timeout=tout,
                                           reset_after=False, done_cb=self.face_detection_cb)
                self.ROOT.add_child(do_recog)

                # Create SpeechTask
                if google_speech_node_found and speech == True:
                    try:
                        SPEECH = ServiceTaskDynamic("SPEECH","speech/say_text", say, request_cb=self.say_request_cb, result_cb=None)
                        self.ROOT.add_child(SPEECH)
                    except rospy.ROSException as excp:
                        print colored("[bt_manager] " + str(excp) + ". Disabling Speech", "red")
                else:
                    rospy.logwarn("[bt_manager] Task without Speech")
                
                # Task was created!
                self.task_created = True

            except rospy.ROSException as excp:
                print colored("[bt_manager] " + str(excp) + ". Skipping task", "red")
                self.task_created = False
        else:
            rospy.logwarn("[bt_manager] ROS face_detector module not found! I Cannot execute a FACE_DETECTION task.")
            self.task_created = False

    
    def face_detection_cb(self, result_state, result):
        if result_state != GoalStatus.SUCCEEDED:
            # The action did not finished for some reason (FAILURE)
            self.ROOT.trace = '"face_detection" : "ActionLib did not finish propertly"'
            self.nFaces = 0
        else:
            # The actionlib SUCCEEDED now lets check if we found users
            if result.success == 1:
                self.nFaces = len(result.detected_users)  # number of detected faces
                self.ROOT.trace = '"face_detection" : "' + str(self.nFaces) + ' faces detected"'
                # display results
                i = 0;
                for u in result.detected_users:
                    rospy.loginfo("user_%u (%.3f,%.3f,%.3f,%.3f)",i,u.image_coords[0],u.image_coords[1],u.image_coords[2],u.image_coords[3])
                    i = i+1
            else:
                self.ROOT.trace = '"face_detection" : "No face detected"'
                self.ROOT.status = TaskStatus.FAILURE
                self.nFaces = 0

    def say_request_cb(self):
        # compose the text to say
        if self.nFaces == 0:
            return "After executing face detection I did not found people arround me"
        else:
            return "Hello human. I'm Giraff. A mobile robot from the mapir lab. See you soon."

    

# =============================================================
# ================= FIND & APPRO PERSON =======================
# =============================================================
class subtree_find_appro_person(subtree):

    def __init__(self, room_labels, room_poses, intensive_search, tout, speech):

        # Action Server to Locate and Approach the User
        if appro_user_node_found:
            goal = appro_user_action_msgGoal()
            goal.method = ""
            goal.time_out = tout
            goal.speech = speech
            goal.labels = room_labels   #string[]
            goal.locations = room_poses #PoseArray
            goal.intensive_search = intensive_search    #bool

            self.ROOT = SimpleActionTask("Locate-Appro_user", "appro_user", appro_user_action_msgAction, goal, done_cb=self.done_cb,
                                        result_timeout=tout,
                                        reset_after=False,
                                        announce=True)
            self.task_created = True
        else:
            rospy.logwarn("[task_manager] Cannot Find and Approach the user.... ApproUser node not found")
            self.task_created = False

    def done_cb(self, result_state, result):
        # Report result
        action_response = result.trace
        if result.user_found == True and action_response == "Completed":
            # SUCCESS
            self.ROOT.trace = '"ApproUser" : "ok"'
        else:
            # ERROR
            self.ROOT.trace = '"ApproUser" : "error - ' + action_response + '"'
            

# =============================================================
# ===================== FIND PERSON ===========================
# =============================================================
class subtree_find_person(subtree):

    def __init__(self, action_type, tout, just_once, speech):
        #rospy.loginfo("[bt_manager] New Face_Detection Task")
        if face_detector_node_found or openpose_node_found:
            try:
                self.pub_twist = rospy.Publisher('cmd_vel', Twist, queue_size=10)
                self.ROOT = Sequence ("FIND_PERSON", reset_after=False, announce=True)

                # turn a bit
                move_a_bit = GenericTask("TURN_A_BIT", self.move_cb, (0.0, 0.15), reset_after=True)
                self.ROOT.add_child(move_a_bit)

                # User_recognizer (ActionServer that only detects faces)
                df = subtree_face_detection(action_type, tout, just_once, speech)
                if df.task_created:
                    self.ROOT.add_child(df.ROOT)

                # stop (just for precaution)
                stop = GenericTask("STOP", self.stop_cb, reset_after=False)
                self.ROOT.add_child(stop)

                # Task was created!
                self.task_created = True

            except rospy.ROSException as excp:
                print colored("[bt_manager] " + str(excp) + ". Skipping task", "red")
                self.task_created = False
        else:
            rospy.logwarn("[bt_manager] ROS face_detector module not found! I Cannot execute a FACE_DETECTION task.")
            self.task_created = False

    def move_cb(self, msg):
        twist = Twist()
        twist.linear.x = float(msg[0])   # m/s
        twist.angular.z = float(msg[1])  # rad
        self.pub_twist.publish(twist)
        return TaskStatus.SUCCESS

    def stop_cb(self, msg):
        twist = Twist()
        twist.linear.x = 0.0  # m/s
        twist.angular.z = 0.0  # rad
        self.pub_twist.publish(twist)
        return TaskStatus.SUCCESS
        

# =============================================================
# ============= HRI - MOVECARE INTERFACE (Action) =============
# =============================================================
class subtree_hri(subtree):
    """
        A BT to call an Action server implementing the HRI MoveCare Interface to the user.(remainders, cognitive games)
    """

    def __init__(self, intervention, user_name):
        self.intervention = intervention
        self.user_name = user_name


        if hri_node_found:
            goal = TaskGoal()
            goal.intervention = self.intervention
            goal.name = self.user_name
            # SimpleActionTask(name, action, action_type, goal, rate=5, connect_timeout=10, result_timeout=30,
            #                  reset_after=False, active_cb=None, done_cb=None, feedback_cb=None):
            self.ROOT = SimpleActionTask("HRI", "dialog_manager", TaskAction, goal, result_timeout=600,
                                         reset_after=False, done_cb=self.done_cb, announce=True)
            self.task_created = True
        else:
            rospy.logwarn("[bt_manager] ROS dialog_manager not available (check for pkg movecare_interface).")
            self.task_created = False

    def done_cb(self, result_state, result):
        # Report decision from the Action Server
        action_response = result.response
        self.ROOT.trace = '"HRI" : "' + action_response + '"'


# =============================================================
# ==================== BATTERY MANAGER ========================
# =============================================================
class subtree_battery_manager(subtree):
    """
        A Subtree to manage the battery and ensure its health.
        It is designed to work in conjunction with:
        - A node that publishes a sensor_msgs::BatteryState msg with info about the battery voltage and charging state
        - An Autodocking node providing the action: autodocking/DoDockingAction
    """

    def __init__(self, battery_topic, low_battery_threshold, critical_battery_threshold, docking_station_pose, speech):
        self.array_battery_obs = []
        self.average_voltage = 0.0
        self.battery_topic = battery_topic
        self.low_battery_threshold = low_battery_threshold
        self.critical_battery_threshold = critical_battery_threshold
        self.docking_station_pose = docking_station_pose
        self.speech = speech
        self.low_battery_notified = False

        self.ROOT = Iterator("BATTERY_MANAGER", reset_after=False, announce=False)

        # Populate BATTERY_MANAGER (Sequence)
        # -----------------------------------
        with self.ROOT:
            # Update battery info
            monitor_battery = MonitorTask("MONITOR_BATTERY", self.battery_topic, BatteryState, self.monitor_battery_cb)
            self.ROOT.add_child(monitor_battery)

            # (A) DISCHARGING
            # ===============
            discharging = Sequence("DISCHARGING", reset_after=True)
            self.ROOT.add_child(discharging)

            # A.1 Check if discharging and low battery
            check_dis = GenericTask("CHECK_DIS", self.check_dis_cb, reset_after=True)
            discharging.add_child(check_dis)
            
            check_low_bat = GenericTask("CHECK_LOW", self.check_low_battery_cb, reset_after=True)
            discharging.add_child(check_low_bat)

            # A.2 If we reach this point, battery is low --> Notify?
            notify_low_bat = Selector("NOTIFY_LOW_BATTERY", reset_after=True)
            discharging.add_child(notify_low_bat)

            # Notify (if necessary)
            check_notification = GenericTask("CHECK", self.check_notification_cb, reset_after=False)
            notify_low_bat.add_child(check_notification)

            # SAY LOW BATTERY (Service)
            if self.speech:
                say_low_bat = subtree_say("Battery level is low. Please consider recharging in a short time.")
                if say_low_bat.task_created:
                    notify_low_bat.add_child(say_low_bat.ROOT)

            # Ensure always success
            return_success = AlwaysSucceed("SUCCESS", reset_after=False)
            notify_low_bat.add_child(return_success)


            # A.3 CRITICAL BATTERY:
            check_critical_battery = GenericTask("CHECK_CRITICAL_BATTERY", self.check_critical_battery_cb, reset_after=True)
            discharging.add_child(check_critical_battery)


            # A.4 SAY CRITICAL BATTERY (Service)
            # if we reach this point, battery is critical --> Command recharge!
            if self.speech:
                say_crit_bat = subtree_say("Battery level is Critial. Commanding an emergency recharge action.")
                if say_crit_bat.task_created:
                    discharging.add_child(say_crit_bat.ROOT)

            # A.5 AUTODOCKING
            dock = subtree_dock(docking_station_pose=self.docking_station_pose, speech=self.speech)
            if dock.task_created:
                discharging.add_child(dock.ROOT)
            else:
                say_help = subtree_say("AutoDocking node not found! I Cannot recharge my batteries. PLEASE HELP.")
                if say_help.task_created:
                    discharging.add_child(say_help.ROOT)

            # A.6 WAIT A BIT
            wait_a_bit = WaitSec("WAIT_A_BIT", 5.0, reset_after=False)
            discharging.add_child(wait_a_bit)


            # (B) CHARGING
            # ===============
            charging = Sequence("CHARGING", reset_after=False)
            self.ROOT.add_child(charging)

            # B.1 Check if charging
            check_charging = GenericTask("CHECK", self.check_charging_cb, reset_after=True)
            charging.add_child(check_charging)

            # B.2 Wait full or High priority task
            wait_full = GenericTask("WAIT_FULL", self.wait_full_cb, reset_after=False)
            charging.add_child(wait_full)


        # Always return True. since the task can be created!
        self.task_created = True



    # MONITOR_BAT: subscribe to battery topic and keep the 20 most recent values
    def monitor_battery_cb(self, msg):
        self.array_battery_obs.append(msg)  # msg is type BatteryState
        if len(self.array_battery_obs) > 20:
            self.array_battery_obs.pop(0)
        return TaskStatus.SUCCESS


    # A.1 Check if discharging
    def check_dis_cb(self, msg):
        # Check charging status
        if len(self.array_battery_obs) > 0:
            self.current_battery = self.array_battery_obs[-1]
        else:
            return TaskStatus.RUNNING
            
        if self.current_battery.power_supply_status == 0:
            # Unknown
            return TaskStatus.RUNNING
        elif self.current_battery.power_supply_status == 2 or self.current_battery.power_supply_status == 3:
            # Discharging
            return TaskStatus.SUCCESS
        else:
            # Charging
            return TaskStatus.FAILURE

    # Check if Low Battery
    def check_low_battery_cb(self, msg):
        # Get average voltage over sliding window (to avoid peaks)
        self.average_voltage = 0.0
        for obs in self.array_battery_obs:
            self.average_voltage += obs.voltage
        self.average_voltage = self.average_voltage / len(self.array_battery_obs)

        if float(self.average_voltage) > float(self.low_battery_threshold):
            # We are safe, nothing to to.
            return TaskStatus.FAILURE
        else:
            # We have crossed the threshold
            return TaskStatus.SUCCESS
        
            
    # A.2 Check if user has been notified
    def check_notification_cb(self, msg):
        if not self.low_battery_notified:
            self.low_battery_notified = True
            print colored("[bt_manager] CAUTION--> BATTERY IS LOW!!" + str(self.average_voltage) + " < " + str(self.low_battery_threshold), "red")
            return TaskStatus.FAILURE
        else:
            return TaskStatus.SUCCESS


    # A.3 Check if critical battery: Returns SUCCESS when th is exceeded
    def check_critical_battery_cb(self, msg):
        if float(self.average_voltage) > float(self.critical_battery_threshold):
            # We are safe, nothing to to.
            return TaskStatus.FAILURE
        else:
            # We have crossed the threshold
            print colored("[bt_manager] CAUTION--> BATTERY IS VERY LOW!!" + str(self.average_voltage) + " < " + str(self.critical_battery_threshold), "red")
            return TaskStatus.SUCCESS


    # B.1 Check if Charging
    def check_charging_cb(self, msg):
        # Check charging status
        self.current_battery = self.array_battery_obs[-1]
        if self.current_battery.power_supply_status == 0:
            return TaskStatus.RUNNING
        if self.current_battery.power_supply_status == 1 or self.current_battery.power_supply_status == 4:
            # Charging/Full
            return TaskStatus.SUCCESS
        else:
            # Discharging / unknown
            return TaskStatus.FAILURE


    # B.2 WAIT FULL: Returns SUCCESS only when battery is completely charged!. RUNNING otherwise
    def wait_full_cb(self, msg):
        if self.current_battery is None:
            return TaskStatus.RUNNING

        if self.current_battery.power_supply_status == 4:
            # rospy.loginfo("[task_manager] BATTERY FULL - level: " + str(float(self.current_battery.voltage)))
            return TaskStatus.SUCCESS
        else:
            # rospy.loginfo("[patrol_pkg] CHARGING BATTERY - level: " + str(float(self.current_battery.voltage)))
            return TaskStatus.RUNNING


# =============================================================
# ==================== BATTERY MANAGER SIMULATION =============
# =============================================================

class subtree_battery_manager_simul(subtree):
    """
        A Subtree to manage the battery and ensure its healthiness in a simulated environment.
        It assummes that a "fake_battery" node is running, offering the srvs:
        "fake_battery/simulate_battery_recharge"
        "fake_battery/simulate_battery_discharge"
        And that it also updates the "battery_topic" with information about battery voltage and charging state.
    """

    def __init__(self, battery_topic, low_battery_threshold, docking_station_pose):
        rospy.loginfo("[bt_manager] New Battery_Manager_Simul Task")
        if fake_battery_node_found:
            self.current_battery = None
            self.battery_topic = battery_topic
            self.low_battery_threshold = low_battery_threshold
            self.docking_station_pose = docking_station_pose
            self.pub_twist = rospy.Publisher('cmd_vel', Twist, queue_size=10)

            self.ROOT = Selector("BATTERY_MANAGER", reset_after=False, announce=False)
            # Populate STAY_HEALTHY (Selector)
            # -----------------------------------
            with self.ROOT:
                MONITOR_BATTERY = MonitorTask("MONITOR_BATTERY", self.battery_topic, BatteryState, self.monitor_battery_cb)
                self.ROOT.add_child(MONITOR_BATTERY)

                CHECK_BATTERY = GenericTask("CHECK_BATTERY", self.check_battery_cb, reset_after=False)  # it implements memory!
                self.ROOT.add_child(CHECK_BATTERY)

                CHARGE_MANEUVER = Sequence("CHARGE_MANEUVER", reset_after=False)
                self.ROOT.add_child(CHARGE_MANEUVER)

                # 1.1 Populate CHARGE_MANEUVER -> NAV_DOCK + DOCK + WAIT_FULL + UMDOCK
                # ------------------------------
                # NAV_DOCK (ActionTask)
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = self.docking_station_pose
                NAV_DOCK = SimpleActionTask("NAV_DOC", "move_base", MoveBaseAction, goal, reset_after=False)
                CHARGE_MANEUVER.add_child(NAV_DOCK)

                # DOCK (Service/Action)
                # (For Simulation) Call the fake_battery service "recharge"
                # (For Real Robot) Substitute this to the call of the service/action that performs Docking!
                DOCK = ServiceTask("DOCK", "fake_battery/simulate_battery_recharge", simulate_battery_recharge, 1,
                                   result_cb=self.dock_cb)
                CHARGE_MANEUVER.add_child(DOCK)

                # WAIT_FULL (GenericTask)
                WAIT_FULL = GenericTask("WAIT_FULL", self.check_full_cb, reset_after=False)
                CHARGE_MANEUVER.add_child(WAIT_FULL)

                # UNDOCK
                DO_UNDOCK = GenericTask("UNDOCK", self.doUndock_cb, reset_after=False)
                CHARGE_MANEUVER.add_child(DO_UNDOCK)

                # UNDOCK (Service/Action)
                # (For Simulation) Call the fake_battery srv discharge
                # (For Real Robot) Substitute this to the call of the service that performs UnDocking!
                UNDOCK = ServiceTask("UNDOCK", "fake_battery/simulate_battery_discharge", simulate_battery_discharge, 1,
                                     result_cb=self.undock_cb, reset_after=False)
                CHARGE_MANEUVER.add_child(UNDOCK)
            self.task_created = True
        else:
            rospy.logwarn("[bt_manager] ROS FakeBattery node not found! I Cannot manage a battery_manager_simul task.")
            self.task_created = False



    # MONITOR_BAT: subscribe to battery topic and keep the most recent value
    def monitor_battery_cb(self, msg):
        self.current_battery = msg  # type is BatteryState
        return TaskStatus.FAILURE   # a failure leads to select the next task -->check_battery


    # CHECK_BAT: Returns SUCCESS only if high voltage and it is NOT charging!
    def check_battery_cb(self, msg):
        if self.current_battery.voltage is None:
            return TaskStatus.RUNNING
        else:
            if self.current_battery.power_supply_status == 2 or self.current_battery.power_supply_status == 3:  # Discharging
                if self.current_battery.voltage > self.low_battery_threshold:  # PERFECT!
                    return TaskStatus.SUCCESS
                else:
                    rospy.loginfo("[patrol_pkg] LOW BATTERY! - level: " + str(float(self.current_battery.voltage)))
                    return TaskStatus.FAILURE  # A failure leads to select the next tast -->stay_charged!
            else:
                return TaskStatus.FAILURE  # Failure becasue we are not ready to navigate! A failure leads to select the next tast -->stay_charged!


    # CHECK_FULL: Returns SUCCESS only when battery is completely charged!. RUNNING otherwise
    def check_full_cb(self, msg):
        if self.current_battery is None:
            return TaskStatus.RUNNING

        if self.current_battery.power_supply_status == 4:
            rospy.loginfo("[patrol_pkg] BATTERY FULL - level: " + str(float(self.current_battery.voltage)))
            return TaskStatus.SUCCESS
        else:
            rospy.loginfo("[patrol_pkg] CHARGING BATTERY - level: " + str(float(self.current_battery.voltage)))
            return TaskStatus.RUNNING


    # (For Simulation) Called when Docking!
    def dock_cb(self, result):
        rospy.loginfo("[patrol_pkg] DOCK!")

    # UNDOCK: Simple Undock - Move back for 2 seconds at slow speed (no reactive!)
    def doUndock_cb(self, msg):
        twist = Twist()
        twist.angular.z = 0.0  # rad
        twist.linear.x = -0.1  # m/s
        t_start = rospy.get_time()
        while rospy.get_time() - t_start < 2.0:
            self.pub_twist.publish(twist)
            rospy.sleep(0.1)  # seconds

        turn_time = 4  # sec
        twist.angular.z = 3.14159 / turn_time  # rad
        twist.linear.x = 0.0  # m/s
        t_start = rospy.get_time()
        while rospy.get_time() - t_start < turn_time:
            self.pub_twist.publish(twist)
            rospy.sleep(0.1)  # seconds

        # stop
        twist.angular.z = 0.0  # rad
        twist.linear.x = 0.0  # m/s
        self.pub_twist.publish(twist)

        return TaskStatus.SUCCESS

    # (For Simulation) Called when unDocking!
    def undock_cb(self, result):
        rospy.loginfo("[patrol_pkg] UNDOCK!")
        
        

# =============================================================
# ====================== PATROL ===============================
# =============================================================
class subtree_patrol(subtree):
    """
        A subtree to implement a patrol task
    """

    def __init__(self, list_poses, random=False, battery_topic="/battery", speech=False):
        self.last_nav_result = 0

        self.ROOT = Sequence("PATROL", reset_after=False, announce=False)

        # Ensure not charging
        ensure_discharging = subtree_ensure_not_charging(battery_topic, speech=False)
        if ensure_discharging.task_created:
            self.ROOT.add_child(ensure_discharging.ROOT)

        # Create iterator
        if random:
            self.it = RandomIterator("PATROL_RANDOM", announce=True)
            self.ROOT.add_child(self.it)
        else:
            self.it = Iterator("PATROL", announce=True)
            self.ROOT.add_child(self.it)

        # Populate the Iterator (with Sequences)
        n_waypoints = len(list_poses)
        for i in range(n_waypoints):
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = list_poses[i]
            action_task = SimpleActionTask("GO_TO_WP_" + str(i), "move_base", MoveBaseAction, goal,
                                           result_timeout=120, reset_after=False, done_cb=self.nav_done_cb, announce=True)
            
            self.it.add_child(action_task)
            """
            if google_speech_node_found and speech:
                S = Iterator("NAV_" + str(i), reset_after=False)
                self.it.add_child(S)
                

                # Talk
                try:
                    srv_say_before = ServiceTask("SAY", "speech/say_text", say, "Starting navigation to Waypoint " + str(i), result_cb=None)
                    S.add_child(srv_say_before)
                except rospy.ROSException as excp:
                    print colored("[bt_manager] " + str(excp) + ". Disabling Speech", "red")
                    
                # Navigate
                S.add_child(action_task)
                
                # Talk
                try:
                    srv_say_after = ServiceTaskDynamic("SAY", "speech/say_text", say, request_cb=self.say_request_cb, result_cb=None)
                    S.add_child(srv_say_after)
                except rospy.ROSException as excp:
                    print colored("[bt_manager] " + str(excp) + ". Disabling Speech", "red")
            else:
                S = Iterator("NAV_" + str(i), reset_after=False)
                self.it.add_child(S)
                
                # Navigate
                S.add_child(action_task)     

                # Find_person
                fd = subtree_find_person(action_type=0, tout=10, just_once=False, speech=True)
                if fd.task_created:
                    S.add_child(fd.ROOT)

                # Wait (2min)
                wt = subtree_wait(120, announce=False)
                if wt.task_created:
                    S.add_child(wt.ROOT)
                """
        self.task_created = True


    #const actionlib::SimpleClientGoalState& state, const ResultConstPtr& result
    def nav_done_cb(self, result_state, result):
        self.last_nav_result = result_state   #3(success), else(failure)
        # Report error if navigation not completed
        if result_state != 3:
            rospy.logwarn("[bt_manager] ROS Navigation NOT ok. Self node  name is: " + str(self.ROOT.name) )
            self.ROOT.trace = '"GO_TO_POINT" : "ERROR navigation not successful"'

    def say_request_cb(self):
        if self.last_nav_result == 3:
            return "Navigation ended successfully"
        else:
            return "Navigation cancelled. I could not complete te action."

    def patrol_feedback_cb(self, msg):
        #msg is of type MoveBaseActionFeedback
        rospy.loginfo("[PATROL FEEDBACK_CB] IM HERE: [" + str(msg.base_position.pose.position.x)+ " , " + str(msg.base_position.pose.position.y) + " , " + str(msg.base_position.pose.orientation.z)+ " ] " )


# =============================================================
# ===============      SEMANTIC - LOAD ONTOLOGY    ============
# =============================================================
class subtree_load_ontology(subtree):
    """
        A Task to load a new ontology to work with
    """

    def __init__(self, ontology_path, ontology_filename, ontology_URI):
        if ontology_bridge_node_found:
            self.ontology_path = ontology_path
            self.ontology_filename = ontology_filename
            self.ontology_URI = ontology_URI


            try:
                self.srv_req = semantic_requestRequest("load_ontology", (self.ontology_path, self.ontology_filename, self.ontology_URI))
                self.ROOT = ServiceTask("LOAD_ONTOLOGY", "ontology_bridge/semantic_request", semantic_request,
                                           self.srv_req, result_cb=self.semantic_request_cb, announce=True)
            except rospy.ROSException as excp:
                print colored("[bt_manager] " + str(excp) + ". Skipping Task", "red")
                self.task_created = False

            self.task_created = True
        else:
            rospy.logwarn("[bt_manager] ROS ontology_bridge not found! I Cannot manage a load_ontology task.")
            self.task_created = False


    # SEMANTIC_REQUEST
    def semantic_request_cb(self, result):
        rospy.loginfo("[iro_bt] Ontology loaded")


# =============================================================
# ===================== GAS SOURCE LOCALIZATION ===============
# =============================================================
class subtree_gsl(subtree):
    """
        A GSL Task based on Action Server
    """
    def __init__(self, gas_conc_topic, gas_conc_threshold, gsl_method):
        if olfaction_node_found and gsl_node_found:
            self.gas_conc_topic = gas_conc_topic
            self.gas_conc_th = gas_conc_threshold
            self.gsl_method = gsl_method

            self.ROOT = Invert("GSL", reset_after=False, announce=True)

            self.S = Sequence("GSL", reset_after=False)
            self.ROOT.add_child(self.S)
            self.array_enose_obs = []       #for gas classification
            self.array_gas_conc_obs = []    #for gas detection (concentration)
            self.most_probable_smell_detected = "demo_smell"
            self.gas_source_candidates_from_semantic = []


            # Populate GSL (Sequence)
            # ------------------------------
            with self.S:
                # MONITOR_GAS_CONC = MonitorTask("MONITOR_GAS_CONC", self.gas_conc_topic, gas_sensor_array, self.monitor_gas_conc_cb)
                # self.S.add_child(MONITOR_GAS_CONC)

                # CHECK_GAS_CONC = GenericTask("CHECK_GAS_CONC", self.check_gas_conc_cb, reset_after=False)
                # self.S.add_child(CHECK_GAS_CONC)

                # Do GSL (Action)
                goal = gsl_action_msgGoal()
                goal.gsl_method = self.gsl_method
                GSL = SimpleActionTask("GSL", "gasSourceLocalization", gsl_action_msgAction, goal, result_timeout=120,
                                           reset_after=False, done_cb=self.gsl_action_done_cb)
                self.S.add_child(GSL)

            self.task_created = True
        else:
            rospy.logwarn("[bt_manager] ROS ontology_bridge or scikit_bridge nodes not found! I Cannot manage a semantic_gsl task.")
            self.task_created = False



    # MONITOR_GAS_CONC: subscribe to the gas_conc sensor topic and keep the 10 most recent observations
    def monitor_gas_conc_cb(self, msg):
        # rospy.loginfo("[MONITOR_GAS_CONC] New Observation")
        self.array_gas_conc_obs.append(msg)    # msg is type olfaction_msgs/gas_sensor_array
        if len(self.array_gas_conc_obs) > 10:
            self.array_gas_conc_obs.pop(0)
        return TaskStatus.SUCCESS

    # MONITOR_SMELL: subscribe to enose topic and keep the 10 most recent observations
    def monitor_smell_cb(self, msg):
        # rospy.loginfo("[MONITOR_SMELL] New Observation")
        self.array_enose_obs.append(msg)  # msg is type olfaction_msgs/gas_sensor_array
        if len(self.array_enose_obs) > 10:
            self.array_enose_obs.pop(0)
        return TaskStatus.SUCCESS

    # CHECK_GAS_CONC: Returns SUCCESS if the gas conc is over the th
    # It considers a sliding_window of 10 samples (smooth) to see if there is significant gas present.
    def check_gas_conc_cb(self, msg):
        obs_data = []
        for obs in self.array_gas_conc_obs:
            sensors_data = []
            sensors_data.append(obs.header.stamp.to_sec())
            sensors_data.append(obs.header.seq)
            for s in obs.sensors:
                sensors_data.append(s.raw)
            # Now append this e-nose observation
            obs_data.append(sensors_data)
        obs_data_array = np.array(obs_data)  # from list to np.array

        # Did any of the sensors in the gas_sensor array smell something?
        if (obs_data_array[:, 2:].mean(axis=0)).max(0) >= self.gas_conc_th:
            rospy.loginfo("[SMELL SOMTHING] " + str(float((obs_data_array[:, 2:].mean(axis=0)).max(0))) + " > " + str(
                float(self.gas_conc_th)))
            return TaskStatus.SUCCESS  # There is gas
        else:
            # rospy.loginfo( "[NO_SMELL] " + str(float( (obs_data_array[:, 2:].mean(axis=0)).max(0) )) + " < " + str(float(self.smell_th)) )
            return TaskStatus.FAILURE  # There is no gas


    def gsl_action_done_cb(self, result_state, result):
        print colored("\n\n\n[iro_bt] GSL ended.\n\n\n", "green")
        return TaskStatus.SUCCESS




# =============================================================
# ===================== MOVECARE: SCENARIO   =================
# =============================================================
class subtree_movecare_intervention(subtree):
    """
        A BT to implement the MoveCare Interventions
    """
    def __init__(self, battery_topic, username, hri_code, locationLabels, locationPoses, text_to_speech):
        self.battery_topic = battery_topic              # type string
        self.username = "Matteo"                        # type string
        self.hri_code = hri_code                        # type int
        self.locationLabels = locationLabels            # type string list
        self.locationPoses = locationPoses              # type PoseArray
        self.text_to_speech = text_to_speech            # type string list

        if hri_code == 1:
            self.ROOT = Sequence("WM", reset_after=False, announce=True)
        elif hri_code == 2:
            self.ROOT = Sequence("CH", reset_after=False, announce=True)
            talk = subtree_say("I received a call for help. Do not worry, I'm going to assist you.")
            if talk.task_created:
                self.ROOT.add_child(talk.ROOT)
        elif hri_code == 3:
            self.ROOT = Sequence("CT", reset_after=False, announce=True)
        elif hri_code == 4:
            self.ROOT = Sequence("CG", reset_after=False, announce=True)
        elif hri_code == 5:
            self.ROOT = Sequence("VQ/GQ", reset_after=False, announce=True)
        elif hri_code == 9:
            self.ROOT = Sequence("TALK", reset_after=False, announce=True)
        else:
            rospy.logerror("[MOVECARE_INT] Error: hri_code not implemented" )
            # Task created successfully
            self.task_created = False
            return;

        # 1. Ensure not charging
        ensure_discharging = subtree_ensure_not_charging(self.battery_topic, speech=False)
        if ensure_discharging.task_created:
            self.ROOT.add_child(ensure_discharging.ROOT)

        # 2.Ensure OpenPose is off (while navigating)
        if openpose_node_found:        
            # Ensure we stop to save CPU
            self.srv_req = StartDetectionHumanRequest("off","off")
            openpose = ServiceTask("OpenPose-off-off", "openpose/start_detection_humans_service", StartDetectionHuman, self.srv_req ,result_cb=None, wait_for_service=True, timeout=5, announce=False)
            self.ROOT.add_child(openpose)
        #    else:
        #        # Normal operation is low freq to save battery
        #        self.srv_req = StartDetectionHumanRequest("low","off")
        #        openpose = ServiceTask("OpenPose-LOW-off", "openpose/start_detection_humans_service", StartDetectionHuman, self.srv_req ,result_cb=None, wait_for_service=True, timeout=5, announce=False)
        #        self.ROOT.add_child(openpose)
        #else:
        #    rospy.logwarn("[bt_manager] OpenPose SRV not available")
        
        # 3. Go to user expected location
        #nav_user = subtree_go_to_point(name="NAV", pose=self.user_expected_pose)
        #if nav_user.task_created:
        #    self.ROOT.add_child(nav_user.ROOT)

        # 3. Find & Approach user
        if hri_code == 29999:
            # Emergency Situation --> intensive human detection
            fd = subtree_find_appro_person(room_labels=self.locationLabels, room_poses=self.locationPoses, intensive_search=True, tout=600, speech=False)
            if fd.task_created:
                self.ROOT.add_child(fd.ROOT)
        else:
            # Normal intervention --> Use low frequency in human detection for battery saving
            fd = subtree_find_appro_person(room_labels=self.locationLabels, room_poses=self.locationPoses, intensive_search=False, tout=600, speech=False)
            if fd.task_created:
                self.ROOT.add_child(fd.ROOT)



        # 5. Look at the User?

        # 6. User Recognition?

        # 7. HRI
        if hri_code == 9:
            #talk2 = subtree_say("Hi " + str(username) + " your weight was just righ. Well Done!")
            talk2 = subtree_say(self.text_to_speech[0])
            if talk2.task_created:
                self.ROOT.add_child(talk2.ROOT)
        else:
            hri = subtree_hri(intervention=self.hri_code, user_name=self.username)
            if hri.task_created == True:
                self.ROOT.add_child(hri.ROOT)
            else:
                rospy.logwarn("[task_manager] Cannot create subtree of type User Dialog(HRI)")

        # Task created successfully
        self.task_created = True




# =============================================================
# ===================== MOVECARE: RFID-SEARCH =================
# =============================================================
class subtree_movecare_rfid_search(subtree):
    """
        A BT to implement the MoveCare Search Lost Object scenario
    """
    def __init__(self, objectID, locationsArray):
        self.object_id = objectID               # type String
        self.locationsArray = locationsArray    # type geometry_msgs/PoseArray

        self.ROOT = Sequence("OS", reset_after=False, announce=True)

        talk = subtree_say("I received your request to help you finding your " + self.object_id + ". Lets do it together.")
        if talk.task_created:
            self.ROOT.add_child(talk.ROOT)


        if object_search_node_found:
            # Execute the rfid_actionServer
            goal = rfidGoal()
            goal.uid = self.object_id               #string
            goal.locations = self.locationsArray    #geometry_msgs/PoseArray
            os = SimpleActionTask("OS", "rfid_server", rfidAction, goal, done_cb=self.done_cb, result_timeout=3000, reset_after=False, announce=True)
            self.ROOT.add_child(os)

            #DEBUG
            #rospy.loginfo("[OS] object_id = %s", str(goal.uid))
            #rospy.loginfo("[OS] object_locations = %u", len(goal.locations.poses) )
            #for p in goal.locations.poses:
            #    rospy.loginfo("[OS]     Pose (x,y)=(%.3f, %.3f)", p.position.x, p.position.y )

            # Task created successfully
            self.task_created = True
        else:
            rospy.logwarn("[task_manager] Cannot create subtree of type Object Search")
            self.task_created = False


    def done_cb(self, result_state, result):
        rospy.loginfo("[bt_manager] Object Search Task completed. Sending result to MQTT.")

        # Report results of the search (found, object_id, location[x,y,phi])

        # Transform from pose to string [x,y,phi]
        obj_loc = result.final_location  # geometry_msg/Pose
        orientation_q = obj_loc.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        obj_loc_str = "[" + str(obj_loc.position.x) + "," + str(obj_loc.position.y) + "," + str(yaw) + "]"

        if result.found == True:
            rospy.loginfo("[bt_manager] Object Search - Object found.")
            self.ROOT.trace = '"found" : "yes", '
        else:
            rospy.loginfo("[bt_manager] Object Search - Object NOT found.")
            self.ROOT.trace = '"found" : "no", '

        self.ROOT.trace += '"objectlocation" : "' + obj_loc_str + '", '
        self.ROOT.trace += '"objectid" : "' + result.uid + '"'




# =============================================================
# ===================== DEMO: SCENARIO   =================
# =============================================================
class subtree_demo(subtree):
    """
        A BT to implement demos
    """
    def __init__(self):
                
        self.ROOT = Sequence("DEMO", reset_after=False, announce=False)
        
        # 1. Ensure not charging
        ensure_discharging = subtree_ensure_not_charging("giraff_battery_filtered", speech=False)
        if ensure_discharging.task_created:
            self.ROOT.add_child(ensure_discharging.ROOT)
        
        # 2. Go To camera location
        pose2D = [5.5, -2.5, 1.57079]
        quat = quaternion_from_euler(0, 0, pose2D[2])
        pose3D = (Pose(Point(pose2D[0], pose2D[1], 0.0), Quaternion(*quat)))
        nav_cam = subtree_go_to_point(name="GO_CAM", pose=pose3D)
        if nav_cam.task_created:
            self.ROOT.add_child(nav_cam.ROOT)
        
        # 3. Talk
        talk = subtree_say("Hello. My name is Giraff X.")
        if talk.task_created:
            self.ROOT.add_child(talk.ROOT)
        talk = subtree_say("I am the robot of the Move Care project.")
        if talk.task_created:
            self.ROOT.add_child(talk.ROOT)
        talk = subtree_say("I have to deliver a remainder to the user. Please come with me.")
        if talk.task_created:
            self.ROOT.add_child(talk.ROOT)
        # Wait 8s
        wait = subtree_wait(8, announce=False)
        if wait.task_created:
            self.ROOT.add_child(wait.ROOT)
            
        # 4. Go to room
        pose2D = [2.7, -3.23, 3.34159]
        quat = quaternion_from_euler(0, 0, pose2D[2])
        pose3D = (Pose(Point(pose2D[0], pose2D[1], 0.0), Quaternion(*quat)))
        nav_room = subtree_go_to_point(name="GO_ROOM", pose=pose3D)
        if nav_room.task_created:
            self.ROOT.add_child(nav_room.ROOT)
            

        # 5. Find & Approach user
        # If user not found (FAILURE) we wont continue (command a global_user_search)
        # If user found (SUCCESS), approach the user to start HRI
        fd = subtree_find_appro_person(tout=120, speech=False)
        if fd.task_created:
            self.ROOT.add_child(fd.ROOT)
            
        # 6. HRI
        talk3 = subtree_say("Hello Paco.")
        if talk3.task_created:
            self.ROOT.add_child(talk3.ROOT)
        talk3 = subtree_say("This is an example of remainder from your caregiver.")
        if talk3.task_created:
            self.ROOT.add_child(talk3.ROOT)            
        # Wait 3s
        wait = subtree_wait(3, announce=False)
        if wait.task_created:
            self.ROOT.add_child(wait.ROOT)

        talk4 = subtree_say("Just remember that I'm here to assit you if you need me. Bye.")
        if talk4.task_created:
            self.ROOT.add_child(talk4.ROOT)     

        # 7. Go To camera location
        pose2D = [5.5, -2.5, 1.57079]
        quat = quaternion_from_euler(0, 0, pose2D[2])
        pose3D = (Pose(Point(pose2D[0], pose2D[1], 0.0), Quaternion(*quat)))
        nav_cam = subtree_go_to_point(name="GO_CAM", pose=pose3D)
        if nav_cam.task_created:
            self.ROOT.add_child(nav_cam.ROOT)
        
        # 3. Talk
        talk = subtree_say("Hello again. I'm back from my duty.")
        if talk.task_created:
            self.ROOT.add_child(talk.ROOT)
        talk = subtree_say("As you can see I'm a fully autonomous robot, looking forward to help people.")
        if talk.task_created:
            self.ROOT.add_child(talk.ROOT)
        talk = subtree_say("Since I have nothing more to do for the moment, I am going to wait in the docking station. See you soon!")
        if talk.task_created:
            self.ROOT.add_child(talk.ROOT)
        # Wait 3s
        wait = subtree_wait(10, announce=False)
        if wait.task_created:
            self.ROOT.add_child(wait.ROOT)
            
        # Dock
        pose2D = [6.0, -6.0, 3.14159]
        quat = quaternion_from_euler(0, 0, pose2D[2])
        pose3D = (Pose(Point(pose2D[0], pose2D[1], 0.0), Quaternion(*quat)))
        dock = subtree_dock(pose3D, False)
        if dock.task_created:
            self.ROOT.add_child(dock.ROOT)
        
        
        """
        # Patrol
        waypoints = []
        manual_wp = [ [15.70, 7.22, 1.5], [23.1, 15.1, 3.14159], [8.0, 10.0, 0.0], [16.0, 14.6, -1.6], [15.70, 7.22, 1.5], [23.1, 15.1, 3.14159], [8.0, 10.0, 0.0], [16.0, 14.6, -1.6] ]
        for i in manual_wp:
            pose2D = i 
            quat = quaternion_from_euler(0, 0, pose2D[2])
            pose3D = (Pose(Point(pose2D[0], pose2D[1], 0.0), Quaternion(*quat)))
            waypoints.append(pose3D)
        # Create Task (subtree)
        p = subtree_patrol(waypoints, random=False, battery_topic="/giraff_battery_filtered", speech=True)
        if p.task_created:
            self.ROOT.add_child(p.ROOT)
        """
        
        # Task created successfully
        self.task_created = True



# =============================================================
# ===================== TELEOPERATION   =======================
# =============================================================
class subtree_teleoperation(subtree):
    """
        A BT to handle teleoperation of the robot via an action server
    """
    def __init__(self, callerID, teleopURL, requestConfirmation):
        self.caller_id = callerID       #string
        self.teleop_url = teleopURL     #string
        self.reqConf = requestConfirmation  #bool
        if teleop_node_found:
            goal = teleop_requestGoal()
            goal.callerID = self.caller_id
            goal.teleoperation_url = self.teleop_url
            goal.confirmation = self.reqConf
            self.ROOT = SimpleActionTask("TO", "teleoperation", teleop_requestAction, goal, done_cb=self.done_cb, result_timeout=36000, reset_after=False, announce=True)
            self.task_created = True
        else:
            rospy.logwarn("[bt_manager] ROS Teleoperation node not found! I Cannot manage a Teleoperation task.")
            self.task_created = False


    def done_cb(self, result_state, result):
        # Report error if not success (3=success)
        if result_state != 3:
            rospy.logwarn("[bt_manager] Teleoperation Finished")
            self.ROOT.trace = '"' + "teleoperation" + '" : "Finished"'
