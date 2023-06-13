#!/usr/bin/env python

"""
This is a compendium of Jobs that can be used by the task_manager.
Each job is implemented following the Behaviour Tree philosophy as a tree.
Upon instantation, these job will become sub-trees and appended to the "tasks" tree 
handled by the task_manager.
"""
import rclpy
import rclpy.node as Node
import py_trees
import py_trees.console as console
import py_trees_ros
import typing

# standar ROS2 msgs
import std_msgs
import diagnostic_msgs.msg
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

"""
Import here all the actions/srv/msgs the TaskManager must deal with
Consider that some utilities may not be availbale (node not executed)
therefore, check imports to avoid runtime errors
"""

# NAV2 NavigateToPose
try:
    from nav2_msgs.action import NavigateToPose
    can_navigate = True
except Exception as excp:
    can_navigate = False

# NAV MapServer/GetMap
try:
    from nav_msgs.srv import GetMap
    can_get_map = True
except Exception as excp:
    can_get_map = False

# Patrol PatrolTimes
try:
    from patrol.action import PatrolTimes
    can_patrol = True
except Exception as excp:
    can_patrol = False


# =============================================================
#                           SAY(dialogue)
# =============================================================
def subtree_say(req)-> py_trees.behaviour.Behaviour:
    """
    A subtree to implement a simple talk
    By default just writes the msg over the /ros2mqtt topic
    """
    try:
        # Create msg to publish (over MQTT) on tic
        say_msg = diagnostic_msgs.msg.KeyValue()
        say_msg.key = "TTS"
        say_msg.value = req.task_args[0]

        # Create Task publisher
        subtree = TaskPublisher(
            name = req.task_name,
            topic_name = "/ros2mqtt",    # publish over MQTT
            topic_type = diagnostic_msgs.msg.KeyValue,
            msg = say_msg,
            qos_profile = 1,
            repetitions = req.task_repetitions
        )
        return subtree

    except Exception as excp:
        job = py_trees.behaviours.Dummy()
        job.name = "invalid"
        job.feedback_message = "[subtree_say] Exception creating job: " + str(excp) + ". Skipping request."
        print(console.red + job.feedback_message + console.reset)
        return job


# =============================================================
#                           GOTO_POSE(pose)
# =============================================================
def subtree_goto_pose(req):
    """
    A subtree to implement an action client for navigation NavigateToPose   
    """
    try:
        if not can_navigate:
            # Nav2 not available, skip request
            raise Exception("Nav2 not available.")
        
        # Create NavigateToPose Goal
        goal_msg = NavigateToPose.Goal()

        # Fill pose msg
        wp = PoseStamped()
        wp.header.frame_id = "map"
        wp.header.stamp = Node.Clock().now().to_msg()
        wp.pose.position.x = float(req.task_args[0])
        wp.pose.position.y = float(req.task_args[1])
        wp.pose.position.z = float(req.task_args[2])
        wp.pose.orientation.x = float(req.task_args[3])
        wp.pose.orientation.y = float(req.task_args[4])
        wp.pose.orientation.z = float(req.task_args[5])
        wp.pose.orientation.w = float(req.task_args[6])
        goal_msg.pose = wp

        # Create Task Action Client
        subtree = TaskActionClient(
            name = "goto_to_pose",
            action_type = NavigateToPose,
            action_name = "navigate_to_pose", # navigation action server of the /bt_navigator?
            action_goal = goal_msg,
            generate_feedback_message = lambda msg: "{}".format(msg.feedback.distance_remaining),
            repetitions = req.task_repetitions
        )    
        return subtree
    
    except Exception as excp:
        job = py_trees.behaviours.Dummy()
        job.name = "invalid"
        job.feedback_message = "[subtree_goto_pose] Exception creating job: " + str(excp) + ". Skipping request."
        print(console.red + job.feedback_message + console.reset)
        return job


# =============================================================
#                           PATROL()
# =============================================================
def subtree_patrol(req):
    """
    A subtree to implement an action client for PatrolTimes
    """
    try:
        if not can_patrol:
            # Patrol node not available, skip request
            raise Exception("Patrol node not available.")
        
        # Create PatrolTimes Goal
        goal_msg = PatrolTimes.Goal()
        goal_msg.times = int(req.task_args[0])      #times to repeat the patrol 

        # Create Task Action Client
        subtree = TaskActionClient(
            name = "patrol",
            action_type = PatrolTimes,
            action_name = "patrol_times",
            action_goal = goal_msg,
            generate_feedback_message = lambda msg: "{}".format(msg.feedback.times_completed),
            repetitions = req.task_repetitions
        )    
        return subtree
    
    except Exception as excp:
        job = py_trees.behaviours.Dummy()
        job.name = "invalid"
        job.feedback_message = "[subtree_goto_pose] Exception creating job: " + str(excp) + ". Skipping request."
        print(console.red + job.feedback_message + console.reset)
        return job

    

# =============================================================
#                           GET_MAP()
# =============================================================
def subtree_get_map(req):
    """
    A subtree to implement a service client for /map_server/map
    """
    try:
        # Create map_server/map requet (Empty)
        
        # Create Task service Client
        subtree = TaskSrvClient(
            name = "get_map",
            srv_type = GetMap,
            srv_name = "/map_server/map",
            srv_request = GetMap.Request(),
            wait_for_server_timeout_sec = -3.0,
            repetitions = req.task_repetitions
        )    
        return subtree
    
    except Exception as excp:
        job = py_trees.behaviours.Dummy()
        job.name = "invalid"
        job.feedback_message = "[subtree_get_map] Exception creating job: " + str(excp) + ". Skipping request."
        print(console.red + job.feedback_message + console.reset)
        return job


# =========================================================================
#               MAPIR TASKS (extend py_trees_ros)
# =========================================================================
class TaskPublisher(py_trees.behaviour.Behaviour):
    """
    This Task just publish an incoming msg to the specified topic
    This is a non-blocking behaviour -always returning:`~py_trees.common.Status.SUCCESS`.

    Args:
        name: name of the behaviour
        topic_name: name of the topic to connect to on ROS2
        topic_type: class of the message type (e.g. :obj:`std_msgs.msg.String`)
        msg: content to be published
        qos_profile: qos profile for the subscriber
        repetitions: number of times to execute this task before prunning (-1 = inf)
    """
    def __init__(self,
                 name: str,
                 topic_name: str,
                 topic_type: typing.Any,                 
                 msg: typing.Any,
                 qos_profile: rclpy.qos.QoSProfile,
                 repetitions: int=1
                 ):
        super().__init__(name=name)
        self.topic_name = topic_name
        self.topic_type = topic_type        
        self.msg_to_publish = msg
        self.qos_profile = qos_profile
        self.repetitions = repetitions
        self.publisher = None           # on setup
        self.node = None                # on setup

    def setup(self, **kwargs):
        """
        Initialises the publisher.

        Args:
            **kwargs (:obj:`dict`): distribute arguments to this
               behaviour and in turn, all of it's children

        Raises:
            KeyError: if a ros2 node isn't passed under the key 'node' in kwargs
        """
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "Didn't find 'node' in setup's kwargs [{}][{}]".format(self.name, self.__class__.__name__)
            raise KeyError(error_message) from e  # 'direct cause' traceability
        
        # Create publisher
        self.publisher = self.node.create_publisher(
            msg_type = self.topic_type,
            topic = self.topic_name,
            qos_profile = self.qos_profile
        )

    def update(self):
        """
        Publish the specified msg on the requested ros2 topic

        Raises:
            TypeError if the msg_to_be_published is not of the required type

        Returns:
            :data:`~py_trees.common.Status.SUCCESS` (published)
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        try:
            if isinstance(self.msg_to_publish, self.topic_type):
                self.publisher.publish(self.msg_to_publish)
            else:
                raise TypeError("{} is not the required type [{}][{}]".format(
                    self.msg_to_publish,
                    self.topic_type,
                    type(self.msg_to_publish))
                )
            self.feedback_message = "topic published"
            return py_trees.common.Status.SUCCESS
        except KeyError:
            self.feedback_message = "Error when publishing"
            return py_trees.common.Status.FAILURE



class TaskSrvClient(py_trees.behaviour.Behaviour):
    """
    This Task calls a service passing as input a msg and returning the reply
    This is a blocking behaviour until the service response arrives

    Args:
        name: name of the behaviour
        srv_name: name of the service to connect to on ROS2
        srv_type: class of the message type (e.g. :obj:`std_msgs.msg.String`)
        srv_request: content used in the request call
        wait_for_server_timeout_sec: use negative values for a blocking but periodic check (default: -3.0)
        repetitions: number of times to execute this task before prunning (-1 = inf)
    """
    def __init__(self,
                 name: str,
                 srv_name: str,
                 srv_type: typing.Any,                 
                 srv_request: typing.Any,
                 wait_for_server_timeout_sec: float=-3.0,
                 repetitions: int=1
                 ):
        super().__init__(name=name)
        self.srv_name = srv_name
        self.srv_type = srv_type        
        self.srv_request = srv_request
        self.wait_for_server_timeout_sec = wait_for_server_timeout_sec
        self.repetitions = repetitions
        self.node = None                 # on setup
        self.srv_client = None           # on setup

    def setup(self, **kwargs):
        """
        Setup the service client.

        Args:
            **kwargs (:obj:`dict`): distribute arguments to this
               behaviour and in turn, all of it's children

        Raises:
            KeyError: if a ros2 node isn't passed under the key 'node' in kwargs
        """
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "Didn't find 'node' in setup's kwargs [{}][{}]".format(self.name, self.__class__.__name__)
            raise KeyError(error_message) from e  # 'direct cause' traceability
        
        # Create service client
        self.srv_client = self.node.create_client(
            srv_type = self.srv_type,
            srv_name = self.srv_name
        )

        # Wait for service server
        if self.wait_for_server_timeout_sec > 0.0:
            result = self.srv_client.wait_for_service(timeout_sec = self.wait_for_server_timeout_sec)
        else:
            iterations = 0
            period_sec = -1.0*self.wait_for_server_timeout_sec
            result = None
            while not result:
                iterations += 1
                result = self.srv_client.wait_for_service(timeout_sec=period_sec)
                if not result:
                    self.node.get_logger().warning(
                        "waiting for service server ... [{}s][{}][{}]".format(
                            iterations * period_sec,
                            self.srv_name,
                            self.qualified_name
                        )
                    )
        if not result:
            self.feedback_message = "timed out waiting for the srv server [{}]".format(self.srv_name)
            self.node.get_logger().error("{}[{}]".format(self.feedback_message, self.qualified_name))
            raise exceptions.TimedOutError(self.feedback_message)
        else:
            self.feedback_message = "... connected to srv server [{}]".format(self.srv_name)
            self.node.get_logger().info("{}[{}]".format(self.feedback_message, self.qualified_name))
        
    
    def initialise(self):
        """
        Reset the internal variables and kick off a new srv request.
        """
        self.logger.debug("{}.initialise()".format(self.qualified_name))

        # initialise some temporary variables
        self.srv_future = None        

        try:
            self.srv_future = self.srv_client.call_async(self.srv_request)
            self.feedback_message = "sending goal ..."            
        except KeyError:
            pass  # self.send_goal_future will be None, check on that


    def update(self):
        """
        Check only to see whether the underlying srv server has
        succeeded, is running, or has cancelled/aborted for some reason and
        map these to the usual behaviour return states.

        Returns:
            :class:`py_trees.common.Status`
        """
        self.logger.debug("{}.update()".format(self.qualified_name))

        if self.srv_future is None:
            self.feedback_message = "no srv_request to send"
            return py_trees.common.Status.FAILURE
        
        if self.srv_future.done():
            res = self.srv_future.result()
            #self.node.get_logger().info("Received service result: {}".format(res))
            
            # There is no common response type, so return "all"
            self.feedback_message = "{}".format(str(res))
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
        


class TaskActionClient(py_trees_ros.action_clients.FromConstant):
    """
    Convenience version of the action client that only ever sends the
    same goal.

    .. see-also: :class:`py_trees_ros.action_clients.FromBlackboard`

    Args:
        name: name of the behaviour
        action_type: spec type for the action (e.g. move_base_msgs.action.MoveBase)
        action_name: where you can find the action topics & services (e.g. "bob/move_base")
        action_goal: the goal to send
        generate_feedback_message: formatter for feedback messages, takes action_type.Feedback
            messages and returns strings (default: None)
        wait_for_server_timeout_sec: use negative values for a blocking but periodic check (default: -3.0)
        repetitions: number of times to tic this task

    .. note::
       The default setting for timeouts (a negative value) will suit
       most use cases. With this setting the behaviour will periodically check and
       issue a warning if the server can't be found. Actually aborting the setup can
       usually be left up to the behaviour tree manager.
    """
    def __init__(self,
                 name: str,
                 action_type: typing.Any,
                 action_name: str,
                 action_goal: typing.Any,
                 generate_feedback_message: typing.Callable[[typing.Any], str]=None,
                 wait_for_server_timeout_sec: float=-3.0,
                 repetitions: int=1,
                 ):
        super().__init__(
            name = name,
            action_type = action_type,
            action_name = action_name,
            action_goal= action_goal,
            generate_feedback_message=generate_feedback_message,
            wait_for_server_timeout_sec=wait_for_server_timeout_sec
        )
        self.repetitions = repetitions        