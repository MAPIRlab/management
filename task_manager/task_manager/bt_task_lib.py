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

import std_msgs
import diagnostic_msgs.msg
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


# Check installed packages to offer sub-trees (tasks)
# If related pkgs are not installed, we wont allow instantiations of such tasks

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
        publisher = PublisherTask(
            name = req.task_name,
            topic_name = "/ros2mqtt",    # publish over MQTT
            topic_type = diagnostic_msgs.msg.KeyValue,
            msg = say_msg,
            qos_profile = 1,
            repetitions = req.task_repetitions
        )
        return publisher

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
    A subtree to implement an action client for navigation    
    """
    try:
        # Create pose msg       
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

        # Create Action Goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = wp

        # Create Task Action Client
        move = py_trees_ros.actions.ActionClient(
            name = "goto_to_pose",
            action_type = NavigateToPose,
            action_name = "navigate_to_pose", # navigation action server of the /bt_navigator?
            action_goal = goal_msg,
            generate_feedback_message = lambda msg: "{}".format(msg.feedback.distance_remaining)
        )        
        return move
    
    except Exception as excp:
        job = py_trees.behaviours.Dummy()
        job.name = "invalid"
        job.feedback_message = "[subtree_goto_pose] Exception creating job: " + str(excp) + ". Skipping request."
        print(console.red + job.feedback_message + console.reset)
        return job

def subtree_patrol():

    tasks_tree = py_trees.composites.Sequence(name="Tasks_tree")

    goal_msg = Undock.Goal()
    goal_msg.undock = True
    undock = py_trees_ros.actions.ActionClient(
        name="Undock",
        action_type=Undock,
        action_name="undock",
        action_goal=goal_msg,  # noqa
        generate_feedback_message=lambda msg: "Undock"
    )
           
    pgoal_msg = Patrol.Goal()
    pgoal_msg.patrol = True
    patrol = py_trees_ros.actions.ActionClient(
        name="Patrol",
        action_type=Patrol,
        action_name="patrol",
        action_goal=pgoal_msg,  # noqa
        generate_feedback_message=lambda msg: "Patrolling"
    )
    
    tasks_tree.add_children([undock,patrol])
    return tasks_tree

def subtree_undock():

    tasks_tree = py_trees.composites.Selector(name="Tasks_tree")
    goal_msg = Undock.Goal()
    goal_msg.undock = True
    undock = py_trees_ros.actions.ActionClient(
        name="Undock",
        action_type=Undock,
        action_name="undock",
        action_goal=goal_msg,  # noqa
        generate_feedback_message=lambda msg: "Undock"
    )
           
    tasks_tree.add_child(undock)
    return tasks_tree


# =========================================================================
#               MAPIR TASKS (extend py_trees_ros)
# =========================================================================
class PublisherTask(py_trees.behaviour.Behaviour):
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
                 repetitions: int
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

