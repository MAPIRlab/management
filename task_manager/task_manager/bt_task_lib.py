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
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


# Check installed packages to offer sub-trees (tasks)
# If related pkgs are not installed, we wont allow instantiations of such tasks

# =============================================================
#                           SAY(dialogue)
# =============================================================
def subtree_say(task_name, dialogue)-> py_trees.behaviour.Behaviour:
    """
    A subtree to implement a simple talk
    By default just writes the msg to the "/say" topic
    """
    try:
        # Create msg to publish
        say_msg = std_msgs.msg.String()
        say_msg.data = dialogue

        # Create publisher
        publisher = simplePublisher(
            name = task_name,
            topic_name = "/say",    # publish over /say (string) ROS2 topic
            topic_type = std_msgs.msg.String,
            qos_profile = 1,
            msg = say_msg
        )
        return publisher

    except Exception as excp:
        job = py_trees.behaviours.Dummy()
        job.name = "invalid"
        job.feedback_message = "[subtree_say] Exception creating job: " + str(excp) + ". Skipping request."
        print(console.red + job.feedback_message + console.reset)
        return job


def subtree_move():
    tasks_tree = py_trees.composites.Selector(name="Tasks_tree")
    wp = PoseStamped()
    wp.header.frame_id = "map"
    wp.header.stamp = Node.Clock().now().to_msg()
    wp.pose.position.x = 2.3
    wp.pose.position.y = 1.9
    wp.pose.position.z = 0.0
    wp.pose.orientation.x = 0.0
    wp.pose.orientation.y = 0.0
    wp.pose.orientation.z = 0.0
    wp.pose.orientation.w = 0.0
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose = wp
    move = py_trees_ros.actions.ActionClient(
        name="navigate_to_pose",
        action_type=NavigateToPose,
        action_name="navigate_to_pose",
        action_goal=goal_msg,  # noqa
        generate_feedback_message=lambda msg: "{}".format(msg.feedback.distance_remaining)
    )
    tasks_tree.add_child(move)
    return tasks_tree


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
#               MAPIR BEHAVIOURS (extend py_trees_ros)
# =========================================================================
class simplePublisher(py_trees.behaviour.Behaviour):
    """
    This behaviour just publish an incoming msg to the specified topic
    This is a non-blocking behaviour -always returning:`~py_trees.common.Status.SUCCESS`.

    Args:
        name: name of the behaviour
        topic_name: name of the topic to connect to on ROS2
        topic_type: class of the message type (e.g. :obj:`std_msgs.msg.String`)
        qos_profile: qos profile for the subscriber
        msg: content to be published
    """
    def __init__(self,
                 name: str,
                 topic_name: str,
                 topic_type: typing.Any,
                 qos_profile: rclpy.qos.QoSProfile,
                 msg: typing.Any
                 ):
        super().__init__(name=name)
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.qos_profile = qos_profile
        self.msg_to_publish = msg
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

