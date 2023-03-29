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

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


# Check installed packages to offer sub-trees (tasks)
# If related pkgs are not installed, we wont allow instantiations of such tasks
try:
    # Google Speech SRV
    from speech.srv import say
    google_speech_node_found = True
except ImportError:
    google_speech_node_found = False

# =============================================================
#                           SAY(dialogue)
# =============================================================
def subtree_say(task_name, dialogue):
    """
    A subtree to implement a simple talk via service call
    """
    try:
        if google_speech_node_found:
            tasks_tree = py_trees.composites.Selector(name="Tasks_tree")
            goal_msg = Say.Goal()
            goal_msg.text_to_say = dialogue
            say = py_trees_ros.actions.ActionClient(
            name="SAY",
            action_type=Say,
            action_name="say",
            action_goal=goal_msg,  # noqa
            generate_feedback_message=lambda msg: "Speech"
            )
            tasks_tree.add_child(say)
            return tasks_tree
        else:
            print(console.red + "[bt_manager] Google Speech module not found!" + console.reset)
            print(console.red + "[bt_manager] Dialogue is: {}".format(str(dialogue))+ console.reset)
            return py_trees.behaviours.Success(task_name)

    except Exception as excp:
        print ("[bt_manager] " + str(excp) + ". Skipping task")


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