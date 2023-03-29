import rclpy

import py_trees
import py_trees_ros.trees
import py_trees.console as console
import py_trees_ros_interfaces.action as py_trees_actions  # noqa
import py_trees_ros_interfaces.srv as py_trees_srvs  # noqa

from task_manager_interfaces.srv import AddTask, RemoveTask
from . import bt_task_lib

import std_msgs.msg as std_msgs
import uuid
import operator
import sys



# Class py_trees_ros.trees.BehaviourTree extends py_trees.trees.BehaviourTree
# Integrates a node: rclpy.node.Node
class DynamicApplicationTree(py_trees_ros.trees.BehaviourTree):
    """
    Wraps the ROS behaviour tree manager in a class that manages loading
    and unloading of jobs.
    """

    # ---------------------------------------------------------------------
    #                               INIT
    # ---------------------------------------------------------------------
    def __init__(self):
        """
        Create the core tree and add post tick handlers for post-execution
        management of the tree.
        """
        super().__init__(
            root=self.create_root(),
            unicode_tree_debug=True
        )

        # After each tick of the root tree, prune completed jobs
        self.add_post_tick_handler(
            self.prune_application_subtree_if_done
        )

    #============================
    # CREATE ROOT
    #============================
    def create_root(self) -> py_trees.behaviour.Behaviour:
        """
        Creates the root of the behaviour tree (parallel composite).
        Then, adds a Sequence for accesing data (Topics2BB) and sharing it througt the blackboard
        and another Sequence, to dynamically host the robot tasks.
        
        Returns:
            the root of the tree
        """
        root = py_trees.composites.Parallel(
            name="Root",
            policy=py_trees.common.ParallelPolicy.SuccessOnAll(
                synchronise=False
            ),
            children=None
        )

        # Bt default the parallel will handle two permanent tasks.
        # 1. Blackboard access (shared data among leafs) 
            # A sequence will progressively tick over each of its children 
            # so long as each child returns SUCCESS. If any child returns FAILURE or RUNNING 
            # the sequence will halt and the parent will adopt the result of this child.
        topics2bb = py_trees.composites.Sequence(
            name="Topics2BB",
            memory=False,
            children=None)
                
        # 2. Robot tasks (initially empty)
        tasks = py_trees.composites.Sequence(
            name="Tasks",
            memory=False,
            children=None)

        # Append to the root
        root.add_child(topics2bb)        
        root.add_child(tasks)
        return root
        
    #============================
    # SETUP
    #============================
    def setup(self, timeout: float):
        """
        Setup the tree and connect additional application management / status
        report subscribers and services, as well as to ROS2.

        Args:
            timeout: time (s) to wait (use common.Duration.INFINITE to block indefinitely)
        """
        # The setup will create the variable "node" as a rclpy.node.Node
        super().setup(timeout=timeout)

        # declare status srv (sever)
        self._report_service = self.node.create_service(
            srv_type = py_trees_srvs.StatusReport,
            srv_name = "~/report",
            callback = self.deliver_status_report,
            qos_profile = rclpy.qos.qos_profile_services_default
        )

        # declare AddTask srv (server)
        self.addTask = self.node.create_service(
            srv_type = AddTask,
            srv_name = "task_manager/add_task",
            callback = self.add_job,
            qos_profile = rclpy.qos.qos_profile_services_default
        )

        # declare RemoveTask srv (server)
        self.RemoveTask = self.node.create_service(
            srv_type = RemoveTask,
            srv_name = "task_manager/remove_task",
            callback = self.remove_job_by_id,
            qos_profile = rclpy.qos.qos_profile_services_default
        )

        # Init job identifiers (unique)
        self.job=0
    

    #============================
    # REMOVE_JOB_BY_ID
    #============================
    def remove_job_by_id(self, req, res):
        """
        Try to remove an existing task given its ID
        If ID = 'all', removes all tasks!
        :param req.task_id: The task id (string)
        :param req.info: Reason to remove the task (string)
        :return: ack (bool)
        """
        
        if req.task_id == "all":
            res.ack = self.remove_all_tasks(req.info)
        else:
            task_iter = 0
            while len(self.root.children[-1].children) > task_iter:
                job = self.root.children[-1].children[task_iter]
                if job.id.hex == req.task_id:
                    print(console.green + "[bt_manager] Task[{}]:{} killed by request".format(str(req.task_id), str(job.name)))
                    # remove job
                    for node in job.iterate():
                        node.shutdown()
                    self.prune_subtree(job.id)
                    res.ack = True
                    return res
                else:
                    task_iter += 1
            
            # Reaching this points mean the task id was not found!
            print(console.red + "[bt_manager] Task[{}]: cannot be removed by request. TaskID not found!".format(str(req.task_id)) + console.reset)
            res.ack = False        
            return res
    
    # ============================
    # REMOVE_ALL_JOBS
    # ============================
    def remove_all_tasks(self, info):
        """
        Removes all pending Tasks
        :return: Bool
        """
        print(console.red + "[bt_manager] Prunning all pending tasks. Reason: {}".format(str(info)) + console.reset)
         
        task_iter = 0
        while len(self.root.children[-1].children) > task_iter:
            job = self.root.children[-1].children[task_iter]
            # remove job
            for node in job.iterate():
                node.shutdown()
            self.prune_subtree(job.id)
        return True


    #============================
    # ADD_JOB
    #============================
    def add_job(self, req, res):
        """
        Incoming job callback. Add new job to the Tasks subtree
        :param req.task_name (string)
        :param req.task_type (string)
        :param rq.task_args (string[])
        :param req.task_priority (1/0)
        :param req.task_repetitions (int)
        :param req.task_impact (string)
        
        :return: 
            bool success            # wether the task was created or not
            uint8[16] task_id       # uuid (only on success)
            string error_msg        # msg (only on error)        
        """

        # Create job according to available list types        
        job = self.create_job(req)
        job_type = py_trees.utilities.get_fully_qualified_name(job)
        print(console.green + "Creating job of type {}".format(str(job_type)) + console.reset)
        print(console.green + "Creating job of type {} and id:{}".format(str(job_type),str(job.id)) + console.reset)
                
        if job.name == "invalid":
            # unable to create job
            res.success = False
            res.error_msg = job.feedback_message
        else:
            # before inserting, run its setup
            try:
                job.setup()
            except Exception as e:
                console.logerror(console.red + "failed to setup {} job, aborting [{}]".formatstr(job.name),(str(e)) + console.reset)
                #sys.exit(1)
            
            # insert job in "tasks" sequence
            if req.task_priority:
                self.root.children[-1].prepend_child(job)
            else:
                self.root.children[-1].add_child(job)

            print(console.green + "SUCCESS on job creation: type:{}, name:{}, id:{}".format(str(job_type),str(job.name),str(job.id)) + console.reset)
            
            # srv response
            res.success = True
            res.task_id = job.id.hex
        return res

    #============================
    # CREATE_JOB
    #============================
    def create_job(self, req) -> py_trees.behaviour.Behaviour:
        """
        Create the job subtree based on the incoming task_name specification.
        Args:            
            :param req.task_name (string)
            :param req.task_type (string)
            :param rq.task_args (string[])
            :param req.task_priority (1/0)
            :param req.task_repetitions (int)
            :param req.task_impact (string)        
        Returns:
        :class:`~py_trees.behaviour.Behaviour`: subtree 
        """

        # List of behaviours (jobs or task the robot can carry out)
        if req.task_type == "say":
            if len(req.task_args) > 0:
                dialogue = req.task_args[0]
                job = bt_task_lib.subtree_say(req.task_name, dialogue)
                return job
            else:
                # incorrect arguments
                job = py_trees.behaviours.Dummy()
                job.name = "invalid"
                job.feedback_message = "[bt_manager] Incorrect number of parameters for task [{}] - ignoring request".format(str(req.task_type)) 
                print(console.red + job.feedback_message + console.reset)
                return job
        else:
            # unknown task_type
            job = py_trees.behaviours.Dummy()
            job.name = "invalid"
            job.feedback_message = "[bt_manager] Failed to AddTask[{}] - task_type not found".format(str(req.task_name))
            print(console.red + job.feedback_message + console.reset)
            return job


    def deliver_status_report(
            self,
            unused_request: py_trees_srvs.StatusReport.Request,  # noqa
            response: py_trees_srvs.StatusReport.Response  # noqa
         ):
        """
        Prepare a status report for an external service client.

        Args:
            unused_request: empty request message
        """
        # last result value or none
        last_result = self.blackboard_exchange.blackboard.get(name="scan_result")
        if self.busy():
            response.report = "executing"
        elif self.root.tip().has_parent_with_name("Battery Emergency"):
            response.report = "battery [last result: {}]".format(last_result)
        else:
            response.report = "idle [last result: {}]".format(last_result)
        return response


    def prune_application_subtree_if_done(self, tree):
        """
        Check if a job is running and if it has finished. If so, prune the job subtree from the tree.
        Additionally, make a status report upon introspection of the tree.
        Args:
            tree (:class:`~py_trees.trees.BehaviourTree`): tree to investigate/manipulate.
        """
        if self.job != 0:
            tree.prune_subtree(self.job.id)
            self.job = 0
        # executing
        elif self.busy():
            job = self.priorities.children[1]
                # finished
            if job.status == py_trees.common.Status.SUCCESS or job.status == py_trees.common.Status.FAILURE:
                self.node.get_logger().info("{0}: finished [{1}]".format(job.name, job.status))
                for node in job.iterate():
                    node.shutdown()
                tree.prune_subtree(job.id)

    def busy(self):
        """
        Check if a job subtree exists and is running. Only one job is permitted at
        a time, so it is sufficient to just check that the priority task selector
        is of length three (note: there is always emergency and idle tasks
        alongside the active job). When the job is not active, it is
        pruned from the tree, leaving just two prioritised tasks (emergency and idle).

        Returns:
            :obj:`bool`: whether it is busy with a job subtree or not
        """
        return len(self.priorities.children) >= 3

    @property
    def priorities(self) -> py_trees.composites.Selector:
        """
        Returns the composite (:class:`~py_trees.composites.Selector`) that is
        home to the prioritised list of tasks.
        """
        return self.root.children[-1]



#===============================0
#               MAIN
#===============================0
def main(args=None):
    """
    Entry point for the TaskManager:bt_manager node
    """
    # Init rclpy
    rclpy.init(args=args)

    # Create a BT (initially empty)
    tree = DynamicApplicationTree()

    # Setup the BT (offers services over ROS2)
    try:
        tree.setup(timeout=15)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # not a warning, nor error, usually a user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.shutdown()
        sys.exit(1)

    # Configure the tree to run "forever"
    tree.tick_tock(period_ms=1000.0)
    
    # Loop (forever)
    try:
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        rclpy.shutdown()
