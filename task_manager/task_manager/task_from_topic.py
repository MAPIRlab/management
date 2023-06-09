import rclpy
from rclpy.node import Node
from task_manager_interfaces.srv import AddTask, RemoveTask

import std_msgs.msg as std_msgs
import diagnostic_msgs.msg
import sys
import json



# Class FromTopic
class FromTopic(Node):
    """
    Subscribes to a diagnostic_msgs type topic and parses its contets to create calls to TaskManager srv
    """

    # ---------------------------------------------------------------------
    #                               INIT
    # ---------------------------------------------------------------------
    def __init__(self):
        super().__init__('task_from_topic')
        
        # subscribe to mqtt2ros topic
        self.subscription = self.create_subscription(
            diagnostic_msgs.msg.KeyValue,
            'mqtt2ros',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # AddTask srv client (task_manager)
        self.srv_cli = self.create_client(AddTask, "task_manager/add_task")
        while not self.srv_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('task_manager/add_task service not available, waiting ...')
        self.srv_req = AddTask.Request()

        # Keep track of service calls (not allow inside callbacks)
        self.srv_futures = []


    def listener_callback(self, msg):
        # mqtt2ros is a shared topic
        # Do only attend msgs with key "tasks"
        if msg.key == "tasks":
            self.get_logger().info("I heard [" + str(msg.key)  + "]: " + msg.value)

            # Parse JSON
            # AddTask example: value = '{ "task_name":"hablar", "task_type":"say", "task_args":["","",""], "task_priority": false, "task_repetitions": 1}'
            # RemoveTask example: value = '{ "task_id":"lkh1lkjh2olilhmhio92343k4", "info":"cancelacion por usuario"}'
            try:                
                # parse msg.value as a Python dictionary:
                d = json.loads(msg.value)

                # Check that task_type is set
                if "task_type" in d.keys():
                    
                    # Fill srv call 
                    self.srv_req.task_type = d["task_type"]

                    # name
                    if "task_name" in d.keys():
                        self.srv_req.task_name = d["task_name"]
                    else:
                        self.srv_req.task_name = d["task_type"]
                    
                    # args
                    if "task_args" in d.keys():
                        self.srv_req.task_args = d["task_args"]
                    else:
                        self.srv_req.task_args= []  # no arguments!
                    
                    # priority
                    if "task_priority" in d.keys():
                        self.srv_req.task_priority = d["task_priority"]
                    else:
                        self.srv_req.task_priority = False
                    
                    # repetitions
                    if "task_repetitions" in d.keys():
                        self.srv_req.task_repetitions = d["task_repetitions"]
                    else:
                        self.srv_req.task_repetitions = 1
                    
                    # srv call is ready. We do no call it inside this callback
                    # Instead we keep track to call from main
                    self.srv_futures.append(self.srv_cli.call_async(self.srv_req))
                    
                    # Call AddTask srv
                    # self.future = self.srv_cli.call_async(self.srv_req)
                    # self.get_logger().info("Called TaskManager addTask srv. Waiting response...")
                    # cant wait for response inside a callback!
                    #rclpy.spin_until_future_complete(self, self.future)
                    #if self.future.done():
                    #    self.get_logger().info("AddTask service result: " + str(self.future.result()))
    
                else:
                    raise Exception("task_type not defined")    
                

            except Exception as excp:
                feedback_message = "Exception parsing tasks (json format invalid?): " + str(excp) + ". Skipping request."
                self.get_logger().info(feedback_message)


    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            incomplete_futures = []
            for f in self.srv_futures:
                if f.done():
                    res = f.result()
                    self.get_logger().info("Received service result: {}".format(res))
                else:
                    incomplete_futures.append(f)
            self.srv_futures = incomplete_futures
            
#===============================0
#               MAIN
#===============================0
def main(args=None):
    """
    Entry point for the TaskManager:task_from_topic node
    """
    # Init rclpy
    rclpy.init(args=args)

    # Create node 
    parser = FromTopic()
        
    # Loop
    parser.spin()
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    parser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()