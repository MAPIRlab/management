# Description
This pkg implements a "task manager" to handle over time a set of tasks the robot must carry out.  
The current implementation is based on **Behaviour Trees (BT)**, using the "py_trees" and "py_trees_ros" library (see http://docs.ros.org/en/kinetic/api/py_trees_ros/html/index.html).

In a nutshell, the bt_manager is the ROOT of the tree, and every task to be executed by the robot becomes no more than a branch (a sub-tree) of the bt_manager. 
The bt_manager is in charge of adding, executing and removing every task (branch) in the ROOT-tree according to its parameters (priority, permanence, etc.)

A "py_trees_ros_viewer" is used to easily visualize the current task-plan, as well as to indicate the task being executted.

## Tasks
A task is a definite piece of work assigned to the robot for its execution. 
It can be a simple "SAY_TEXT" (contining a string that will be forwarded to a text2speech node) or a more elaborated task like "GO_DOCK" (in charge of commanding the robot to go back to the docking station and recharge batteries).

All tasks share a common API with the following fields:  
  * **name**: The Task's Type. This "name" must correspond with an existing task type declared in this pkg as a BT
  * **priority**: [1-9]  0=low priority, 9=high priority
  * **permanence**: False (after 1 execution the task is removed from the tree), True (the task is never removed, so executed periodically)
  * **args[]** List of parameters own of each task

## Adding a new Task to the Tree
There are two ways of including a new task to the current plan:
1. Call the **/tree/add_new_task** service:  
Calling from Terminal: `ros2 service call task task_type parametres_as_a_dictionary`
e.g. Adding a "say_text" task: `ros2 service call /tree/add_new_task task_manager_interfaces/srv/AddTask "{task_name: "say", task_priority: 2, task_permanence: False, task_impact: "no", task_args: ["'cosa para decir'"]}"`  
e.g. Adding a "go_to_point" task: `ros2 service call /tree/add_new_task task_manager_interfaces/srv/AddTask "{task_name: "move", task_priority: 1, task_permanence: False, task_impact: "no", task_args: ["'cosa para decir'"]}"`  

2. Set a list of tasks to be executed on start-up
You can see an example [here](https://gitlab.com/mapir/mapir-ros-sources/blob/kinetic-dev/missions_pkg/launch/jgmonroy/jgmonroy_simbot_iro_initial_tasks.yaml)

## Creating a new Task Type
As explained above, the current implementation is based on **Behaviour Trees (BT)**, so every Task must follow the BT convention and be implemented as a Tree.  
In a nutshell, the bt_manager is the ROOT of the tree, and every task to be executed is no more than a branch (a sub-tree). 
The bt_manager is in charge of positioning every task (branch) in the ROOT-tree according to its priority, and removing it if the permanence param is set to False.

## Dependencies
sudo apt install ros-<rosdistro>-py-trees