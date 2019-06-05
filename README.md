# grasp_set_skill
## [DEPRECATED] See [Grasp Estimation](https://github.com/ItzMeJP/grasp_estimation)

ROS skill package to read the teached grasp candidates pose associated with a specific object, and convert it in the ROS structure.
Each grasp candidate is a pose related to origin base frame of the object with an associated gripper type.
The gripper types are: 
- 0 - SUCTION
- 1 - ROBOTIQ_2F_C40666

The run.launch in grasp_set_skill_server/launch is resposible to load the parameters and run the node. The parameters are load using at least two .yaml files: the config.yaml and the $(object_name)\_candidates.yaml. The config.yaml has general parameters and the $(object_name)\_candidates.yaml is where all the objects grasp candidates are located (one file per object is needed to describe all the associated grasps points). 

## Installation
- Download the package inside a worskpace directory.

> cd $(catkin_ws)/src

> git clone https://github.com/ItzMeJP/grasp_set.git

- Compile the grasp_set_skill_msgs.

> catkin build grasp_set_skill_msgs

- Compile the others packages.

> catkin build grasp_set_skill_server

> catkin build grasp_set_skill_client

## How to use

Run the launch:
> roslaunch grasp_set_skill_server run.launch

Request using goal action:

> rostopic pub /grasp_set_skill/goal grasp_set_skill_msgs/GraspSetActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  object_name: '$(name_of_object)'" 

See the result using result action:

> rostopic pub /grasp_set_skill/result

### Parameters
- **ros_verbosity_level**: set the ros verbosity level (DEBUG, INFO, WARN, ERROR, FATAL)
- **number_of_candidates**: number of grasp candidates that each object has (all the objects need to have the same number of grasp candidates).
- **object_tf_origin**: name of the object origin (all grasps candidates will be related to this frame).
- **candidates_grasps_tf_base_name:** just a base name, the grasps candidates will have the name base_name_($NUMBER).


