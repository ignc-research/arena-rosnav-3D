# Introduction
In this *.md we will provide an overview of the most important scripts and packages to work on development for the DRL Pipeline.

# Run script
[Here](https://github.com/ignc-research/arena-rosnav-3D/blob/eval_noetic/arena_navigation/arena_local_planer/learning_based/arena_local_planner_drl/scripts/deployment/drl_agent_node.py) you can find the script or node that is responsible for starting the DRL agent. The core feature for modularity, are the `encoders`- to better understand those and which arguments you have to additionally pass when starting the script, we reffer to the corresponding [TODO: Encoder Readme](). 
If you want to include your own, trained model, you will have to provide a respective encoder and action_space definition (See ##Action space). You can select a trained model by providing its name either at the end of the script itself (`AGENTNAME`), or in the [rosnav launch file](https://github.com/ignc-research/arena-rosnav-3D/blob/eval_noetic/arena_bringup/launch/sublaunch_testing/move_base/move_base_rosnav.launch). All agents can be found [here](https://github.com/ignc-research/arena-rosnav-3D/tree/eval_noetic/arena_navigation/arena_local_planer/learning_based/arena_local_planner_drl/agents).

> ___NOTE___: Running the _drl_agent_node_ from the launch file is not possible for all models, since some models are trained with python3.6. For this reason it might be necessary to seperately launch the `arena_start_gazebo.launch` with `local_planer:=rosnav` and use `python drl_agent_node.py` in a new terminal with a venv with python3.6. 
For all models, that are trainied with python3.8, this is not necessary, and the `drl_agent_node` can be simple started with the `arena_start_gazebo.launch` file.

To use your pretrained models from a different project/ repo, the necessary AGENT files have to be provided (_best_model.zip_, _hyperparameters.json_, _hyperparameters.json.lock_). Additionally, you have to provide the respective encoders and action_space definitions. The latter will be briefly discussed in the following.

# DRL-Model specifications
Since _arena-rosnav_ enables the use of different robots, those robots can have differnt properties! Not only do they have different footprints, but they can also have different observation and action spaces. For each different robot model a new DRL model has to be trained with the respective footprint, action, and observaton space.

## Action space
The _drl_agent_node.py_ script inherits from the [_base_agent_wrapper_](https://github.com/ignc-research/arena-rosnav-3D/blob/eval_noetic/arena_navigation/arena_local_planer/learning_based/arena_local_planner_drl/rl_agent/base_agent_wrapper.py) class. Here we have the `setup_action_space()` function where the action space is defined and the `publish_action()` function, where the actions are transformed into a message for the _cmd_vel_ - topic. Nevertheless, defining the action_space is parameterised and automated.

When adding a new robot, or changing the action space of an existing robot, you only have to add/ adjust the respective `default_settings_{robot_name}.yaml` file in [this](https://github.com/ignc-research/arena-rosnav-3D/tree/main/arena_navigation/arena_local_planer/learning_based/arena_local_planner_drl/configs) directory. Please, compare the formatting to already existing files and be aware of the differences between holonomic and non-holonomic robots.

## Observation space
To receive the observations, the _drl_agent_node_ calls the function `get_observations()` from the [observation_collector](https://github.com/ignc-research/arena-rosnav-3D/blob/eval_noetic/arena_navigation/arena_local_planer/learning_based/arena_local_planner_drl/rl_agent/utils/observation_collector.py) class. The class subscribes to the _scan_, _odom_, _subgoal_, and _global_plan_ topic and returns several observations. On the one hand, it returns merged observations, but also (more importantly) all observations in form of a dictionary, which can be used to construct the correct input for your own model/ network. See [TODO: Encoder Readme]().
If necessary, the topics can be adjusted accordingly.


| Name                  | content         |  description                                  |
| :---:                 | :---:           | :---:                                         |
| *laser_scan*          | float[]         | all scan ranges from the _scan_ topic         |
| *goal_in_robot_frame* | [float, float]  | [distance, angle] from robot to subgoal       |
| *global_plan*         | np.array        | global path provided by globalPlan topic      |
| *robot_pose*          | Pose2D          | x and y position of robot based on odometry   |
| *subgoal*             | Pose2D          | x and y position of subgoal                   |
| *robot_vel*           | Twist           | Twist message containing velocity of robot    |

