# Introduction
In this *.md we will provide an overview of the most important scripts and packages to work on development for the DRL Pipeine.

# Run script
[Here](https://github.com/ignc-research/arena-rosnav-3D/blob/eval_noetic/arena_navigation/arena_local_planer/learning_based/arena_local_planner_drl/scripts/deployment/drl_agent_node.py) you can find the script or node that is responsible for starting the DRL agent. You can select a trained model by providing its name either at the end of the script itself (`AGENTNAME`), or in the [rosnav launch file](https://github.com/ignc-research/arena-rosnav-3D/blob/eval_noetic/arena_bringup/launch/sublaunch_testing/move_base/move_base_rosnav.launch). All agents can be found [here](https://github.com/ignc-research/arena-rosnav-3D/tree/eval_noetic/arena_navigation/arena_local_planer/learning_based/arena_local_planner_drl/agents), *but* not all are compatible with all robot types!

> ___NOTE___: Running the _drl_agent_node_ from the launch file is not yet fully supported, since most models are trained with python3.6. For this reason it is necessary to regularly launch the `arena_start_gazebo.launch` with `local_planer:=rosnav` and use `python drl_agent_node.py` in a new terminal with a venv with python3.6 and located in the respective directory. 
For future models, that are trainied with python3.8, this will not be necessary.

To use are pretrained model from a different project/ repo, the necessary AGENT files have to be provided (_best_model.zip_, _hyperparameters.json_, _hyperparameters.json.lock_). Additionally, it might be necessary to provide wrappers for action and observation space.

# DRL-Model specifications
Since _arena-rosnav_ enables the use of different robots, those robots can have differnt properties! Not only do they have different footprints, but they can also have different observation and action spaces. For each different robot model a new DRL model has to be trained with the respective footprint, action, and observaton space.

## Action space
The _drl_agent_node.py_ script inherits from the [_base_agent_wrapper_](https://github.com/ignc-research/arena-rosnav-3D/blob/eval_noetic/arena_navigation/arena_local_planer/learning_based/arena_local_planner_drl/rl_agent/base_agent_wrapper.py) class. Here we have the `setup_action_space()` function where the action space is defined and the `publish_action()` function, where the actions are transformed into a message for the _cmd_vel_ - topic.
> __TODO__: Parameterise action space depending on robot.

## Observation space
To receive the observations, the _drl_agent_node_ calls the function `get_observations()` from the [observation_collector](https://github.com/ignc-research/arena-rosnav-3D/blob/eval_noetic/arena_navigation/arena_local_planer/learning_based/arena_local_planner_drl/rl_agent/utils/observation_collector.py) class. In the `__init()__` of this class the observation space is defined and can be adjusted accordingly.
> __TODO__: Parameterise observation space depending on robot.

