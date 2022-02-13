import rospy
import rospkg
import yaml
import os
from std_msgs.msg import Bool
import subprocess
import time

use_recorder = "false"

def read_evals_file():
    file_path = os.path.join(rospkg.RosPack().get_path("task_generator"), "scripts", "evals.yaml")
    with open(file_path, "r") as file:
        evals = yaml.safe_load(file)

        should_launch_docker = evals["shouldLaunchDocker"]
        
        if should_launch_docker:
            dockerName = evals["dockerName"]
        
        return (
            evals["trainings_env"], 
            evals["network_type"], 
            evals["scenarios"], 
            dockerName if should_launch_docker else None
        )

trainings_env, network_type, scenarios, launch_docker = read_evals_file()
arena_rosnav_3d_path = os.path.join(rospkg.RosPack().get_path("task_generator"), "..")

def build_startup_command(robot, world, scenario):
    return f"gnome-terminal -- /bin/sh -c \"roslaunch arena_bringup start_arena_gazebo.launch use_recorder:={use_recorder} gui:=false trainings_environment:={trainings_env} network_type:={network_type} model:={robot} local_planner:=rosnav world:={world} scenario_file:={scenario}\""

docker_command = f"{arena_rosnav_3d_path}/docker/drl_agent_node/start_docker.sh {arena_rosnav_3d_path} {launch_docker}"

if __name__ == "__main__":

    for scenario in scenarios:
        robot = scenario["robot"]
        worlds = scenario["worlds"]

        for world in worlds:
            current_world = world["world"]
            scenario_file = world["scenario"]

            print(f"Starting scenario {scenario_file} in world {current_world}")

            startup_command = build_startup_command(robot, current_world, scenario_file)

            process = subprocess.Popen(startup_command, shell=True)

            if launch_docker:
                rospy.sleep(2)
                docker_process = subprocess.Popen(docker_command, shell=True)
            
            time.sleep(1)
            rospy.init_node("run_evals_node")

            rospy.wait_for_message("/End_of_scenario", Bool)

            process.kill()
            docker_process.kill()

            print(f"Scenario {scenario_file} in world {current_world} finished")