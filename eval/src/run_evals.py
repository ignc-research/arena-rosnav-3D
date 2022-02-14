import rospy
import rospkg
import yaml
import os, sys
from std_msgs.msg import Bool
import subprocess
import time
import signal
import argparse

use_recorder = "true"

def arguments():
    parser = argparse.ArgumentParser()

    parser.add_argument("-f", "--eval_file")

    return parser.parse_args()

args = arguments()
print(args.eval_file)

def read_evals_file():
    file_path = os.path.join(rospkg.RosPack().get_path("eval"), "src", f"{args.eval_file}.yaml")
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
arena_rosnav_3d_path = os.path.join(rospkg.RosPack().get_path("eval"), "..")

def build_startup_command(robot, world, scenario):
    return f"roslaunch arena_bringup start_arena_gazebo.launch use_recorder:={use_recorder} use_rviz:=false gui:=false trainings_environment:={trainings_env} network_type:={network_type} model:={robot} local_planner:=rosnav world:={world} scenario_file:={scenario}"

docker_command = f"{arena_rosnav_3d_path}/docker/drl_agent_node/start_docker.sh {arena_rosnav_3d_path} {launch_docker}"

if __name__ == "__main__":
    rospy.init_node("run_evals_node")

    for scenario in scenarios:
        robot = scenario["robot"]
        worlds = scenario["worlds"]

        for world in worlds:
            current_world = world["world"]
            scenario_file = world["scenario"]

            print("===============================================================")
            print(f"Starting scenario {scenario_file} in world {current_world}")

            startup_command = build_startup_command(robot, current_world, scenario_file)

            process = subprocess.Popen(startup_command, shell=True, preexec_fn=os.setsid)

            if launch_docker:
                time.sleep(2)
                print("Launching docker")
                docker_process = subprocess.Popen(docker_command, shell=True, preexec_fn=os.setsid)
            
            time.sleep(1)

            rospy.wait_for_message("/End_of_scenario", Bool)

            print("Killing processes")

            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            os.killpg(os.getpgid(docker_process.pid), signal.SIGINT)

            time.sleep(1)

            os.system("killall roslaunch")
            os.system("killall docker")
            
            docker_process.wait(10)
            print("Docker down")

            process.wait(10)
            print("Roslaunch down")
            
            time.sleep(10)

            print("===============================================================")
            print(f"Scenario {scenario_file} in world {current_world} finished")