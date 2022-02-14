import rospy
import rospkg
import yaml
import os, sys
from std_msgs.msg import Bool
import subprocess
import time
import signal
import argparse


def read_evals_file(eval_file: str):
    file_path = os.path.join(
        rospkg.RosPack().get_path("task_generator"),
        "scripts",
        f"{eval_file}.yaml",
    )
    with open(file_path, "r") as file:
        evals = yaml.safe_load(file)

        should_launch_docker = evals["shouldLaunchDocker"]

        if should_launch_docker:
            dockerName = evals["dockerName"]

        return (
            evals["trainings_env"],
            evals["network_type"],
            evals["scenarios"],
            dockerName if should_launch_docker else None,
        )


def main(args):
    trainings_env, network_type, scenarios, launch_docker = read_evals_file(
        eval_file=args.eval_file
    )
    arena_rosnav_3d_path = os.path.join(
        rospkg.RosPack().get_path("task_generator"), ".."
    )

    def build_startup_command(robot, world, scenario):
        return f"roslaunch arena_bringup start_arena_gazebo.launch use_recorder:={args.use_recorder} use_rviz:=false gui:=false trainings_environment:={trainings_env} network_type:={network_type} model:={robot} local_planner:=rosnav world:={world} scenario_file:={scenario}"

    docker_command = f"{arena_rosnav_3d_path}/docker/drl_agent_node/start_docker.sh {arena_rosnav_3d_path} {launch_docker}"

    try:
        for scenario in scenarios:
            robot = scenario["robot"]
            worlds = scenario["worlds"]

            for world in worlds:
                current_world = world["world"]
                scenario_file = world["scenario"]

                print(
                    f"Starting scenario {scenario_file} in world {current_world}"
                )

                startup_command = build_startup_command(
                    robot, current_world, scenario_file
                )

                process = subprocess.Popen(
                    startup_command, shell=True, preexec_fn=os.setsid
                )

                if launch_docker:
                    rospy.sleep(2)
                    docker_process = subprocess.Popen(
                        docker_command, shell=True, preexec_fn=os.setsid
                    )

                time.sleep(5)
                rospy.init_node("run_evals_node")

                rospy.wait_for_message("/End_of_scenario", Bool)

                os.killpg(os.getpgid(process.pid), signal.SIGINT)
                if launch_docker:
                    os.killpg(os.getpgid(docker_process.pid), signal.SIGINT)

                time.sleep(1)

                os.system("killall roslaunch")

                process.wait()
                if launch_docker:
                    docker_process.wait()

                time.sleep(10)

                print(
                    f"Scenario {scenario_file} in world {current_world} finished"
                )
    except KeyboardInterrupt:
        os.system("killall roslaunch")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("-f", "--eval_file")
    parser.add_argument("-r", "--use_recorder", action="store_true")

    main(parser.parse_args())
