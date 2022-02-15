from typing import Tuple, Union
import rospy
import rospkg
import yaml
import os
from std_msgs.msg import Bool
import subprocess
import time
import signal
import argparse

__all__ = ["main", "evaluate_scenario"]


""" GLOBAL PARAMS """
ROSNAV_3D_BASEPATH = os.path.join(rospkg.RosPack().get_path("eval"), "..")
BUILD_STARTUP_CMD = (
    lambda use_recorder, trainings_env, network_type, robot, world, scenario: "roslaunch arena_bringup "
    f"start_arena_gazebo.launch use_recorder:={use_recorder} use_rviz:=false gui:=false trainings_environment"
    f"trainings_environment:={trainings_env} network_type:={network_type} model:={robot} "
    f"local_planner:=rosnav world:={world} scenario_file:={scenario}"
)
DOCKER_CMD = (
    lambda launch_docker: f"{ROSNAV_3D_BASEPATH}/docker/drl_agent_node/start_docker.sh {ROSNAV_3D_BASEPATH} {launch_docker}"
)


def read_evals_file(file_name: str) -> Union[Tuple[str, str, str, str], None]:
    file_path = os.path.join(
        rospkg.RosPack().get_path("eval"), "src", f"{file_name}.yaml"
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


def evaluate_scenario(
    scenario_file: str,
    current_world: str,
    trainings_env: str,
    network_type: str,
    robot: str,
    launch_docker: bool,
    use_recorder: bool,
    verbose: bool = False,
) -> None:
    """_summary_

    Args:
        scenario_file (str): _description_
        current_world (str): _description_
        use_recorder (bool): _description_
        trainings_env (str): _description_
        network_type (str): _description_
        robot (str): _description_
        launch_docker (bool): _description_
    """
    print("===============================================================")
    print(
        f"[{robot}] Starting scenario {scenario_file} in world {current_world}"
    )

    startup_command = BUILD_STARTUP_CMD(
        use_recorder,
        trainings_env,
        network_type,
        robot,
        current_world,
        scenario_file,
    )
    if verbose:
        startup_command += "> eval_log.txt"

    process = subprocess.Popen(
        startup_command, shell=True, preexec_fn=os.setsid
    )

    if launch_docker:
        time.sleep(2)
        print("Launching docker")
        docker_process = subprocess.Popen(
            DOCKER_CMD(launch_docker), shell=True, preexec_fn=os.setsid
        )

    time.sleep(1)

    rospy.init_node("run_evals_node", anonymous=True, disable_signals=True)
    rospy.wait_for_message("/End_of_scenario", Bool)

    os.killpg(os.getpgid(process.pid), signal.SIGINT)
    os.killpg(os.getpgid(docker_process.pid), signal.SIGINT)

    time.sleep(1)

    os.system("killall roslaunch")

    if launch_docker:
        os.system("killall docker")
        docker_process.wait(10)

    process.wait(10)

    time.sleep(10)

    print("===============================================================")
    print(
        f"[{robot}] Scenario {scenario_file} in world {current_world} finished"
    )


def main(args):
    use_recorder = args.use_recorder
    eval_file = args.eval_file
    verbose = args.verbose

    trainings_env, network_type, scenarios, launch_docker = read_evals_file(
        eval_file
    )

    for scenario in scenarios:
        robot = scenario["robot"]
        worlds = scenario["worlds"]

        for world in worlds:
            current_world = world["world"]
            scenario_file = world["scenario"]

            evaluate_scenario(
                scenario_file,
                current_world,
                trainings_env,
                network_type,
                robot,
                launch_docker,
                use_recorder,
                verbose,
            )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("-f", "--eval_file")
    parser.add_argument("-r", "--use_recorder", action="store_true")
    parser.add_argument("-v", "--verbose", action="store_true")

    main(parser.parse_args())
