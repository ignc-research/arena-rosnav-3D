import numpy as np
import os, rospkg
import yaml
import json
from .PedsimAgent import *
from .FlatlandModel import *
from .HelperFunctions import *


class ArenaScenario:
    def __init__(self):
        self.pedsimAgents = []  # list of PedsimAgent objects
        self.interactiveObstacles = []  # list of InteractiveObstacle messages
        self.staticObstacles = []  # list of FlatlandObjects
        self.robotPosition = np.zeros(2)  # starting position of robot
        self.robotGoal = np.zeros(2)  # robot goal
        self.mapPath = ""  # path to map file
        self.resets = 0
        self.path = ""  # path to file associated with this scenario

    def toDict(self):
        d = {}

        d["pedsim_agents"] = [a.toDict() for a in self.pedsimAgents]
        d["static_obstacles"] = [o.toDict() for o in self.staticObstacles]
        # d["interactive_obstacles"] = TODO...
        d["robot_position"] = [float(value) for value in self.robotPosition]
        d["robot_goal"] = [float(value) for value in self.robotGoal]
        d["resets"] = self.resets
        d["map_path"] = self.mapPath
        d["format"] = "arena-tools"

        return d

    @staticmethod
    def fromDict(d):
        # type: (dict) -> None
        scenario = ArenaScenario()
        scenario.loadFromDict(d)
        return scenario

    def loadFromDict(self, d):
        # type: (dict) -> None
        self.pedsimAgents = [PedsimAgent.fromDict(a) for a in d["pedsim_agents"]]
        self.staticObstacles = [
            FlatlandObject.fromDict(o) for o in d["static_obstacles"]
        ]
        # self.interactiveObstacles = ...TODO
        self.robotPosition = np.array([d["robot_position"][0], d["robot_position"][1]])
        self.robotGoal = np.array([d["robot_goal"][0], d["robot_goal"][1]])
        self.mapPath = get_current_user_path(d["map_path"])
        if ("resets") in d.keys():
            self.resets = d["resets"]
        else:
            self.resets = 0

    def saveToFile(self, path_in=""):
        # type: (str) -> bool
        """
        Save Scenario in file.
        - path_in: path to save file
        - format: format of save file, can be "json" or "yaml"
        """
        if os.path.exists(path_in):
            self.path = path_in

        if self.path == "":
            return False

        _, file_extension = os.path.splitext(self.path)
        with open(self.path, "w") as file:
            data = self.toDict()
            if file_extension == ".json":
                json.dump(data, file, indent=4)
            elif file_extension == ".yaml":
                yaml.dump(data, file, default_flow_style=None)
            else:
                raise Exception(
                    "wrong format. file needs to have 'json' or 'yaml' file ending."
                )

        return True

    def loadFromFile(self, path_in):
        # type: (str) -> str
        if os.path.exists(path_in):
            _, file_extension = os.path.splitext(path_in)
            with open(path_in, "r") as f:
                data = None
                if file_extension == ".json":
                    data = json.load(f)
                elif file_extension == ".yaml":
                    data = yaml.safe_load(f)
                else:
                    raise Exception(
                        "wrong format. file needs to have 'json' or 'yaml' file ending."
                    )

                self.loadFromDict(data)
                self.path = path_in

        else:
            raise Exception("file ", path_in, " does not exist")

    def createSimplePed(self, ids, s_pos, w_pos, speed=.3):
        # type: (list, list, list, float) -> None

        # creates pedsim-agents
        peds = []
        for id, spos, wpos in zip(ids, s_pos, w_pos):
            peds.append(
                {
                    "name": "Pedestrian",
                    "id": id,
                    "pos": [*spos],
                    "type": "adult",
                    "yaml_file": "arena-rosnav-3D/simulator_setup/dynamic_obstacles/person_two_legged.model.yaml",
                    "number_of_peds": 1,
                    "vmax": speed,
                    "start_up_mode": "default",
                    "wait_time": 0.0,
                    "trigger_zone_radius": 0.0,
                    "chatting_probability": 0.01,
                    "tell_story_probability": 0,
                    "group_talking_probability": 0.01,
                    "talking_and_walking_probability": 0.01,
                    "requesting_service_probability": 0.01,
                    "requesting_guide_probability": 0.01,
                    "requesting_follower_probability": 0.01,
                    "max_talking_distance": 5,
                    "max_servicing_radius": 5,
                    "talking_base_time": 10,
                    "tell_story_base_time": 0,
                    "group_talking_base_time": 10,
                    "talking_and_walking_base_time": 6,
                    "receiving_service_base_time": 20,
                    "requesting_service_base_time": 30,
                    "force_factor_desired": 1,
                    "force_factor_obstacle": 1,
                    "force_factor_social": 5,
                    "force_factor_robot": 1,
                    "waypoints": [[*spos], [*wpos]],
                    "waypoint_mode": 0,
                }
            )

        # loading the an empty pedsim-scenario file and inserting peds
        path = (
            rospkg.RosPack().get_path("simulator_setup")
            + "/scenarios/utils/empty_ped_scenario.json"
        )

        if os.path.exists(path):
            _, file_extension = os.path.splitext(path)
            with open(path, "r") as f:
                data = json.load(f)

        data["pedsim_agents"] = peds
        self.loadFromDict(data)
