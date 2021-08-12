import numpy as np
import os
import yaml
import json
from PedsimAgent import *
from FlatlandModel import *
from HelperFunctions import *

class ArenaScenario():
    def __init__(self):
        self.pedsimAgents = []  # list of PedsimAgent objects
        self.interactiveObstacles = []  # list of InteractiveObstacle messages
        self.staticObstacles = []  # list of FlatlandObjects
        self.robotPosition = np.zeros(2)  # starting position of robot
        self.robotGoal = np.zeros(2)  # robot goal
        self.mapPath = ""  # path to map file

        self.path = ""  # path to file associated with this scenario

    def toDict(self):
        d = {}

        d["pedsim_agents"] = [a.toDict() for a in self.pedsimAgents]
        d["static_obstacles"] = [o.toDict() for o in self.staticObstacles]
        # d["interactive_obstacles"] = TODO...
        d["robot_position"] = [float(value) for value in self.robotPosition]
        d["robot_goal"] = [float(value) for value in self.robotGoal]
        d["map_path"] = self.mapPath
        d["format"] = "arena-tools"

        return d

    @staticmethod
    def fromDict(d : dict):
        scenario = ArenaScenario()
        scenario.loadFromDict(d)
        return scenario

    def loadFromDict(self, d: dict):
        self.pedsimAgents = [PedsimAgent.fromDict(a) for a in d["pedsim_agents"]]
        self.staticObstacles = [FlatlandObject.fromDict(o) for o in d["static_obstacles"]]
        # self.interactiveObstacles = ...TODO
        self.robotPosition = np.array([d["robot_position"][0], d["robot_position"][1]])
        self.robotGoal = np.array([d["robot_goal"][0], d["robot_goal"][1]])
        self.mapPath = get_current_user_path(d["map_path"])

    def saveToFile(self, path_in: str = "") -> bool:
        '''
        Save Scenario in file.
        - path_in: path to save file
        - format: format of save file, can be "json" or "yaml"
        '''
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
                raise Exception("wrong format. file needs to have 'json' or 'yaml' file ending.")

        return True

    def loadFromFile(self, path_in: str):
        if os.path.exists(path_in):
            _, file_extension = os.path.splitext(path_in)
            with open(path_in, "r") as f:
                data = None
                if file_extension == ".json":
                    data = json.load(f)
                elif file_extension == ".yaml":
                    data = yaml.safe_load(f)
                else:
                    raise Exception("wrong format. file needs to have 'json' or 'yaml' file ending.")

                self.loadFromDict(data)
                self.path = path_in

        else:
            raise Exception(f"file '{path_in}' does not exist")
