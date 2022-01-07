#!/usr/bin/env python
from abc import abstractmethod

class BaseEncoder:
    @abstractmethod
    def __init__(self, agent_name: str, model_dir: str, hyperparams):
        raise NotImplementedError()
    
    @abstractmethod
    def _load_model(self, model_path: str):
        raise NotImplementedError()

    @abstractmethod
    def _load_vecnorm(self, vecnorm_path: str):
        raise NotImplementedError()

    @abstractmethod
    def get_observation(self, obs):
        raise NotImplementedError()

    @abstractmethod
    def get_action(self, action):
        raise NotImplementedError()