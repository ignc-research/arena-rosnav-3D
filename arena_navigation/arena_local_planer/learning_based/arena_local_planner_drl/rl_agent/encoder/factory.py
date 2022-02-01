from rl_agent.encoder import BaseEncoder


class EncoderFactory:
    registry = {}

    @classmethod
    def register(self, trainings_environment, network_type, robot):

        def wrapper(wrapped_class):
            name = trainings_environment + "_" + network_type + "_" + robot

            assert name not in self.registry, f"Encoder '{name}' already exists!"
            assert issubclass(wrapped_class, BaseEncoder), f"Wrapped class {wrapped_class.__name__} is not of type 'BaseEncoder'!"

            self.registry[name] = wrapped_class
        
        return wrapper

    @classmethod
    def instantiate(self, trainings_environemnt, network_type, robot):
        name = trainings_environemnt + "_" + network_type + "_" + robot
        
        assert name in self.registry, f"Encoder '{name}' is not registered!"

        return self.registry[name]
