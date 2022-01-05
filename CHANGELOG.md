Reyk Carstens 05.01.2022

- Refactoring base_agent_wrapper -> Removed unused and useless code, better comments | Retuning values from function instead of direct assign for better overview
- Added directory for virtual environments in arena_local_planner_drl for switching between python versions more easily
- created subdirectories in agents folder for different trainig environments
- Added first navrep models
- Created Encoder base class and rosnav and navrep encoder -> Serves as converter between desired model in- and outputs and rosnav topics | Is easily adaptable and extendable to add new trainig environments and models
- Adapted arena_local_planner_drl setup.py to include encoders
- Added CHANGELOG for better comprehend what is happening to the codebase and as first quality of life update
