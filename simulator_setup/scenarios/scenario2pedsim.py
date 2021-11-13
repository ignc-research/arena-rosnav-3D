import json
import yaml
import argparse

def parsing(): # define what flags user can/must give as terminal input
    parser = argparse.ArgumentParser(description='Create pedsim scenario from regular scenario.json') # create parser object
    parser.add_argument('path_to_json', type=str, action='store',
                        help='path to JSON file')
    parser.add_argument('path_to_store_pedsim', type=str, action='store',
                        help='directory to store pedsim scenario')               


    # parser.add_argument('--quantity', action='store', nargs='?', default='obs', choices=['obs','vel'],
    #                     help='plot evaluation graphs over obstacle number (obs) or velocity (vel)') # store over which quantity metrics should be evaluated, default = obs                        
    # parser.add_argument('--allplot_quantity', action='store', nargs='?', default='none', choices=['obs','vel'],
    #                     help='plot all in one plot over obstacle number (obs) or velocity (vel)') # store over which quantity metrics should be evaluated, default = none                        
    # parser.add_argument('--latex', action='store_true',
    #                     help='flag: write dataframe as latex table into a txt file') # store TRUE if flag given, else FALSE per default, optional                        
    # parser.add_argument('--legendsoff', action='store_false',
    #                     help='flag: print graphs without legends') # store False if flag given, else True per default, optional
    # parser.add_argument('--show', action='store_true',
    #                     help='flag:show plots via plt.show()') # store TRUE if flag given, else FALSE per default, optional
    # parser.add_argument('--csv', action='store_true',
    #                     help='flag:export dfs to csv') # store TRUE if flag given, else FALSE per default, optional                        
    # parser.add_argument('--withclassic', action='store_true',
    #                     help='flag:plot classic planners into metrics plots') # store TRUE if flag given, else FALSE per default, optional     
    # parser.add_argument('--byplanner', action='store_true',
    #                     help='flag:plot metrics divided into waypoint generator for every local planner') # store TRUE if flag given, else FALSE per default, optional                               
    # parser.add_argument('--nosubtitle', action='store_false',
    #                     help='flag:plot metrics without subtitle specifying planner/wpgen') # store TRUE if flag given, else FALSE per default, optional                               
    args = parser.parse_args()
    return args

def import_scenario_json(path_to_json_file):
    with open(path_to_json_file) as f:
        return json.load(f)

def write_pedsim_yaml(scenario_json,path_to_store_pedsim,file_name):
    agent_yaml = {}
    agent_yaml["waypoints"] = {}
    for agent in scenario_json["pedsim_agents"]:
        waypoint_ids = [""]*len(agent["waypoints"])
        for i,waypoint in enumerate(agent["waypoints"]):
            waypoint_ids[i] = "waypoint_id_"+agent["name"]+"_"+str(i)
            agent_yaml["waypoints"][waypoint_ids[i]] = waypoint + [1]
        agent_yaml["agent_"+agent["name"]] = {
            "n": 1,
            "x": agent["pos"][0],
            "y": agent["pos"][0],
            "w": waypoint_ids[:2]
        }
    with open(path_to_store_pedsim+"/{}.yaml".format(file_name), 'w') as outfile:
        yaml.dump(agent_yaml, outfile, sort_keys=False,default_flow_style=None)














if __name__ == '__main__':
    args = parsing()
    path_to_json_file = args.path_to_json
    path_to_store_pedsim = args.path_to_store_pedsim
    file_name = path_to_json_file.split("/")[-1].split(".json")[0] # 

    scenario_json = import_scenario_json(path_to_json_file)
    write_pedsim_yaml(scenario_json,path_to_store_pedsim,file_name)