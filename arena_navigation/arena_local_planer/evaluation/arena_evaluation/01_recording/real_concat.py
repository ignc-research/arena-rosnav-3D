import glob
import os
import pandas as pd
import numpy as np
import time

file_path = os.path.dirname(os.path.abspath(__file__))
files = glob.glob("{0}/*.csv".format(file_path)) # get all the csv files paths in the directory where this script is located
planner = []
scenario = []
for file in files:
    planner.append(file.split("/")[-1].split("--")[0])
    scenario.append(file.split("/")[-1].split("--")[1])
if len(np.unique(planner)) != 1 or len(np.unique(scenario)) != 1:
    print("ERROR: Planners and scenarios in the files provided don't match. Terminating script.")
    print("Files provided:")
    for file in files:
        print(file.split("/")[-1])
elif len(np.unique(planner)) == 1 and len(np.unique(scenario)) == 1:
    data = pd.DataFrame()
    episode_overview = []
    for i,file in enumerate(files):
        episode_data = pd.read_csv(file)
        episode_data.episode = [i]*len(episode_data.episode)
        data = pd.concat([data,episode_data])
        episode_overview.append("{}\t{}".format(i,file.split("/")[-1]))
    file_name = "{}/{}--{}--real_concat_{}.csv".format(file_path,np.unique(planner)[0],np.unique(scenario)[0],time.strftime("%y-%m-%d_%H-%M-%S"))
    data.to_csv(file_name, index = False)
    print("INFO: Successfully concatenated {}".format(file_name.split("/")[-1]))
    with open(file_path+"/{}.txt".format(file_name.split("/")[-1].replace(".csv","")), "w") as outfile:
        outfile.write("\n".join(episode_overview))
else:
    print("ERROR: Some error occured. Terminating script.")
    print("Files provided:")
    for file in files:
        print(file.split("/")[-1])

