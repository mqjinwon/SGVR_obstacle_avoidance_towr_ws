import pandas as pd
import os
import numpy as np

#get the path of the script
path = os.path.dirname(os.path.abspath(__file__))

#get file names
files = os.listdir(path)
files.sort()

#filter out the files that are not csv files
files = [f for f in files if f.endswith('.csv')]

header = ("terrain_id", 
            "coll_mode",
            "total_success",
            "optimization_success",
            "total_cost",
            "total_collision_count",
            "swing_link_collision_count",
            "stance_link_collision_count",
            "optimization_time",
            "final_x",
            "final_y",
            "final_yaw",
            "edge_dist")

result = pd.DataFrame()

# concatenate all the files, data doesn't have index
for file_name in files:
    df = pd.read_csv(path + "/" + file_name)
    result = pd.concat([result, df], axis=0)

#write the data to a new file
result.to_csv(path + "/result/merged.csv", index=False)