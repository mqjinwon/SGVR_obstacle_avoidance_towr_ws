import pandas as pd
import os
import numpy as np
import logging

#get the path of the script
path = os.path.dirname(os.path.abspath(__file__))

data_path = path + ""

#get file names ordered by name
files = os.listdir(data_path)
files.sort()

#filter out the files that are not csv files
files = [f for f in files if f.endswith('.csv')]

terrain_id_dic = {1: 'random_stair', 2: 'random_block', 3: 'random_huddle', 4: 'perlin noise', 
                6: 'random_stair', 7: 'random_block', 8: 'random_huddle', 9: 'perlin noise',}
coll_mode_dic = {0: 'None', 1: "Grad_stance", 3: "Grad_stance, Grad_swing", 4: "Heuristic_Stance", 6: "Heuristic_Stance, Grad_Swing"}

optimize_result_dic ={"timeout":-4 , "failed": -2,
                        "success":0, "success_local_min":1, "infeasible":2}

header_list = ("terrain_id", 
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

statistic_index_list = ("terrain_id", 
                        "coll_mode", 
                        "success_rate", 
                        "op_total_fail_rate",
                        "op_timeout_rate",
                        "op_fail_rate",
                        "op_success_rate",
                        "total_collision_count", 
                        "swing_link_collision_count", 
                        "stance_link_collision_count",
                        "mean_optimization_time", 
                        "std_optimization_time",
                        "edge_dist")


result = pd.DataFrame()

for file_name in files:
        
    df = pd.read_csv(data_path + "/" + file_name)

    # edge_dist
    edge_keys = df[header_list[-1]].unique()

    for edge_key in edge_keys:
        df = pd.read_csv(data_path + "/" + file_name)
        df = df[df[header_list[-1]] == edge_key]

        data_size = df.shape[0] #len(df)
        if data_size == 0:
            continue
        terrain_id = 0
        coll_mode = 0
        success_rate = 0
        total_collision_count = 0
        swing_link_collision_count = 0
        stance_link_collision_count = 0
        mean_optimization_time = 0
        std_optimization_time = 0
        optimize_timeout = 0
        optimize_failed = 0
        optimize_success = 0
        optimize_success_local_min = 0
        failed_optimize_df = df[df['total_success'] == 0].reset_index(drop=True)
        success_optimize_df = df[df['total_success'] == 1].reset_index(drop=True)

        # test!!
        print("path: " + data_path + "/" + file_name)
        print("size: " + str(data_size))
        optimize_result = df[header_list[3]].value_counts()

        for key in df[header_list[3]].unique():
            for optimize_key in optimize_result_dic:
                if key == optimize_result_dic[optimize_key]:
                    if optimize_key == "timeout":
                        optimize_timeout = optimize_result[key]
                    elif optimize_key == "failed":
                        optimize_failed = optimize_result[key]
                    elif optimize_key == "infeasible":
                        optimize_failed = optimize_result[key]
                    elif optimize_key == "success":
                        optimize_success = optimize_result[key]
                    elif optimize_key == "success_local_min":
                        optimize_success_local_min = optimize_result[key]

        op_total_fail_rate = (optimize_timeout+optimize_failed) / data_size
        op_timeout_rate = optimize_timeout / data_size
        op_fail_rate = optimize_failed / data_size
        op_success_rate = (optimize_success+optimize_success_local_min) / data_size

        terrain_id = df['terrain_id'].iloc[0]
        coll_mode = df['coll_mode'].iloc[0]
        success_rate = df['total_success'].mean()
        
        if len(failed_optimize_df) > 0:
            total_collision_count = failed_optimize_df['total_collision_count'].mean()
            swing_link_collision_count = failed_optimize_df['swing_link_collision_count'].mean()
            stance_link_collision_count = failed_optimize_df['stance_link_collision_count'].mean()
        if len(success_optimize_df) > 0:
            mean_optimization_time = success_optimize_df['optimization_time'].mean()
            std_optimization_time = success_optimize_df['optimization_time'].std()

        result = result.append(pd.DataFrame([[terrain_id_dic[terrain_id], 
                                            coll_mode_dic[coll_mode], 
                                            success_rate, 
                                            op_total_fail_rate,
                                            op_timeout_rate,
                                            op_fail_rate,
                                            op_success_rate,
                                            total_collision_count, 
                                            swing_link_collision_count, 
                                            stance_link_collision_count,
                                            mean_optimization_time, 
                                            std_optimization_time,
                                            edge_key]],
                                            columns=statistic_index_list))

result = result.round(4)
print(result)

result.to_csv(path + "/result/statistic.csv", index=False)