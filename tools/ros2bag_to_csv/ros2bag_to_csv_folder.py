import os
import subprocess


topics = ["/auto/raw_command", "/vehicle/state",
          "/auto/raw_command2", "/lqg_telemetry"]
out_names = ["pp_output", "vehicle_state", "lqg_output", "lqg_telemetry"]
i = 0
folder_name = "/home/art/adehome/rosbags/8-24-22"
out_folder_name = "dom-csv-8-24-22"
rosbags = os.listdir(folder_name)

if os.path.exists(out_folder_name):
    print("Folder already exists")
else:
    os.makedirs(out_folder_name)
    for rosbag in rosbags:
        for j in range(len(topics)):
            subprocess.call(
                [
                    "python3",
                    "ros2bag_to_csv.py",
                    "-i",
                    os.path.join(folder_name, rosbag),
                    "-t",
                    topics[j],
                    "-o",
                    os.path.join(out_folder_name,
                                 f"{out_names[j]}_{rosbag}_{i}.csv"),
                ]
            )
        i += 1
