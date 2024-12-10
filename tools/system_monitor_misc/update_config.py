#!/usr/bin/env python3
import os
import subprocess
import yaml
from ament_index_python.packages import get_package_share_directory

# Run 'free -h' command to get total RAM
result = subprocess.run(['free', '-h'], capture_output=True, text=True)
output = result.stdout
lines = output.splitlines()
total_memory_line = lines[1].split()
total_memory = int(total_memory_line[1][:-2])  # Remove the 'G' at the end and convert to int
total_memory_mb = total_memory * 1024  # Convert to megabytes

# Read the RAM Memory config file
ram_config_path = os.path.join(get_package_share_directory("system_monitor"), "config", "mem_monitor.param.yaml")

if os.path.isfile(ram_config_path):
    with open(ram_config_path, 'r') as file:
        metadata = yaml.safe_load(file)
else:
    print(f"RAM config file not found: {ram_config_path}")
    exit(1)

# Update the available_size in the metadata
metadata['/**']['ros__parameters']['available_size'] = total_memory_mb

# Write the updated metadata back to the file
written = False
with open(ram_config_path, 'w') as file:
    yaml.safe_dump(metadata, file)
    written = True

if written:
    print(f"Total RAM: {total_memory_mb} MB")
else:
    print("Failed to update RAM config file")