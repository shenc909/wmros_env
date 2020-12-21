import subprocess
import sys

for i in range(2):
    result = subprocess.run("python 01_generate_data.py SingleRacecar --render 1 --total_episodes 1 --time_steps=500", shell=True)