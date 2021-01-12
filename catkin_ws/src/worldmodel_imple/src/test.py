import subprocess
import sys

for i in range(1400):
    result = subprocess.run("python 01_generate_data.py SingleRacecar --render 1 --time_steps=300", shell=True)