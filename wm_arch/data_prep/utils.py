import numpy as np
import argparse
import os

from os import listdir
from os.path import isfile, join

def process_observations():
    pass

def normalize_observations(directory):
    parent_dir = directory
    normalized_folder = 'normalized_obs'
    normalized_dir = os.path.join(parent_dir, normalized_folder)
    try:
        os.mkdir(normalized_dir)
    except FileExistsError:
        pass

    obs_files = [f for f in listdir(parent_dir) if isfile(join(parent_dir, f))]
    obs_files_dir = [join(parent_dir, f) for f in obs_files]

    for filepath, name in zip(obs_files_dir, obs_files):
        if name == '.gitignore':
            continue
        print(f'Normalizing {name}...')
        data = np.load(filepath)
        obs = data['obs']
        action = data['action']
        reward = data['reward']
        done = data['done']

        normalized_obs = [img/255 for img in obs]

        np.savez_compressed(join(normalized_dir, name), obs=normalized_obs, action=action,
                                reward=reward, done=done)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=('Data processing utilities'))
    parser.add_argument('--normalize', action ='store_true', help='renormalize image data from 0 - 255 to 0 - 1')
    parser.add_argument('--directory', type=str, help='directory to process')

    args = parser.parse_args()

    if args.normalize:
        if args.directory != None:
            normalize_observations(args.directory)