import argparse
import numpy as np
import cv2
import random

def readfile(args):
    filepath = args.filepath

    with np.load(filepath) as data:
        obs = data['obs']
        random_idx = random.randint(0, len(obs)-1)
        img = obs[random_idx]
        imgBig = cv2.resize(img, (256, 256))
        cv2.namedWindow("window", cv2.WINDOW_AUTOSIZE) 
        cv2.imshow('window', imgBig)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=('Check recorded observations'))
    parser.add_argument('filepath', type=str, help='path to the file to check')
    
    args = parser.parse_args()
    readfile(args)