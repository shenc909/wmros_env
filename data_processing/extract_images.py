import rosbag
import numpy as np
import cv2
from cv_bridge import CvBridge

from os import listdir
from os.path import isfile, join

# path to recorded bag files
BAG_FILE_PATH = './data/recorded/'
# path to generated images npz
GENERATED_PATH = './data/generated_images/'
# image dims
IMAGE_WIDTH = 64
IMAGE_HEIGHT = 64

def main():
    # get list of files
    bag_files = [f for f in listdir(BAG_FILE_PATH) if isfile(join(BAG_FILE_PATH, f))]

    if len(bag_files) == 0:
        raise IOError('No bag files found!')
    
    # generate full file paths
    bag_files_dir = [BAG_FILE_PATH + f for f in bag_files]

    for filename, bag_file in zip(bag_files, bag_files_dir):
        print(f'Extracting images from {filename}')
        bridge = CvBridge()
        # read bag file
        bag = rosbag.Bag(bag_file)
        images = []
        # get messages from bag file
        for topic, msg, t in bag.read_messages(topics=['/ackermann_vehicle/camera1/image_raw']):
            # convert from cv_bridge BGR8 format to RGB8 format
            image = cv2.cvtColor(bridge.imgmsg_to_cv2(msg,desired_encoding='rgb8'), cv2.COLOR_BGR2RGB)
            # resize image to VAE input size
            images.append(cv2.resize(image, (IMAGE_WIDTH, IMAGE_HEIGHT)))
        filename = GENERATED_PATH + filename + '_images'
        np.savez_compressed(filename, obs=images)

if __name__ == "__main__":
    main()
