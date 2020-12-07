import ezdxf
import numpy as np

from os import listdir
from os.path import isfile, join

file_path = './track_dxf/'

def arcRadius(width, height):
    return (height/2) + (width**2)/(8*height)

def getPolylinePoints(dxf_file_dir):
    doc = ezdxf.readfile(dxf_file_dir)
    msp = doc.modelspace()
    line = msp.query('LWPOLYLINE').first
    return line.points('xyb')

def walker(coords):
    if coords[0][0:2] != coords[-1][0:2]:
        print("No closed loop in polyline!")
        return
    itr = 0
    
    while itr < len(coords):
        

def main():
    dxf_files = [f for f in listdir(file_path) if isfile(join(file_path, f))]
    dxf_files = [file_path + f for f in dxf_files]
    # print(dxf_files)
    for dxf in dxf_files:
        coords = getPolylinePoints(dxf)


if __name__ == "__main__":
    main()