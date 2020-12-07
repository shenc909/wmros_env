import ezdxf
import numpy as np
import matplotlib.pyplot as plt

from os import listdir
from os.path import isfile, join

file_path = './track_dxf/'

def arcRadius(width, height):
    return (height/2) + (width**2)/(8*height)

def getPolylinePoints(dxf_file_dir):
    doc = ezdxf.readfile(dxf_file_dir)
    msp = doc.modelspace()
    line = msp.query('LWPOLYLINE').first
    with line.points('xyb') as points:
        return points


def walker(coords, spacing):
    if coords[0][0:2] != coords[-1][0:2]:
        print("No closed loop in polyline!")
        return
    length = len(coords)
    waypoints = []

    for i in range(length):
        print(f'processing point number {i}')
        j = i + 1
        if j >= length:
            j = 0
        residual = 0
        if coords[i][2] != 0:
            additional_waypoints, residual = arc_calc(coords[i], coords[j], residual, spacing)
        else:
            additional_waypoints, residual = line_calc(coords[i], coords[j], residual, spacing)
        
        waypoints = waypoints + additional_waypoints
    
    return waypoints
    
def line_calc(coord1, coord2, residual, spacing):
    x1, y1, _ = coord1
    x2, y2, _ = coord2
    line_len = np.linalg.norm([[x1, y1],[x2, y2]])
    
    if line_len < residual:
        return [], residual - line_len
    
    coords = []

    angle = np.arctan2(y2-y1, x2-x1)
    last_coord = np.array([x1, y1])

    # Residual handling
    dx = np.cos(residual)
    dy = np.sin(residual)
    d_coord = np.array([dx, dy])
    last_coord = last_coord + d_coord
    coords.append(last_coord)
    line_length_remaining = line_len - residual
    
    # line handling
    dx = np.cos(spacing)
    dy = np.sin(spacing)
    d_coord = np.array([dx, dy])

    while line_length_remaining > spacing:
        last_coord = last_coord + d_coord
        coords.append(last_coord)
        line_length_remaining = line_length_remaining - spacing
    
    return coords, line_length_remaining

def arc_calc(coord1, coord2, residual, spacing):
    x1, y1, _ = coord1
    x2, y2, _ = coord2
    _, _, bulge = coord1
    arc_width = np.linalg.norm([[x1, y1],[x2, y2]])
    arc_height = abs(bulge) * arc_width/2
    arc_radius = arcRadius(arc_width, arc_height)
    small_angle = np.arccos((arc_width**2)/(2*arc_width*arc_radius))
    arc_angle = np.pi - 2*small_angle
    angle = np.arctan2(y2-y1, x2-x1)
    total_angle = angle + small_angle
    dx = np.cos(total_angle)
    dy = np.sin(total_angle)
    center = np.array([x1+dx, y1+dy])

    arc_length = arc_radius * arc_angle

    coords = []

    circumference = np.pi * 2 * arc_radius
    # residual angle in rads
    residual_angle = residual/circumference
    # base angle of starting coordinates
    base_angle = np.arctan2(dy, dx)

    new_angle = residual_angle + base_angle
    new_coord = np.array([center[0] + np.cos(new_angle), center[1] + np.sin(new_angle)])
    coords.append(new_coord)
    arc_length_remaining = arc_length - residual

    spacing_angle = spacing/circumference

    while arc_length_remaining > spacing:
        new_angle = new_angle + spacing_angle
        new_coord = np.array([center[0] + np.cos(new_angle), center[1] + np.sin(new_angle)])
        coords.append(new_coord)
        arc_length_remaining = arc_length_remaining - spacing
    
    return coords, arc_length_remaining


def main():
    dxf_files = [f for f in listdir(file_path) if isfile(join(file_path, f))]
    dxf_files = [file_path + f for f in dxf_files]
    # print(dxf_files)
    print(dxf_files[0])
    coords = getPolylinePoints(dxf_files[0])
    waypoints = walker(coords, 0.01)
    x = []
    y = []
    for i in waypoints:
        x.append(i[0])
        y.append(i[1])
    plt.plot(x, y)
    plt.show()


if __name__ == "__main__":
    main()