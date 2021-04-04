import ezdxf
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm

from os import listdir
from os.path import isfile, join

file_path = './track_dxf/'
dest_path = './track_waypoints/'

CHECK_PLOT = False

# Function for determining start and direction of generated waypoints (in loop)
def startZero(waypoints):
    
    # Find starting point (closest to zero)
    min_dist = np.inf
    min_index = -1
    for i in range(len(waypoints)):
        dist = np.linalg.norm(waypoints[i])
        if dist < min_dist:
            min_dist = dist
            min_index = i
        
    # print(f'min_index: {min_index}, min_dist: {min_dist}')
    output_waypoints = waypoints[min_index:] + waypoints[0:min_index]

    # Find direction (defaults to rightwards in x dir)
    if output_waypoints[0][0] > output_waypoints[1][0]:
            output_waypoints = output_waypoints[::-1]

    return output_waypoints

# Find radius of arc given width and height
def arcRadius(width, height):
    return (height/2) + (width**2)/(8*height)

# Find centre of circle of the arc given start and end points of arc and bulge
def arcCenter(r, x1, y1, x2, y2, bulge):
    # Centre of arc calculation
    base_x = (x2+x1)/2
    base_y = (y2+y1)/2

    d = np.sqrt((x2-x1)**2 + (y2-y1)**2)
    n_perpendicular = np.array([-(y2-y1), (x2-x1)])
    n_perpendicular = n_perpendicular/np.linalg.norm(n_perpendicular)

    h = np.sqrt(r**2 - (d**2)/4)
    diff_vector = h * n_perpendicular
    eps = np.sign(bulge)
    if abs(bulge) > 1:
        eps *= -1 
    center = np.array([base_x + eps*diff_vector[0], base_y + eps*diff_vector[1]])

    return center

# Find starting angle (in polar coordinates) of the starting point of arc
def startAngle(center, start):
    angle = np.arccos(np.dot(start - center, np.array([1,0]))/(np.linalg.norm(start - center)))
    # print(f'angle: {angle}')
    if center[1] > start[1]:
        angle = -angle
    return angle

# Function to get Polyline points given DXF file
def getPolylinePoints(dxf_file_dir):
    doc = ezdxf.readfile(dxf_file_dir)
    msp = doc.modelspace()
    line = msp.query('LWPOLYLINE').first
    with line.points('xyb') as points:
        return points

# Generate loop of waypoints given DXF polyline coordinates and desired spacing between waypoints
def walker(coords, spacing):
    loop_closure = 1
    if coords[0][0:2] != coords[-1][0:2]:
        # print("No closed loop in polyline!")
        loop_closure = 0
        # return
    length = len(coords)
    waypoints = []
    residual = 0
    for i in range(length-loop_closure):
        # print(f'processing point number {i}')
        j = i + 1
        if j >= length:
            j = 0
        if coords[i][2] != 0:
            # print("detected curve")
            additional_waypoints, residual = arcCalc(coords[i], coords[j], residual, spacing)
        else:
            # print("detected line")
            additional_waypoints, residual = lineCalc(coords[i], coords[j], residual, spacing)
        
        waypoints = waypoints + additional_waypoints
    
    return waypoints

# Generate waypoints for line segment
def lineCalc(coord1, coord2, residual, spacing):
    x1, y1, _ = coord1
    x2, y2, _ = coord2
    line_len = np.linalg.norm([x1-x2, y1-y2])
    # print(f'line len: {line_len}')
    
    if line_len < residual:
        return [], residual - line_len
    
    coords = []

    angle = np.arctan2(y2-y1, x2-x1)
    last_coord = np.array([x1, y1])

    # Residual handling
    dx = residual * np.cos(angle)
    dy = residual * np.sin(angle)
    d_coord = np.array([dx, dy])
    last_coord = last_coord + d_coord
    coords.append(last_coord)
    line_length_remaining = line_len - residual
    
    # line handling
    dx = spacing * np.cos(angle)
    dy = spacing * np.sin(angle)
    d_coord = np.array([dx, dy])

    while line_length_remaining > spacing:
        last_coord = last_coord + d_coord
        coords.append(last_coord)
        line_length_remaining = line_length_remaining - spacing
    
    return coords, spacing - line_length_remaining

# Generate waypoints for arc segments
def arcCalc(coord1, coord2, residual, spacing):
    x1, y1, bulge = coord1
    x2, y2, bulge2 = coord2
    # print(f'coord1: {x1},{y1},{bulge}, coord2: {x2},{y2},{bulge2}')
    arc_width = np.linalg.norm([x1-x2, y1-y2])
    arc_height = abs(bulge) * arc_width/2
    arc_radius = arcRadius(arc_width, arc_height)
    # print(f'radius: {arc_radius}')

    c1 = np.array([x1, y1])
    c2 = np.array([x2, y2])

    center = arcCenter(arc_radius, x1, y1, x2, y2, bulge)
    # plt.scatter(center[0], center[1], color='green', s=1.5)
    plt.axis('equal')

    arc_angle = np.arcsin(np.linalg.norm((c1-c2)/2)/arc_radius)
    # print(f'arc_angle: {arc_angle}')
    if abs(bulge) > 1:
        arc_angle = np.pi - arc_angle
    arc_angle *= 2

    arc_length = arc_radius * arc_angle
    # print(f'arc length: {arc_length}')

    coords = []
    if arc_length > residual:
        # residual angle in rads
        residual_angle = residual/arc_radius
        # base angle of starting coordinates
        base_angle = startAngle(center, np.array([x1, y1]))

        new_angle = base_angle + np.sign(bulge) * residual_angle
        new_coord = np.array([center[0] + arc_radius * np.cos(new_angle), center[1] + arc_radius * np.sin(new_angle)])
        coords.append(new_coord)
        arc_length_remaining = arc_length - residual

        spacing_angle = spacing/arc_radius
        # print(f'spacing angle: {spacing_angle}')

        while arc_length_remaining > spacing:
            new_angle += np.sign(bulge) * spacing_angle
            new_coord = np.array([center[0] + arc_radius * np.cos(new_angle), center[1] + arc_radius * np.sin(new_angle)])
            coords.append(new_coord)
            arc_length_remaining -= spacing
            # print(f'arc length remaining: {arc_length_remaining}')
    
        return coords, spacing - arc_length_remaining
    else:
        return [], residual - arc_length


def main():
    # Read all files in file_path
    dxf_files = [f for f in listdir(file_path) if isfile(join(file_path, f))]
    dxf_files_dir = [file_path + f for f in dxf_files]
    # print(dxf_files)
    # print(dxf_files[10])
    # dxf_files = ['C:/Users/shenc/Documents/GitHub/wmros_env/data_processing/track_dxf/track14.dxf']
    for dxf_file, filename in zip(dxf_files_dir, dxf_files):
        coords = getPolylinePoints(dxf_file)
        print(f'Extracting from {dxf_file}')

        # create waypoints, second arg is spacing between waypoints
        waypoints = walker(coords, 0.3)
        waypoints = startZero(waypoints)

        if CHECK_PLOT:
            x = []
            y = []
            colors = cm.rainbow(np.linspace(0, 1, len(waypoints)))
            for i, c in zip(waypoints, colors):
                x.append(i[0])
                y.append(i[1])
                plt.scatter(i[0], i[1], s=2, color=c)

            # x2 = []
            # y2 = []
            # for j in coords:
            #     x2.append(j[0])
            #     y2.append(j[1])
            # plt.scatter(x2, y2, s=2, color='red')

            plt.hlines(0, -0.5, 0.5, linewidths=1)
            plt.vlines(0, -0.5, 0.5, linewidths=1)
            plt.show()

        np.save(f'{dest_path+filename}.npy', waypoints)


if __name__ == "__main__":
    main()