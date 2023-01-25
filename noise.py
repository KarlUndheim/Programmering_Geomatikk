import pylas
import numpy as np
from scipy.spatial import KDTree
import math
import time

start_time = time.time()

# Read in the LAS file
las = pylas.read("0-0_clean.las")
point_count = las.header.point_count

# Once again I process the point cloud chunk by chunk.
chunk_las = pylas.create_from_header(las.header)


# Here I remove noise by looking at how many neighbors each point has within a radius r. 
# Additinally I remove points with an intensity over a certain value.
for chunk in range(0, point_count-1, 10000):
    print(chunk)
    chunk_las.points = las.points[chunk:chunk+10000]
    chunk_point_count = chunk_las.header.point_count

    # Extract the point cloud from the LAS file
    pcd = np.column_stack((chunk_las.x, chunk_las.y, chunk_las.z))

    # Build a KD-tree data structure for the point cloud
    kd_tree = KDTree(pcd)

    for i in range(0, chunk_point_count-1):
        # Specify the point for which to find neighbors
        point = np.array([pcd[i][0], pcd[i][1], pcd[i][2]])

        # Find the neighbors of the point within the radius r
        indices = kd_tree.query_ball_point(point, r=0.5)

        # Extract the coordinates of the neighbors
        neighbors = pcd[indices]
        # Most of the scalar fields are not present in 0-0.las 
        # so I use gps_time as a placeholder attribute 
        # to store relevant information.
        if len(neighbors)<60:
            las.gps_time[i+chunk] = 7
        elif las.intensity[chunk+i]>183:
            las.gps_time[i+chunk] = 7

noise = pylas.create_from_header(las.header)
noise.points = las.points[las.gps_time==7]
noise.write("0-0_noise.las")
            
road = pylas.create_from_header(las.header)
road.points= las.points[las.gps_time!=7]
road.write("0-0_Processed.las")

print("My program took", time.time() - start_time, " s to run")


