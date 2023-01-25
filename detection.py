import pylas
import numpy as np
from scipy.spatial import KDTree
import math
import time

start_time = time.time()

# Read in the LAS file
las = pylas.read("0-0_processed.las")

point_count = las.header.point_count

chunk_las = pylas.create_from_header(las.header)

# This algorithm detects potential speed bumps. 
# It works in a similar way as the noise removal algorithm, by using neighbors of each point.

for chunk in range(0, point_count-1, 20000):
    print(chunk)
    chunk_las.points = las.points[chunk:chunk+20000+5000]
    chunk_point_count = chunk_las.header.point_count-5000

    # Extract the point cloud from the LAS file
    pcd = np.column_stack((chunk_las.x, chunk_las.y, chunk_las.z))

    # Build a KD-tree data structure for the point cloud
    kd_tree = KDTree(pcd)

    for i in range(0, chunk_point_count-1):
        # Specify the point for which to find neighbors
        point = np.array([pcd[i][0], pcd[i][1], pcd[i][2]])

        # Find the neighbors of the point within the radius r
        indices = kd_tree.query_ball_point(point, r=1.0)

        # Extract the z-coordinates of the neighbors

        z_neighbors = pcd[indices][:,2]

        meanZ = np.mean(z_neighbors)

        # If a point is significantly higher elevated than its neighbors, it could be a speed bump.
        if chunk_las.z[i]>meanZ+0.025:
            las.gps_time[i+chunk]=8
            

notbumps = pylas.create_from_header(las.header)
notbumps.points = las.points[las.gps_time!=8]

# I continue to save both relevant and non relevant points for visualization.
las.write("TOTAL_BUMPS.las")
notbumps.write("a_TOTAL_NOTBUMPS.las")


print("My program took", time.time() - start_time, " s to run")


