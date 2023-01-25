import pylas
import open3d as o3d
import numpy as np

las = pylas.read("road_0_clean.las")

pcount = las.header.point_count

print(np.max(las.z)-np.min(las.z))

minZ = np.min(las.z)

for i in range(0, pcount-1):
    intensity = las.intensity[i]
    diff = las.z[i]-minZ

    if intensity>160 and diff>0.1:
        las.gps_time[i]=8
    
newLas = pylas.create_from_header(las.header)
newLas.points = las.points[las.gps_time==8]

newLas.write("BUMPS.las")