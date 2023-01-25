import pylas
import numpy as np

# Read in the LAS file
las = pylas.read("a_TOTAL_NOTBUMPS.las")
point_count = las.header.point_count
chunk_las = pylas.create_from_header(las.header)
surrounding = pylas.create_from_header(las.header)


# I process the point cloud chunk by chunk.
for chunk in range(0, point_count-1, 10000):
    # These two if statements ignores early and late chunks as these doesnt have points on both sides when comparing heights.
    if chunk<=10000:
        continue
    if chunk>=point_count-30000:
        break

    # Current chunk
    chunk_las.points = las.points[chunk:chunk+10000]
    # Slightly bigger chunk expanding the current one on both sides.
    surrounding.points = las.points[chunk-3000:chunk+10000+3000]
    chunk_point_count = chunk_las.header.point_count

    local_meanZ = np.mean(chunk_las.z)
    meanZ = np.mean(surrounding.z)

    #If the current chunk is significantly higher elevated than the extended chunk we have detected a big speed bump.
    if local_meanZ-meanZ>0.013:
        for i in range(0, chunk_point_count-1):
            las.gps_time[i+chunk]=8
            
bumpsegment = pylas.create_from_header(las.header)
other = pylas.create_from_header(las.header)

bumpsegment.points = las.points[las.gps_time==8]
other.points = las.points[las.gps_time!=8]

bumpsegment.write("a_BIG_BUMPS.las")
other.write("other.las")