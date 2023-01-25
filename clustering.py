import pylas
import open3d as o3d
import numpy as np
from sklearn.cluster import DBSCAN
from functions import findCentroid, covMatrix
from numpy.linalg import eig

las = pylas.read("TOTAL_BUMPS.las")
las.points = las.points[las.gps_time==8]
point_count = las.header.point_count

X = np.array([[las.x[i], las.y[i]] for i in range(0, point_count-1)])

# I use sklearn's DBSCAN on the potential bumps with the goal of removing sidewalk wrongly classified as bumps. 
clustering = DBSCAN(eps=0.5, min_samples=90).fit(X)

labels = clustering.labels_
unique_labels = list(set(labels))

result = []

# For each cluster I analyse its size and its linearity to determine if it is a speed bump or not.
for label in unique_labels:
    temp = pylas.create_from_header(las.header)
    indices = np.where(labels == label)
    temp.points = las.points[indices]
    if temp.header.point_count<800:

        # I calculate the covariance matrix to find its eigenvalues, which I use to calculate the linearity of the cluster.
        cov = covMatrix(temp)
        eigs = eig(cov)
        eigvals = eigs[0]

        eigvals.sort()

        eigvals = eigvals[::-1]
        e1 = eigvals[0]
        e2 = eigvals[1]
        e3 = eigvals[2]

        if e1==0:
            print("NULL HER")
            continue

        linearity = (e1-e2)/e1

        # Speed bumps are generally not linear so I remove all clusters with a linearity above 0.8
        if linearity<0.80:
            result.append(temp)
        else:
            print("Removed")

# Merge the clusters which survived. This is the final result
ret = pylas.merge([p for p in result])

ret.write("clustering.las")

