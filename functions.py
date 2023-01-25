import numpy as np

def findCentroid(neighborhood):
    pointCount = neighborhood.header.point_count
    sumx = np.sum(neighborhood.x)
    sumy = np.sum(neighborhood.y)
    sumz = np.sum(neighborhood.z)
    x_c = sumx/pointCount
    y_c = sumy/pointCount
    z_c = sumz/pointCount

    return np.vstack([x_c, y_c, z_c])

def covMatrix(neighborhood):

    pointCount = neighborhood.header.point_count

    centroid = findCentroid(neighborhood)
    matrix = np.zeros((3,3))

    for i in range(0, pointCount):
        v = np.vstack([neighborhood.x[i], neighborhood.y[i], neighborhood.z[i]])
        matrix+=np.dot((v-centroid), (v-centroid).T)

    cov = (1/pointCount)*matrix

    return cov

