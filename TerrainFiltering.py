import numpy as np
import pylas
import random
import math
# import pandas as pd


class RANSAC:
    """
    RANSAC algorithm
        INPUT:
            pcd:            np.array()     # point clouds data
            MAX_Iteration:  int            # the number you want for max iterations
            Thres_dis:      int            # distance threshold to
        OUTPUT:
            self.pcd
            best_interior:      np.array()
            best_interior_idx:  list
            best_plane:         list
    """
    def __init__(self, pcd, MAX_Iteration, Thres_dis):
        self.pcd = pcd
        self.point_count = self.pcd.header.point_count
        self.pcd_p = np.array([self.pcd.x, self.pcd.y, self.pcd.z]) # 3* n_points
        self.pcd_p = self.pcd_p.T   # n_points* 3
        """ print('point clouds size: ', self.pcd_p.shape) """
        self.Max_Iter = MAX_Iteration
        self.thres_dis = Thres_dis

    def RanSac_algthm(self):
        """
        INPUT: self.pcd;    self.Max_Iter;  self.thres_dis
        :return: self.pcd, best_interior, best_interior_idx, best_plane
        """
        best_interior = np.array([])
        best_interior_idx = np.array([])
        best_plane = []

        for i in range(self.Max_Iter):
            # Step 1: add 3 random points
            random.seed()

            start_points_idx = random.sample(range(0, self.point_count-1), 3) # start_points = np.random.randint(0, self.point_count, size=3)

            p1 = [self.pcd.x[start_points_idx[0]], self.pcd.y[start_points_idx[0]], self.pcd.z[start_points_idx[0]]]
            p2 = [self.pcd.x[start_points_idx[1]], self.pcd.y[start_points_idx[1]], self.pcd.z[start_points_idx[1]]]
            p3 = [self.pcd.x[start_points_idx[2]], self.pcd.y[start_points_idx[2]], self.pcd.z[start_points_idx[2]]]
            
            # Step 2: Calculate plane contains p1, p2 and p3
            # Plane equation: ax + by + cz + d = 0
            # p1: x1, y1, z1
            # p2: x2, y2, z2
            # p3: x3, y3, z3
            a = (p2[1] - p1[1]) * (p3[2] - p1[2]) - (p2[2] - p1[2]) * (p3[1] - p1[1]) # a = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1)
            b = (p2[2] - p1[2]) * (p3[0] - p1[0]) - (p2[0] - p1[0]) * (p3[2] - p1[2]) # b = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1)
            c = (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0]) # c = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1)
            d = -(a * p1[0] + b * p1[1] + c * p1[2])# d = -(a*x1 + b*y1 + c*z1)
            plane_len = max(0.00001, math.sqrt(a*a + b*b + c*c))

            # Step 3: add points into interior points
            # Skip points have be chosen as seed
            # pcd_p_search = np.delete(self.pcd_p, start_points_idx, axis=0)

            #  Calculate distance between the point and the plane
            #  (now all point are in calculating, including 3 samples)
            dist = np.abs((a*self.pcd_p[:, 0] + b * self.pcd_p[:, 1] + c * self.pcd_p[:, 2] + d)) / plane_len

            # Add points in distance threshold as interior points

            interior_idx = np.where((dist <= self.thres_dis))[0]

            pcd_p_interior = self.pcd_p[interior_idx, :]

            # update the best model
            if (pcd_p_interior.shape[0] > best_interior.shape[0]) and (pcd_p_interior.shape[0]< self.point_count): # and (pcd_p_interior.shape[0]< self.point_count/2 * 1.15):
                best_interior = pcd_p_interior
                best_interior_idx = interior_idx
                best_plane = [a, b, c, d, plane_len]

        # I also want to store the indeces of the remaining points, to use for visualization later.
        rest = [a for a in range(0, self.point_count-1) if a not in best_interior_idx]

        return best_interior_idx, rest



if __name__=='__main__':
    ## read point clouds
    pcd = pylas.read("0-0.las")
    
    # point cloud for storing points chunk by chunk.
    temp = pylas.create_from_header(pcd.header)

    # Point cloud which will store the result
    result = pylas.create_from_header(pcd.header)

    # The points which do not fit the plane(non road points)
    rests = pylas.create_from_header(pcd.header)

    all_point_count = pcd.header.point_count

    # Lists for storing the chunks, which will be merged together at the end.
    pcds = []
    restpcds = []


    # Here I segment the point cloud into chunks of 10000 points. For each chunk I find the points that are inliers (these are ground points).
    for i in range(0, all_point_count, 10000):
        temp.points = pcd.points[i:i+10000]

        # Parameters found by trial and error.
        rans = RANSAC(temp, 200, 0.09)
        best_interior_idx, rest = rans.RanSac_algthm()

        # save result
        newLas = pylas.create_from_header(temp.header)
        restLas = pylas.create_from_header(temp.header)
        newLas.points = temp.points[best_interior_idx]
        restLas.points = temp.points[rest]

        pcds.append(newLas)
        restpcds.append(restLas)
    
    # Merge all the chunks together to regain the original point cloud separated into ground points and non-ground points.
    result = pylas.merge([p for p in pcds])
    rests = pylas.merge([p for p in restpcds])
    
    result.write("0-0_clean.las")
    rests.write("0-0_rest.las")

