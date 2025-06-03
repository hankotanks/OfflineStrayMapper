import sys
import os
import numpy as np
import vtk, vtkmodules
import sklearn.cluster as skcluster
import open3d as o3d

from util import pcio

def cluster_point_cloud(pc_path, min_samples):
    pc = pcio.load_point_cloud(pc_path)
    pc_size = len(pc.points)
    
    pc_pts_avg_dist = np.mean(pc.compute_nearest_neighbor_distance())
    print('Mean NN distance is %f.' % pc_pts_avg_dist)

    pc_pts = np.asarray(pc.points)
    pc_rgb = np.asarray(pc.colors)

    pc_dbscan = skcluster.DBSCAN(eps=pc_pts_avg_dist, min_samples=min_samples)
    pc_labels = pc_dbscan.fit_predict(pc_pts)

    pc_cleaned_mask = (pc_labels != -1)
    pc_cleaned = o3d.geometry.PointCloud()
    pc_cleaned.points = o3d.utility.Vector3dVector(pc_pts[pc_cleaned_mask])
    pc_cleaned.colors = o3d.utility.Vector3dVector(pc_rgb[pc_cleaned_mask])
    pc_cleaned_size = len(pc_cleaned.points)
    print('Clustering removed %d points, new point cloud has %d points.' % (pc_size - pc_cleaned_size, pc_cleaned_size))

    pcio.write_point_cloud(pc_cleaned, pc_path, "clustered")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python clustering.py <path_to_vtk_or_pcd> <min_samples>")
        sys.exit(1)    

    cluster_point_cloud(sys.argv[1], int(sys.argv[2]))