import sys
import os
import numpy as np
import vtk, vtkmodules
import sklearn.cluster as skcluster
import scipy.spatial as scispatial
import open3d as o3d

from util import pcio

def triangulate_point_cloud(pc_path, num_std):
    pc = pcio.load_point_cloud(pc_path)

    pc_pts = np.asarray(pc.points)
    pc_rgb = np.asarray(pc.colors)

    pc_nn = pc.compute_nearest_neighbor_distance()
    pc_nn_std = np.std(pc_nn)

    radius = np.mean(pc_nn)
    print('Mean NN distance is %f.' % radius)

    pc.estimate_normals(o3d.geometry.KDTreeSearchParamRadius(radius * 2.0))
    print('Estimated surface normals.')

    radii = np.arange(0, pc_nn_std * num_std, pc_nn_std) + radius * 2.0
    print('Using', end=' ')
    print(radii, end=' as ball pivot radii.\n')
    radii = o3d.utility.DoubleVector(radii)

    pc_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pc, radii)
    print('Constructed mesh.')

    o3d.visualization.draw_geometries([pc_mesh])

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python triangulation.py <path_to_vtk_or_pcd> <num_std>")
        sys.exit(1)    

    triangulate_point_cloud(sys.argv[1], int(sys.argv[2]))