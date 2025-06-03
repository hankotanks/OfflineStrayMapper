import sys
import os
import numpy as np
import vtk, vtkmodules
import sklearn.cluster as skcluster
import open3d as o3d

from util import pcio

def triangulate_point_cloud(pc_path):
    pc = pcio.load_point_cloud(pc_path)

    pc_nn = pc.compute_nearest_neighbor_distance()
    pc_nn_avg = np.mean(pc_nn)
    pc = pc.voxel_down_sample(pc_nn_avg)
    pc_nn_std = np.std(pc_nn)
    print('Mean NN distance is %f.' % pc_nn_avg)

    pc.estimate_normals()
    print('Estimated surface normals.')

    pc_mesh_rad = o3d.utility.DoubleVector([pc_nn_avg, pc_nn_avg * 2.0, pc_nn_avg * 4.0, pc_nn_avg * 8.0, pc_nn_avg * 16.0, pc_nn_avg * 32.0])
    pc_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pc, pc_mesh_rad)
    print('Constructed mesh.')

    o3d.visualization.draw_geometries([pc_mesh])

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python triangulation.py <path_to_vtk_or_pcd>")
        sys.exit(1)    

    triangulate_point_cloud(sys.argv[1])