import sys
import os
import vtk, vtkmodules
import numpy as np
import open3d as o3d

from util import pcio

def resample_point_cloud(pc_path, num_samples, nb_neighbors, std_ratio):
    pc = pcio.load_point_cloud(pc_path)

    every_k_points = int(len(pc.points) / num_samples)
    print('Sampling every %d points' % every_k_points)

    pc_downsampled = pc.uniform_down_sample(every_k_points)
    print('Completed downsampling.')

    _, ind = pc_downsampled.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    pc_cleaned = pc_downsampled.select_by_index(ind)
    print('Removed outliers.')

    pc_cleaned_size = len(pc_cleaned.points)
    print('Resampled point cloud has %d points.' % pc_cleaned_size)

    o3d.visualization.draw_geometries([pc_cleaned], window_name="Output")

    pcio.write_point_cloud(pc_cleaned, pc_path, "resampled")

if __name__ == "__main__":
    if len(sys.argv) != 5:
        print("Usage: python resampling.py <path_to_vtk_or_pcd> <num_samples> <nb_neighbors> <std_ratio>")
        sys.exit(1)

    resample_point_cloud(sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), float(sys.argv[4]))