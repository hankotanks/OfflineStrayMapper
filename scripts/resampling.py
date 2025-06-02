import sys
import os
import vtk, vtkmodules
import numpy as np
import open3d as o3d

def load_point_cloud(pc_path):
    _, ext = os.path.splitext(pc_path)
    
    if ext == '.pcd':
        print('Loading PCD point cloud.')
        return o3d.io.read_point_cloud(pc_path)
    elif ext == '.vtk':
        print('Loading VTK point cloud.')
        vtk_reader = vtk.vtkPolyDataReader()
        vtk_reader.SetFileName(pc_path)
        vtk_reader.Update()
        vtk_output = vtk_reader.GetOutput()
        print('Finished loading VTK point cloud.')
        
        vtk_pts = vtk_output.GetPoints()
        vtk_pts_data = vtk_pts.GetData()
        vtk_pts_arr = vtkmodules.util.numpy_support.vtk_to_numpy(vtk_pts_data)
        
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(vtk_pts_arr)

        # I'm assuming that all scalar fields are colors
        # this might cause problems in the future        
        if vtk_output.GetPointData().GetScalars() is not None:
            vtk_colors = vtk_output.GetPointData().GetScalars()
            vtk_colors_arr = vtkmodules.util.numpy_support.vtk_to_numpy(vtk_colors)

            pc.colors = o3d.utility.Vector3dVector(vtk_colors_arr / 255.0)
        
        print('Finished converting VTK point cloud to open3d.')
        
        return pc

def write_point_cloud(pc, pc_path):
    _, ext = os.path.splitext(pc_path)

    pc_path_out = os.path.splitext(os.path.basename(pc_path))[0] + "_resampled" + ext
    pc_path_out = os.path.join(os.path.dirname(pc_path), pc_path_out)

    if ext == '.pcd':
        print('Writing PCD point cloud.')
        o3d.io.write_point_cloud(pc_path_out, pc)
        print('Finished writing PCD point cloud.')
    elif ext == '.vtk':
        print('Writing VTK point cloud.')
        pc_pts = np.asarray(pc.points)
        pc_colors = np.asarray(pc.colors) if pc.has_colors() else None

        vtk_pts = vtk.vtkPoints()
        vtk_pts.SetData(vtkmodules.util.numpy_support.numpy_to_vtk(pc_pts, deep=True))

        vtk_polydata = vtk.vtkPolyData()
        vtk_polydata.SetPoints(vtk_pts)

        if pc_colors is not None:
            vtk_colors = vtkmodules.util.numpy_support.numpy_to_vtk((pc_colors * 255.0).astype(np.uint8), deep=True)
            vtk_colors.SetName("Colors")
            vtk_polydata.GetPointData().SetScalars(vtk_colors)

        writer = vtk.vtkPolyDataWriter()
        writer.SetFileName(pc_path_out)
        writer.SetInputData(vtk_polydata)
        writer.Write()
        print('Finished writing VTK point cloud.')

def resample_point_cloud(pc_path, num_samples, nb_neighbors, std_ratio):
    pc = load_point_cloud(pc_path)
    pc_size = len(pc.points)
    print('Loaded point cloud with %d points.' % pc_size)

    every_k_points = int(len(pc.points) / num_samples)
    print('Sampling every %d points' % every_k_points)

    pc_downsampled = pc.uniform_down_sample(every_k_points)
    print('Completed downsampling.')

    cl, ind = pc_downsampled.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    pc_cleaned = pc_downsampled.select_by_index(ind)
    print('Removed outliers.')

    pc_cleaned_size = len(pc_cleaned.points)
    print('Resampled point cloud has %d points.' % pc_cleaned_size)

    o3d.visualization.draw_geometries([pc_cleaned], window_name="Output")

    write_point_cloud(pc_cleaned, pc_path)
    print('Finished writing resampled point cloud.')

if __name__ == "__main__":
    if len(sys.argv) != 5:
        print("Usage: python process_pcd.py <path_to_vtk_or_pcd> <num_samples> <nb_neighbors> <std_ratio>")
        sys.exit(1)    

    resample_point_cloud(sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), float(sys.argv[4]))