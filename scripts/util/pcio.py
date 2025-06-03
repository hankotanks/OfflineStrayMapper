import os
import vtk, vtkmodules
import numpy as np
import open3d as o3d

def load_point_cloud(pc_path):
    _, ext = os.path.splitext(pc_path)
    
    pc = None
    if ext == '.pcd':
        print('Loading PCD point cloud.')
        pc = o3d.io.read_point_cloud(pc_path)
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
    else:
        raise Exception("Unsupported point cloud format. Must provide VTK or PCD.")
    
    pc_size = len(pc.points)
    print('Prepared point cloud with %d points.' % pc_size)

    return pc

def write_point_cloud(pc, pc_path, pc_path_postfix):
    _, ext = os.path.splitext(pc_path)

    pc_path_out = os.path.splitext(os.path.basename(pc_path))[0] + "_" + pc_path_postfix + ext
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