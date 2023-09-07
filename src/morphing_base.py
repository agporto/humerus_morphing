import open3d as o3d
import numpy as np
import copy

### LOCAL MODULES ###
from preprocessing.preprocessing import Preprocessing 
from registration.registration import Registration

import constants.constants as CONST

### FUNCTIONS ###
def load_meshes():
        source_mesh = o3d.io.read_triangle_mesh("../meshes/O35_M_humerus_source.stl")
        target_mesh = o3d.io.read_triangle_mesh("../meshes/U35_F_humerus_partial_target.stl")
        return source_mesh, target_mesh

def convert_to_point_cloud(mesh: o3d.geometry.TriangleMesh):
    ''' Converts a mesh to a open3d.geometry.PointCloud'''

    mesh_center = mesh.get_center()
    mesh.translate(-mesh_center, relative=False)
    pcd = o3d.geometry.PointCloud()
    pcd.points = mesh.vertices
    pcd.colors = mesh.vertex_colors
    pcd.normals = mesh.vertex_normals

    return pcd

def draw_point_clouds(source_pcd: o3d.geometry.PointCloud, target_pcd: o3d.geometry.PointCloud):
    source_pcd.paint_uniform_color(SOURCE_PCD_COLOR)
    target_pcd.paint_uniform_color(TARGET_PCD_COLOR)
    o3d.visualization.draw_geometries([source_pcd_downsampled,target_pcd_downsampled])


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

 
VOXEL_SIZE_DENOMINATOR = 55 # Note: JH - Why 55??
SOURCE_PCD_COLOR = [1, 0.706, 0]
TARGET_PCD_COLOR = [0, 0.706, 1]

if __name__ == "__main__":

    source_mesh, target_mesh = load_meshes()

    source_pcd = convert_to_point_cloud(source_mesh)
    target_pcd = convert_to_point_cloud(target_mesh)
    
    #Calculate object size
    size = np.linalg.norm(np.asarray(target_pcd.get_max_bound()) - np.asarray(target_pcd.get_min_bound()))
    voxel_size = float(size / VOXEL_SIZE_DENOMINATOR)
    
    source_pcd_downsampled, source_pcd_fpfh = Preprocessing.preprocess_point_cloud(source_pcd, voxel_size)
    target_pcd_downsampled, target_pcd_fpfh = Preprocessing.preprocess_point_cloud(target_pcd, voxel_size)
    
    draw_point_clouds(source_pcd_downsampled, target_pcd_downsampled)
    
    # Global registration
    max_attempts = 20
    result_ransac = Registration.ransac_pcd_registration(source_pcd_downsampled, 
                                                         target_pcd_downsampled,
                                                         source_pcd_fpfh,
                                                         target_pcd_fpfh,
                                                         voxel_size, 
                                                         max_attempts)
    if(result_ransac == None):
        print("No ideal fit was found using the RANSAC algorithm. Cancelling further operations...")
        exit(CONST.EXIT_OK) 

    draw_registration_result(source_pcd_downsampled, target_pcd_downsampled, result_ransac.transformation)

    result_icp = Registration.refine_registration(source_pcd, target_pcd,result_ransac, voxel_size)
    print(result_icp)
    draw_registration_result(source_pcd, target_pcd, result_icp.transformation)
    
    # Deformable registration
    deformed = Registration.deformable_registration(source_mesh.transform(result_icp.transformation),target_mesh)
    deformed.compute_vertex_normals()
    target_mesh.compute_vertex_normals()
    deformed.paint_uniform_color([0,0.706, 0])
    o3d.visualization.draw_geometries([deformed, target_mesh])

    # Combine meshes (alternative - to crop the first before merging)
    combined = deformed + target_mesh
    combined.compute_vertex_normals()
    combined.paint_uniform_color([0,0.706, 0])
    o3d.visualization.draw_geometries([combined])

    # Simplify mesh (smoothing and filtering)
    mesh_smp = combined.simplify_vertex_clustering(voxel_size/3, contraction=o3d.geometry.SimplificationContraction.Average)
    mesh_smp = mesh_smp.filter_smooth_simple(number_of_iterations=2)
    mesh_smp = mesh_smp.filter_smooth_taubin(number_of_iterations=100)
    mesh_smp.compute_vertex_normals()
    o3d.visualization.draw_geometries([mesh_smp])

