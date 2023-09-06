import open3d as o3d

class Preprocessing:
    
    @staticmethod
    def preprocess_point_cloud(pcd: o3d.geometry.PointCloud, voxel_size: float): 
        ''' 
            Perform downsampling of a mesh, normal estimation and computing FPFH feature of the point cloud.

            Returns
            -------
            Tuple: [Downsampled PCD: open3d.geometry.PointCloud,
                    FPFH: open3d.pipelines.registration.Feature]
        '''
        # NOTE: JH - Why these constants? 
        radius_normal = voxel_size * 4
        radius_feature = voxel_size * 10
        max_nn_normals = 30
        max_nn_fpfh = 100

        print("Downsampling mesh with a voxel size %.3f." % voxel_size)
        pcd_down: o3d.geometry.PointCloud = pcd.voxel_down_sample(voxel_size)

        print("Estimating normal with search radius %.3f." % radius_normal)
        pcd_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=max_nn_normals))

        print("Compute FPFH feature with search radius %.3f." % radius_feature)
        pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(pcd_down, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=max_nn_fpfh))

        return pcd_down, pcd_fpfh

