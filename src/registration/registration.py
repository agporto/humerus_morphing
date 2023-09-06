import numpy as np
import open3d as o3d
import ctypes

__BCPD = ctypes.CDLL("../../bcpd/libbcpd.dylib")

class Registration:

    @staticmethod
    def ransac_pcd_registration(source_pcd_down: o3d.geometry.PointCloud, 
                                target_pcd_down: o3d.geometry.PointCloud, 
                                source_fpfh: o3d.pipelines.registration.Feature, 
                                target_fpfh: o3d.pipelines.registration.Feature,
                                voxel_size: float, 
                                ransac_max_attempts: int):
        ''' Perform a registration of nearest neighbourgs using the RANSAC algorithm.
            Distance threshold will be set as 1.5 times the voxel size.
            
            Parameters
            ----------
            source_pcd_down: Downsampled SOURCE point cloud
            target_pcd_down: Downsampled TARGET point cloud
            source_fpfh: Source PCD Fast-Point-Feature-Histogram
            target_fpfh: Target PCD Fast-Point-Feature-Histogram
            voxel_size: Difference between source and target PCD max voxel_size
            ransac_max_attempts: Maximum number of iterations of the RANSAC algorithm

            Returns
            -------
            Best PCD fit: open3d.pipelines.registration.RegistrationResult

        '''
        distance_threshold = voxel_size * 1.5
        fitness = 0
        count = 0
        best_result = None
        fitness_min, fitness_max = 0.999, 1

        while (fitness < fitness_min and fitness < fitness_max and count < ransac_max_attempts):
            result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
                source_pcd_down,
                target_pcd_down,
                source_fpfh,
                target_fpfh,
                True,
                distance_threshold,
                o3d.pipelines.registration.TransformationEstimationPointToPoint(True),
                3,
                [   
                    o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                    o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
                ],
                o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999) # NOTE: JH - Would this not make a good fit? Why is the outer loop neccessary? 
            )
            if result.fitness > fitness and result.fitness < 1:
              fitness = result.fitness
              best_result = result
            count += 1
        return best_result

    @staticmethod
    def refine_registration(source_pcd_down: o3d.geometry.PointCloud, 
                            target_pcd_down: o3d.geometry.PointCloud, 
                            # source_fpfh: o3d.pipelines.registration.Feature, 
                            # target_fpfh: o3d.pipelines.registration.Feature,
                            ransac_result: o3d.pipelines.registration.RegistrationResult,
                            voxel_size: float):
        ''' Perform a point-clouds' registration refinement.
            Distance threshold for refinement is 0.4 times voxel_size
            
            Parameters
            ----------
            source_down: Downsampled SOURCE point cloud
            target_down: Downsampled TARGET point cloud
            source_fpfh: Source PCD Fast-Point-Feature-Histogram
            target_fpfh: Target PCD Fast-Point-Feature-Histogram
            voxel_size: Difference between source and target PCD max voxel_size

        '''
        distance_threshold = voxel_size * 0.4
        print(":: Point-to-plane ICP registration is applied on original point")
        print("   clouds to refine the alignment. This time we use a strict")
        print("   distance threshold %.3f." % distance_threshold)
        result = o3d.pipelines.registration.registration_icp(
            source_pcd_down, 
            target_pcd_down, 
            distance_threshold, 
            ransac_result.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPlane())
        return result

    # @staticmethod
    # def deformable_registration(source_pcd, target_pcd):
    #     bcpd_result = __BCPD.main(targetPath, # TODO: JH - don't use the main method, rather use the 
    #                 sourcePath, 
    #                 "10", 
    #                 "10", 
    #                 "0.1"
    #                 "140",
    #                 "500",
    #                 "1e-6",
    #                 "p",
    #                 "7",
    #                 "0.3",
    #                 "0.3",
    #                 "ux", 
    #                 "DB",
    #                 "5000",
    #                 "0.08" 
    #                 "ux",
    #                 "N1")
    #     if (bcpd_result == None):
    #         print("There was an error ")
    #     deformed = o3d.geometry.TriangleMesh()
    #     deformed.vertices = o3d.utility.Vector3dVector(np.asarray(bcpd_result))
    #     deformed.triangles = source_pcd.triangles
    #     return deformed

