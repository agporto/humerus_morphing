import numpy as np
import open3d as o3d
import ctypes
import glob
import os

BCPD = ctypes.CDLL("./bcpd/libbcpd.dylib") # Note: Needs to be relative to the main run script, not this one

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

    @staticmethod
    def deformable_registration(source_pcd, target_pcd):
        sourceArray = np.asarray(source_pcd.vertices,dtype=np.float32)
        targetArray = np.asarray(target_pcd.vertices,dtype=np.float32)
        targetPath = './target.txt'
        sourcePath = './source.txt'
        np.savetxt (targetPath, targetArray, delimiter=',')
        np.savetxt (sourcePath, sourceArray, delimiter=',')

        cmd = f'bcdp -x {targetPath} -y {sourcePath} -l10 -b10 -g0.1 -K140 -J500 -c1e-6 -p -d7 -e0.3 -f0.3 -ux -DB,5000,0.08 -ux -N1'

        # Split the cmd string into individual arguments
        cmd_parts = cmd.split()

        # Convert the Python list of arguments to a C-style array of char pointers
        argv_c = (ctypes.c_char_p * len(cmd_parts))()
        argv_c[:] = [arg.encode('utf-8') for arg in cmd_parts]

        # Set argc to the number of arguments
        argc = len(cmd_parts)

        BCPD.main.argtypes = (ctypes.c_int, ctypes.POINTER(ctypes.c_char_p))

        # TODO: JH - Better usage, rather than the main method
        # Might be considered in the future, but this should be enough, for now
        BCPD.main(argc, argv_c)

        bcpd_result = np.loadtxt('output_y.interpolated.txt')

        for fl in glob.glob("output*.txt"):
            os.remove(fl)
        os.remove(targetPath)
        os.remove(sourcePath)

        deformed = o3d.geometry.TriangleMesh()
        deformed.vertices = o3d.utility.Vector3dVector(np.asarray(bcpd_result))
        deformed.triangles = source_pcd.triangles
        return deformed

