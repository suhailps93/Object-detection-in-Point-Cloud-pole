
import numpy as np
import copy
import argparse
import sys
def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    draw_geometries([source_temp, target_temp])

def draw_result(source, target):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    draw_geometries([source_temp, target_temp])

def prepare_dataset(voxel_size):
    print(":: Load two point clouds and disturb initial pose.")
    source = read_point_cloud("../pointclouds/pole1.pcd")
    target = read_point_cloud("../pointclouds/no_plane.pcd")
    trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0],
                            [1.0, 0.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0, 1.0]])

    # trans_init = np.asarray([[1.0, 0.0, 0.0, 1.0],
    #                         [0.0, 1.0, 0.0, 0.0],
    #                         [0.0, 0.0, 1.0, 0.0],
    #                         [0.0, 0.0, 0.0, 1.0]])
    source.transform(trans_init)
    draw_registration_result(source, target, np.identity(4))

    print(":: Downsample with a voxel size %.3f." % voxel_size)
    source_down = source#voxel_down_sample(source, voxel_size)
    target_down = target#voxel_down_sample(target, voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    estimate_normals(source_down, KDTreeSearchParamHybrid(
            radius = radius_normal, max_nn = 30))
    estimate_normals(target_down, KDTreeSearchParamHybrid(
            radius = radius_normal, max_nn = 30))

    # radius_feature = voxel_size * 5
    radius_feature = 10
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    source_fpfh = compute_fpfh_feature(source_down,
            KDTreeSearchParamHybrid(radius = radius_feature, max_nn = 100))
    target_fpfh = compute_fpfh_feature(target_down,
            KDTreeSearchParamHybrid(radius = radius_feature, max_nn = 100))
    return source, target, source_down, target_down, source_fpfh, target_fpfh

def execute_global_registration(
        source_down, target_down, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = registration_ransac_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh, 0.005,
            TransformationEstimationPointToPoint(False), 4,
            [CorrespondenceCheckerBasedOnEdgeLength(0.9),
            CorrespondenceCheckerBasedOnDistance(0.005)],
            RANSACConvergenceCriteria(400000, 500))
    return result

def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.4
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = registration_icp(source, target, distance_threshold,
            result_ransac.transformation,
            TransformationEstimationPointToPlane())
    return result

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description = "Description for my parser")
    parser.add_argument("-p", "--path", help = "Example: Path to py3D.so", required = False, default = "/home/suhailps/Code/Open3D/build/lib")
    argument = parser.parse_args()

    if argument.path:
        print("You have used {0} for path to py3D.so".format(argument.path))
        sys.path.append(argument.path)
        from py3d import *
        # from open3d import *
    final=read_point_cloud("../pointclouds/pole1.pcd")
    for x in xrange(1,10):

        voxel_size = 0.1 # means 5cm for the dataset
        # source, target, source_down, target_down, source_fpfh, target_fpfh = \
                # prepare_dataset(voxel_size)



        source = read_point_cloud("../pointclouds/pole"+str(x)+".pcd")
        target = read_point_cloud("../pointclouds/no_plane.pcd")
        trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0],
                                [1.0, 0.0, 0.0, 0.0],
                                [0.0, 1.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0, 1.0]])

        # trans_init = np.asarray([[1.0, 0.0, 0.0, 1.0],
        #                         [0.0, 1.0, 0.0, 0.0],
        #                         [0.0, 0.0, 1.0, 0.0],
        #                         [0.0, 0.0, 0.0, 1.0]])
        source.transform(trans_init)
        draw_registration_result(source, target, np.identity(4))

        source_down = source#voxel_down_sample(source, voxel_size)
        target_down = target#voxel_down_sample(target, voxel_size)

        radius_normal = voxel_size * 2
        print(":: Estimate normal with search radius %.3f." % radius_normal)
        estimate_normals(source_down, KDTreeSearchParamHybrid(
                radius = radius_normal, max_nn = 30))
        estimate_normals(target_down, KDTreeSearchParamHybrid(
                radius = radius_normal, max_nn = 30))

        # radius_feature = voxel_size * 5
        radius_feature = 10
        print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
        source_fpfh = compute_fpfh_feature(source_down,
                KDTreeSearchParamHybrid(radius = radius_feature, max_nn = 100))
        target_fpfh = compute_fpfh_feature(target_down,
                KDTreeSearchParamHybrid(radius = radius_feature, max_nn = 100))



        fitness=0

        while fitness < 0.9:

            result_ransac = execute_global_registration(source_down, target_down,
                    source_fpfh, target_fpfh, voxel_size)
            print(result_ransac)

            fitness=result_ransac.fitness
            draw_registration_result(source_down, target_down,
                    result_ransac.transformation)

            result_icp = refine_registration(source, target,
                    source_fpfh, target_fpfh, voxel_size)
            print(result_icp)
            fitness=result_icp.fitness
            draw_registration_result(source, target, result_icp.transformation)

            source.transform(result_icp.transformation)
            final=final+source

            draw_result(final,target)
