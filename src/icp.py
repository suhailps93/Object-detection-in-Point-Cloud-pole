
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

if __name__ == "__main__":


    parser = argparse.ArgumentParser(description = "Description for my parser")
    parser.add_argument("-p", "--path", help = "Example: Path to py3D.so", required = False, default = "/home/suhailps/Code/Open3D/build/lib")
    argument = parser.parse_args()

    if argument.path:
        print("You have used {0} for path to py3D.so".format(argument.path))
        sys.path.append(argument.path)
        from py3d import *


    source = read_point_cloud("../pole.pcd")
    target = read_point_cloud("../final_project_point_cloud.pcd")
    threshold = 0.2
    trans_init = np.asarray(
                [[0.862, 0.011, -0.507,  0.5],
                [-0.139, 0.967, -0.215,  0.7],
                [0.487, 0.255,  0.835, -1.4],
                [0.0, 0.0, 0.0, 1.0]])
    # trans_init = np.asarray(
    #             [[1.0, 0.0, 0.0,  0.0],
    #             [0.0, 1.0, 0.0,  0.0],
    #             [0.0,0.0,  1.0, 0.0],
    #             [0.0, 0.0, 0.0, 1.0]])
    draw_registration_result(source, target, trans_init)
    print("Initial alignment")
    evaluation = evaluate_registration(source, target,
            threshold, trans_init)
    print(evaluation)

    print("Apply point-to-point ICP")


    reg_p2p = registration_icp(source, target, threshold, trans_init,
            TransformationEstimationPointToPoint())
    print(reg_p2p)
    print("Transformation is:")
    print(reg_p2p.transformation)
    print("")
    draw_registration_result(source, target, reg_p2p.transformation)

    print("Apply point-to-plane ICP")
    fitness=0
    while fitness<0.8:
        reg_p2l = registration_icp(source, target, threshold, trans_init,
                TransformationEstimationPointToPlane())
        print(reg_p2l)
        print("Transformation is:")
        print(reg_p2l.fitness)
        fitness=reg_p2l.fitness
        print("")
        draw_registration_result(source, target, reg_p2l.transformation)
