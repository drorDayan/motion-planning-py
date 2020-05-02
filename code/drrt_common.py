from config import *
from math import sqrt
import matplotlib.pyplot as plt
import random
import time
from neighbor_finder import NeighborsFinder
from rrt_common import *
from drrt_collision_detector import RobotsCollisionDetector
import sr_prm
from sr_collision_detector import SRCollisionDetectorSlow, SRCollisionDetectorFast
from collision_detector import CollisionDetectorFast, CollisionDetectorSlow

ROBOTS_COUNT = Config().general_config['ROBOTS_COUNT']
if ROBOTS_COUNT is None:
    from arr2_epec_seg_ex import *
elif ROBOTS_COUNT == 2:
    from libs.release_cgal_binddings.d4.arr2_epec_seg_ex import *
elif ROBOTS_COUNT == 3:
    from libs.release_cgal_binddings.d6.arr2_epec_seg_ex import *
elif ROBOTS_COUNT == 4:
    from libs.release_cgal_binddings.d8.arr2_epec_seg_ex import *
elif ROBOTS_COUNT == 5:
    from libs.release_cgal_binddings.d10.arr2_epec_seg_ex import *
elif ROBOTS_COUNT == 6:
    from libs.release_cgal_binddings.d12.arr2_epec_seg_ex import *
elif ROBOTS_COUNT == 7:
    from libs.release_cgal_binddings.d14.arr2_epec_seg_ex import *
elif ROBOTS_COUNT == 8:
    from libs.release_cgal_binddings.d16.arr2_epec_seg_ex import *
elif ROBOTS_COUNT == 9:
    from libs.release_cgal_binddings.d18.arr2_epec_seg_ex import *
elif ROBOTS_COUNT == 10:
    from libs.release_cgal_binddings.d20.arr2_epec_seg_ex import *


def direction_oracle(prm_graphs, robot_num, near, new_point, is_sparse=False):
    direction = [new_point[i]-near[i] for i in range(2*robot_num)]
    res_arr = [0 for _ in range(2*robot_num)]
    for rid in range(robot_num):
        next_p = prm_graphs[rid].sr_direction_oracle(sr_prm.xy_to_2n_d_point(near[2*rid], near[2*rid+1]),
                                                     sr_prm.xy_to_2n_d_point(direction[2*rid], direction[2*rid+1]),
                                                     is_sparse)
        # x = prm_graphs[rid].points_to_nodes[sr_prm.xy_to_2n_d_point(near[2*rid], near[2*rid+1])]
        # next_p = random.choice(list(x.connections.keys())).point  # TODO this is rand, not direction
        res_arr[2*rid], res_arr[2*rid+1] = next_p[0], next_p[1]
    return Point_d(2*robot_num, res_arr)


def create_prm_graphs(robot_num, obstacles, start_point, dest_point, robot_width, create_sparse=False):
    if Config().general_config['USE_FAST_CD']:
        prm_cd = SRCollisionDetectorFast(robot_width, obstacles)
    else:
        prm_cd = SRCollisionDetectorSlow(robot_width, obstacles)
    prm_graphs = []
    for rid in range(robot_num):
        is_valid, g = sr_prm.generate_graph(obstacles,
                                            Point_d(2, [start_point[2*rid], start_point[2*rid+1]]),
                                            Point_d(2, [dest_point[2*rid], dest_point[2*rid+1]]),
                                            prm_cd, create_sparse)
        if not is_valid:
            print("robot", rid, "failed to find a valid path in prm")
            return []
        prm_graphs.append(g)
    return prm_graphs


def print_ver_graph(c_time, ver_list, robot_num):
    for i in range(robot_num):
        x_s = [v[2*i].to_double() for v in ver_list]
        y_s = [v[2*i+1].to_double() for v in ver_list]
        plt.plot(x_s, y_s, 'o', color='black')
        plt.savefig("temp/R"+str(i)+"T"+c_time+".png")
        plt.close()
