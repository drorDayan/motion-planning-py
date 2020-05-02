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
from drrt_common import *

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


class DrrtNode:
    def __init__(self, pt, pr=None):
        self.point = pt
        self.parent = pr
        self.srm_counter = Config().srm_drrt_config['mr_add_srm_once_in']
        if pr is None:
            self.cost = FT(0)
        else:
            self.cost = pr.cost + Euclidean_distance().transformed_distance(pt, pr.point)

    def get_path_to_here(self, ret_path):
        cur = self
        while cur is not None:
            ret_path.insert(0, cur.point)
            cur = cur.parent
        return ret_path


# TODO this is not the way to do it by the paper
def try_connect_to_dest(graph, neighbor_finder, dest_point, collision_detector):
    nn = neighbor_finder.get_k_nearest(dest_point, Config().drrt_config['k_nearest_to_connect_to_dest'])
    for neighbor in nn:
        free = collision_detector.path_collision_free(neighbor, dest_point)
        if free:
            graph[dest_point] = DrrtNode(dest_point, graph[neighbor])
            # print(graph[dest_point].cost)
            return True
        return False


def expand(robot_num, min_coord, max_coord, neighbor_finder, prm_g, vertices, robots_collision_detector, graph):
    num_of_points_to_add_in_expand = Config().drrt_config['num_of_points_to_add_in_expand']
    is_sparse = Config().drrt_config['use_sparse']

    for _ in range(num_of_points_to_add_in_expand):
        new_point = Point_d(2 * robot_num, [FT(random.uniform(min_coord, max_coord)) for _ in range(2 * robot_num)])
        near = neighbor_finder.get_nearest(new_point)
        new = direction_oracle(prm_g, robot_num, near, new_point, is_sparse=is_sparse)
        if new in vertices:
            continue
        free = robots_collision_detector.path_collision_free(near, new)
        if free:
            vertices.append(new)
            graph[new] = DrrtNode(new, graph[near])
            neighbor_finder.add_points([new])
        else:
            new_data = [near[j] for j in range(2 * robot_num)]
            for rid in range(robot_num):
                if prm_g[rid].points_to_nodes[sr_prm.xy_to_2n_d_point(new[2 * rid], new[2 * rid + 1])].srm_counter > 0:
                    prm_g[rid].points_to_nodes[sr_prm.xy_to_2n_d_point(new[2 * rid], new[2 * rid + 1])].srm_counter -= 1
                else:
                    new_data[2 * rid] = new[2 * rid]
                    new_data[2 * rid + 1] = new[2 * rid + 1]
                    my_new = Point_d(2 * robot_num, new_data)
                    if my_new in vertices:
                        continue
                    new_data[2 * rid] = near[2 * rid]
                    new_data[2 * rid + 1] = near[2 * rid + 1]
                    free = robots_collision_detector.srm_path_collision_free(near, my_new, rid)
                    if free:
                        vertices.append(my_new)
                        graph[my_new] = DrrtNode(my_new, graph[near])
                        neighbor_finder.add_points([my_new])
                        prm_g[rid].points_to_nodes[sr_prm.xy_to_2n_d_point(new[2 * rid], new[2 * rid + 1])].srm_counter\
                            = Config().srm_drrt_config['sr_add_srm_once_in']


def generate_path(path, robots, obstacles, destination):
    # random.seed(1)  # for tests
    # init config stuff
    start_time = time.time()
    timeout = Config().drrt_config['timeout']
    is_sparse = Config().drrt_config['use_sparse']
    robot_num = len(robots)
    robot_width = FT(1)
    min_coord, max_coord = get_min_max(obstacles)
    start_point, dest_point = get_start_and_dest(robots, destination)
    robots_collision_detector = RobotsCollisionDetector(robot_width, robot_num)
    if Config().general_config['USE_FAST_CD']:
        connector_cd = CollisionDetectorFast(robot_width, obstacles, robot_num)
    else:
        connector_cd = CollisionDetectorSlow(robot_width, obstacles, robot_num)

    validate_input(robots, destination, robot_width)

    prm_graphs = create_prm_graphs(robot_num, obstacles, start_point, dest_point, robot_width, create_sparse=is_sparse)
    if len(prm_graphs) == 0:
        return 0, 0
    print("calculated prm maps, time= ", time.time() - start_time)

    vertices = [start_point]
    graph = {start_point: DrrtNode(start_point)}
    neighbor_finder = NeighborsFinder(vertices)
    connected = False

    while not connected:
        expand(robot_num, min_coord, max_coord, neighbor_finder, prm_graphs, vertices, robots_collision_detector, graph)
        connected = try_connect_to_dest(graph, neighbor_finder, dest_point, connector_cd)
        # print("srm, time= ", time.time() - start_time, "vertices amount: ", len(vertices))
        # print_ver_graph(str(time.time() - start_time), vertices, robot_num)
        if timeout is not None and (time.time() - start_time > timeout):
            print("dRRT timed out")
            return 0, 0

    # write path to output
    d_path = []
    graph[dest_point].get_path_to_here(d_path)
    for dp in d_path:
        path.append([Point_2(dp[2*i], dp[2*i+1]) for i in range(robot_num)])
    print("finished, time= ", time.time() - start_time, "vertices amount: ", len(vertices))
    return time.time() - start_time, len(vertices)
