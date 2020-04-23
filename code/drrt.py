from config import *
from math import sqrt
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


class DrrtNode:
    def __init__(self, pt, pr=None, n=0):
        self.point = pt
        self.parent = pr

    def get_path_to_here(self, ret_path):
        cur = self
        while cur is not None:
            ret_path.insert(0, cur.point)
            cur = cur.parent
        return ret_path


def direction_oracle(prm_graphs, robot_num, near, new_point):
    res_arr = [0 for _ in range(2*robot_num)]
    for rid in range(robot_num):
        x = prm_graphs[rid].points_to_nodes[sr_prm.xy_to_2n_d_point(near[2*rid], near[2*rid+1])]
        next_p = random.choice(x.connections).point  # TODO this is rand, not direction
        res_arr[2*rid], res_arr[2*rid+1] = next_p[0], next_p[1]
    return Point_d(2*robot_num, res_arr)


def try_connect_to_dest(graph, neighbor_finder, dest_point, collision_detector):
    nn = neighbor_finder.get_k_nearest(dest_point, Config().drrt_config['k_nearest_to_connect_to_dest'])
    for neighbor in nn:
        free = collision_detector.path_collision_free(neighbor, dest_point)
        if free:
            graph[dest_point] = DrrtNode(dest_point, graph[neighbor])
            return True
    return False



def generate_path(path, robots, obstacles, destination):
    # random.seed(1)  # for tests
    start = time.time()
    robot_num = len(robots)
    robot_width = FT(1)
    num_of_points_to_add_in_expand = Config().drrt_config['num_of_points_to_add_in_expand']
    validate_input(robots, destination, robot_width)
    min_coord, max_coord = get_min_max(obstacles)
    start_point, dest_point = get_start_and_dest(robots, destination)
    collision_detector = RobotsCollisionDetector(robot_width, robot_num)
    prm_graphs = []
    if Config().general_config['USE_FAST_CD']:
        prm_cd = SRCollisionDetectorFast(robot_width, obstacles)
        connector_cd = CollisionDetectorFast(robot_width,obstacles, robot_num)
    else:
        prm_cd = SRCollisionDetectorSlow(robot_width, obstacles)
        connector_cd = CollisionDetectorSlow(robot_width,obstacles, robot_num)

    for rid in range(robot_num):
        is_valid, g = sr_prm.generate_graph(obstacles,
                                            Point_d(2, [start_point[rid], start_point[rid+1]]),
                                            Point_d(2, [dest_point[rid], dest_point[rid+1]]),
                                            prm_cd)
        if not is_valid:
            print("robot", rid, "failed to find a valid path in prm")
            return
        prm_graphs.append(g)
    print("finished with prm maps, time= ", time.time() - start)
    vertices = [start_point]
    graph = {start_point: DrrtNode(start_point)}
    neighbor_finder = NeighborsFinder(vertices)
    connected = False
    while not connected:
        for _ in range(num_of_points_to_add_in_expand):
            new_point = Point_d(2*robot_num, [FT(random.uniform(min_coord, max_coord)) for _ in range(2*robot_num)])
            near = neighbor_finder.get_nearest(new_point)
            new = direction_oracle(prm_graphs, robot_num, near, new_point)
            if new in vertices:
                continue
            free = collision_detector.path_collision_free(near, new)
            if free:
                vertices.append(new)
                graph[new] = DrrtNode(new, graph[near])
                neighbor_finder.add_points([new])
        if try_connect_to_dest(graph, neighbor_finder, dest_point, connector_cd):
            break
    d_path = []
    graph[dest_point].get_path_to_here(d_path)
    for dp in d_path:
        path.append([Point_2(dp[2*i], dp[2*i+1]) for i in range(robot_num)])
    print("finished, time= ", time.time() - start, "vertices amount: ", len(vertices))
