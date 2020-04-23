from config import *
from math import sqrt
import arr2_epec_seg_ex as srpb
import random
import time
from neighbor_finder import NeighborsFinder
from rrt_common import *
from drrt_collision_detector import RobotsCollisionDetector
import sr_prm


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


def generate_path(path, robots, obstacles, destination):
    # random.seed(1)  # for tests
    start = time.time()
    robot_num = len(robots)
    robot_width = FT(1)
    steer_eta = FT(Config().drrt_config['steer_eta'])
    validate_input(robots, destination, robot_width)
    min_coord, max_coord = get_min_max(obstacles)
    start_point, dest_point = get_start_and_dest(robots, destination)
    collision_detector = RobotsCollisionDetector(robot_width, robot_num)
    prm_graphs = []

    obstacles0 = [[srpb.Point_2(srpb.FT(p.x().to_double()), srpb.FT(p.y().to_double())) for p in obs] for obs in obstacles]
    for rid in range(robot_num):
        is_valid, g = sr_prm.generate_graph(obstacles0,
                                            srpb.Point_d(2, [srpb.FT(start_point[rid].to_double()),
                                                         srpb.FT(start_point[rid+1].to_double())]),
                                            srpb.Point_d(2, [srpb.FT(dest_point[rid].to_double()),
                                                         srpb.FT(dest_point[rid+1].to_double())]),
                                            srpb.FT(robot_width.to_double()))
        if not is_valid:
            print("robot:", rid, "failed to find a valid path in prm")
            return
        prm_graphs.append(g)
    vertices = [start_point]
    graph = {start_point: DrrtNode(start_point)}
    neighbor_finder = NeighborsFinder(vertices)
    i = 0
    while True:
        i += 1
        new_point = Point_d(2*robot_num, [FT(random.uniform(min_coord, max_coord)) for _ in range(2*robot_num)])
        while not collision_detector.is_valid_conf(new_point):  # this hurts performance hard!
            new_point = Point_d(2 * robot_num, [FT(random.uniform(min_coord, max_coord)) for _ in range(2 * robot_num)])
        near = neighbor_finder.get_nearest(new_point)
        new = steer(robot_num, near, new_point, steer_eta)
        free = collision_detector.path_collision_free(near, new)
        if free:
            vertices.append(new)
            graph[new] = RrtNode(new, graph[near])
            neighbor_finder.add_points([new])
        if i % 100 == 0:
            if try_connect_to_dest(graph, neighbor_finder, dest_point, collision_detector):
                break
    d_path = []
    graph[dest_point].get_path_to_here(d_path)
    for dp in d_path:
        path.append([Point_2(dp[2*i], dp[2*i+1]) for i in range(robot_num)])
    print("finished, time= ", time.time() - start, "vertices amount: ", len(vertices), "steer_eta = ", steer_eta)
