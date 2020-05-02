import random
import time
from collision_detector import CollisionDetectorFast, CollisionDetectorSlow
from neighbor_finder import NeighborsFinder
from rrt_common import *
from config import *


class RrtNode:
    def __init__(self, pt, pr=None, n=0):
        self.point = pt
        self.parent = pr
        self.nval = n  # Will be used to store distance to this point, etc.

    def get_path_to_here(self, ret_path):
        cur = self
        while cur is not None:
            ret_path.insert(0, cur.point)
            cur = cur.parent
        return ret_path


def try_connect_to_dest(graph, neighbor_finder, dest_point, collision_detector):
    nn = neighbor_finder.get_k_nearest(dest_point, Config().rrt_config['k_nearest_to_connect_to_dest'])
    for neighbor in nn:
        free = collision_detector.path_collision_free(neighbor, dest_point)
        if free:
            graph[dest_point] = RrtNode(dest_point, graph[neighbor])
            return True
    return False


# slower then old impl, why? TODO check this
def generate_path(path, robots, obstacles, destination):
    # random.seed(1)  # for tests
    start_time = time.time()
    timeout = Config().rrt_config['timeout']
    robot_num = len(robots)
    robot_width = FT(1)
    steer_eta = FT(Config().rrt_config['steer_eta'])
    validate_input(robots, destination, robot_width)
    min_coord, max_coord = get_min_max(obstacles)
    start_point, dest_point = get_start_and_dest(robots, destination)
    collision_detector = CollisionDetectorFast(robot_width, obstacles, robot_num)

    vertices = [start_point]
    graph = {start_point: RrtNode(start_point)}
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
        if timeout is not None and (time.time() - start_time > timeout):
            print("rrt timed out")
            return 0, 0

    d_path = []
    graph[dest_point].get_path_to_here(d_path)
    for dp in d_path:
        path.append([Point_2(dp[2*i], dp[2*i+1]) for i in range(robot_num)])
    finish_time = time.time() - start_time
    print("finished, time= ", finish_time, "vertices amount: ", len(vertices), "steer_eta = ", steer_eta)
    return finish_time, len(vertices)
