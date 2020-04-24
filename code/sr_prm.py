from config import *
import time
import random
from sr_neighbor_finder import NeighborsFinder
import queue

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


class PrmNode:
    def __init__(self, pt):
        self.point = pt
        self.connections = []


class PrmGraph:
    def __init__(self):
        self.points_to_nodes = {}

    def insert_edge(self, p1, p2):
        if p1 not in self.points_to_nodes.keys():
            p1_node = PrmNode(p1)
            self.points_to_nodes[p1] = p1_node
        else:
            p1_node = self.points_to_nodes[p1]
        if p2 not in self.points_to_nodes.keys():
            p2_node = PrmNode(p2)
            self.points_to_nodes[p2] = p2_node
        else:
            p2_node = self.points_to_nodes[p2]
        if p1_node == p2_node:
            return
        dist = Euclidean_distance().transformed_distance(p1_node.point, p2_node.point)
        p1_node.connections.append((p2_node, dist))
        p2_node.connections.append((p1_node, dist))

    def has_path(self, p1, p2):
        if p1 not in self.points_to_nodes.keys() or p2 not in self.points_to_nodes.keys():
            return False
        q = queue.Queue()
        visited = {p1: True}
        q.put(p1)
        while not q.empty():
            curr = q.get()
            if curr == p2:
                return True
            else:
                for next_n, _ in self.points_to_nodes[curr].connections:
                    next_p = next_n.point
                    if next_p not in visited:
                        visited[next_p] = True
                        q.put(next_p)
        return False


def two_d_point_to_2n_d_point(p):
    n = Config().general_config['ROBOTS_COUNT']
    return Point_d(2*n, [p[0], p[1]] + [FT(Gmpq(0))] * (2*(n-1)))


def xy_to_2n_d_point(x, y):
    n = Config().general_config['ROBOTS_COUNT']
    return Point_d(2*n, [x, y] + [FT(Gmpq(0))] * (2*(n-1)))


def get_min_max(obstacles):
    max_x = max(max(v.x() for v in obs) for obs in obstacles)
    max_y = max(max(v.y() for v in obs) for obs in obstacles)
    min_x = min(min(v.x() for v in obs) for obs in obstacles)
    min_y = min(min(v.y() for v in obs) for obs in obstacles)
    return max_x.to_double(), max_y.to_double(), min_x.to_double(), min_y.to_double()


def generate_milestones(cd, n, max_x, max_y, min_x, min_y):
    v = []
    while len(v) < n:
        x = FT(random.uniform(min_x, max_x))
        y = FT(random.uniform(min_y, max_y))
        if cd.is_valid_conf(Point_2(x, y)):
            v.append(xy_to_2n_d_point(x, y))
    return v


def make_graph(nn, cd, milestones):
    g = PrmGraph()
    for milestone in milestones:
        # the + 1 to number_of_neighbors is to count for count v as it's neighbor
        nearest = nn.k_nn(milestone, Config().sr_prm_config['number_of_neighbors_to_connect'])
        for neighbor in nearest[1:]:  # first point is self and no need for edge from v to itself
            if cd.path_collision_free(milestone, neighbor):
                g.insert_edge(milestone, neighbor)
    return g


def generate_graph(obstacles, origin, destination, cd):
    # start = time.time()
    origin = two_d_point_to_2n_d_point(origin)
    destination = two_d_point_to_2n_d_point(destination)
    max_x, max_y, min_x, min_y = get_min_max(obstacles)
    if not cd.is_valid_conf(origin) or not cd.is_valid_conf(destination):
        print("invalid input")
        return False
    number_of_points_to_find = Config().sr_prm_config['number_of_milestones_to_find']
    milestones = [origin, destination] + generate_milestones(cd, number_of_points_to_find, max_x, max_y, min_x, min_y)
    nn = NeighborsFinder(milestones)
    g = make_graph(nn, cd, milestones)
    if g.has_path(origin, destination):
        return True, g
    else:
        print("failed to find a valid path in prm")
        return False, PrmGraph()
