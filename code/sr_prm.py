from config import *
import time
import random
from sr_neighbor_finder import NeighborsFinder
import queue
import heapq
import math
import numpy as np
import matplotlib.pyplot as plt

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
    def __init__(self, pt, is_sparse=False):
        self.point = pt
        self.connections = {}
        self.bfs_dist_from_t = None
        self.father_in_bfs_dist_from_t = None
        self.real_dist_from_t = None
        self.father_in_dist_from_t = None
        self.srm_counter = Config().srm_drrt_config['sr_add_srm_once_in']
        self.is_sparse = is_sparse
        self.sparse_connections = {}
        self.sparse_rep = None
        self.rep_of = {}


class PrmGraph:
    def __init__(self):
        self.points_to_nodes = {}

    def add_node(self, p, is_sparse=False):
        if p not in self.points_to_nodes.keys():
            p1_node = PrmNode(p, is_sparse)
            self.points_to_nodes[p] = p1_node

    def insert_edge(self, p1, p2, is_sparse=False):
        if p1 not in self.points_to_nodes.keys():
            p1_node = PrmNode(p1, is_sparse=is_sparse)
            self.points_to_nodes[p1] = p1_node
        else:
            p1_node = self.points_to_nodes[p1]
        if p2 not in self.points_to_nodes.keys():
            p2_node = PrmNode(p2, is_sparse=is_sparse)
            self.points_to_nodes[p2] = p2_node
        else:
            p2_node = self.points_to_nodes[p2]
        if p1_node == p2_node:
            return
        dist = math.sqrt(Euclidean_distance().transformed_distance(p1_node.point, p2_node.point).to_double())
        if is_sparse:
            p1_node.sparse_connections[p2_node] = dist
            p2_node.sparse_connections[p1_node] = dist
        else:
            p1_node.connections[p2_node] = dist
            p2_node.connections[p1_node] = dist

    def has_path(self, p1, p2, is_sparse=False):
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
                if is_sparse:
                    keys = self.points_to_nodes[curr].sparse_connections.keys()
                else:
                    keys = self.points_to_nodes[curr].connections.keys()
                for next_n in keys:
                    next_p = next_n.point
                    if next_p not in visited:
                        visited[next_p] = True
                        q.put(next_p)
        return False

    def calc_bfs_dist_from_t(self, t):
        if t not in self.points_to_nodes.keys():
            return False
        self.points_to_nodes[t].bfs_dist_from_t = 0
        temp_i = 0
        q = [(0, temp_i, t)]
        heapq.heapify(q)
        visited = {t: True}
        while len(q) > 0:
            c_dist, _, curr = heapq.heappop(q)
            curr_n = self.points_to_nodes[curr]
            for next_n in curr_n.connections.keys():
                next_p = next_n.point
                if next_p not in visited:
                    next_n.bfs_dist_from_t = c_dist + 1
                    next_n.father_in_bfs_dist_from_t = curr_n
                    visited[next_p] = True
                    temp_i += 1
                    heapq.heappush(q, (next_n.bfs_dist_from_t, temp_i, next_p))
        return True

    def calc_real_dist_from_t(self, t):
        if t not in self.points_to_nodes.keys():
            return False
        self.points_to_nodes[t].real_dist_from_t = 0
        temp_i = 0
        q = [(0, temp_i, t)]
        heapq.heapify(q)
        done = {}
        while len(q) > 0:
            c_dist, _, curr = heapq.heappop(q)
            done[curr] = True
            curr_n = self.points_to_nodes[curr]
            for next_n in curr_n.connections.keys():
                next_p = next_n.point
                if next_p not in done:
                    temp_i += 1
                    alt = c_dist + next_n.connections[curr_n]
                    if next_n.real_dist_from_t is None or alt < next_n.real_dist_from_t:
                        next_n.real_dist_from_t = alt
                        next_n.father_in_dist_from_t = curr_n
                        heapq.heappush(q, (next_n.real_dist_from_t, temp_i, next_p))
        return True

    def sr_direction_oracle(self, source, direction):
        direction_vec = np.array([direction[0].to_double(), direction[1].to_double()])
        direction_norm = np.linalg.norm(direction_vec)
        source_n = self.points_to_nodes[source]
        max_cos = -1
        found_neighbor = None
        for neighbor in source_n.connections.keys():
            neighbor_vec = np.array([neighbor.point[0].to_double()-source[0].to_double(),
                                     neighbor.point[1].to_double()-source[1].to_double()])
            neighbor_norm = np.linalg.norm(neighbor_vec)
            neighbor_res = abs(np.dot(direction_vec, neighbor_vec)/(direction_norm * neighbor_norm))
            if neighbor_res > max_cos:
                max_cos = neighbor_res
                found_neighbor = neighbor.point
        # the else in the next line will only happen if source is dest and not connected to anything
        return found_neighbor if found_neighbor is not None else source

    # def update_sparse_rep(self, old_p, new_p, cd):
    #     items_to_remove = []
    #     for v, d in self.points_to_nodes[old_p].rep_of.items():
    #         dist_to_new = Euclidean_distance().transformed_distance(v.point, new_p)
    #         if dist_to_new < d and cd.path_collision_free(v.point, new_p):
    #             self.points_to_nodes[new_p].rep_of[v] = dist_to_new
    #             items_to_remove.append(v)
    #             v.sparse_rep = new_p
    #     for v in items_to_remove:
    #         self.points_to_nodes[old_p].rep_of.pop(v)


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


def make_graph(cd, milestones, origin, destination):
    sparse_nn = NeighborsFinder([origin, destination])
    milestones += [origin, destination]
    nn = NeighborsFinder(milestones)
    g = PrmGraph()
    # init sparse graph
    if cd.path_collision_free(origin, destination):
        g.insert_edge(origin, destination, is_sparse=True)
        # print("0od:", origin, destination)
    else:
        g.add_node(origin, is_sparse=True)
        g.add_node(destination, is_sparse=True)
        # print("1od:", origin, destination)

    for milestone in milestones:
        # the + 1 to number_of_neighbors is to count for count v as it's neighbor
        nearest = nn.k_nn(milestone, Config().sr_prm_config['number_of_neighbors_to_connect'])
        for neighbor in nearest[1:]:  # first point is self and no need for edge from v to itself
            if cd.path_collision_free(milestone, neighbor):
                g.insert_edge(milestone, neighbor)

        # from here we build the sparse graph
        if g.points_to_nodes[milestone].is_sparse:
            # to remove origin and dest
            continue
        s_ns = sparse_nn.neighbors_in_radius(milestone, FT(Config().sr_prm_config['sparse_radius']))
        ok_s_ns = []
        best_s_n = None
        for s_n in s_ns:
            if s_n == milestone:
                continue
            if cd.path_collision_free(milestone, s_n):
                ok_s_ns.append(s_n)
                dist = Euclidean_distance().transformed_distance(milestone, s_n)
                if best_s_n is None or best_s_n[1] < dist:
                    best_s_n = (s_n, dist)
        if best_s_n is not None:
            g.points_to_nodes[milestone].sparse_rep = best_s_n[0]
            g.points_to_nodes[best_s_n[0]].rep_of[g.points_to_nodes[milestone]] = best_s_n[1]
        # if no one is close this is now a sparse milestone
        if best_s_n is None:
            sparse_nn.add_point(milestone)
            g.points_to_nodes[milestone].is_sparse = True
            for s_n in s_ns:
                if s_n == milestone:
                    continue
                # g.update_sparse_rep(s_n, milestone, cd)
            # print("1:", milestone)
            continue
        # add bridge in sparse spanner
        is_bridge = False
        for i in range(len(ok_s_ns)):
            for j in range(i + 1, len(ok_s_ns)):
                # TODO use union-find, this is very in efficient
                if not g.has_path(ok_s_ns[i], ok_s_ns[j], is_sparse=True):
                    is_bridge = True
                    if cd.path_collision_free(ok_s_ns[i], ok_s_ns[j]):
                        g.insert_edge(ok_s_ns[i], ok_s_ns[j], is_sparse=True)
                        # print("e:", ok_s_ns[i], ok_s_ns[j])
                        # print_sr_sparse_graph(g)
                    else:
                        if not g.points_to_nodes[milestone].is_sparse:
                            g.points_to_nodes[milestone].is_sparse = True
                            sparse_nn.add_point(milestone)
                            # print("2:", milestone)
                        g.insert_edge(ok_s_ns[i], milestone, is_sparse=True)
                        g.insert_edge(ok_s_ns[j], milestone, is_sparse=True)
                        # g.update_sparse_rep(ok_s_ns[i], milestone, cd)
                        # g.update_sparse_rep(ok_s_ns[j], milestone, cd)
                        # print_sr_sparse_graph(g)
        if is_bridge:
            continue
        # # pair spanners
        # m_rep = best_s_n[0]
        # for c_n in g.points_to_nodes[milestone].connections.keys():
        #     n_rep = c_n.sparse_rep
        #     if n_rep is None:
        #         continue
        #     if m_rep != n_rep and\
        #             not g.points_to_nodes[n_rep] in g.points_to_nodes[m_rep].sparse_connections.keys():
        #         if cd.path_collision_free(m_rep, n_rep):
        #             g.insert_edge(m_rep, n_rep, is_sparse=True)
        #             # print("e_ps:", m_rep, n_rep)
        #             print_sr_sparse_graph(g)
        #         else:
        #             if not g.points_to_nodes[milestone].is_sparse:
        #                 g.points_to_nodes[milestone].is_sparse = True
        #                 sparse_nn.add_point(milestone)
        #                 print("3a:", milestone)
        #             if not c_n.is_sparse:
        #                 c_n.is_sparse = True
        #                 sparse_nn.add_point(c_n.point)
        #                 print("3b:", c_n.point)
        #             g.insert_edge(m_rep, milestone, is_sparse=True)
        #             g.insert_edge(c_n.sparse_rep, c_n.point, is_sparse=True)
        #             g.insert_edge(milestone, c_n.point, is_sparse=True)
        #             g.update_sparse_rep(m_rep, milestone, cd)
        #             g.update_sparse_rep(c_n.sparse_rep, c_n.point, cd)
        #             g.update_sparse_rep(milestone, c_n.point, cd)
        #             g.update_sparse_rep(c_n.point, milestone, cd)
        #
        #             print_sr_sparse_graph(g)

    return g


def print_sr_sparse_graph(g):
    for v in g.points_to_nodes.values():
        if v.is_sparse:
            plt.plot([v.point[0].to_double()], [v.point[1].to_double()], 'o', color='black')
            # print("p:", v.point, "con", len(v.sparse_connections))
            for con in v.sparse_connections.keys():
                plt.plot([v.point[0].to_double(), con.point[0].to_double()],
                         [v.point[1].to_double(), con.point[1].to_double()])
    plt.savefig("temp/sparse_graph, T" + str(time.time()) + ".png")
    plt.close()


def generate_graph(obstacles, origin, destination, cd):
    # start = time.time()
    origin = two_d_point_to_2n_d_point(origin)
    destination = two_d_point_to_2n_d_point(destination)
    max_x, max_y, min_x, min_y = get_min_max(obstacles)
    if not cd.is_valid_conf(origin) or not cd.is_valid_conf(destination):
        print("invalid input")
        return False
    number_of_points_to_find = Config().sr_prm_config['number_of_milestones_to_find']
    milestones = generate_milestones(cd, number_of_points_to_find, max_x, max_y, min_x, min_y)
    g = make_graph(cd, milestones, origin, destination)
    if g.has_path(origin, destination):
        g.calc_bfs_dist_from_t(destination)
        g.calc_real_dist_from_t(destination)
        # print_sr_sparse_graph(g)
        return True, g
    else:
        print("failed to find a valid path in prm")
        return False, PrmGraph()
