from config import *
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
import heapq


# TODO find best
smallest_tree_size = 200
tree_size_mul = 100


# noinspection PyArgumentList
class NeighborsFinder:
    def __init__(self, points):
        # trees array will have max size, points_array, kd_tree
        self.trees = [[smallest_tree_size, points, Kd_tree(points)]]

    def __tree_k_nn(self, query, k):
        search_nearest = True
        sort_neighbors = True
        epsilon = FT(0)
        res = []
        for _, points_array, kd_tree in self.trees:
            if len(points_array) > 0:
                search = K_neighbor_search(kd_tree, query, k, epsilon, search_nearest, Euclidean_distance(),
                                           sort_neighbors)
                lst = []
                search.k_neighbors(lst)
                res += lst
        return res

    def add_points(self, points):
        num_of_points = len(points)
        i = 0
        while i < len(self.trees):
            if self.trees[i][0] > num_of_points+len(self.trees[i][1]):
                points_to_add = points
                for j in range(0, i):
                    points_to_add += self.trees[j][1]
                    self.trees[j][1] = []
                    self.trees[j][2] = Kd_tree([])
                self.trees[i][1] += points_to_add
                self.trees[i][2].insert(points_to_add)
                return
            num_of_points += len(self.trees[i][1])
            i += 1
        points_to_add = points
        for i in range(0, len(self.trees)):
            points_to_add += self.trees[i][1]
            self.trees[i][1] = []
            self.trees[i][2] = Kd_tree([])
        self.trees.append([self.trees[len(self.trees) - 1][0] * tree_size_mul, points_to_add, Kd_tree(points_to_add)])
        # print(self.trees[len(self.trees) - 1][0])
        return

    def get_nearest(self, point):
        nn = self.__tree_k_nn(point, 1)
        nn_min = min(nn, key=lambda n: n[1])
        return nn_min[0]

    def get_k_nearest(self, point, k):
        nn = self.__tree_k_nn(point, k)
        heap_nn = [(d, n) for n, d in nn]
        heapq.heapify(heap_nn)
        return [heapq.heappop(heap_nn)[1] for _ in range(min(k, len(heap_nn)))]


