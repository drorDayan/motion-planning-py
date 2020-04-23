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


# noinspection PyArgumentList
class NeighborsFinder:
    def __init__(self, points):
        self.tree = Kd_tree()
        self.tree.insert(points)
        self.points = points

    def tree_k_nn(self, query, k):
        search_nearest = True
        sort_neighbors = True
        epsilon = FT(0)

        search = K_neighbor_search(self.tree, query, k, epsilon, search_nearest, Euclidean_distance(), sort_neighbors)
        lst = []
        search.k_neighbors(lst)
        return [x for x, y in lst]

    def k_nn(self, query, k):
        return self.tree_k_nn(query, k)
        sorted_vals = sorted(self.points, key=lambda a: Euclidean_distance().transformed_distance(query, a))
        return sorted_vals[:k]

