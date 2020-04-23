import arr2_epec_seg_ex as srpb


# noinspection PyArgumentList
class NeighborsFinder:
    def __init__(self, points):
        self.tree = srpb.Kd_tree(points)

    def tree_k_nn(self, query, k):
        search_nearest = True
        sort_neighbors = True
        epsilon = srpb.FT(0)

        search = srpb.K_neighbor_search(self.tree, query, k, epsilon, search_nearest, srpb.Euclidean_distance(),
                                        sort_neighbors)
        lst = []
        search.k_neighbors(lst)
        return lst

