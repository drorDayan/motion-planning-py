import arr2_epec_seg_ex as srpb


# noinspection PyArgumentList
class NeighborsFinder:
    def __init__(self, points):
        self.tree = srpb.Kd_tree()
        self.tree.insert(points)
        self.points = points

    def tree_k_nn(self, query, k):
        search_nearest = True
        sort_neighbors = True
        epsilon = srpb.FT(0)

        search = srpb.K_neighbor_search(self.tree, query, k, epsilon, search_nearest, srpb.Euclidean_distance(),
                                        sort_neighbors)
        lst = []
        search.k_neighbors(lst)
        return [x for x, y in lst]

    def k_nn(self, query, k):
        return self.tree_k_nn(query, k)
        sorted_vals = sorted(self.points, key=lambda a: srpb.Euclidean_distance().transformed_distance(query, a))
        return sorted_vals[:k]

