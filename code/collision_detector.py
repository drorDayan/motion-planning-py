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
from math import sqrt

FREESPACE = 'freespace'
DENSE = 'dense'
DENSE_VISITED = 'dense_visited'


# noinspection PyArgumentList
def polygon_with_holes_to_arrangement(poly):
    assert isinstance(poly, Polygon_with_holes_2)
    arr = Arrangement_2()
    insert(arr, [Curve_2(edge) for edge in poly.outer_boundary().edges()])

    # set the freespace flag for the only current two faces
    for f in arr.faces():
        assert isinstance(f, Face)
        f.set_data({FREESPACE: f.is_unbounded()})

    for hole in poly.holes():
        insert(arr, [Curve_2(edge) for edge in hole.edges()])

    for f in arr.faces():
        assert isinstance(f, Face)
        if f.data() is None:
            f.set_data({FREESPACE: True})
    return arr


def get_origin_robot_coord(width):
    robot_width = width / FT(2)
    v1 = Point_2(robot_width, robot_width)
    v2 = Point_2(robot_width * FT(-1), robot_width)
    v3 = Point_2(robot_width * FT(-1), robot_width * FT(-1))
    v4 = Point_2(robot_width, robot_width * FT(-1))
    return [v1, v2, v3, v4]


def merge_faces_by_freespace_flag(x, y):
    return {FREESPACE: x[FREESPACE] and y[FREESPACE]}


# noinspection PyArgumentList
def overlay_multiple_arrangements(arrs, face_merge_func):
    final_arr = arrs[0]
    for arr in arrs[1:]:
        temp_res = Arrangement_2()

        overlay(final_arr, arr, temp_res, Arr_face_overlay_traits(face_merge_func))
        final_arr = temp_res
    return final_arr


# noinspection PyArgumentList
def is_in_free_face(point_locator, point):
    face = Face()
    # locate can return a vertex or an edge or a face
    located_obj = point_locator.locate(point)
    # if located_obj.is_vertex():
    #     return False
    # if located_obj.is_halfedge():
    #     return False
    if located_obj.is_face():
        located_obj.get_face(face)
        return face.data()[FREESPACE]
    return False


# noinspection PyArgumentList
def vertical_decompose(arr):
    assert isinstance(arr, Arrangement_2)
    d = []
    verticals = Arrangement_2()
    decompose(arr, d)
    for pair in d:
        # pair is a tuple
        # pair[0] is an arrangement vertex
        # pair[1] is a pair holding the objects (vertex, halfedge, or face) above and below the vertex,
        # that is, the objects hit by the vertical walls emanating from the vertex
        v0 = pair[0]
        for obj in pair[1]:
            if obj.is_vertex():
                v1 = Vertex()
                obj.get_vertex(v1)
                insert(verticals, Curve_2(Segment_2(v0.point(), v1.point())))
            elif obj.is_halfedge():
                he = Halfedge()
                obj.get_halfedge(he)
                v1 = Point_2(v0.point().x(), he.curve().line().y_at_x(v0.point().x()))
                insert(verticals, Curve_2(Segment_2(v0.point(), v1)))
            else:  # obj is a face
                # can only happen for the vertices of the bbox, so IGNORE
                pass

    res = Arrangement_2()
    for f in verticals.faces():
        f.set_data({FREESPACE: True})
    overlay(arr, verticals, res, Arr_face_overlay_traits(merge_faces_by_freespace_flag))

    return res


# obs collision detection code:
# noinspection PyArgumentList
class CollisionDetectorFast:
    def __init__(self, robot_width, obstacles, robot_num):
        # init obs for collision detection
        one_width_square = Polygon_2(get_origin_robot_coord(robot_width))
        double_width_square = Polygon_with_holes_2(Polygon_2(get_origin_robot_coord(FT(2) * robot_width)))
        inflated_obstacles = [Polygon_2([p for p in obs]) for obs in obstacles]
        c_space_obstacles = [minkowski_sum_by_full_convolution_2(one_width_square, obs) for obs in inflated_obstacles]
        c_space_arrangements = [polygon_with_holes_to_arrangement(obs) for obs in c_space_obstacles]
        self.obstacles_arrangement = overlay_multiple_arrangements(c_space_arrangements, merge_faces_by_freespace_flag)
        self.obstacles_point_locator = Arr_trapezoid_ric_point_location(self.obstacles_arrangement)
        self.double_width_square_arrangement = polygon_with_holes_to_arrangement(double_width_square)
        self.double_width_square_point_locator = Arr_trapezoid_ric_point_location(self.double_width_square_arrangement)
        self.robot_num = robot_num
        self.robot_width = robot_width.to_double()

    @staticmethod
    def get_normal_movement_vector(p1, p2, i, j):
        start_point = [p1[2*i] - p1[2*j], p1[2*i+1] - p1[2*j+1]]
        diff_vec = [(p2[2*i] - p1[2*i])-(p2[2*j] - p1[2*j]), (p2[2*i+1] - p1[2*i+1])-(p2[2*j+1] - p1[2*j+1])]
        normal_movement_vector = Curve_2(Point_2(start_point[0], start_point[1]),
                                         Point_2(start_point[0]+diff_vec[0], start_point[1]+diff_vec[1]))
        return normal_movement_vector

    def __two_robot_intersect(self, p1, p2, i, j):
        mov_vec = self.get_normal_movement_vector(p1, p2, i, j)
        return do_intersect(self.double_width_square_arrangement, mov_vec)

    # checks for collisions return:
    # True if collision free
    # False, if not
    def path_collision_free(self, p1, p2):
        # check for obs collision
        for i in [i for i in range(self.robot_num)]:
            if do_intersect(self.obstacles_arrangement, Curve_2(Point_2(p1[2*i], p1[2*i+1]),
                                                                Point_2(p2[2*i], p2[2*i+1]))):
                return False
        # check for robot to robot collision
        for i in range(self.robot_num):
            for j in range(i + 1, self.robot_num):
                if self.__two_robot_intersect(p1, p2, i, j):
                    return False
        return True

    # checks for collisions return:
    # True if collision free
    # False, if not
    def srm_path_collision_free(self, p1, p2, rid):
        # check for obs collision
        if do_intersect(self.obstacles_arrangement, Curve_2(Point_2(p1[2*rid], p1[2*rid+1]),
                                                            Point_2(p2[2*rid], p2[2*rid+1]))):
            return False
        # check for robot to robot collision
        for j in range(self.robot_num):
            if j == rid:
                continue
            if self.__two_robot_intersect(p1, p2, rid, j):
                return False
        return True

    def is_valid_conf(self, p):
        for robot_index in range(self.robot_num):
            if not is_in_free_face(self.obstacles_point_locator, Point_2(p[robot_index*2], p[robot_index*2+1])):
                return False
        for r1 in range(self.robot_num):
            for r2 in range(r1+1, self.robot_num):
                if abs(FT.to_double(p[2 * r1] - p[2 * r2])) < self.robot_width and \
                        abs(FT.to_double(p[2 * r1 + 1] - p[2 * r2 + 1])) < self.robot_width:
                    return False
        return True


# noinspection PyArgumentList
class CollisionDetectorSlow:
    inflation_epsilon = FT(0.01)

    def __init__(self, robot_width, obstacles, robot_num):
        self.robot_num = robot_num
        self.robot_width = robot_width.to_double()
        inf_sq_coord = (robot_width+CollisionDetectorSlow.inflation_epsilon)/FT(2)
        v1 = Point_2(inf_sq_coord, inf_sq_coord)
        v2 = Point_2(inf_sq_coord * FT(-1), inf_sq_coord)
        v3 = Point_2(inf_sq_coord * FT(-1), inf_sq_coord*FT(-1))
        v4 = Point_2(inf_sq_coord, inf_sq_coord * FT(-1))
        inflated_square = Polygon_2([v1, v2, v3, v4])
        cgal_obstacles = [Polygon_2([p for p in obs]) for obs in obstacles]
        c_space_obstacles = [minkowski_sum_by_full_convolution_2(inflated_square, obs) for obs in cgal_obstacles]
        arrangements = [polygon_with_holes_to_arrangement(obs) for obs in c_space_obstacles]
        single_arrangement = overlay_multiple_arrangements(arrangements, merge_faces_by_freespace_flag)
        self.point_locator = Arr_landmarks_point_location(single_arrangement)

    def is_valid_conf(self, p):
        eps = CollisionDetectorSlow.inflation_epsilon.to_double()
        for j in range(self.robot_num):
            if not is_in_free_face(self.point_locator, Point_2(p[2 * j], p[2 * j + 1])):
                return False
            for k in range(j + 1, self.robot_num):
                if abs(FT.to_double(p[2 * j] - p[2 * k])) < self.robot_width + eps and \
                        abs(FT.to_double(p[2 * j + 1] - p[2 * k + 1])) < self.robot_width + eps:
                    return False
        return True

    # checks for collisions return:
    # True if collision free
    # False, if not
    def path_collision_free(self, p1, p2):
        max_robot_path_len = FT(0)
        for i in range(self.robot_num):
            robot_path_len = (p2[2 * i] - p1[2 * i]) * (p2[2 * i] - p1[2 * i]) + \
                             (p2[2 * i + 1] - p1[2 * i + 1]) * (p2[2 * i + 1] - p1[2 * i + 1])
            if robot_path_len > max_robot_path_len:
                max_robot_path_len = robot_path_len
        sample_amount = FT(sqrt(max_robot_path_len.to_double())) / CollisionDetectorSlow.inflation_epsilon + FT(1)
        diff_vec = [((p2[i] - p1[i]) / sample_amount) for i in range(2 * self.robot_num)]
        curr = [p1[i] for i in range(2 * self.robot_num)]
        for i in range(int(sample_amount.to_double())):
            curr = [sum(x, FT(0)) for x in zip(curr, diff_vec)]
            if not self.is_valid_conf(curr):
                return False
        return True


# noinspection PyArgumentList
class DenseSpaceQuery:
    big_value = FT(9999999)

    def __init__(self, robot_width, obstacles, robot_num):
        # init obs for collision detection
        one_width_square = Polygon_2(get_origin_robot_coord(robot_width))
        inflated_obstacles = [Polygon_2([p for p in obs]) for obs in obstacles]
        c_space_obstacles = [minkowski_sum_by_full_convolution_2(one_width_square, obs) for obs in inflated_obstacles]
        c_space_arrangements = [polygon_with_holes_to_arrangement(obs) for obs in c_space_obstacles]
        obstacles_arrangement = overlay_multiple_arrangements(c_space_arrangements, merge_faces_by_freespace_flag)
        self.vertical_decomposition = vertical_decompose(obstacles_arrangement)
        self.mark_dense()
        self.obstacles_point_locator = Arr_trapezoid_ric_point_location(self.vertical_decomposition)
        self.robot_num = robot_num

    # TODO currently we assume no 2 touching free faces (each face has it's own dense value calculated separately)
    #  this in not good
    # TODO bug in scene6
    # noinspection PyArgumentList
    def mark_dense(self):
        assert isinstance(self.vertical_decomposition, Arrangement_2)
        for f in self.vertical_decomposition.faces():
            f.data()[DENSE] = DenseSpaceQuery.big_value
        for h_edge in self.vertical_decomposition.edges():
            if not h_edge.face().data().get(FREESPACE):
                continue
            if h_edge.face().data().get(DENSE_VISITED):
                continue
            if h_edge.face().is_unbounded():
                continue
            if h_edge.source().point().x() != h_edge.target().point().x():
                continue
            next_h_edge = h_edge.next()
            # from here we take care of horizontal dense
            # skip all edges in a row over me
            while next_h_edge.source().point().x() == h_edge.source().point().x():
                next_h_edge = next_h_edge.next()
            # skip non vertical edges
            while next_h_edge.source().point().x() != next_h_edge.target().point().x():
                next_h_edge = next_h_edge.next()
            if next_h_edge == h_edge:
                # triangle
                diff = h_edge.next().point.x() - h_edge.source().point().x()
                if diff < FT(0):
                    diff = diff*FT(-1)
                h_edge.face().data()[DENSE] = diff
                h_edge.face().data()[DENSE_VISITED] = True
                # in any case (dense or not) we are done with this triangle
                continue
            else:
                diff = next_h_edge.source().point().x() - h_edge.source().point().x()
                if diff < FT(0):
                    diff = diff*FT(-1)
                h_edge.face().data()[DENSE] = diff
                h_edge.face().data()[DENSE_VISITED] = True
                # from here take care of non-horizontal dense
                next_h_edge = h_edge.next()
                # skip all vertical edges
                while next_h_edge.source().point().x() == next_h_edge.target().point().x():
                    next_h_edge = next_h_edge.next()
                first_non_ver_h_edge = next_h_edge
                next_h_edge = next_h_edge.next()
                while next_h_edge.source().point().x() == next_h_edge.target().point().x():
                    next_h_edge = next_h_edge.next()
                second_non_ver_h_edge = next_h_edge
                if first_non_ver_h_edge.source().point().y() < second_non_ver_h_edge.source().point().y():
                    temp = first_non_ver_h_edge
                    first_non_ver_h_edge = second_non_ver_h_edge
                    second_non_ver_h_edge = temp
                # first is above second
                first_min_y = min(first_non_ver_h_edge.source().point().y(), first_non_ver_h_edge.target().point().y())
                second_max_y = max(second_non_ver_h_edge.source().point().y(), second_non_ver_h_edge.target().point().y())
                if first_min_y-second_max_y < diff:
                    h_edge.face().data()[DENSE] = first_min_y-second_max_y
        return

    def is_in_dense_face(self, point):
        face = Face()
        # locate can return a vertex or an edge or a face
        located_obj = self.obstacles_point_locator.locate(point)
        # if located_obj.is_vertex():
        #     return False
        # if located_obj.is_halfedge():
        #     return False
        if located_obj.is_face():
            located_obj.get_face(face)
            return face.data()[DENSE]
        return DenseSpaceQuery.big_value

    def robots_is_in_dense(self, p):
        return [self.is_in_dense_face(Point_2(p[2 * rid], p[2 * rid + 1])) for rid in range(self.robot_num)]
