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

offset = Vector_2(FT(0.5), FT(0.5))

def minkowski_sum_convex_polygon_segment(polygon, segment, translation_vector):
  vector = Vector_2(segment)
  arr = Arrangement_2()
  lst = []
  for e in polygon.edges():
    translated_edge = Segment_2(e.source() + translation_vector, e.target() + translation_vector)
    lst.append(Curve_2(translated_edge))
  insert(arr, lst)

  for e in arr.edges():
    begin0 = e.source().point()
    end0 = begin0 + vector
    lst.append(Curve_2(Segment_2(begin0, end0)))
    begin1 = e.target().point()
    end1 = begin1 + vector
    lst.append(Curve_2(Segment_2(begin1, end1)))
    lst.append(Curve_2(Segment_2(end0, end1)))

  insert(arr, lst)
  lst = []
  polygon = next(arr.unbounded_face().inner_ccbs())
  for edge in polygon:
    lst.append(edge.source().point())
  result = Polygon_2(lst)
  if(result.orientation() == CLOCKWISE): result.reverse_orientation()
  return result

def minkowski_sum_polygon_segment(polygon, segment):
  translation_vector = Vector_2(Point_2(0, 0), segment.source()) - Vector_2(Point_2(0, 0), polygon.vertex(0) + offset)
  res = []
  approx_convex_partition_2(polygon, res)
  ms_parts = [minkowski_sum_convex_polygon_segment(c, segment, translation_vector) for c in res]
  ps = Polygon_set_2()
  ps.join_polygons(ms_parts)
  res = []
  ps.polygons_with_holes(res)
  pwh = res[0]
  return pwh

def minkowski_sum_polygon_point(polygon, point):
  translation_vector = Vector_2(Point_2(0, 0), point) - Vector_2(Point_2(0, 0), polygon.vertex(0) + offset)
  lst = []
  for v in polygon.vertices():
    lst.append(v + translation_vector)
  p = Polygon_2(lst)
  pwh = Polygon_with_holes_2(p, [])
  return pwh