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

def point_2_to_xy(p):
  return (p.x().to_double(), p.y().to_double())

def xy_to_point_2(x, y):
  return Point_2(x, y)

def coords_list_to_polygon_2(lst):
  lst0 = []
  for i in range(len(lst)//2):
    lst0.append(Point_2(lst[2*i], lst[2*i+1]))
  p = Polygon_2(lst0)
  if p.is_clockwise_oriented(): p.reverse_orientation()
  return p

def tuples_list_to_polygon_2(lst):
  lst0 = []
  for tuple in lst:
    lst0.append(Point_2(tuple[0], tuple[1]))
  p = Polygon_2(lst0)
  if p.is_clockwise_oriented(): p.reverse_orientation()
  return p

def polygon_2_to_tuples_list(polygon):
  lst = [(p.x().to_double(), p.y().to_double()) for p in polygon.vertices()]
  return lst