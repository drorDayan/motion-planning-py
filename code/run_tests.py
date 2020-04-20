import srm_rrt
import rrt
from arr2_epec_seg_ex import *
import time
import matplotlib.pyplot as plt
import gc


def generate_path(path, robots, obstacles, destination):
    print("running tests")
    num_of_runs = 30
    print("srm_rrt:")
    for _ in range(num_of_runs):
        srm_rrt.generate_path(path, robots, obstacles, destination)
        path = []
        gc.collect()
    print("rrt:")
    for _ in range(num_of_runs):
        rrt.generate_path(path, robots, obstacles, destination)
        path = []
        gc.collect()
    print("finish")
