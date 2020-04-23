import srm_rrt
import rrt
import drrt
import time
import matplotlib.pyplot as plt
import gc
from config import *
from arr2_epec_seg_ex import *


def generate_path(path, robots, obstacles, destination):
    print("running tests")
    num_of_runs = 3
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
    print("drrt:")
    for number_of_milestones_to_find in [40, 100, 200, 500]:
        print("number_of_milestones_to_find=", number_of_milestones_to_find)
        Config().sr_prm_config["number_of_milestones_to_find"] = number_of_milestones_to_find
        for _ in range(num_of_runs):
            drrt.generate_path(path, robots, obstacles, destination)
            path = []
            gc.collect()
    print("finish")
