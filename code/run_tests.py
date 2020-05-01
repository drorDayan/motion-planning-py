import srm_rrt
import rrt
import drrt
import srm_drrt
import time
import matplotlib.pyplot as plt
import gc
from config import *
from arr2_epec_seg_ex import *


def test_alg(alg, number_of_milestones_to_find_list, num_of_runs, robots, obstacles, destination):
    alg_name = alg.__name__
    print("testing", alg_name)
    path = []
    res = []
    for number_of_milestones_to_find in number_of_milestones_to_find_list:
        t_sum = 0
        t_v = 0
        ok_r = 0
        print("number_of_milestones_to_find=", number_of_milestones_to_find)
        Config().sr_prm_config["number_of_milestones_to_find"] = number_of_milestones_to_find
        for _ in range(num_of_runs):
            start_t = time.time()
            t, v = alg.generate_path(path, robots, obstacles, destination)
            if t != 0:
                ok_r += 1
                t_sum += float(time.time()-start_t)
                t_v += v
            path = []
            gc.collect()
        if ok_r != 0:
            print(t_sum / ok_r)
            print(t_v / ok_r)
            print(ok_r)
            res.append([t_sum / ok_r, t_v / ok_r, ok_r])
        else:
            print("0\n0\n0")
            res.append([0, 0, 0])
    # add graphs here


def generate_path(path, robots, obstacles, destination):
    print("running tests")

    num_of_runs = 10
    number_of_milestones_to_find_list = [10, 15, 25, 35, 50, 70, 100, 150, 200, 300, 400, 500, 1000]
    # number_of_milestones_to_find_list = [15, 20]
    test_alg(srm_drrt, number_of_milestones_to_find_list, num_of_runs, robots, obstacles, destination)
    test_alg(drrt, number_of_milestones_to_find_list, num_of_runs, robots, obstacles, destination)

    print("finish")
