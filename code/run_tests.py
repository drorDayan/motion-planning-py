import srm_rrt
import rrt
import drrt
import srm_drrt
import time
import matplotlib.pyplot as plt
import gc
from config import *
from arr2_epec_seg_ex import *


def generate_path(path, robots, obstacles, destination):
    print("running tests")
    num_of_runs = 10

    print("srm_drrt:")
    for number_of_milestones_to_find in [10, 15, 25, 50]:
        t_sum = 0
        t_v = 0
        ok_r = 0
        print("number_of_milestones_to_find=", number_of_milestones_to_find)
        Config().sr_prm_config["number_of_milestones_to_find"] = number_of_milestones_to_find
        for _ in range(num_of_runs):
            start_t = time.time()
            t, v = srm_drrt.generate_path(path, robots, obstacles, destination)
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
        else:
            print("0\n0\n0")

    print("drrt:")
    for number_of_milestones_to_find in [10, 15, 25, 35, 50, 70, 100]:
        t_sum = 0
        t_v = 0
        ok_r = 0
        print("number_of_milestones_to_find=", number_of_milestones_to_find)
        Config().sr_prm_config["number_of_milestones_to_find"] = number_of_milestones_to_find
        for _ in range(num_of_runs):
            start_t = time.time()
            t, v = drrt.generate_path(path, robots, obstacles, destination)
            if t != 0:
                ok_r += 1
                t_sum += float(time.time() - start_t)
                t_v += v
            path = []
            gc.collect()
        if ok_r != 0:
            print(t_sum / ok_r)
            print(t_v / ok_r)
            print(ok_r)
        else:
            print("0\n0\n0")

    # print("drrt:")
    # for number_of_milestones_to_find in [40, 60]:
    #     t_sum = 0
    #     print("number_of_milestones_to_find=", number_of_milestones_to_find)
    #     Config().sr_prm_config["number_of_milestones_to_find"] = number_of_milestones_to_find
    #     for _ in range(num_of_runs):
    #         start_t = time.time()
    #         drrt.generate_path(path, robots, obstacles, destination)
    #         path = []
    #         gc.collect()
    #         t_sum += int(time.time() - start_t)
    #     print(t_sum / num_of_runs)
    # print("srm_rrt:")
    # for _ in range(num_of_runs):
    #     srm_rrt.generate_path(path, robots, obstacles, destination)
    #     path = []
    #     gc.collect()
    # print("rrt:")
    # for _ in range(num_of_runs):
    #     rrt.generate_path(path, robots, obstacles, destination)
    #     path = []
    #     gc.collect()

    print("finish")
