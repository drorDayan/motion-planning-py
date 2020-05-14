import srm_rrt
import rrt
import drrt
import srm_drrt
import time
import matplotlib.pyplot as plt
import gc
from config import *
from arr2_epec_seg_ex import *
import pickle


def run_alg(alg, num_of_runs, robots, obstacles, destination):
    path = []
    t_sum = 0
    prm_t_sum = 0
    t_v = 0
    ok_r = 0
    for _ in range(num_of_runs):
        t, v, prm_t = alg.generate_path(path, robots, obstacles, destination)
        if t != 0:
            ok_r += 1
            t_sum += t
            prm_t_sum += prm_t
            t_v += v
        path = []
        gc.collect()
    if ok_r != 0:
        print(t_sum / ok_r)
        print(t_v / ok_r)
        print(ok_r)
        print(prm_t_sum)
        return t_sum / ok_r, t_v / ok_r, ok_r, prm_t_sum / ok_r
    else:
        print("0\n0\n0\n")
        return 0, 0, 0, 0


def test_drrt_alg(alg, number_of_milestones_to_find_list, num_of_runs, robots, obstacles, destination, is_sparse):
    alg_name = alg.__name__
    Config().drrt_config['use_sparse'] = is_sparse
    sparse_str = "sparse" if is_sparse else ""
    res = []
    print("testing", sparse_str, alg_name)
    for number_of_milestones_to_find in number_of_milestones_to_find_list:
        print("number_of_milestones_to_find=", number_of_milestones_to_find)
        Config().sr_prm_config["number_of_milestones_to_find"] = number_of_milestones_to_find
        c_res = run_alg(alg, num_of_runs, robots, obstacles, destination)
        res.append(c_res)
        # t_sum = 0
        # t_v = 0
        # ok_r = 0
        # for _ in range(num_of_runs):
        #     start_t = time.time()
        #     t, v = alg.generate_path(path, robots, obstacles, destination)
        #     if t != 0:
        #         ok_r += 1
        #         t_sum += float(time.time()-start_t)
        #         t_v += v
        #     path = []
        #     gc.collect()
        # if ok_r != 0:
        #     print(t_sum / ok_r)
        #     print(t_v / ok_r)
        #     print(ok_r)
        #     res.append([t_sum / ok_r, t_v / ok_r, ok_r])
        # else:
        #     print("0\n0\n0")
        #     res.append([0, 0, 0])
    return res


def generate_path(path, robots, obstacles, destination):
    print("running tests")
    print("drrt timeout=", Config().drrt_config['timeout'])
    print("sparse factor=", Config().sr_prm_config['sparse_radius'])

    num_of_runs = 10
    # run_alg(srm_rrt, num_of_runs, robots, obstacles, destination)
    # run_alg(rrt, num_of_runs, robots, obstacles, destination)
    # return
    #  number_of_milestones_to_find_list = [25, 50, 75, 100, 150, 200, 300, 400, 500, 750, 1000, 1250, 1500]
    # number_of_milestones_to_find_list = [100, 250, 500, 750, 1000, 1250, 1500, 1750, 2000]
    number_of_milestones_to_find_list = [100, 250, 500, 750, 1000, 1250, 1500, 1750, 2000]
    # number_of_milestones_to_find_list = [10*t for t in range(1, 21)]
    res = []
    for alg in [srm_drrt, drrt]:
        for is_sparse in [True, False]:
            t_res = test_drrt_alg(alg, number_of_milestones_to_find_list, num_of_runs, robots, obstacles,
                                  destination, is_sparse)
            res.append(t_res)

    # here we create some graphs
    plt.plot(number_of_milestones_to_find_list, [t for t, v, s, prm_t in res[0]], linestyle='-', marker='o',
             label='sparse srm_drrt')
    plt.plot(number_of_milestones_to_find_list, [t for t, v, s, prm_t in res[1]], linestyle='-', marker='o',
             label='srm_drrt')
    plt.plot(number_of_milestones_to_find_list, [t for t, v, s, prm_t in res[2]], linestyle='-', marker='o',
             label='sparse drrt')
    plt.plot(number_of_milestones_to_find_list, [t for t, v, s, prm_t in res[3]], linestyle='-', marker='o',
             label='drrt')
    plt.legend()
    plt.xlabel("number of milestones per prm road-map")
    plt.title("average time to find solution (counting only successful tries)")
    plt.savefig("output_graphs/average time to find solution.png")
    plt.close()

    plt.plot(number_of_milestones_to_find_list, [v for t, v, s, prm_t in res[0]], linestyle='-', marker='o',
             label='sparse srm_drrt')
    plt.plot(number_of_milestones_to_find_list, [v for t, v, s, prm_t in res[1]], linestyle='-', marker='o',
             label='srm_drrt')
    plt.plot(number_of_milestones_to_find_list, [v for t, v, s, prm_t in res[2]], linestyle='-', marker='o',
             label='sparse drrt')
    plt.plot(number_of_milestones_to_find_list, [v for t, v, s, prm_t in res[3]], linestyle='-', marker='o',
             label='drrt')
    plt.legend()
    plt.xlabel("number of milestones per prm road-map")
    plt.title("C-space vertices until we found a solution (counting only successful tries)")
    plt.savefig("output_graphs/average vertices to find solution.png")
    plt.close()

    plt.plot(number_of_milestones_to_find_list, [s for t, v, s, prm_t in res[0]], linestyle='-', marker='o',
             label='sparse srm_drrt')
    plt.plot(number_of_milestones_to_find_list, [s for t, v, s, prm_t in res[1]], linestyle='-', marker='o',
             label='srm_drrt')
    plt.plot(number_of_milestones_to_find_list, [s for t, v, s, prm_t in res[2]], linestyle='-', marker='o',
             label='sparse drrt')
    plt.plot(number_of_milestones_to_find_list, [s for t, v, s, prm_t in res[3]], linestyle='-', marker='o',
             label='drrt')
    plt.legend()
    plt.xlabel("number of milestones per prm road-map")
    plt.title("success rate")
    plt.savefig("output_graphs/success rate.png")
    plt.close()

    plt.plot(number_of_milestones_to_find_list, [prm_t for t, v, s, prm_t in res[0]], linestyle='-', marker='o',
             label='sparse srm_drrt')
    plt.plot(number_of_milestones_to_find_list, [prm_t for t, v, s, prm_t in res[1]], linestyle='-', marker='o',
             label='srm_drrt')
    plt.plot(number_of_milestones_to_find_list, [prm_t for t, v, s, prm_t in res[2]], linestyle='-', marker='o',
             label='sparse drrt')
    plt.plot(number_of_milestones_to_find_list, [prm_t for t, v, s, prm_t in res[3]], linestyle='-', marker='o',
             label='drrt')
    plt.legend()
    plt.xlabel("number of milestones per prm road-map")
    plt.title("average time to build prm maps (counting only successful tries)")
    plt.savefig("output_graphs/prm maps time.png")
    plt.close()

    test_info_file = open("output_graphs/test_info.txt", "w")
    test_info_file.write(f"drrt timeout={Config().drrt_config['timeout']}\n")
    test_info_file.write(f"sparse factor={Config().sr_prm_config['sparse_radius']}\n")
    test_info_file.close()

    pickle.dump(res, open("output_graphs/pickled_results.p", "wb"))
    pickle.dump(Config(), open("output_graphs/pickled_config.p", "wb"))

    # res = pickle.load(open("output_graphs/pickled_results.p", "rb"))

    plt.plot(number_of_milestones_to_find_list, [t-prm_t for t, v, s, prm_t in res[0]], linestyle='-', marker='o',
             label='sparse srm_drrt')
    plt.plot(number_of_milestones_to_find_list, [t-prm_t for t, v, s, prm_t in res[1]], linestyle='-', marker='o',
             label='srm_drrt')
    plt.plot(number_of_milestones_to_find_list, [t-prm_t for t, v, s, prm_t in res[2]], linestyle='-', marker='o',
             label='sparse drrt')
    plt.plot(number_of_milestones_to_find_list, [t-prm_t for t, v, s, prm_t in res[3]], linestyle='-', marker='o',
             label='drrt')
    plt.legend()
    plt.xlabel("number of milestones per prm road-map")
    plt.title("average time the dRRT run on the composite road-maps (counting only successful tries)")
    plt.savefig("output_graphs/time the dRRT run on the composite.png")
    plt.close()

    # run_alg(srm_rrt, num_of_runs, robots, obstacles, destination)
    # run_alg(rrt, num_of_runs, robots, obstacles, destination)
    print("finish")
