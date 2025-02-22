class Config(object):
    __instance = None

    def __new__(cls):
        if Config.__instance is None:
            Config.__instance = object.__new__(cls)
            Config.__instance.reset()
        return Config.__instance

    def reset(self):
        # General configuration
        self.general_config = dict(
            ROBOTS_COUNT=3,  # Amount of robots. Should match scene file.
            USE_FAST_CD=True,  # TODO not tested on false
            INFLATION_EPS=0.01
        )
        self.sr_prm_config = dict(
            number_of_milestones_to_find=1000,
            number_of_neighbors_to_connect=15,
            sparse_radius=1,
            use_grid=False,
            grid_size=0.01
        )
        self.rrt_config = dict(
            k_nearest_to_connect_to_dest=30,
            steer_eta=4,
            timeout=120
        )
        self.srm_rrt_config = dict(
            k_nearest_to_connect_to_dest=30,
            steer_eta=4,
            timeout=120
        )
        self.drrt_config = dict(
            num_of_points_to_add_in_expand=100,
            k_nearest_to_connect_to_dest=30,
            timeout=120,
            use_sparse=True
        )
        self.srm_drrt_config = dict(  # only the extra from drrt_config
            sr_add_srm_once_in=3,
            mr_add_srm_once_in=3  # unused
        )
