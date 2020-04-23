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
            ROBOTS_COUNT=3,  # Amount of robots. Should match scene file. If lower, excess robots are ignored.
            USE_FAST_CD=True, # not tested on false
            INFLATION_EPS=0.01
        )
        self.sr_prm_config = dict(
            number_of_milestones_to_find=1000,
            number_of_neighbors_to_connect=15
        )
        self.rrt_config = dict(
            k_nearest_to_connect_to_dest=30,
            steer_eta=4
        )
        self.srm_rrt_config = dict(
            k_nearest_to_connect_to_dest=30,
            steer_eta=4
        )
        self.drrt_config = dict(
            steer_eta=4
        )
