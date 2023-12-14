import pickle
import sympy as sp
import numpy as np
import sys
import os
import random

# Get the parent directory
parent_dir = os.path.dirname(os.path.realpath('/home/rushi/mobile_robotics/final_project/car-racing/src/car_racing/racing'))

# Add the parent directory to sys.path
sys.path.append(parent_dir)

from racing import offboard
from utils import base, racing_env
from utils.constants import *


def racing(args):
    track_layout = args["track_layout"]
    track_spec = np.genfromtxt("data/track_layout/" + track_layout + ".csv", delimiter=",")
    if args["simulation"]:
        track = racing_env.ClosedTrack(track_spec, track_width=1.0)
        # setup ego car
        ego = offboard.DynamicBicycleModel(name="ego", param=base.CarParam(edgecolor="black"), system_param = base.SystemParam())
        mpc_cbf_param = base.MPCCBFRacingParam(vt=1.6)
        ego.set_state_curvilinear(np.zeros((X_DIM,)))
        ego.set_state_global(np.zeros((X_DIM,)))
        ego.start_logging()
        ego.set_ctrl_policy(offboard.MPCCBFRacing(mpc_cbf_param, ego.system_param))
        ego.ctrl_policy.set_timestep(0.1)
        ego.set_track(track)
        ego.ctrl_policy.set_track(track)
        # setup surrounding cars
        t_symbol = sp.symbols("t")
        # car1 = offboard.DynamicBicycleModel(name="car1", param=base.CarParam(edgecolor="orange"), system_param = base.SystemParam())
        # mpc_cbf_param = base.MPCCBFRacingParam(vt=0.8)
        # car1.set_state_curvilinear(np.zeros((X_DIM,)))
        # car1.set_state_global(np.zeros((X_DIM,)))
        # car1.start_logging()
        # car1.set_ctrl_policy(offboard.iLQRRacing(mpc_cbf_param, car1.system_param))
        # car1.ctrl_policy.set_timestep(0.1)
        # car1.set_track(track)
        # car1.ctrl_policy.set_track(track)

        # car2 = offboard.NoDynamicsModel(name="car2", param=base.CarParam(edgecolor="orange"))
        # car2.set_track(track)
        # car2.set_state_curvilinear_func(t_symbol, 0.1 * random.randint(0, 10) * t_symbol + 3 + random.randint(0, 14), -0.1 + 0.0 * t_symbol)
        # car2.start_logging()
        car3 = offboard.NoDynamicsModel(name="car3", param=base.CarParam(edgecolor="orange"))
        car3.set_track(track)
        car3.set_state_curvilinear_func(t_symbol, 1 * t_symbol + 3.0, t_symbol)
        car3.start_logging()
        # setup simulation
        simulator = offboard.CarRacingSim()
        simulator.set_timestep(0.1)
        simulator.set_track(track)
        simulator.add_vehicle(ego)
        ego.ctrl_policy.set_racing_sim(simulator)
        simulator.add_vehicle(car3)
        # simulator.add_vehicle(car2)
        simulator.sim(sim_time=20.0)
        with open("data/simulator/racing.obj", "wb") as handle:
            pickle.dump(simulator, handle, protocol=pickle.HIGHEST_PROTOCOL)
    else:
        with open("data/simulator/racing.obj", "rb") as handle:
            simulator = pickle.load(handle)
    if args["plotting"]:
        simulator.plot_simulation()
        simulator.plot_state("ego")
    if args["animation"]:
        simulator.animate(filename="racing", imagemagick=True)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulation", action="store_true")
    parser.add_argument("--plotting", action="store_true")
    parser.add_argument("--animation", action="store_true")
    parser.add_argument("--track-layout", type=str)
    args = vars(parser.parse_args())
    racing(args)
