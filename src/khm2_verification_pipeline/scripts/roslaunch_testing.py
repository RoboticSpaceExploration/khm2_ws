#!/usr/bin/env python3

import roslaunch
import xml.etree.ElementTree as ET # GO HOME

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

trial_params_args = ['khm2_verification_pipeline', 'trial_params.launch']
trial_params_launch = roslaunch.rlutil.resolve_launch_arguments(trial_params_args)[0]
print(trial_params_launch)

sim_args = ['khm2_bringup', 'simulation.launch']
sim_launch = roslaunch.rlutil.resolve_launch_arguments(sim_args)[0]

rover_args = ['khm2_bringup', 'rover.launch']
rover_launch = roslaunch.rlutil.resolve_launch_arguments(rover_args)[0]

launch_files = [trial_params_launch, sim_launch, rover_launch]
parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)

parent.start()
parent.spin()
