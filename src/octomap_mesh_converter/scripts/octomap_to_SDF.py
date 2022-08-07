#!/usr/bin/env python
# license removed for brevity

import rospy
import roslaunch
import subprocess 
import sys
import os
from datetime import datetime
from os.path import exists

def octomap_to_SDF(filename_in):

    # 1: Check input and retrieve file locations.

    working_directory = os.getcwd()
    maps_directory = working_directory + "/src/octomap_mesh_converter/maps/"
    simulation_directory = working_directory + "/src/octomap_mesh_converter/simulation/"
    worlds_directory = working_directory + "/src/octomap_mesh_converter/worlds/"
    launch_directory = working_directory + "/src/octomap_mesh_converter/launch/"

    output_name = "octomap_to_sdf_" + datetime.now().strftime("%d_%m_%Y__%H_%M_%S")

    if(filename_in.endswith('.bt')):

        if(exists(maps_directory + filename_in)):

            output_name = output_name + "_" + filename_in.replace('.bt', '')
            print("Success! Found file " + filename_in + " at location " + maps_directory)
            print("Output mesh file will be saved at " + simulation_directory + " as " + output_name + ".dae")
            print("Output world file will be saved at " + worlds_directory + " as " + output_name + ".world")
            print("Output launch file will be saved at " + launch_directory + " as " + output_name + ".launch")

            # 2: Call the roslaunch file which converts filename_in to a mesh and saves it in the 'simulation_directory' folder.

            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            cli_args = [launch_directory + "/convert_to_SDF.launch", "package:=" + working_directory + "/src/octomap_mesh_converter", "map_path:=maps/" + filename_in, "model_path:=simulation/" + output_name + ".dae"]
            roslaunch_args = cli_args[1:]
            roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
            parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
            parent.start()

            # 3: Generate a world file and corresponding launch file for the generated mesh.

        else:
            print("Could not find " + filename_in + " in " + maps_directory)
    else:    
        print("Invalid input! filename must end in '.bt'")



if __name__ == "__main__":
    if(len(sys.argv) != 2):
        print("Invalid input! usage: rosrun octomap_mesh_converter octomap_to_SDF.py filename_in")
    else:
        octomap_to_SDF(sys.argv[1]);
