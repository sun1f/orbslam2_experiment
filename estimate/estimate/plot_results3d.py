#!/usr/bin/python
#
# Plots the results from the 2D pose graph optimization. It will draw a line
# between consecutive vertices.  The commandline expects two optional filenames:
#
#   ./plot_results.py --initial_poses optional --optimized_poses optional
#
# The files have the following format:
#   ID x y yaw_radians

import matplotlib.pyplot as plot
import numpy
import sys
from optparse import OptionParser

parser = OptionParser()
parser.add_option("--initial_poses", dest="initial_poses",
                  default="/home/sun/slambook2/ch10/result/txt/3D_before_opt.txt", help="The filename that contains the original poses.")
parser.add_option("--optimized_poses1", dest="optimized_poses1",
                  default="/home/sun/slambook2/ch10/result/txt/3D_after_GN.txt", help="The filename that contains the optimized poses.")
parser.add_option("--optimized_poses2", dest="optimized_poses2",
                  default="/home/sun/slambook2/ch10/result/txt/3D_after_LM.txt", help="The filename that contains the optimized poses.")
parser.add_option("--optimized_poses3", dest="optimized_poses3",
                  default="/home/sun/slambook2/ch10/result/txt/3D_after_DL.txt", help="The filename that contains the optimized poses.")
(options, args) = parser.parse_args()

# Read the original and optimized poses files.
poses_original = None
if options.initial_poses != '':
    poses_original = numpy.genfromtxt(options.initial_poses, usecols=(1, 3))

poses_optimized1 = None
if options.optimized_poses1 != '':
    poses_optimized1 = numpy.genfromtxt(
        options.optimized_poses1, usecols=(1, 3))

poses_optimized2 = None
if options.optimized_poses2 != '':
    poses_optimized2 = numpy.genfromtxt(
        options.optimized_poses2, usecols=(1, 3))

poses_optimized3 = None
if options.optimized_poses3 != '':
    poses_optimized3 = numpy.genfromtxt(
        options.optimized_poses3, usecols=(1, 3))

# Plots the results for the specified poses.
plot.figure()
if poses_original is not None:
    plot.plot(poses_original[:, 0], poses_original[:, 1], poses_original[:, 2], '-', label="Original",
              alpha=0.5, color="green")

if poses_optimized1 is not None:
    plot.plot(poses_optimized1[:, 0], poses_optimized1[:, 1], poses_optimized1[:, 2], '-', label="GN",
              alpha=0.5, color="red")
if poses_optimized2 is not None:
    plot.plot(poses_optimized2[:, 0], poses_optimized2[:, 1], poses_optimized2[:, 2], '-', label="LM",
              alpha=0.5, color="blue")
if poses_optimized3 is not None:
    plot.plot(poses_optimized3[:, 0], poses_optimized3[:, 1], poses_optimized3[:, 2], '-', label="DogLeg",
              alpha=0.5, color="yellow")

plot.title("city10000")
plot.axis('equal')
plot.legend()
# Show the plot and wait for the user to close.
plot.show()
