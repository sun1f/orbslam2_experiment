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
                  default="/home/sun/slambook2/ch10/result/txt/before_opt.txt", help="The filename that contains the original poses.")
(options, args) = parser.parse_args()

# Read the original and optimized poses files.
poses_original = None
if options.initial_poses != '':
    poses_original = numpy.genfromtxt(options.initial_poses, usecols=(1, 2))

# Plots the results for the specified poses.
plot.figure()
if poses_original is not None:
    plot.plot(poses_original[:, 0], poses_original[:, 1], '-', label="Original",
              alpha=0.5, color="green")

plot.title("Title")
plot.axis('equal')
plot.legend()
# Show the plot and wait for the user to close.
plot.show()
