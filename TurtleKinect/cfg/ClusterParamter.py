#!/usr/bin/env python
PACKAGE = "TurtleKinect"
import roslib;roslib.load_manifest(PACKAGE)

from dynamic_reconfigure.parameter_generator import *

gen = ParameterGenerator()

gen.add("ClusterTolerance", double_t, 0, "Euclidean Cluster Tolerance", .3, 0.0001, 2.0)
gen.add("MinClusterSize", int_t, 0, "Min Cluster Points", 50, 0, 400)
gen.add("MaxClusterSize", int_t, 0, "Max Cluster Points", 250000, 400, 1000000)


exit(gen.generate(PACKAGE, "turtle_cluster", "Cluster"))