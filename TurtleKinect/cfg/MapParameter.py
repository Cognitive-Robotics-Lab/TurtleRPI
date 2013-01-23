#!/usr/bin/env python
PACKAGE = "TurtleKinect"
import roslib;roslib.load_manifest(PACKAGE)

from dynamic_reconfigure.parameter_generator import *

gen = ParameterGenerator()
gen.add("DistanceThreshold", double_t, 0, "Distance Threshold For Segmentation", 0.2,0 ,1)
gen.add("MaxIterations", int_t, 0, "Max Iterations For Segmentation", 100, 10, 200)


exit(gen.generate(PACKAGE, "map_maker", "Map"))