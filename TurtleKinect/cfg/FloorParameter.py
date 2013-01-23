#!/usr/bin/env python
PACKAGE = "TurtleKinect"
import roslib;roslib.load_manifest(PACKAGE)

from dynamic_reconfigure.parameter_generator import *

gen = ParameterGenerator()
gen.add("ModelType", int_t, 0, "SAC Model Type", 0, 0, 17)
gen.add("MaxIterations", int_t, 0, "Max Iterations For Segmentation", 100, 10, 200)
gen.add("DistanceThreshold", double_t, 0, "Distance Threshold For Segmentation", 0.2,0 ,1)
gen.add("RatioLimit", double_t, 0, "Percent Of Remaing Cloud To Stop Segmentation", 0.3, 0 ,1)
gen.add("ExtractNegative", bool_t, 0, "False To Extract Floor As PointCloud", True)


exit(gen.generate(PACKAGE, "floor_segment", "Floor"))