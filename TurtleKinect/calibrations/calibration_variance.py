import numpy, scipy
import matplotlib.pyplot as plt

def main():
  matL1 = [[574.326,518.278,541.508,534.621,534.485],[0.0,0.0,0.0,0.0,0.0],[314.756,300.897,311.604,327.068,316.927],[0.0,0.0,0.0,0.0,0.0],
  [539.829,574.027,522.978,548.985,537.631],[245.037,278.126,286.686,263.143,238.930],[0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0],
  [1.0,1.0,1.0,1.0,1.0]]
  DistMatList = [[0.184173,-0.371247,-0.004344,0.004530,0.000000],[0.162593,-0.253174,0.024391,-0.010942,0.000000],
  [0.150006,-0.206318,0.023749,0.000153,0.000000],[0.177905,-0.313835,-0.005280,-0.001105,0.000000],[0.179406,-0.311723,-0.009299,-0.007484,0.000000]]
  ProjMatList = [[586.293979,0.000000,316.542009,0.000000,0.000000,587.729848,261.160195,0.000000,0.000000,0.000000,1.000000,0.000000],
  [535.540318,0.000000,303.712920,0.000000,0.000000,534.386823,296.609057,0.000000,0.000000,0.000000,1.000000,0.000000],
  [563.533500,0.000000,326.787549,0.000000,0.000000,560.170252,287.050754,0.000000,0.000000,.000000,1.000000,0.000000],
  [547.009548,0.000000,315.707304,0.000000,0.000000,550.441589,242.678852,0.000000,0.000000,0.000000,1.000000,0.000000],
  [546.58914728524076, 0.0, 295.35747112606435, 0.0, 0.0, 555.24654486040083, 234.885989409789, 0.0, 0.0, 0.0, 1.0, 0.0]]

  projMatArr = numpy.array(matL1)
  projMat = numpy.transpose(numpy.mat(projMatArr))
  print projMat
  projVar = numpy.var(projMat,0)
  projMean = numpy.mean(projMat,0)
  print projMean
  projCalib = projMat
  projDiff = [sum(x-projMean) for x in projCalib]
  for x in projDiff:
    print
    print x

#  DistMatArr = numpy.array(DistMatList)
#  DistMat = (numpy.mat(DistMatArr))
#  DistVar = numpy.var(DistMat,0)
#  DistMean = numpy.mean(DistMat,0)
#  DistDiff = [sum(x-DistMean) for x in DistMat]
#  for x in DistDiff:
#    print
#    print x

#  matA1 = numpy.array(ProjMatList)
#  mat1 = numpy.mat(matA1)
#  mat1 = numpy.transpose(matA1) 
#  V1 = numpy.var(mat1,0)
#  M1 = numpy.mean(mat1,0)
#  print mat1
#  print V1
#  print M1
#  calbs = mat1
#  diff = [sum(x-M1) for x in calbs]
#  for x in diff:
#    print
#    print x
#  mn = calbs[diff.index(min(diff))]
#  print mn

if __name__ == '__main__':
	main()
