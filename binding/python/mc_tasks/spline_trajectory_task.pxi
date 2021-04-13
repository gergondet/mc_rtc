cdef class PairDoubleMatrix3d(object):
  cdef pair[double, c_eigen.Matrix3d] impl
  def __cinit__(self, *args):
    if len(args) == 1 and isinstance(args[0], tuple) and len(args[0]) == 2:
      tup = args[0]
      self.impl = pair[double, c_eigen.Matrix3d](tup[0], eigen.Matrix3d(tup[1]).impl)

cdef class VectorPairDoubleMatrix3d(object):
  cdef vector[pair[double, c_eigen.Matrix3d]] impl
  def __cinit__(self, *args):
    if len(args) == 1 and isinstance(args[0], list):
      for p in args[0]:
        self.impl.push_back(PairDoubleMatrix3d(p).impl)

cdef class PairDoubleVector3d(object):
  cdef pair[double, c_eigen.Vector3d] impl
  def __cinit__(self, *args):
    if len(args) == 1 and isinstance(args[0], tuple) and len(args[0]) == 2:
      tup = args[0]
      self.impl = pair[double, c_eigen.Vector3d](tup[0], eigen.Vector3d(tup[1]).impl)
      print "pair {}".format(tup[0])

cdef class VectorPairDoubleVector3d(object):
  cdef vector[pair[double, c_eigen.Vector3d]] impl
  def __cinit__(self, *args):
    if len(args) == 1 and isinstance(args[0], list):
      for p in args[0]:
        self.impl.push_back(PairDoubleVector3d(p).impl)
