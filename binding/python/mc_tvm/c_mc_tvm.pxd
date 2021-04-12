#
# Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
#

cdef extern from "<mc_tvm/CoMFunction.h>" namespace "mc_tvm":
    cdef cppclass CoMFunction:
        pass

cdef extern from "<mc_tvm/CoMInConvexFunction.h>" namespace "mc_tvm":
    cdef cppclass CoMInConvexFunction:
        pass

cdef extern from "<mc_tvm/MomentumFunction.h>" namespace "mc_tvm":
    cdef cppclass MomentumFunction:
        pass

cdef extern from "<mc_tvm/OrientationFunction.h>" namespace "mc_tvm":
    cdef cppclass OrientationFunction:
        pass

cdef extern from "<mc_tvm/PositionBasedVisServoFunction.h>" namespace "mc_tvm":
    cdef cppclass PositionBasedVisServoFunction:
        pass

cdef extern from "<mc_tvm/PositionFunction.h>" namespace "mc_tvm":
    cdef cppclass PositionFunction:
        pass

cdef extern from "<mc_tvm/PostureFunction.h>" namespace "mc_tvm":
    cdef cppclass PostureFunction:
        pass

cdef extern from "<mc_tvm/TransformFunction.h>" namespace "mc_tvm":
    cdef cppclass TransformFunction:
        pass

cdef extern from "<mc_tvm/VectorOrientationFunction.h>" namespace "mc_tvm":
    cdef cppclass VectorOrientationFunction:
        pass

