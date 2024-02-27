# distutils: language = c++

from RobotParams cimport RobotParams

# Create a Cython extension type which holds a C++ instance
# as an attribute and create a bunch of forwarding methods
# Python extension type.
cdef class PyRobotParams:
    cdef RobotParams c_paramStruct  # Hold a C++ instance which we're wrapping
    cdef int i
    def __cinit__(self):
      pass
    def __dealloc__(self):
      pass
    # Attribute access
    @property
    def c_paramStruct(self):
       return self.c_paramStruct
