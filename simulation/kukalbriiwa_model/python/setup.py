from setuptools import setup
from Cython.Build import cythonize
def compile_setup(path = ""):
    setup(include_dirs = ['.','../cpp/parameters/withoutLinAxes'], package_dir={'python': ''},ext_modules=cythonize("RobotParams.pyx"))
if __name__ == '__main__':
    compile_setup();
