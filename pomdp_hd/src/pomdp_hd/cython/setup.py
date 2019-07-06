from distutils.core import setup
from distutils.extension import Extension
from Cython.Build import cythonize
import eigency

model_name = "drawer"

if model_name == "drawer":
    PROBLEM_FILENAME = "../src/problem_definitions/drawer.cpp"
    DYNAMICS = "../src/dynamics_models/generic_kinematic_pair.cpp" ## Drawer or Microwave
    DYNAMICS_MODEL = "../src/dynamics_models/prismatic_pair.cpp"
    FILTER = "../src/filters/ukf.cpp"

elif model_name == "stapler":
    PROBLEM_FILENAME = "../src/problem_definitions/stapler.cpp"
    DYNAMICS = "../src/dynamics_models/stapler/stapler_dynamics.cpp" ## Drawer or Microwave
    DYNAMICS_MODEL = "../src/dynamics_models/revolute_pair.cpp" ## Micorwave or Stapler
    FILTER = "../src/filters/kalman_filter.cpp"

else:
    PROBLEM_FILENAME = "../src/problem_definitions/microwave_door.cpp"
    DYNAMICS = "../src/dynamics_models/generic_kinematic_pair.cpp" ## Drawer or Microwave
    DYNAMICS_MODEL = "../src/dynamics_models/revolute_pair.cpp" ## Micorwave or Stapler
    FILTER = "../src/filters/ukf.cpp"


# # Compilation parameters
# PROBLEM_FILENAME = "../src/problem_definitions/drawer.cpp"
# # PROBLEM_FILENAME = "../src/problem_definitions/microwave_door.cpp"
# # PROBLEM_FILENAME="../src/problem_definitions/stapler.cpp"

# # DYNAMICS = "../src/dynamics_models/stapler/stapler_dynamics.cpp"
# DYNAMICS = "../src/dynamics_models/generic_kinematic_pair.cpp" ## Drawer or Microwave


# # DYNAMICS_MODEL = "../src/dynamics_models/revolute_pair.cpp" ## Micorwave or Stapler
# DYNAMICS_MODEL = "../src/dynamics_models/prismatic_pair.cpp"

# # FILTER = "../src/filters/ukf.cpp"
# FILTER = "../src/filters/kalman_filter.cpp"


setup(
    # Information
    name="beliefEvolution",
    version="1.0.0",
    license="BSD",
    # Build instructions
    ext_modules=cythonize(
        [Extension("beliefEvolution",
                   ["beliefEvolution.pyx",
                    "../src/belief_evolution.cpp",
                    DYNAMICS,
                    "../src/dynamics_models/linear_dynamics.cpp",
                    "../src/dynamics_models/generic_kinematic_pair.cpp",
                    DYNAMICS_MODEL,
                    FILTER,
                    PROBLEM_FILENAME,
                    "../src/simulator.cpp",
                    "../src/utils.cpp"],
                   include_dirs=["../include",
                                 "/usr/include/eigen3",
                                 "/opt/ros/kinetic/include"] +
                   eigency.get_includes(include_eigen=False),
                   library_dirs=["/opt/ros/kinetic/lib"],
                   libraries=['rosconsole'],
                   extra_compile_args=[
                       "-std=c++11", "-fopenmp"],
                   extra_link_args=['-lgomp', '-lboost_system'], language='c++')]),
)
