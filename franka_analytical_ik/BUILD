load("@rules_cc//cc:defs.bzl", "cc_library")
load("@pybind11_bazel//:build_defs.bzl", "pybind_extension")
load("@rules_python//python:packaging.bzl", "py_wheel", "py_package")

# Header-only library for the Franka IK solver
cc_library(
    name = "franka_ik_lib",
    hdrs = ["franka_ik_He.hpp"],
    deps = ["@eigen//:eigen"],
    visibility = ["//visibility:public"],
)

# Python binding extension
pybind_extension(
    name = "_franka_ik",
    srcs = ["franka_analytic_ik_bindings.cpp"],
    deps = [":franka_ik_lib"],
)

# Python package
py_library(
    name = "franka_ik_pylib",
    srcs = ["__init__.py"],
    deps = ["_franka_ik"],
    imports = [".."],
)

py_package(
    name = "franka_ik_package",
    deps = [":franka_ik_pylib"],
)

# Wheel distribution
py_wheel(


## WHEEL BUILDING IS STILL BROKEN YOU NEED TO MAKE A FOLDER THAT CONTAINS ALL THE CORRECT STUFF LIKE IN CSDECOMP ALSO GENERATE THE PYSTUBS.

    name,, = "franka_ik_wheel",
    distribution = "franka_analytical_ik",
    version = "1.0.0",
    deps = [":franka_ik_package"],
    platform = select({
        "@platforms//os:linux": "manylinux2014_x86_64",
    }),
    python_tag = "cp310",
    author = "Yanhao He, Steven Liu, and Peter Werner",
    author_email = "",
    homepage = "https://github.com/wernerpe/franka_analytical_ik",
    license = "MIT",
 
)