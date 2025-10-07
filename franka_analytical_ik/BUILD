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
    visibility = ["//visibility:public"],
)

# Python package
py_library(
    name = "franka_ik_package",
    srcs = ["__init__.py",
            "__init__.pyi",  # Type stubs
    ],
    deps = ["_franka_ik"],
    imports = ["."],
    visibility = ["//visibility:public"],
)

# py_package(
#     name = "franka_ik_package",
#     deps = [":franka_ik_pylib"],
#     visibility = ["//visibility:public"],
# )
# Wheel distribution
