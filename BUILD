load("@rules_python//python:packaging.bzl", "py_wheel", "py_package")
# Root BUILD file
exports_files([
    "README.md",
    "LICENSE",
])

py_package(
    name = "franka_analytical_ik_package",
    packages = ["franka_analytical_ik"],
    deps = ["//franka_analytical_ik:franka_ik_package"],
)

py_wheel(
    name = "franka_ik_wheel",
    distribution = "franka_analytical_ik",
    version = "1.0.0",
    deps = [":franka_analytical_ik_package"],
    platform = select({
        "@platforms//os:linux": "manylinux2014_x86_64",
    }),
    python_tag = "cp310",
    author = "Yanhao He, Steven Liu, and Peter Werner",
    author_email = "",
    homepage = "https://github.com/wernerpe/franka_analytical_ik",
    license = "MIT",
 
)