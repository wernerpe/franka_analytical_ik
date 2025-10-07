# Build Instructions

This document explains how to build the Franka Analytical IK Python bindings using Bazel.

## Prerequisites

1. **Bazel** (version 7.0 or higher recommended)
   ```bash
   # On Ubuntu/Debian
   sudo apt install bazel
   
   # Or using Bazelisk (recommended)
   npm install -g @bazel/bazelisk
   ```

2. **Python 3.10** (configured in MODULE.bazel)
   ```bash
   python3 --version  # Should be 3.10.x
   ```

3. **C++ compiler** with C++17 support
   ```bash
   # On Ubuntu/Debian
   sudo apt install build-essential
   ```

## Project Structure

```
.
├── MODULE.bazel              # Bazel module configuration
├── MODULE.bazel.lock         # Lock file for dependencies
├── BUILD                     # Root build file
├── .bazelrc                  # Bazel configuration
├── README.md                 # Original project README
├── LICENSE                   # License file
├── build_wheel.sh           # Helper script to build wheel
├── example_usage.py         # Example Python script
└── src/
    ├── BUILD                # Build rules for the library
    ├── __init__.py          # Python package initialization
    ├── franka_ik_He.hpp     # C++ header (IK solver)
    └── franka_analytic_ik_bindings.cpp  # pybind11 bindings
```

## Building the Project

### Option 1: Build and Install Wheel (Recommended)

1. **Build the wheel package:**
   ```bash
   ./build_wheel.sh
   ```
   
   Or manually:
   ```bash
   bazel build //src:franka_ik_wheel
   ```

2. **Install the wheel:**
   ```bash
   pip install dist/franka_analytical_ik-1.0.0-*.whl
   ```

3. **Test the installation:**
   ```bash
   python3 -c "from franka_analytical_ik import solve_ik; print('Import successful!')"
   ```

### Option 2: Build Extension Only

To build just the Python extension (for development):

```bash
bazel build //src:_franka_ik
```

The compiled extension will be at:
```
bazel-bin/src/_franka_ik.so
```

### Option 3: Build C++ Library

To build just the C++ header library:

```bash
bazel build //src:franka_ik_lib
```

## Usage Examples

### Basic Usage

```python
import numpy as np
from franka_analytical_ik import solve_ik, solve_ik_cc

# Define desired end-effector pose (4x4 transformation matrix)
O_T_EE = np.eye(4)
O_T_EE[0:3, 3] = [0.3, 0.0, 0.5]  # Set position

# Flatten in column-major order
O_T_EE_flat = O_T_EE.flatten('F')

# Current joint configuration
q_actual = np.array([0.0, -0.3, 0.0, -2.0, 0.0, 1.5, 0.0])

# Redundant parameter (7th joint)
q7 = 0.0

# Get all solutions (up to 4)
solutions = solve_ik(O_T_EE_flat, q7, q_actual)

# Get case-consistent solution
solution = solve_ik_cc(O_T_EE_flat, q7, q_actual)
```

### Running the Example

```bash
python3 example_usage.py
```

## Troubleshooting

### Issue: Bazel not found
**Solution:** Install Bazel or Bazelisk (recommended)

### Issue: Python version mismatch
**Solution:** Update MODULE.bazel to match your Python version:
```python
python.toolchain(python_version = "3.X")  # Change to your version
```

### Issue: Compiler errors
**Solution:** Ensure you have a C++17 compatible compiler:
```bash
g++ --version  # Should be 7.0 or higher
```

### Issue: Module not found after installation
**Solution:** Make sure pip install completed successfully and try:
```bash
pip install --force-reinstall dist/franka_analytical_ik-*.whl
```

### Issue: Import errors with Eigen
**Solution:** The Eigen dependency should be automatically fetched by Bazel. If issues persist:
```bash
bazel clean --expunge
bazel build //src:franka_ik_wheel
```

## Development Workflow

1. **Make changes to C++ code:**
   - Edit `src/franka_ik_He.hpp` or `src/franka_analytic_ik_bindings.cpp`

2. **Rebuild:**
   ```bash
   bazel build //src:_franka_ik
   ```

3. **Test changes:**
   ```bash
   python3 -c "import sys; sys.path.insert(0, 'bazel-bin/src'); from _franka_ik import solve_ik"
   ```

4. **Build new wheel:**
   ```bash
   ./build_wheel.sh
   pip install --force-reinstall dist/*.whl
   ```

## Clean Build

To perform a clean build:

```bash
bazel clean
bazel build //src:franka_ik_wheel
```

To completely reset Bazel's cache:

```bash
bazel clean --expunge
bazel build //src:franka_ik_wheel
```

## Advanced Configuration

### Changing Optimization Level

Edit `.bazelrc`:
```
build --copt=-O2  # Change from -O3
```

### Debug Build

```bash
bazel build -c dbg //src:_franka_ik
```

### Verbose Build

```bash
bazel build --subcommands //src:franka_ik_wheel
```

## Testing

To add tests, create a `test` directory and add test targets in BUILD files:

```python
py_test(
    name = "ik_test",
    srcs = ["test_ik.py"],
    deps = ["//src:franka_ik"],
)
```

Run tests:
```bash
bazel test //test:ik_test
```

## Distribution

The wheel file can be distributed and installed on any compatible system:

```bash
# On the build machine
./build_wheel.sh

# On target machine
pip install franka_analytical_ik-1.0.0-*.whl
```

## References

- [Bazel Documentation](https://bazel.build/docs)
- [pybind11 Documentation](https://pybind11.readthedocs.io/)
- [Original Paper](paper_preprint.pdf)