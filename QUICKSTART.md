# Quick Start Guide

## TL;DR - Build and Install

```bash
# Make build script executable
chmod +x build_wheel.sh

# Build the wheel
./build_wheel.sh

# Install
pip install dist/franka_analytical_ik-1.0.0-*.whl

# Test
python3 -c "from franka_analytical_ik import solve_ik; print('Success!')"
```

## Complete File Structure

After setting up, your repo should look like this:

```
franka_analytical_ik/
├── MODULE.bazel              # Bazel dependencies (updated)
├── MODULE.bazel.lock        # Auto-generated lock file
├── BUILD                    # Root build file (new)
├── .bazelrc                 # Bazel config (new)
├── build_wheel.sh          # Build helper script (new)
├── example_usage.py        # Usage example (new)
├── BUILD_INSTRUCTIONS.md   # Detailed docs (new)
├── README.md               # Original README
├── LICENSE
├── paper_preprint.pdf
└── src/
    ├── BUILD                      # Build rules (updated)
    ├── __init__.py               # Python package (new)
    ├── franka_ik_He.hpp          # Original C++ header
    └── franka_analytic_ik_bindings.cpp  # Bindings (updated)
```

## Files You Need to Create

1. **Update existing files:**
   - `MODULE.bazel` - Use the updated version
   - `src/BUILD` - Use the new BUILD file
   - `src/franka_analytic_ik_bindings.cpp` - Use the complete bindings

2. **Create new files:**
   - `BUILD` (root directory)
   - `.bazelrc`
   - `src/__init__.py`
   - `build_wheel.sh` (and make it executable)
   - `example_usage.py`
   - `BUILD_INSTRUCTIONS.md`

## Minimal Test

After installation, create a test file `test.py`:

```python
import numpy as np
from franka_analytical_ik import solve_ik

# Identity transformation
O_T_EE = np.eye(4).flatten('F')
q_actual = np.zeros(7)
q7 = 0.0

solutions = solve_ik(O_T_EE, q7, q_actual)
print(f"Got {len(solutions)} solutions")
print(f"First solution: {solutions[0]}")
```

Run it:
```bash
python3 test.py
```

## Common Commands

```bash
# Build everything
bazel build //src:all

# Build just the extension
bazel build //src:_franka_ik

# Build the wheel
bazel build //src:franka_ik_wheel

# Clean build
bazel clean
bazel build //src:franka_ik_wheel

# Install wheel
pip install dist/franka_analytical_ik-*.whl

# Uninstall
pip uninstall franka_analytical_ik
```

## Key Points

1. **Column-Major Order**: The transformation matrix must be flattened in column-major (Fortran) order:
   ```python
   O_T_EE_flat = O_T_EE.flatten('F')  # NOT 'C'!
   ```

2. **Joint Limits**: Solutions that violate joint limits return NaN arrays

3. **Four Solutions**: `solve_ik()` returns up to 4 solutions for different configurations (elbow-up/down, etc.)

4. **Case-Consistent**: `solve_ik_cc()` returns only one solution that matches the current configuration

5. **Redundant Parameter**: q7 (the last joint) is the redundant parameter you specify

## Next Steps

- Read `BUILD_INSTRUCTIONS.md` for detailed information
- Check out `example_usage.py` for complete examples
- Refer to the original README for algorithm details