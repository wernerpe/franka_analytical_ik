"""
Franka Analytical IK - Analytical Inverse Kinematics solver for Franka Emika Panda

This package provides fast analytical inverse kinematics solutions for the 
Franka Emika Panda 7-DOF robot manipulator.
"""

try:
    from . import _franka_ik
except ImportError:
    # Fallback for direct imports during development
    import _franka_ik

__version__ = _franka_ik.__version__
__all__ = ['solve_ik', 'solve_ik_cc']

# Re-export the main functions
solve_ik = _franka_ik.solve_ik
solve_ik_cc = _franka_ik.solve_ik_cc