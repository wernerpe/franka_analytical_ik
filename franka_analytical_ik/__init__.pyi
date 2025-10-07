"""
Type stubs for franka_analytical_ik

Analytical Inverse Kinematics solver for Franka Emika Panda robot.
"""

from typing import List
import numpy as np
import numpy.typing as npt

__version__: str

def solve_ik(
    O_T_EE_array: npt.NDArray[np.float64],
    q7: float,
    q_actual_array: npt.NDArray[np.float64],
) -> List[npt.NDArray[np.float64]]:
    """
    Compute inverse kinematics for Franka Emika Panda robot.

    This function returns up to 4 different joint configurations for a given 
    end-effector pose. Solutions that violate joint limits are set to NaN.

    Parameters
    ----------
    O_T_EE_array : numpy.ndarray
        Desired Cartesian pose as a 4x4 transformation matrix in column-major 
        format (flattened to 16 elements). Must be float64 dtype.
    q7 : float
        Last joint angle (wrist) as redundant parameter in radians.
    q_actual_array : numpy.ndarray
        Current joint configuration (7 elements) in radians. Must be float64 dtype.

    Returns
    -------
    list of numpy.ndarray
        List of 4 solutions, each as a 7-element float64 array. Invalid solutions 
        contain NaN values.

    Raises
    ------
    ValueError
        If O_T_EE_array doesn't have 16 elements or q_actual_array doesn't have 7 elements.

    Examples
    --------
    >>> import numpy as np
    >>> from franka_analytical_ik import solve_ik
    >>> O_T_EE = np.eye(4, dtype=np.float64).flatten('F')  # Column-major format
    >>> q7 = 0.0
    >>> q_actual = np.array([0.0, 0.0, 0.0, -1.5, 0.0, 1.5, 0.0], dtype=np.float64)
    >>> solutions = solve_ik(O_T_EE, q7, q_actual)
    >>> # Check which solutions are valid
    >>> valid_solutions = [sol for sol in solutions if not np.any(np.isnan(sol))]
    
    Notes
    -----
    - The transformation matrix must be in column-major (Fortran) order
    - Joint limits are checked; solutions outside limits return NaN
    - Up to 4 solutions may be returned representing different configurations 
      (e.g., elbow-up, elbow-down, etc.)
    - Assumes Franka Hand is installed (d7e = 0.2104m)
    """
    ...

def solve_ik_cc(
    O_T_EE_array: npt.NDArray[np.float64],
    q7: float,
    q_actual_array: npt.NDArray[np.float64],
) -> npt.NDArray[np.float64]:
    """
    Compute case-consistent inverse kinematics for Franka Emika Panda robot.

    This function returns only the solution that belongs to the same "case" as 
    the current joint configuration. This prevents unexpected switching between 
    different solution cases (e.g., elbow-up and elbow-down) during continuous 
    motion planning.

    Parameters
    ----------
    O_T_EE_array : numpy.ndarray
        Desired Cartesian pose as a 4x4 transformation matrix in column-major 
        format (flattened to 16 elements). Must be float64 dtype.
    q7 : float
        Last joint angle (wrist) as redundant parameter in radians.
    q_actual_array : numpy.ndarray
        Current joint configuration (7 elements) in radians. Must be float64 dtype.

    Returns
    -------
    numpy.ndarray
        Single solution as a 7-element float64 array. If no valid solution exists in 
        the current case, returns NaN values.

    Raises
    ------
    ValueError
        If O_T_EE_array doesn't have 16 elements or q_actual_array doesn't have 7 elements.

    Examples
    --------
    >>> import numpy as np
    >>> from franka_analytical_ik import solve_ik_cc
    >>> O_T_EE = np.eye(4, dtype=np.float64).flatten('F')  # Column-major format
    >>> q7 = 0.0
    >>> q_actual = np.array([0.0, 0.0, 0.0, -1.5, 0.0, 1.5, 0.0], dtype=np.float64)
    >>> solution = solve_ik_cc(O_T_EE, q7, q_actual)
    >>> if not np.any(np.isnan(solution)):
    ...     print("Valid solution found!")
    
    Notes
    -----
    - The transformation matrix must be in column-major (Fortran) order
    - Returns solution consistent with current joint configuration "case"
    - Useful for trajectory planning to avoid configuration jumps
    - Assumes Franka Hand is installed (d7e = 0.2104m)
    """
    ...

__all__ = ['solve_ik', 'solve_ik_cc', '__version__']