#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include "franka_ik_He.hpp"

namespace py = pybind11;

PYBIND11_MODULE(_franka_ik, m) {
    m.doc() = "Analytical Inverse Kinematics solver for Franka Emika Panda robot";

    m.def("solve_ik", 
        [](py::array_t<double> O_T_EE_array, double q7, py::array_t<double> q_actual_array) {
            // Validate input sizes
            if (O_T_EE_array.size() != 16) {
                throw std::invalid_argument("O_T_EE_array must have 16 elements");
            }
            if (q_actual_array.size() != 7) {
                throw std::invalid_argument("q_actual_array must have 7 elements");
            }

            // Convert numpy arrays to std::array
            std::array<double, 16> O_T_EE;
            std::array<double, 7> q_actual;
            
            auto O_T_EE_buf = O_T_EE_array.unchecked<1>();
            auto q_actual_buf = q_actual_array.unchecked<1>();
            
            for (size_t i = 0; i < 16; i++) {
                O_T_EE[i] = O_T_EE_buf(i);
            }
            for (size_t i = 0; i < 7; i++) {
                q_actual[i] = q_actual_buf(i);
            }

            // Call the IK solver
            auto result = franka_IK_EE(O_T_EE, q7, q_actual);

            // Convert result to list of numpy arrays
            py::list solutions;
            for (const auto& sol : result) {
                py::array_t<double> sol_array(7);
                auto sol_buf = sol_array.mutable_unchecked<1>();
                for (size_t i = 0; i < 7; i++) {
                    sol_buf(i) = sol[i];
                }
                solutions.append(sol_array);
            }

            return solutions;
        },
        py::arg("O_T_EE_array"),
        py::arg("q7"),
        py::arg("q_actual_array"),
        R"pbdoc(
            Compute inverse kinematics for Franka Emika Panda robot.

            This function returns up to 4 different joint configurations for a given 
            end-effector pose. Solutions that violate joint limits are set to NaN.

            Parameters
            ----------
            O_T_EE_array : numpy.ndarray
                Desired Cartesian pose as a 4x4 transformation matrix in column-major 
                format (flattened to 16 elements)
            q7 : float
                Last joint angle (wrist) as redundant parameter in radians
            q_actual_array : numpy.ndarray
                Current joint configuration (7 elements) in radians

            Returns
            -------
            list of numpy.ndarray
                List of 4 solutions, each as a 7-element array. Invalid solutions 
                contain NaN values.

            Examples
            --------
            >>> import numpy as np
            >>> O_T_EE = np.eye(4).flatten('F')  # Column-major format
            >>> q7 = 0.0
            >>> q_actual = np.array([0.0, 0.0, 0.0, -1.5, 0.0, 1.5, 0.0])
            >>> solutions = solve_ik(O_T_EE, q7, q_actual)
        )pbdoc"
    );

    m.def("solve_ik_cc", 
        [](py::array_t<double> O_T_EE_array, double q7, py::array_t<double> q_actual_array) {
            // Validate input sizes
            if (O_T_EE_array.size() != 16) {
                throw std::invalid_argument("O_T_EE_array must have 16 elements");
            }
            if (q_actual_array.size() != 7) {
                throw std::invalid_argument("q_actual_array must have 7 elements");
            }

            // Convert numpy arrays to std::array
            std::array<double, 16> O_T_EE;
            std::array<double, 7> q_actual;
            
            auto O_T_EE_buf = O_T_EE_array.unchecked<1>();
            auto q_actual_buf = q_actual_array.unchecked<1>();
            
            for (size_t i = 0; i < 16; i++) {
                O_T_EE[i] = O_T_EE_buf(i);
            }
            for (size_t i = 0; i < 7; i++) {
                q_actual[i] = q_actual_buf(i);
            }

            // Call the case-consistent IK solver
            auto result = franka_IK_EE_CC(O_T_EE, q7, q_actual);

            // Convert result to numpy array
            py::array_t<double> sol_array(7);
            auto sol_buf = sol_array.mutable_unchecked<1>();
            for (size_t i = 0; i < 7; i++) {
                sol_buf(i) = result[i];
            }

            return sol_array;
        },
        py::arg("O_T_EE_array"),
        py::arg("q7"),
        py::arg("q_actual_array"),
        R"pbdoc(
            Compute case-consistent inverse kinematics for Franka Emika Panda robot.

            This function returns only the solution that belongs to the same "case" as 
            the current joint configuration. This prevents unexpected switching between 
            different solution cases (e.g., elbow-up and elbow-down) during continuous 
            motion planning.

            Parameters
            ----------
            O_T_EE_array : numpy.ndarray
                Desired Cartesian pose as a 4x4 transformation matrix in column-major 
                format (flattened to 16 elements)
            q7 : float
                Last joint angle (wrist) as redundant parameter in radians
            q_actual_array : numpy.ndarray
                Current joint configuration (7 elements) in radians

            Returns
            -------
            numpy.ndarray
                Single solution as a 7-element array. If no valid solution exists in 
                the current case, returns NaN values.

            Examples
            --------
            >>> import numpy as np
            >>> O_T_EE = np.eye(4).flatten('F')  # Column-major format
            >>> q7 = 0.0
            >>> q_actual = np.array([0.0, 0.0, 0.0, -1.5, 0.0, 1.5, 0.0])
            >>> solution = solve_ik_cc(O_T_EE, q7, q_actual)
        )pbdoc"
    );

    m.attr("__version__") = "1.0.0";
}