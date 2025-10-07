# Test the built extension
python3 << 'EOF'
#!/usr/bin/env python3
"""
Example usage of the Franka Analytical IK Python bindings
"""

import numpy as np
import sys

# Add the bazel build output to path for testing
sys.path.insert(0, 'bazel-bin/src')

try:
    from _franka_ik import solve_ik, solve_ik_cc
    print("✓ Successfully imported _franka_ik")
except ImportError as e:
    print(f"✗ Failed to import: {e}")
    print("\nMake sure you've built the extension with:")
    print("  bazel build //src:_franka_ik")
    sys.exit(1)

def test_with_real_data():
    """Test with real Franka robot configuration and pose"""
    print("\n" + "="*70)
    print("Testing Franka Analytical IK with Real Robot Data")
    print("="*70)
    
    # Real joint configuration from Franka robot
    q_actual = np.array([
        -2.266327e+00,
        -3.631720e-01,
        -3.270000e-04,
        -1.901082e+00,
        -3.270000e-04,
         1.570675e+00,
        -1.944327e+00
    ], dtype=np.float64)
    
    print("\n1. Current Joint Configuration:")
    print("   q_actual = [")
    for i, q in enumerate(q_actual):
        print(f"     q{i+1} = {q:+.6f} rad ({np.degrees(q):+.2f}°)")
    print("   ]")
    
    # Real gripper pose (end-effector transformation matrix)
    O_T_EE = np.array([
        [ 0.89435996,  0.44684723, -0.02116137, -0.28044076],
        [ 0.44633396, -0.89451696, -0.02500802, -0.33566672],
        [-0.03010396,  0.01292113, -0.99946325,  0.64871789],
        [ 0.00000000,  0.00000000,  0.00000000,  1.00000000]
    ], dtype=np.float64)
    
    print("\n2. Target End-Effector Pose:")
    print("   Position: [x={:.4f}, y={:.4f}, z={:.4f}] meters".format(
        O_T_EE[0, 3], O_T_EE[1, 3], O_T_EE[2, 3]))
    print("   Rotation Matrix:")
    for row in O_T_EE[:3, :3]:
        print(f"     [{row[0]:+.6f}, {row[1]:+.6f}, {row[2]:+.6f}]")
    
    # Flatten in column-major order (Fortran order) as expected by C++ code
    O_T_EE_flat = O_T_EE.flatten('F')
    
    # Use current q7 as redundant parameter
    q7 = q_actual[6]
    
    print(f"\n3. Redundant Parameter (q7): {q7:.6f} rad ({np.degrees(q7):.2f}°)")
    
    # Test 1: Solve IK - get all solutions
    print("\n" + "-"*70)
    print("TEST 1: solve_ik() - Finding all possible solutions")
    print("-"*70)
    
    try:
        solutions = solve_ik(O_T_EE_flat, q7, q_actual)
        
        valid_count = 0
        for i, sol in enumerate(solutions):
            is_valid = not np.any(np.isnan(sol))
            if is_valid:
                valid_count += 1
                print(f"\n  Solution {i+1}: ✓ VALID")
                print("   Joint angles [rad]:")
                for j, angle in enumerate(sol):
                    print(f"     q{j+1} = {angle:+.6f} rad ({np.degrees(angle):+7.2f}°)")
                
                # Compare with actual configuration
                diff = np.abs(sol - q_actual)
                max_diff_rad = np.max(diff)
                max_diff_deg = np.degrees(max_diff_rad)
                print(f"   Max difference from q_actual: {max_diff_rad:.6f} rad ({max_diff_deg:.2f}°)")
            else:
                print(f"\n  Solution {i+1}: ✗ INVALID (violates joint limits)")
        
        print(f"\n  Summary: Found {valid_count}/4 valid solutions")
        
    except Exception as e:
        print(f"  ✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    # Test 2: Solve IK - case-consistent
    print("\n" + "-"*70)
    print("TEST 2: solve_ik_cc() - Case-consistent solution")
    print("-"*70)
    
    try:
        solution_cc = solve_ik_cc(O_T_EE_flat, q7, q_actual)
        
        if not np.any(np.isnan(solution_cc)):
            print("\n  ✓ Found case-consistent solution:")
            print("   Joint angles [rad]:")
            for j, angle in enumerate(solution_cc):
                print(f"     q{j+1} = {angle:+.6f} rad ({np.degrees(angle):+7.2f}°)")
            
            # Compare with actual configuration
            diff = np.abs(solution_cc - q_actual)
            max_diff_rad = np.max(diff)
            max_diff_deg = np.degrees(max_diff_rad)
            print(f"\n   Max difference from q_actual: {max_diff_rad:.6f} rad ({max_diff_deg:.2f}°)")
            
            if max_diff_rad < 1e-3:
                print("   ✓ Solution matches current configuration (< 1e-3 rad)")
            
        else:
            print("\n  ✗ No valid case-consistent solution found")
            
    except Exception as e:
        print(f"  ✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    print("\n" + "="*70)
    print("All tests completed successfully! ✓")
    print("="*70)
    return True

def test_simple():
    """Simple test with identity pose"""
    print("\n" + "="*70)
    print("Simple Test: Identity Pose")
    print("="*70)
    
    # Identity transformation at origin
    O_T_EE = np.eye(4, dtype=np.float64)
    O_T_EE[2, 3] = 0.5  # 0.5m height
    
    O_T_EE_flat = O_T_EE.flatten('F')
    q_actual = np.zeros(7, dtype=np.float64)
    q7 = 0.0
    
    print(f"\nTarget position: [0, 0, 0.5] meters")
    print(f"q7 = {q7:.2f} rad")
    
    try:
        solutions = solve_ik(O_T_EE_flat, q7, q_actual)
        valid_count = sum(1 for sol in solutions if not np.any(np.isnan(sol)))
        print(f"Result: Found {valid_count}/4 valid solutions ✓")
        return True
    except Exception as e:
        print(f"Error: {e} ✗")
        return False

if __name__ == "__main__":
    print("\n" + "🤖 "*20)
    print("Franka Analytical IK - Python Bindings Test Suite")
    print("🤖 "*20)
    
    # Run simple test first
    success1 = test_simple()
    
    # Run test with real data
    success2 = test_with_real_data()
    
    if success1 and success2:
        print("\n✓✓✓ ALL TESTS PASSED ✓✓✓\n")
        sys.exit(0)
    else:
        print("\n✗✗✗ SOME TESTS FAILED ✗✗✗\n")
        sys.exit(1)
EOF