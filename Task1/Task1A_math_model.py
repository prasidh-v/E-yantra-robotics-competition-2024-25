import sympy as sp
import numpy as np
import control as ctrl
 
########## Initialization of  variables & provided  
# Define the symbolic variables
x1, x2, u = sp.symbols('x1 x2 u')
 
# Define the differential equations
x1_dot = -x1 + 2 * x1 ** 3 + x2 + 4 * u
x2_dot = -x1 - x2 + 2 * u
 
eq1 = sp.Eq(x1_dot, 0)
eq2 = sp.Eq(x2_dot, 0)
##################################################
 
 
def find_equilibrium_points():
    '''
    1. Substitute input(u) = 0 in both equation for finding equilibrium points
    2. Equate x1_dot, x2_dot equal to zero for finding equilibrium points
    3. solve the x1_dot, x2_dot equations for the unknown variables and save the value to the variable namely "equi_points"
    '''
 
    eq1_sub = eq1.subs(u, 0)
    eq2_sub = eq2.subs(u, 0)
 
    equi_points = sp.solve([eq1_sub, eq2_sub], (x1, x2))
 
    return equi_points
 
def find_A_B_matrices(equi_points):
    '''
    1. Substitute every equilibrium points that you have already found in the find_equilibrium_points() function
    2. After substituting the equilibrium points, Save the Jacobian matrices A and B as A_matrices, B_matrices
    '''
    A_matrix = sp.Matrix([
        [sp.diff(x1_dot, x1), sp.diff(x1_dot, x2)],
        [sp.diff(x2_dot, x1), sp.diff(x2_dot, x2)]
    ])
 
    B_matrix = sp.Matrix([
        [sp.diff(x1_dot, u)],
        [sp.diff(x2_dot, u)]
    ])
    A_matrices = []
    B_matrices = []
 
    ###### WRITE YOUR CODE HERE ################
    for equi_point in equi_points:
        x1_val, x2_val = equi_point
        jacobians_A = A_matrix.subs({x1: x1_val, x2: x2_val})
        jacobaian_B = B_matrix.subs({x1: x1_val, x2: x2_val})
 
        A_matrices.append(jacobians_A)
        B_matrices.append(jacobaian_B)
 
       # print(f"Jacobian A at equi point {equi_point}:")
       #print(A_matrices)
 
       # print(f"Jacobian B at equi point {equi_point}:")
       # print(B_matrices)
       # print()
 
    ############################################
    
    return A_matrices,B_matrices
 
 
def find_eigen_values(A_matrices):
    '''
    1.  Find the eigen values of all A_matrices (You can use the eigenvals() function of sympy) 
        and append it to the 'eigen_values' list
    2.  With the eigen values, determine whether the system is stable or not and
        append the string 'Stable' if system is stable, else append the string 'Unstable'
        to the 'stability' list 
    '''
    
    eigen_values = []
    stability = []
 
    ###### WRITE YOUR CODE HERE ################
    for matrix in A_matrices:
 
        eigen_val = matrix.eigenvals()
        eigen_values.append(eigen_val)
 
        is_stable = all(sp.re(ev) < 0 for ev in eigen_val.keys())
 
        if is_stable:
            stability.append("Stable")
        else:
            stability.append("Unstable")
 
    ############################################
    return eigen_values, stability
 
def compute_lqr_gain(jacobians_A, jacobians_B):
    K = 0
    '''
    This function is use to compute the LQR gain matrix K
    1. Use the Jacobian A and B matrix at the equilibrium point (-1,1) and assign it to A and B respectively for computing LQR gain
    2.  Compute the LQR gain of the given system equation (You can use lqr() of control module) 
    3. Take the A matrix corresponding to the Unstable Equilibrium point (-1,1) that you have already found for computing LQR gain.
    4. Assign the value of gain to the variable K
    '''
 
    # Define the Q and R matrices
    Q = np.eye(2)  # State weighting matrix
    R = np.array([1])  # Control weighting matrix
 
    ###### WRITE YOUR CODE HERE ################ 
     
    jacobians_A = np.matrix([[5,1], [-1,-1]])
    jacobians_B = np.matrix([[4], [2]])
    K, S, R = ctrl.lqr(jacobians_A, jacobians_B, Q, R)
 
    ############################################
    return K
 
def main_function(): # Don't change anything in this function
    # Find equilibrium points
    eq_points = find_equilibrium_points()
 
    if not eq_points:
        print("No equilibrium points found.")
        return None, None, None, None, None, None
 
    # Find Jacobian matrices
    jacobians_A, jacobians_B = find_A_B_matrices(eq_points)
 
    # For finding eigenvalues and stability of the given equation
    eigen_values, stability = find_eigen_values(jacobians_A)
 
 
    # Compute the LQR gain matrix K
    K = compute_lqr_gain(jacobians_A, jacobians_B)
 
    return eq_points, jacobians_A,  eigen_values, stability, K
 
 
def task1a_output():
    '''
    This function will print the results that you have obtained
    '''
    print("Equilibrium Points:")
    for i, point in enumerate(eq_points):
        print(f"  Point {i + 1}: x1 = {point[0]}, x2 = {point[1]}")
 
    print("\nJacobian Matrices at Equilibrium Points:")
    for i, matrix in enumerate(jacobians_A):
        print(f"  At Point {i + 1}:")
        print(sp.pretty(matrix, use_unicode=True))
 
    print("\nEigenvalues at Equilibrium Points:")
    for i, eigvals in enumerate(eigen_values):
        eigvals_str = ', '.join([f"{val}: {count}" for val, count in eigvals.items()])
        print(f"  At Point {i + 1}: {eigvals_str}")
 
    print("\nStability of Equilibrium Points:")
    for i, status in enumerate(stability):
        print(f"  At Point {i + 1}: {status}")
 
    print("\nLQR Gain Matrix K at the selected Equilibrium Point:")
    print(K)
 
 
if __name__ == "__main__":
    # Run the main function
    results = main_function()
 
    # This will get the equilibrium points, A_matrix, eigen values, stability of the system
    eq_points, jacobians_A, eigen_values, stability, K = results
 
    # print the results
    task1a_output()