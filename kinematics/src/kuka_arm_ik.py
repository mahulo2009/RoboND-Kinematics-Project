from sympy import symbols, cos, sin, pi, simplify
from sympy.matrices import Matrix

import numpy as np


q1, q2, q3 = symbols('q1:4')


R_z = Matrix([[ cos(np.pi),     -sin(np.pi),    0],
               [sin(np.pi),     cos(np.pi),     0],
               [0,              0,              0],
               ])

R_y = Matrix([[ cos(-np.pi/2),  0,              sin(-np.pi/2)],
              [ 0,              1,              0,],
              [ -sin(-np.pi/2), 0,              cos(-np.pi/2)],
              ])

R_corr = simplify(R_z * R_y)


EE_P = Matrix([[1.718],[1.930],[1.400]])

R_EEx = Matrix([[ 1,           0,   0],
              [ 0,      cos(q1),    -sin(q1)],
              [ 0,      sin(q1),    cos(q1)]])


R_EEy = Matrix([[ cos(q2),        0,  sin(q2)],
              [       0,        1,        0],
              [-sin(q2),        0,  cos(q2)]])

R_EEz = Matrix([[ cos(q3), -sin(q3),        0],
              [ sin(q3),  cos(q3),        0],
              [ 0,              0,        1]])
 

  
R_EE = simplify ( R_EEz * R_EEy * R_EEx * R_corr )

print(R_EE.evalf(subs={q1: -2.622, q2:-0.239, q3:1.236}))


Z_AXIS = Matrix([[0],[0],[1]])
 
WC = EE_P  - (0.303) * R_EE *  Z_AXIS

print ("WC->" , WC.evalf(subs={q1: -2.622, q2:-0.239, q3:1.236}))





