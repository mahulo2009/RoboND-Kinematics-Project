from sympy import symbols, cos, sin, pi, simplify
from sympy.matrices import Matrix

import numpy as np


q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5 ,a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')


# DH Parameters
s = {
        alpha0: 0,         a0: 0,                           d1: 0.75, 
        alpha1: -pi/2,     a1: 0.35,        q2: q2-pi/2,    d2:0.0,  
        alpha2: 0,         a2: 1.25,                        d3:0.0,
        alpha3: -pi/2,     a3: -0.054,                      d4:1.5,
        alpha4: pi/2,      a4: 0.0,                         d5:0.0,
        alpha5: -pi/2,     a5: 0.0,                         d6:0.0,
        alpha6: 0,         a6: 0.0,         q7:0.0,         d7:0.303
     
     }

# Homogeneous Transforms Matrix
T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
               [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
               [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
               [                   0,                   0,            0,               1]])
T0_1 = T0_1.subs(s)

T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
               [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
               [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
               [                   0,                   0,            0,               1]])
T1_2 = T1_2.subs(s)

T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
               [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
               [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
               [                   0,                   0,            0,               1]])
T2_3 = T2_3.subs(s)

T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
               [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
               [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
               [                   0,                   0,            0,               1]])
T3_4 = T3_4.subs(s)

T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
               [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
               [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
               [                   0,                   0,            0,               1]])
T4_5 = T4_5.subs(s)

T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
               [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
               [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
               [                   0,                   0,            0,               1]])
T5_6 = T5_6.subs(s)

T6_7 = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
               [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
               [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
               [                   0,                   0,            0,               1]])
T6_7 = T6_7.subs(s)

# Rotation to have the same orientation than rviz
R_z = Matrix([[ cos(np.pi),     -sin(np.pi),    0,  0],
               [sin(np.pi),     cos(np.pi),     0,  0],
               [0,              0,              1,  0],
               [0,              0,              0,  1]])

R_y = Matrix([[ cos(-np.pi/2),  0,              sin(-np.pi/2),   0],
              [ 0,              1,              0,               0],
              [ -sin(-np.pi/2), 0,              cos(-np.pi/2),   0],
              [ 0,              0,              0,               1]])

R_corr = simplify(R_z * R_y)


# Intermediate transformations
T0_2 = simplify( T0_1 * T1_2 )
T0_3 = simplify( T0_2 * T2_3 )
T0_4 = simplify( T0_3 * T3_4 )
T0_5 = simplify( T0_4 * T4_5 )
T0_6 = simplify( T0_5 * T5_6 )
T0_G = simplify( T0_6 * T6_7 )
T_total = simplify(T0_G * R_corr)



print("T0_1->",T0_1)
print("T1_2->",T1_2)
print("T2_3->",T2_3)
print("T3_4->",T3_4)
print("T4_5->",T4_5)
print("T5_6->",T5_6)
print("T6_7->",T6_7)



#print("T0_1->",(T0_1*P_0_0).evalf(subs={q1: q1_in, q2:q2_in, q3:q3_in, q4:q4_in, q5:q5_in, q6:q6_in}))
#print("T0_2->",(T0_2*P_0_0).evalf(subs={q1: q1_in, q2:q2_in, q3:q3_in, q4:q4_in, q5:q5_in, q6:q6_in}))
#print("T0_3->",(T0_3*P_0_0).evalf(subs={q1: q1_in, q2:q2_in, q3:q3_in, q4:q4_in, q5:q5_in, q6:q6_in}))
#print("T0_4->",(T0_4*P_0_0).evalf(subs={q1: q1_in, q2:q2_in, q3:q3_in, q4:q4_in, q5:q5_in, q6:q6_in}))
#print("T0_5->",(T0_5*P_0_0).evalf(subs={q1: q1_in, q2:q2_in, q3:q3_in, q4:q4_in, q5:q5_in, q6:q6_in}))
#print("T0_6->",(T0_6*P_0_0).evalf(subs={q1: q1_in, q2:q2_in, q3:q3_in, q4:q4_in, q5:q5_in, q6:q6_in}))
#print("T0_G->",(T0_G*P_0_0).evalf(subs={q1: q1_in, q2:q2_in, q3:q3_in, q4:q4_in, q5:q5_in, q6:q6_in}))
#print("T_total->",(T_total * P_0_0).evalf(subs={q1: q1_in, q2:q2_in, q3:q3_in, q4:q4_in, q5:q5_in, q6:q6_in}))




