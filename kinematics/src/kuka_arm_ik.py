from sympy import symbols, cos, sin, pi, simplify
from sympy.matrices import Matrix

import numpy as np
import math



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
R0_1 = Matrix([[             cos(q1),            -sin(q1),            0],
               [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0)],
               [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0)]])
R0_1 = R0_1.subs(s)

R1_2 = Matrix([[             cos(q2),            -sin(q2),            0],
               [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1)],
               [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1)]])
R1_2 = R1_2.subs(s)

R2_3 = Matrix([[             cos(q3),            -sin(q3),            0],
               [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2)],
               [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2)]])
R2_3 = R2_3.subs(s)


R3_4 = Matrix([[             cos(q4),            -sin(q4),            0],
               [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3)],
               [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3)]])
R3_4 = R3_4.subs(s)

R4_5 = Matrix([[             cos(q5),            -sin(q5),            0],
               [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4)],
               [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4)]])
R4_5 = R4_5.subs(s)

R5_6 = Matrix([[             cos(q6),            -sin(q6),            0],
               [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5)],
               [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5)]])
R5_6 = R5_6.subs(s)


# Rotation to have the same orientation than rviz
R_z = Matrix([[ cos(np.pi),     -sin(np.pi),    0],
               [sin(np.pi),     cos(np.pi),     0],
               [0,              0,              1],
               ])

R_y = Matrix([[ cos(-np.pi/2),  0,              sin(-np.pi/2)],
              [ 0,              1,              0,],
              [ -sin(-np.pi/2), 0,              cos(-np.pi/2)],
              ])

R_corr = simplify(R_z * R_y)

# End efector position: for testing
EE_Position = Matrix([[0.714], [2.241], [2.332]])
roll=2.241
pitch=-1.054
yaw=-2.756


# End efector orientation: for testing
R_EEx = Matrix([[ 1,           0,   0],
              [ 0,      cos(alpha0),    -sin(alpha0)],
              [ 0,      sin(alpha0),    cos(alpha0)]])

R_EEy = Matrix([[ cos(alpha1),        0,  sin(alpha1)],
              [       0,        1,        0],
              [-sin(alpha1),        0,  cos(alpha1)]])

R_EEz = Matrix([[ cos(alpha2), -sin(alpha2),        0],
              [ sin(alpha2),  cos(alpha2),        0],
              [ 0,              0,        1]])

R_EE_corr = simplify ( R_EEz * R_EEy * R_EEx * R_corr )
R_EE_corr_eval =R_EE_corr.evalf(subs={alpha0: roll, alpha1:pitch, alpha2:yaw})


# DH parameters
d12=0.75
a12=0.35
a23=1.25
a35=-0.054
d35=1.5
d67=0.303


WC_location_eval = EE_Position  - (d67) * R_EE_corr_eval *  Matrix([[0],[0],[1]])


print ("WC->" , WC_location_eval)

#Angle Q1
q_1 = math.atan2(WC_location_eval[1],WC_location_eval[0])
print("q1---->",q_1)

L=WC_location_eval[2]-d12
print("L->",L)
W=math.sqrt(math.pow(WC_location_eval[0],2)+math.pow(WC_location_eval[1],2))-a12
print("W->",W)

A = math.sqrt(math.pow(a35,2)+ math.pow(d35,2))
print("A->",A)
B = math.sqrt(math.pow(L,2)+math.pow(W,2))
print("B->",B)
C = a23
print("C->",C)

alpha =  math.atan2(L,W)
print("alpha->",math.degrees(alpha))


a = math.acos(np.clip((B*B+C*C-A*A)/(2*B*C),-1,1))
print("a->",math.degrees(a))
q_2 = math.pi/2-a-alpha
print("q2---->",q_2)


b = math.acos(np.clip((A*A+C*C-B*B)/(2*A*C),-1,1))
print("b->",math.degrees(b))

q_3 = math.pi/2-(b+0.036)
print("q3---->",q_3)

R_0_3 = simplify(R0_1  * R1_2 * R2_3)
R_0_3_Inv=R_0_3.inv("LU")


R_3_6 = simplify(R_0_3_Inv * R_EE_corr_eval)
R_3_6_eval= R_3_6.evalf(subs={q1: q_1, q2:q_2, q3:q_3})


q_5 = math.acos(R_3_6_eval[1,2])
print("q5--->",q_5)

q_4 =math.acos(-R_3_6_eval[0,2]/math.sin(q_5))
print("q4--->",q_4)

q_6=math.acos(R_3_6_eval[1,0]/math.sin(q_5))
print("q6--->",q_6)



print("-------------------------------------------")
print("q1---->",q_1)
print("q2---->",q_2)
print("q3---->",q_3)
print("q4--->",q_4)
print("q5--->",q_5)
print("q6--->",q_6)