from sympy import symbols, cos, sin, pi, simplify
from sympy.matrices import Matrix

import numpy as np
import math



# Create symbols
#
#   
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5 ,a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
beta0, beta1, beta2 = symbols('beta0:3')

# Create Modified DH parameters
#
#
s = {
    alpha0: 0,         a0: 0,                           d1: 0.75, 
    alpha1: -pi/2,     a1: 0.35,        q2: q2-pi/2,    d2:0.0,  
    alpha2: 0,         a2: 1.25,                        d3:0.0,
    alpha3: -pi/2,     a3: -0.054,                      d4:1.5,
    alpha4: pi/2,      a4: 0.0,                         d5:0.0,
    alpha5: -pi/2,     a5: 0.0,                         d6:0.0,
    alpha6: 0,         a6: 0.0,         q7:0.0,         d7:0.303
 
}

# Define Modified DH Transformation matrix
#
#
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


R_z = Matrix([[ cos(np.pi),     -sin(np.pi),    0],
       [sin(np.pi),     cos(np.pi),     0],
       [0,              0,              1],
       ])

R_y = Matrix([[ cos(-np.pi/2),  0,              sin(-np.pi/2)],
      [ 0,              1,              0,],
      [ -sin(-np.pi/2), 0,              cos(-np.pi/2)],
      ])
R_corr = simplify(R_z * R_y)


R_EEx = Matrix([[ 1,           0,   0],
      [ 0,      cos(beta0),    -sin(beta0)],
      [ 0,      sin(beta0),    cos(beta0)]])

R_EEy = Matrix([[ cos(beta1),        0,  sin(beta1)],
      [       0,        1,        0],
      [-sin(beta1),        0,  cos(beta1)]])

R_EEz = Matrix([[ cos(beta2), -sin(beta2),        0],
      [ sin(beta2),  cos(beta2),        0],
      [ 0,              0,        1]])

R_EE_corr = simplify ( R_EEz * R_EEy * R_EEx * R_corr )


# Extract rotation matrices from the transformation matrices
#
#
R0_1 = T0_1.extract([0,1,2],[0,1,2])
R1_2 = T1_2.extract([0,1,2],[0,1,2])
R2_3 = T2_3.extract([0,1,2],[0,1,2])

R_0_3 = simplify(R0_1  * R1_2 * R2_3)
R_0_3_Inv = R_0_3.inv("LU")

R_3_6 = simplify(R_0_3_Inv * R_EE_corr)


px = 0.714
py = 2.241
pz = 2.332

roll = 2.241
pitch = -1.054
yaw = -2.756

print("-----------------------------------------------------")


R_EE_corr_eval = R_EE_corr.evalf(subs={beta0: roll, beta1:pitch, beta2:yaw})

EE_Position = Matrix ( [[px],[py],[pz]] ) 
WC_location_eval = EE_Position  - (0.303) * R_EE_corr_eval *  Matrix([[0],[0],[1]])

print("WC_location= ",WC_location_eval)

L=WC_location_eval[2]-0.75
print("L->",L)
W=math.sqrt(math.pow(WC_location_eval[0],2)+math.pow(WC_location_eval[1],2))-0.35
print("W->",W)

A = math.sqrt(math.pow(1.5,2)+ math.pow(-0.054,2))
print("A->",A)
B = math.sqrt(math.pow(L,2)+math.pow(W,2))
print("B->",B)
C = 1.25
print("B->",C)

angle_a = math.acos(np.clip((B*B+C*C-A*A)/(2*B*C),-1,1))
print("angle_a->",angle_a)

theta1 = math.atan2(WC_location_eval[1],WC_location_eval[0])
theta2 = math.pi/2-angle_a-math.atan2(L,W)
theta3 = math.pi/2-(math.acos(np.clip((A*A+C*C-B*B)/(2*A*C),-1,1))+0.036)

R_3_6_eval = R_3_6.evalf(subs={beta0:roll, beta1:pitch, beta2: yaw,q1: theta1, q2:theta2, q3:theta3})

theta5 = math.acos(R_3_6_eval[1,2])
theta4 = math.acos(-R_3_6_eval[0,2]/math.sin(theta5))
theta6 = math.acos(R_3_6_eval[1,0]/math.sin(theta5))

print("theta1= ",theta1)
print("theta2= ",theta2)
print("theta3= ",theta3)
print("theta4= ",theta4)
print("theta5= ",theta5)
print("theta6= ",theta6)

print("-----------------------------------------------------")

# End efector position: for testing
EE_Position = Matrix([[0.714], [2.241], [2.332]])


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
print("angle_a->",a)
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