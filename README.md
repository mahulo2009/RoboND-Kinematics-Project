# Project: Kinematics Pick & Place

[//]: # (Image References)

[image1]: ./misc_images/Kuka_arm_001.png
[image2]: ./misc_images/Kuka_arm_002.png
[image3]: ./misc_images/Kuka_arm_003.png
[image4]: ./misc_images/Kuka_arm_004.png
[image5]: ./misc_images/Kuka_arm_005.png
[image6]: ./misc_images/Kuka_arm_006.png
[image7]: ./misc_images/Kuka_arm_007.png
[image8]: ./misc_images/math-001.png
[image9]: ./misc_images/math-002.png


## Kinematic Analysis

### Forward Kinematic

#### Overview KR210 Forward Kinematic

In this section we are going to do the forward kinematic of the Kuka KR210 serial manipulator. The KR210 is a robot arm with six degrees of freedom. The following picture show the schematic of the arm in its zero configuration: when all the joint variables are equal to zero.

![alt text][image1]

#### KR210 Forward Kinematic DH Parameters


To construct the DH parameter table for the manipulator the following steps must be made:

* Labeling the joints from 1 to n.
* Labeling each link from 0 to n.
* Draw lines defining each joint axes.
* Define directions for the positive Z axes.
* Define directions for the X axes as the common normals between Zi-1 and Zi axis.
  * For skew axes, along the normal between Zi an Zi-1, and pointing from i to i+1
  * For intersecting axes, normal to the plane containing Zi and Zi+1
  * For parallel or coincident axes, the way to make other DH Parameters equal to zero.
* Define the base and end efector frame.

![alt text][image2]

The DH Parameter table is filled up:

* α​i−1 (twist angle) = angle between Z^​​​i−1 and Z^​i measured about X^​i−1 in a right-hand sense.
* ai−1(link length) = distance from Z​^​i−1 to Z^​i measured along X^​i−1 where X^​i−1 is perpendicular to both Z^​i−1 to Z^​i
* di (link offset) = signed distance from X^​​i−1 to X^​i measured along Z^​i.
* θi (joint angle) = angle between X^​​​i−1 to X^​​​i measured about Z^​i in a right-hand sense

![alt text][image3]

The DH Parameter table:

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | d1 | q1
1->2 | -pi/2 | a1 | 0 | q2 -pi/2
2->3 | 0 | a2 | 0 | q3
3->4 | -pi/2 | a3 | d4 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | d7 | 0

The URDF file contains the information to get the numerical values for a's and d's. The following table shows the information extracted from this file.


Joint Name | Parent Link | Child Link | x(m) | y(m) | z(m)
--- | --- | --- | --- | --- | ---
joint_1 | base_link | link_1 | 0 | 0 | 0.33
joint_2 | link_1 | link_2 | 0.35 | 0 | 0.42
joint_3 | link_2 | link_3 | 0 | 0 | 1.25
joint_4 | link_3 | link_4 | 0.96 | 0 | -0.054
joint_5 | link_4 | link_5 | 0.54 | 0 | 0
joint_5 | link_5 | link_6 | 0.193 | 0 | 0
gripper_joint | link_6 | gripper_link | 0.11 | 0 | 0

In most case the joint origins in the URDF file are not consistent with the frame origins created with the DH parameter convetions nor do they have the same orientation. The following picture show the reference frames as defined in the URDF file. The difference are highlighted in red squares.

* X4 is located in Joint 4 rathen than Joint 5.
* X5 different orientation.
* Frame gripper different orientation.

![alt text][image4]

Taking in consideration these differences the a's and d's can be calculated as follow:

* d1 =  0.330 + 0.42 = 0.75
* a1 =  0.35
* a2 =  1.25
* a3 = -0.0054
* d4 =  0.960 + 0.54 = 1.5
* d7 =  0.193 + 0.11 = 0.303

This is the final DH parameter table.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | -pi/2 | 0.35 | 0 | q2 -pi/2
2->3 | 0 | 1.25 | 0 | q3
3->4 | -pi/2 | -0.054 | 1.5| q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0  |  q6
6->EE | 0 | 0 | 0.303 | 0


#### Homogeneous transform

From frame 0 to frame 1:
[  
[cos(q1), -sin(q1), 0,    0],  
[sin(q1),  cos(q1), 0,    0],  
[      0,        0, 1, 0.75],  
[      0,        0, 0,    1]  
]  

From frame 1 to frame 2:
[  
[sin(q2),  cos(q2), 0, 0.35],  
[      0,        0, 1,  0.0],  
[cos(q2), -sin(q2), 0,    0],  
[      0,        0, 0,    1]  
]  

From frame 2 to frame 3:
[  
[cos(q3), -sin(q3), 0, 1.25],  
[sin(q3),  cos(q3), 0,    0],  
[      0,        0, 1,  0.0],  
[      0,        0, 0,    1]  
]  

From frame 3 to frame 4:
[  
[ cos(q4), -sin(q4), 0, -0.054],  
[       0,        0, 1,    1.5],  
[-sin(q4), -cos(q4), 0,      0],  
[       0,        0, 0,      1]  
]  

From frame 4 to frame 5:
[  
[cos(q5), -sin(q5),  0, 0.0],  
[      0,        0, -1,   0],  
[sin(q5),  cos(q5),  0,   0],  
[      0,        0,  0,   1]  
]  

From frame 5 to frame 6:  
[  
[ cos(q6), -sin(q6), 0, 0.0],  
[       0,        0, 1, 0.0],  
[-sin(q6), -cos(q6), 0,   0],  
[       0,        0, 0,   1]  
]  

From frame 6 to frame 7:  
[  
[1, 0, 0,   0.0],  
[0, 1, 0,     0],  
[0, 0, 1, 0.303],  
[0, 0, 0,     1]  
]  


From frame 0 to frame gripper (making the rotation correction of the gripper frame):  

[  
[-7.49879891330929e-33*((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) - 6.12323399573677e-17*((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) - 1.0*(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) - 6.12323399573677e-17*(sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*sin(q6) + 7.49879891330929e-33*(sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*cos(q6) + 1.0*cos(q1)*cos(q5)*cos(q2 + q3), 1.0*((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) - 1.22464679914735e-16*((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) - 1.22464679914735e-16*(sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*sin(q6) - 1.0*(sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*cos(q6), 1.22464679914735e-16*((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) + 1.0*((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) - 6.12323399573677e-17*(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + 1.0*(sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*sin(q6) - 1.22464679914735e-16*(sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*cos(q6) + 6.12323399573677e-17*cos(q1)*cos(q5)*cos(q2 + q3), -0.303*(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*cos(q1) + 0.303*cos(q1)*cos(q5)*cos(q2 + q3)],  
[-7.49879891330929e-33*((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) - 6.12323399573677e-17*((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) + 6.12323399573677e-17*(sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*sin(q6) - 7.49879891330929e-33*(sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*cos(q6) - 1.0*(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + 1.0*sin(q1)*cos(q5)*cos(q2 + q3), 1.0*((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) - 1.22464679914735e-16*((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) + 1.22464679914735e-16*(sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*sin(q6) + 1.0*(sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*cos(q6), 1.22464679914735e-16*((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) + 1.0*((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) - 1.0*(sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*sin(q6) + 1.22464679914735e-16*(sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*cos(q6) - 6.12323399573677e-17*(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + 6.12323399573677e-17*sin(q1)*cos(q5)*cos(q2 + q3), -0.303*(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + (1.25*sin(q2) - 0.054*sin(q2 + q3) + 1.5*cos(q2 + q3) + 0.35)*sin(q1) + 0.303*sin(q1)*cos(q5)*cos(q2 + q3)],  
[7.49879891330929e-33*(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) + 6.12323399573677e-17*(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) + 6.12323399573677e-17*sin(q4)*sin(q6)*cos(q2 + q3) -7.49879891330929e-33*sin(q4)*cos(q6)*cos(q2 + q3) - 1.0*sin(q5)*cos(q4)*cos(q2 + q3) - 1.0*sin(q2 + q3)*cos(q5),-1.0*(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) + 1.22464679914735e-16*(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) + 1.22464679914735e-16*sin(q4)*sin(q6)*cos(q2 + q3) + 1.0*sin(q4)*cos(q6)*cos(q2 + q3),-1.22464679914735e-16*(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) - 1.0*(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) - 1.0*sin(q4)*sin(q6)*cos(q2 + q3) + 1.22464679914735e-16*sin(q4)*cos(q6)*cos(q2 + q3) - 6.12323399573677e-17*sin(q5)*cos(q4)*cos(q2 + q3) - 6.12323399573677e-17*sin(q2 + q3)*cos(q5),-0.303*sin(q5)*cos(q4)*cos(q2 + q3) - 0.303*sin(q2 + q3)*cos(q5) - 1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75],  
[0, 0, 0,     1]  
] 

### Inverse Kinematic

The last three joints of the Kuka arm are revolute and theis joint axes intersect at a single point. The joint_5 is the wrist center, the common intersection point. This allows us to kinematically decouple the IK problem into Inverse Position and Inverse Orientation problem. 

#### Inverse Position

Starting from the pose of End Efector: position and orientation; the location of the wrist can be found :
![alt text][image8]

In this case, d equals 0.303, the distance from joint 5 to the final effector. The Rotation Matrix of frame 0 to frame EE can be found using the roll, pitch and yaw of the EE. Finally, because the orientation of the EE differs from that calculated with the DH parameters, it is necessary to apply a rotation matrix that corrects this difference.


Matrix to correct orientation of the EE:

R_z = Matrix([[ cos(np.pi),    -sin(np.pi),    0],  
              [sin(np.pi),     cos(np.pi),     0],  
              [0,              0,              1],])  

R_y = Matrix([[ cos(-np.pi/2.),  0,  sin(-np.pi/2.)],  
              [ 0,               1,              0,],  
              [ -sin(-np.pi/2.), 0,  cos(-np.pi/2.)],])  
R_corr = R_z * R_y


Orientation Matrix of the EE based on roll, pitch and yaw:

R_EEx = Matrix([[ 1,         0,           0],  
                [ 0, cos(roll),  -sin(roll)],  
                [ 0, sin(roll),  cos(roll)]])  

R_EEy = Matrix([[ cos(pitch),  0,  sin(pitch)],  
                [          0,  1,           0],  
                [-sin(pitch),  0,  cos(pitch)]])  

R_EEz = Matrix([[ cos(yaw), -sin(yaw),        0],  
                [ sin(yaw),  cos(yaw),        0],  
                [ 0,                0,        1]])  

R_EE_corr = R_EEz * R_EEy * R_EEx * R_corr 

Location of the WC:

WC_location = EE_Position - (0.303) * R_EE_corr_eval[:,2]

#### Orientation

The next step is to find the joint variables, q1, q2, and q3, such that the WC has the coordinates calculated previously. The following image shows the way to do it.

![alt text][image5]

theta1 = math.atan2(WC_y,WC_x)

![alt text][image6]

theta2 =  pi/2 - alfa - beta

To calculate alfa the cosine law is used.

A² =   B² + C² - 2*B*C*cos(alfa)
alfa = acos((B²+C²-A²)/(2*B*C))

From the image it can see that:

B = sqrt ( W² + L² )
W = sqrt ( WC_x² + WC_y² ) - a1
L = Wc_z - d1

C = a2
A = sqrt ( d4² + a3² )

beta = atan(L,W)


![alt text][image7]

theta3 = pi/2 - alfa - beta

To calculate alfa the cosine law is used.

B² =   A² + C² - 2*A*C*cos(alfa)
alfa = acos((A²+C²-B²)/(2*A*C))

From the image it can see that:

beta = atan(a3,d4)

Once the first three joint variables are known, calculate the Rotation Matrix from frame 3 to frame 6 using the following formula:

![alt text][image9]

The symbolic formula for the rotation matrix from frame three to frame six is equal to:

('R_3_6->', Matrix([
[-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4)],  
[                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5)],  
[-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5)]]))  

R_3_6[2,2]/R_3_6[0,2]= sin(q4)*sin(q5)/-sin(q5)*cos(q4)=-sin(q4)/cos(q4) 
theta4 = atan(R_3_6[2,2],-R_3_6[0,2] 

R_3_6[0,2]² + R_3_6[2,2]² = (-sin(q5)*cos(q4))² + (sin(q4)*sin(q5))² = sin(q5)² * ( sin(q4)² + cos(q4) ² ) = sin(q5)²  
R_3_6[1,2] = cos(q5) 
theta5 = atan( sqrt (  R_3_6[0,2]² + R_3_6[2,2]² ) , R_3_6[1,2] ) 


R_3_6[1,1]/R_3_6[1,0]= -sin(q5)*sin(q6) / sin(q5)*cos(q6) = -sin(q6)/cos(q6) 
theta6 = atan(-R_3_6[1,1],R_3_6[1,0]) 



## Project Implementation

This section will explain the IK_server code in detail


A function to create the transform between adjacent links:

```python

def DH_matrix (alpha,a,q,d):
    T = Matrix([   [cos(q),            -sin(q),            0,                         a],
                 [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha),   -sin(alpha)*d],
                 [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),    cos(alpha)*d],
                 [                 0,                 0,           0,               1]])
    T = T.subs(s)
    return T
```

Functions to create the rotation matrix x, y and z axes:

```python
def Rot_X_matrix (alpha):
    Rx = Matrix([[ 1,           0,   0],
          [ 0,      cos(alpha),    -sin(alpha)],
          [ 0,      sin(alpha),    cos(alpha)]])
    return Rx

def Rot_Y_matrix (alpha):
    Ry = Matrix([[ cos(alpha),    0,  sin(alpha)],
            [    0,    1,    0],
            [-sin(alpha),    0,    cos(alpha)]])
    return Ry

def Rot_Z_matrix (alpha):
    Rz = Matrix([[ cos(alpha), -sin(alpha),        0],
            [ sin(alpha),  cos(alpha),        0],
            [ 0,              0,        1]])
    return Rz
```

The individual transform matrix between adjacents links and the transform from base like to the End Efector.


```python
# Individual Transformation matrix
#
#
T0_1 = DH_matrix(alpha0,a0,q1,d1)
T1_2 = DH_matrix(alpha1,a1,q2,d2)
T2_3 = DH_matrix(alpha2,a2,q3,d3)
T3_4 = DH_matrix(alpha3,a3,q4,d4)
T4_5 = DH_matrix(alpha4,a4,q5,d5)
T5_6 = DH_matrix(alpha5,a5,q6,d6)
T6_7 = DH_matrix(alpha6,a6,q7,d7)

# Transformation matrix from 0 to 7
#
#
T0_7 = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_7
```

Matrix to correct the gripper reference orientation as defined in the URDF files versus de DH parameters.

```python
# Orientation correction matrix
#
#
R_z = Rot_Z_matrix(np.pi)
R_y = Rot_Y_matrix(-np.pi/2.0)
R_corr = R_z * R_y
```
 
Orientation matrix calculate from the roll, pitch and yaw values. I will be evaluated for every end-effector pose.

```python
# Orientation matrix: roll, pitch and yaw
#
#
beta0, beta1, beta2 = symbols('beta0:3')    
R_EEx = Rot_X_matrix(beta0)
R_EEy = Rot_Y_matrix(beta1)
R_EEz = Rot_Z_matrix(beta2)

R_EE_corr = R_EEz * R_EEy * R_EEx * R_corr 
```
Symbolic Orientation matrix from  reference frame 0 to reference frame 3 (WC).


```python
# Orientation matrix from frame 0 to 3
#
#
R0_1 = T0_1.extract([0,1,2],[0,1,2])
R1_2 = T1_2.extract([0,1,2],[0,1,2])
R2_3 = T2_3.extract([0,1,2],[0,1,2])

R_0_3 = R0_1  * R1_2 * R2_3
```

```python
        # Extract end-effector position and orientation from request
        # px,py,pz = end-effector position
        # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

        # Compensate for rotation discrepancy between DH parameters and Gazebo
        #
        #
            R_EE_corr_eval = R_EE_corr.evalf(subs={beta0: roll, beta1:pitch, beta2:yaw})

```

```python
        # Calculate joint angles using Geometric IK method
        #
        #
            EE_Position = Matrix ( [[px],[py],[pz]] ) 
            # WC position
            WC_location_eval = EE_Position  - (0.303) * R_EE_corr_eval[:,2]
            
            # Calculate the first three joint angles using trigronometry	
            L=WC_location_eval[2]-0.75
            W=math.sqrt(math.pow(WC_location_eval[0],2)+math.pow(WC_location_eval[1],2))-0.35
            
            A = 1.500971685275908
            B = math.sqrt(math.pow(L,2)+math.pow(W,2))
            C = 1.25
            
            angle_a = math.acos((B*B+C*C-A*A)/(2*B*C))
            angle_b = math.acos((A*A+C*C-B*B)/(2*A*C))
    
            theta1 = math.atan2(WC_location_eval[1],WC_location_eval[0])
            theta2 = math.pi/2.0-angle_a-math.atan2(L,W)
            theta3 = math.pi/2.0-(angle_b+0.036)
```

```python
            # Calculate the rotation matrix from frame 0 to frame 3
            R_0_3_eval = R_0_3.evalf(subs={q1: theta1, q2:theta2, q3:theta3})
            R_0_3_Inv = Transpose(R_0_3_eval)
            # Calculate the rotation matrix from frame 3 to frame 6.
            R_3_6 = R_0_3_Inv * R_EE_corr_eval   

            # Calculate the last three angles using the simbolyc matrix rotation from frame 3 to frame 6
            theta4 = math.atan2(R_3_6[2,2],-R_3_6[0,2])
            theta5 = math.atan2( math.sqrt( math.pow(R_3_6[0,2],2)+math.pow(R_3_6[2,2],2)), R_3_6[1,2] )
            theta6 = math.atan2(-R_3_6[1,1],R_3_6[1,0])

```