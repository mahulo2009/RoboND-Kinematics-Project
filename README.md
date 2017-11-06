# Project: Kinematics Pick & Place

[//]: # (Image References)

[image1]: ./misc_images/Kuka_arm_001.png
[image2]: ./misc_images/Kuka_arm_002.png
[image3]: ./misc_images/Kuka_arm_003.png
[image4]: ./misc_images/Kuka_arm_004.png
[image5]: ./misc_images/Kuka_arm_005.png
[image6]: ./misc_images/Kuka_arm_006.png
[image7]: ./misc_images/Kuka_arm_007.png


## Kinematic Analysis

### Forward Kinematic

#### Overview KR210 Forward Kinematic

In this section we are going to do the forward kinematic of the Kuka KR210 serial manipulator. The KR210 is a robot arm with six degrees of freedom. The following picture show the schematic of the arm in its zero configuration: when all the joint variables are equal to zero.

![alt text][image1]

#### KR210 Forward Kinematic DH Parameters


To construct the DH parameter table for the manipulator the following steps must be made:

* Labeling the joints from 1 to n.
* Labeling each link from 0 to n.
* Draw lines defining each joint axis.
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
3->4 | -pi/2 | a3 | q4 | d4
4->5 | pi/2 | 0 | q5 | 0
5->6 | -pi/2 | 0 | q6 | 0
6->EE | 0 | 0 | 0 | d7

The URDF file contains the information to get the numerical values for a's and d's. The following table show the information extracted from this file.


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
3->4 | -pi/2 | -0.054 | q4 | 1.5
4->5 | pi/2 | 0 | q5 | 0
5->6 | -pi/2 | 0 | q6 | 0
6->EE | 0 | 0 | 0 | 0.303


#### Homogeneous transform



### Inverse Kinematic

#### Position: Wrist center location
![alt text][image5]
![alt text][image6]
![alt text][image7]
#### Orientation

## Project Implementation
