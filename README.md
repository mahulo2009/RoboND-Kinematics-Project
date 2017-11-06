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
![alt text][image1]
#### KR210 Forward Kinematic reference frames
![alt text][image2]
#### KR210 Forward Kinematic DH parameters
![alt text][image3]

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | d1 | q1
1->2 | -pi/2 | a1 | 0 | q2 -pi/2
2->3 | 0 | a2 | 0 | q3
3->4 | -pi/2 | a3 | q4 | d4
4->5 | pi/2 | 0 | q5 | 0
5->6 | -pi/2 | 0 | q6 | 0
6->EE | 0 | 0 | 0 | d7


#### KR210 Forward Kinematic modified DH parameters
![alt text][image4]

Joint Name | Parent Link | Child Link | x(m) | y(m) | z(m)
--- | --- | --- | --- | --- | ---
joint_1 | base_link | link_1 | 0 | 0 | 0.33
joint_2 | link_1 | link_2 | 0.35 | 0 | 0.42
joint_3 | link_2 | link_3 | 0 | 0 | 1.25
joint_4 | link_3 | link_4 | 0.96 | 0 | -0.054
joint_5 | link_4 | link_5 | 0.54 | 0 | 0
joint_5 | link_5 | link_6 | 0.193 | 0 | 0 
gripper_joint | link_6 | gripper_link | 0.11 | 0 | 0


d1 =  0.330 + 0.42 = 0.75
a1 =  0.35
a2 =  1.25
a3 = -0.0054
d4 =  0.960 + 0.54 = 1.5
d7 =  0.193 + 0.11 = 0.303


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
