## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
[image4]: ./misc_images/misc4.png
[image5]: ./misc_images/misc5.png
[image6]: ./misc_images/misc6.png
[image7]: ./misc_images/misc7.png
[image8]: ./misc_images/misc8.png
[image9]: ./misc_images/misc9.png
[image10]: ./misc_images/misc10.png
[image11]: ./misc_images/misc11.png
[image12]: ./misc_images/misc12.png
[image13]: ./misc_images/misc13.png
[image14]: ./misc_images/misc14.png
[image15]: ./misc_images/misc15.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Here is an image with the reference frames shown. 

![alt text][image1]

We can drive the DH parameters as follows:

* alpha(i-1) : Angle between Z_(i-1) and Z_(i) measured along X_(i-1)

* a_(i-1) : Link length, distance between Z_(i-1) and Z_(i), measured along X_(i-1)

* d_(i) : Link offset, distance between X_(i-1) and X_(i), measured along Z_(i-1), variable in prismatic joints(there are no prismatic joints in the given problem)

* theta_(i) : Joint angle, Angle between X_(i-1), X_(i) measured along Z_(i), variable in revolute joints.

The table is shown below

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  - pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.193 | 0



#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The transform from reference frame_i to frame_(i-1) will be as shown below:

![alt text][image4]

This matrix is the homogeneous transform between two frames. In order to make a generalized transform between the base_link and the gripper_link, we can simply multiply the transformation matrices with each other to move from the end effector to the base frame.
This will be on this shape: `T0_EE = T0_1*T1_2*T2_3*T3_4*T4_5*T5_6*T6_EE`

In our case, the reference frames should be rotated as the urdf reference and our specified reference were different from one another. So, we added a correction rotation matrix.

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

here are two images showing the decoupling:

![alt text][image5]

![alt text][image6]

The first one i got from the web. However, i think it's great from our purpose. 
Using that decoupling, we can now have 2 seperate small problems. The first one is to get the position of the wrist center, and the second one is to get the orientation of the EE. 
Since we have the position of the EE, we can get the position of the wrist center as follows

![alt text][image7]

Where,

Px, Py, Pz = end-effector positions

Wx, Wy, Wz = wrist positions

d6 = from DH table

we can also get the rotation matrix using the following formula

![alt text][image8]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


The whole idea of my solution was that the inverse kinematics problem can be decoupled into Inverse Position Kinematics and Inverse Orientation Kinematics, which are a good problem to solve. 

The hardest problem was that of the angles. The result is good though. Every can goes to the bin eventually. One of the potential imporvement is that the angles can be normalized into the range between 0 to pi so that one joint would not rotate one whole circle but do nothing in fact.  

Here are some images showing the process:

![alt text][image9]

![alt text][image10]

![alt text][image11]

![alt text][image12]

![alt text][image13]

![alt text][image14]

![alt text][image15]

That's it for my implementation. Hope everything is clear.

