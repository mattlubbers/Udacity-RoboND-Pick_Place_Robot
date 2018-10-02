## Project: Kinematics Pick & Place
**Steps to complete the project:**  

1. Set up ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 

---
### Kinematic Analysis of Kuka KR210 Robot
We will be using the Kuka KR210 for this project in a simulated environment to perform the task of calculating necessary joint movement to allow the gripper to pick an item from the shelf, and successfully placing the item into the desired storage bin. A representative image of this task is shown below:

![PickPlaceRobot](/assets/PickPlaceRobot.png)

Before we begin with the implementation of this project, we need to perform the Forward Kinematic Analysis of the Kuka KR210 robot arm by referencing the Kuka URDF for the relationship between the joint links:

![KukaURDF](/assets/KukaURDF.PNG)

#### DH Parameter Tables
Now that we have analyzed the URDF for the Forward Kinematics, we can construct the Denavit-Hartenberg Parameters for the Kuka KR210 are as follows:

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 |- pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |- pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

The DH parameter table is now implemented:
```
    	DH = { 
		        alpha0:    0,  a0:     0, d1: 0.75, q1:          q1,
		        alpha1:-pi/2., a1:  0.35, d2:    0, q2: -pi/2. + q2,
		        alpha2:    0,  a2:  1.25, d3:    0, q3:          q3,
		         alpha3:-pi/2., a3:-0.054, d4:  1.5, q4:          q4,
		        alpha4: pi/2,  a4:     0, d5:    0, q5:          q5,
		        alpha5:-pi/2., a5:     0, d6:    0, q6:          q6,
		        alpha6:    0,  a6:     0, d7:0.303, q7:           0
             }
```
**Transform Matrix:**

With the DH parameters defined, next we can identify the homogeneous transform from the i-1 to i frame. This will consist of:
- Four Transformations
- Two Rotations
- Two Translations

```
    	def TF_Matrix(alpha, a, d, q):
	    	TF = Matrix([
			[           cos(q),           -sin(q),           0,             a],
			[sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
			[sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
			[                0,                 0,           0,             1]
		    	])
		return TF
 ```
 **Call TF_Matrix function to calculate link Transforms:**
 
 We now call the defined function to create the link-link transforms for each adjacent link ranging from the base (T0_1) to the End Effector (T6_EE):
 ```
        T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(DH)
        T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(DH)
        T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(DH)
        T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(DH)
        T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(DH)
        T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(DH)
        T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(DH)
	print('Transforms Calculated')
```
**Transformation Matrix from Base Link to End Effector**

We can now multiply all link to link transforms to calculate a single transform that represents the robot base all the way to the gripper!
```
        T0_EE = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE)
```
### Inverse Position Kinematics
We begin solving the Inverse Position Kinematics by calculating the rotation error between the DH parameters and the URDF. To do this, we must first define the corresponding matricies for rotation of Roll, Yaw, and Pitch:

**Roll, Pitch, and Yaw Rotation Calculation**
```
	# Compensate for rotation discrepancy between DH parameters and Gazebo
	r, p, y = symbols('r p y')	

	#Roll Rotation
   	rotation_x = Matrix([
			 [1,      0,       0],
			 [0, cos(r), -sin(r)],
			 [0, sin(r),  cos(r)]
		       ])
    	#Pitch Rotation
    	rotation_y = Matrix([
                         [ cos(p), 0, sin(p)],
                         [      0, 1,      0],
                         [-sin(p), 0, cos(p)]
                       ])
    	#Yaw Rotation
    	rotation_z = Matrix([
                         [cos(y), -sin(y), 0],
                         [sin(y),  cos(y), 0],
                         [     0,       0, 1]
                       ])
```
**Rotation and Rotation Error at the End Effector**

Now that these matricies have been defined, we call the functions to calculate the Rotation Error for the gripper:
```
#Rotation End Effector
    	rotation_EE = rotation_z * rotation_y * rotation_x
    	rotation_error = rotation_z.subs(y, radians(180)) * rotation_y.subs(p, radians(-90))
    	rotation_EE = rotation_EE * rotation_error
```
**Theta Angle Calculations**

Next, we will begin the process of calculating the theta angles through 4 key steps:

> 1.) Calculate **Theta 1** and **r**

> 2.) Lengths of the triangle between Joint 2, Joint 3, and the Wrist Center (**A**, **B**, **C**)

> 3.) Angles of the triangle between Joint 2, Joint 3, and the Wrist Center (**a**, **b**, **c**)

> 4.) Calculate **Theta 2** and **Theta 4**


**1.) Calculate Theta 1 and r**

First we start by calculating the theta 0 value as well as the r value from the diagram below:

![Wrist_Center](/assets/WristCenter.png)

The calculation for the theta 1 and r values can be found equations below:

![thetaOne_radius_calc](/assets/thetaOne_radius_calc.PNG)

This is implemented as:

```
    	    theta1 = atan2(wrist_center[1], wrist_center[0])
```

**2.) Lengths of the triangle between Joint 2, Joint 3, and the Wrist Center (A, B, C):**

Let's label each of the sides of the triangle, side A and C are already of known length:

![triangle_sides](/assets/triangle_sides.PNG)

If 2 of the 3 sides are of known length, we can calculate the length of side B through the Law of Cosines SSS (3 Sides) as can be found from the reference [here](http://2000clicks.com/mathhelp/geometrylawofsines.aspx).

This can be broken into 3 separate equations which are color coded for reference to indicate how they are used within the calculation for the length of side B:

![side_b_calc](/assets/side_b_calc.PNG)

```
	        # Calculate joint angles using Geometric IK method
    	    s_a = 1.501
    	    s_b = sqrt(pow((sqrt(wrist_center[0] * wrist_center[0] + wrist_center[1] * wrist_center[1]) - 0.35),2)
		 + pow((wrist_center[2] - 0.75),2))
    	    s_c = 1.25
```
**3.) Angles of the triangle between Joint 2, Joint 3, and the Wrist Center (a, b, c)**

After the calculation of the side lengths, we can now proceed to calculate the angles of the triangle seen below as **a**, **b**, and **c**:

![theta_diagram](/assets/theta_diagram.PNG)

These are also simple trigonometry equations using acos of the relationship between the triangle side lengths:

![angle_calcs](/assets/angle_calcs.PNG)

```
    	    #Angles
    	    a_a = acos((s_b * s_b + s_c * s_c - s_a * s_a) / (2 * s_b * s_c))
    	    a_b = acos((s_a * s_a + s_c * s_c - s_b * s_b) / (2 * s_a * s_c))
    	    a_c = acos((s_a * s_a + s_b * s_b - s_c * s_c) / (2 * s_a * s_b))
```
**4.) Calculate Theta 2 and Theta 4**

Now that we have the triangle angles, we can finally calculate theta 2 and theta 3!

![thetaTwo_thetaThree_calc](/assets/thetaTwo_thetaThree_calc.PNG)

```
    	    #Theta 2 and 3
    	    theta2 = pi / 2 - a_a - atan2(wrist_center[2] - 0.75, sqrt(wrist_center[0] * wrist_center[0] +
                                                                wrist_center[1] * wrist_center[1]) - 0.35)
    	    theta3 = pi /2 - (a_b + 0.036)
```
### Inverse Orientation Kinematics
Now that the Inverse Position Kinematics are defined, we need to define the Inverse Orientation Kinematics by calculating the orientations of the gripper by extracting the pose:  

```
       # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            joint_trajectory_point = JointTrajectoryPoint()

	        # Extract end-effector position and orientation from request
     	    pose_x = req.poses[x].position.x
    	    pose_y = req.poses[x].position.y
	        pose_z = req.poses[x].position.z

    	    #End Effector Orientation
    	    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([
								req.poses[x].orientation.x,
								req.poses[x].orientation.y,
								req.poses[x].orientation.z,
								req.poses[x].orientation.w
								])
```
**Calculate the Final Rotation at the End Effector**
```
    	    rotation_EE = rotation_EE.subs({'r': roll, 'p': pitch, 'y': yaw})
```
**Apply End Effector Matrix and Calculate Wrist Center**

Calculate the Wrist Center from the End Effector matrix and the gripper offset of 0.303 meters:
```
    	    #End Effector Matrix
    	    EE_matrix = Matrix([[pose_x], [pose_y], [pose_z]])
    	    wrist_center = EE_matrix - (0.303) * rotation_EE[:,2]
```
**Calculate Euler angles through link Transforms**
```
    	    r0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
            r0_3 = r0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
    	    #r3_6 = r0_3.inv("LU") * rotation_EE
    	    r3_6 = r0_3.transpose() * rotation_EE
    	    #print(simplify(r3_6))
```
**Calculate Theta Angles 4 - 6**
```
    	    #Remaining Joint Angles
    	    theta4 = atan2(r3_6[2,2], -r3_6[0,2])
    	    theta5 = atan2(sqrt(r3_6[0,2] * r3_6[0,2] + r3_6[2,2] * r3_6[2,2]), r3_6[1,2])
    	    theta6 = atan2(-r3_6[1,1], r3_6[1,0])
```
**Append Joint Trajectory Points**

Now that we've done all the difficult work of calculating our theta values, we need to add them to a list for use in the pick and place functionality!
```
            # Populate response for the IK request
	        joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	        joint_trajectory_list.append(joint_trajectory_point)
```
**Run Main** 

Finally, let's write the init main to continue running through the program and making all of the appropriate function calls:
```
def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()
```
### Conclusion and Future Enhancements
This project was very intensive and helped provide a practical approach to the design, implementation, and testing of a real robot that exists throughout the industry!

While the Kinematic calculations were necessary to really understand how the robot motion is defined, I would imagine that there are modeling tools that exist in the industry that do the busy work, and allow the Engineer to focus on other aspects of the design such as controls and communication protocol. It would have been interesting to explore these tools, as well as dive a bit deeper on the specific controls interface for this robot arm application.

Things that could be future enhancements to this base project could include:
- Variation of retrieval object size
- Difficult pick locations with barriers that require specific routing
- Pick locations that are not arranged in a symmetric grid
- Multiple placement locations
- More precise placement location requirements (often we will not be able to drop into a large bin)
- Optimization for speed of processing (pick and place seemed slow compared to manufacturing use-cases)
















