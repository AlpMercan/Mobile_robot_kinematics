# Mobile_robot_kinematics
An attempt to model turtlebot3 burger on a matlab and using forward and inward kinematics 
make the robot do specific actions
![Step1](https://github.com/AlpMercan/Mobile_robot_kinematics/assets/112685013/cf2d8ccd-05b4-4682-8572-c90ba5bb4b3a)

Part#1: Make an animation in MATLAB and control a robot-like element (like a box with two
wheels on each side) in the plot part (considered as a map). Don’t forget to differentiate the front
of the robot from the back in the animation (like we do on the board by adding the spherical wheel).
You don’t need to derive equations of motion yet; you only need to be able to generate an
animation of this object.
![Step2ileri](https://github.com/AlpMercan/Mobile_robot_kinematics/assets/112685013/d0d69cc8-c41f-4ebf-a5b3-01e961ffb5f7)

vr=0.1,vl=0.1

![Step2dönme](https://github.com/AlpMercan/Mobile_robot_kinematics/assets/112685013/824b7be1-593c-4697-8117-43c9ba3a9d74)

vr=0.1 vl=-0.1

Part#2: Derive and code the TurtleBot’s full kinematics, explaining all assumptions, constraints,
and steps in MATLAB. Additional documents about TurtleBot’s design will be uploaded
separately. The kinematics equations will be almost identical to what we have done in class. Show
that if you provide specific wheel velocities, the robot will turn in place, steer right or left, or go
straight.

![Step3](https://github.com/AlpMercan/Mobile_robot_kinematics/assets/112685013/019ae1cf-04ec-4c85-b775-4bf2982f0b9c)

a robot following a path with open loop control

Part#3: Open-loop control the robot through a pre-defined path using the kinematics you coded in
part 2 in MATLAB – assuming no-slip condition without any noise. You will need the inverse
differential kinematics for this part. The path can be anything you choose but should not be just a
straight line.

![Step4](https://github.com/AlpMercan/Mobile_robot_kinematics/assets/112685013/9813544b-4959-42bb-8575-256c7871d017)

a robot following a path with open loop control closed loop control

Part#4: Write a feedback controller for the robot using the pose as the robot feedback. Move the
robot through a pre-defined path with the help of the controller. You can use any feedbac kcontroller; however,
using the feedback controller we discussed in class might be easier. The path can be anything you choose but should not be just a straight line.
