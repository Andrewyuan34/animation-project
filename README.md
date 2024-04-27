# Inverse Kinematics

In this assignment, your task is to develop an Inverse Kinematics engine for animating a human character. The human character is to be positioned in front of a blackboard. Your program should read an arbitrary shape represented by a 2D spline (use code from assignment 1 or the spline class provided). The character should be able to draw (i.e trace) the spline on the blackboard.

The program should work as follows. Through the scripting, the commands read an arbitrary spline. Once the spline is read, pressing the key “s” should make the character draw the spline repeatedly. The drawing should stop when “s” is pressed again. Please read the entirety of this assignment text as it includes important requirements and several notes at the end. There is also a short lecture associated with this assignment to help guide you. which we will cover in class.

## Tasks

1.Modeled a classroom using one wall whose visible surface is the z = 0 plane, where the blackboard is attached, and a planar floor with the y-axis as its normal.
2.The character is initially in the rest pose. After a spline is loaded, the character switches to the Ready pose for drawing on the blackboard.
3.Implemented functionality to read arbitrary splines and project them onto the blackboard.
4.Modeled a human character using ellipses.
5.Implemented CCDIK and PseudoInverse methods to optimize arm motion paths for smooth animations.
6.Developed interpolation system, including Hermite and Catmull-Rom splines, for controlled motion curves.
7.Employed Point Picking with automatic interpolation technology to accurately add control points from any viewing angle.
