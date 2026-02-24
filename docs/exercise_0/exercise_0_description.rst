Exercise 0: Coordinate Transformations
==================================================

In this exercise, you will learn how to implement a coordinate transform for a drone using euler angles.
This will introduce you to Webots and help you to get familiar with some of the tools you will be using throughout the course.

Task Overview
-------------
For this task you will need to convert commands for a drone from the inertial frame to the body frame.
While normally for First Person View (FPV) flying we want commands in the body frame, sometimes we want them in the inertial frame instead, such as when flying from an external viewpoint like the 3D view in Webots. This requires a transformation to account for the drone's current orientation.

To start, you can run the simulation in Webots by opening the world file (**crazyflie_world_exercise.wbt**) and clicking the play button.
You should see a drone in the simulation that you can control using the keys on your keyboard as follows:
  - **W**: Move the drone forward
  - **S**: Move the drone backward
  - **A**: Move the drone left
  - **D**: Move the drone right
  - **Q**: Turn the drone left
  - **E**: Turn the drone right
  - **X**: Move the drone up
  - **Z**: Move the drone down

By looking at the 3D view of the scene and the FPV camera from the drone, you can see that the drone moves
relative to its own orientation rather than the world, since the controller operates in the body frame.

This behaviour works well for FPV piloting, where a human operator controls the drone relative to
the FPV view. However, there are many scenarios where it is more natural or necessary to issue
commands in the inertial (world) frame. A common example is **headless mode**: a flight mode popular
in consumer drones where "forward" always means the same fixed direction in the world, regardless of
where the drone is pointing. This is useful for autonomous waypoint navigation, GPS-based control, or
simply when the operator has lost track of the drone's heading.

.. image:: before_transform.gif
  :width: 650
  :alt: Without the transformation the drone moves relative to its own orientation

To fix this, you will need to implement a function to convert the control commands from the inertial
frame to the body frame. This will involve using the drone's current orientation, given as a set of
Euler angles, to calculate the inverse rotation that maps world-frame velocity commands into the
drone's local frame. Once implemented, pressing "forward" will always move the drone in the same
world-space direction regardless of its yaw, enabling headless-mode behaviour.

.. image:: after_transform.gif
  :width: 650
  :alt: When the rotation is implemented correctly the drone moves in the inertial frame regardless of its orientation

After making changes in the code you need to reload the world file in Webots to see the changes. This is done by clicking the **Reload World** button in the Webots interface.

.. image:: webots_reload_button.png
  :width: 650
  :alt: Overview of the frame transformation process


Exercise
---------

1. Start by opening the **ex0_rotations.py** file and locating the **euler2rotmat(euler_angs)** function. Implement the function to calculate the rotation matrix based on the given Euler angles. Remember, Euler angles represent roll, pitch, and yaw of the drone in the inertial frame.

2. Next, implement the **rot_inertial2body(control_commands, euler_angs)** function. Use the rotation matrix you obtained from **euler2rotmat(euler_angs)** to transform the velocity commands from the inertial frame into the body frame. Recall that the inverse of a rotation matrix is simply its transpose. You do not need to rotate the altitude command, just the horizontal velocity commands.

3. Test your implementation in the Webots simulation environment (**crazyflie_world_exercise**). You should observe that the drone now operates in headless mode: the control commands are always interpreted relative to the inertial frame, so pressing "forward" moves the drone in the same world-space direction regardless of its current yaw.

Bonus challenge
---------------
To further test your skills, see if you can complete the same task using quaternions instead of Euler angles.

====================================================================================
Any questions about the exercise, please contact Benjamin Jarvis (benjamin.jarvis@epfl.ch).
