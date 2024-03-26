Simulation quickstart
=====================

Choose world
--------------
Make sure you load **crazyflie_world_assignment.wbt** in Webots.

Remember to select 'Close without Saving' when you close Webots software, such that you will not change the simulated world environments.

Install relevant packages
--------------------------

Besides the previously installed packages, install OpenCV by executing:

**pip install opencv-python**

Controller switch
-----------------
In **main.py**, **choose exp_num = 3** for this assignment. 

Switch **control_syle = 'keyboard'** to fly on your own (W: forward, S: backward, A: left, D: right, Q: yaw left, E: yaw right, C: lower altitude, V: higher altitude). 

Switch **control_syle = 'autonomous'** to choose automatic control. You can edit your own algorithm in **my_control.py**, **get_command()**. This function maps sensor inputs to control commands (velocity forwards, velocity sidewards, altitude, yaw rate).

For submission and grading, we only consider the **my_control.py** file. Details on how to submit can be found in the section “Leaderboard”.