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
In **main.py**, **choose exp_num = 4** for this assignment. 

Switch **control_syle = 'keyboard'** to fly on your own. 

Switch **control_syle = 'path_planner'** to choose automatic control. You can edit your own algorithm in **my_assignment.py** and **ex1_pid_control.py** (if you want to tune the PID gains).

For submission and grading, we only consider those two files. Details on how to submit can be found in the section “Leaderboard”.