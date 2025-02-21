Webots
==================================================
This page will give you a short introduction about Webots and how to install it.
If you want additional information, Webtos is very well documented and you can find the documentation here: `Webots Documentation <https://cyberbotics.com/doc/guide/index>`_.

Webots Introduction
-------------------
During this course we will use the Webots simulation environment to test code on a simulated drone.
Webots is an open-source robot simulator that provides a complete development environment to model, program, and simulate robots.
It is widely used in research and education to develop and test algorithms for robotics.

A webots simulation is defined by a world file, which specifies the environment and any robots or objects within it.
You can interact with the simulation using the Webots GUI, which allows you to control the robot, view sensor data, and visualize the environment.
You can even open code files and modify them in Webots, though you will need to reload the world for these changes to take effect.
If you prefer, you can also use your favorite text editor or IDE to edit the code files.

Webots Installation
-------------------
Download Webots R2023b according to the operation system of your laptop and the following commands.
If necessary, more information can be found here: `Webots Documentation <https://cyberbotics.com/doc/guide/installation-procedure>`_.

- For Ubuntu 20.04 & 22.04, you will download the `webots_2023b_amd64.deb <https://github.com/cyberbotics/webots/releases/download/R2023b/webots_2023b_amd64.deb>`_. To install the deb file, run the following command in your terminal:

.. code-block:: console

	$ sudo apt install ./webots_2023b_amd64.deb

- For Ubuntu 24.04, you will download the `webots_2025a_amd64.deb <https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb>`_. To install the deb file, run the following command in your terminal:

.. code-block:: console

	$ sudo apt install ./webots_2025a_amd64.deb

- For Windows 10, you will download and install the `webots-R2023b_setup.exe <https://github.com/cyberbotics/webots/releases/download/R2023b/webots-R2023b_setup.exe>`_.
- For macOS, you will download and install the `webots-R2023b.dmg <https://github.com/cyberbotics/webots/releases/download/R2023b/webots-R2023b.dmg>`_. For error like 'unidentified developer', try 'right-click' the application and choose 'open'. For error of 'Python was not found' in Webots, put the correct Python path in 'Webots->preferences->python command'. To find the python path, open terminal, type 'python3', type 'import sys', and type 'print(sys.executable)'.

For library missing error, you can type 'pip3 install numpy' or 'pip3 install matplotlib' in the terminal. For more information about Webots, refere to `Webots website <https://cyberbotics.com/>`_.