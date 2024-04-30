Hardware quickstart
===================

Hardware unpacking
------------------
Every team will receive a box with the necessary hardware. This includes:

* One Crazyflie package

  * 1 x Crazyflie 2.1 with all components mounted
  * 1 x Crazy-radio dongle
  * 3 x LiPo battery (240mAh)
  * 1 x Battery charger
  * 1 x USB cable

* One spare parts set

  * 1 x spare motor
  * 2 x spare motor mounts
  * 5 x spare propellers CW
  * 5 x spare propellers CCW

* One flow deck v2
* One multi-ranger deck

Change battery and broken parts
-------------------------------
The Crazyflie has been already assembled. However, in case you need to replace the broken parts like propellers, refer to the instructions `here <https://www.bitcraze.io/getting-started-with-the-crazyflie-2-0/#assembling>`_.

Before powering on the drone, ensure that the multi-ranger and flow **decks are mounted correctly**, with the forward arrows aligned with the antenna on the Crazyflie body, as shown in the left animation.
**To mount a new battery**, follow the steps shown in the right animation.
Note that there is **a small power button** located in front of the quadrotor that can be used to restart the drone if needed.

.. image:: battery_deck_direction.gif
  :width: 550
  :alt: battery_and_deck_direction

.. image:: on_off.gif
  :width: 300
  :alt: on_off

**To replace the propellers**, make sure to identify the correct type of propeller.
Look for the arrows on the drone arm near the motors.
Arrows with a clockwise direction indicate Type A propellers, while arrows with a counterclockwise direction indicate Type B propellers.

Onboaed LEDs indicate drone states such as low power, self-test fail, etc. Refer to `bitcraze page <https://www.bitcraze.io/documentation/tutorials/getting-started-with-crazyflie-2-x/#leds>`_.

Software installation
---------------------
For developing the code for the Crazyflie, you will first need to install Python 3.8+ if you don't have it yet.
Then, you can clone and install the Crazyflie Python Library, to communicate with and control the Crazyflie.

For both Ubuntu and Windows, you can install the library by running following commands in your terminal:

.. code-block:: bash

    git clone https://github.com/bitcraze/crazyflie-lib-python.git
    cd crazyflie-lib-python
    git checkout tags/0.1.22 -b v0.1.22-branch
    pip3 install -e .

Possible installation issues: 1. In Windows, if pip3 command is not found, then you need to use pip instead; 2. Useful `links <https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/installation/install/>`_ to Python and pip issues on Windows.

To connect to the drone and visualize its measurements, you can install the Crazyflie client. To do this, follow the instructions under the section "Installing from source" on the site:
`links <https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/installation/install/>`_. If there are issue with launching the crazyflie client, install both the crazyflie-lib and crazyflie-client it in a new conda / virtual environment.


The next step is to configure the radio driver:

- Ubuntu: Look at the `usb permission instructions <https://github.com/bitcraze/crazyflie-lib-python/blob/master/docs/installation/usb_permissions.md>`_ to setup udev on linux.
- Windows: Look at the `Zadig crazyradio instructions <https://www.bitcraze.io/documentation/repository/crazyradio-firmware/master/building/usbwindows/>`_ to install crazyradio on Windows

Change radio address
--------------------
Each drone has a unique address for communication between your laptop and the drone.
**Crazyflie address** = 0xE7E7E7E7XX (XX is your team number such as 01, 02, ..., 10, 11, ..., 16).
**Radio channel** = 10 * (group_number % 10), such as (10, 20, ..., **100**, 10, ..., 60).
When developing your algorithm or running the following examples, **be sure to update the uri** in your code to reflect the correct address and radio channel for your team.

For example, 'uri = uri_helper.uri_from_env(default='radio://0/10/2M/E7E7E7E701')' for group 1.

Example - log
-------------
Now you can test the communication with the drone by running this log example: `log.py <https://github.com/dronecourse-epfl/crazy-practical-tutorial/tree/main/docs/log.py>`_.
For this example you can put the drone on desk as there is no control.
If the library and radio driver is configured correctly, you should see sensor data printed in your ternimal when running this example (remember changing the uri).
Try moving your hand closer and farther away from the multi-ranger sensors and observe the sensor data change.

To log any other sensor data from the drone, refer to `this page <https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/api/logs/#estimator>`_.

Example - log_and_control
-------------------------
This example code will control the drone to fly a figure-eight trajectory, while also logging all sensor data at the same time.
The example code is at: `log_and_control.py <https://github.com/dronecourse-epfl/crazy-practical-tutorial/tree/main/docs/log_and_control.py>`_
Please ensure that you place the drone on the ground before testing this example, as the drone is programmed to take off and fly.
Additionally, it is recommended to take off from a white part of the ground for best performance.

These two examples are sufficient for finishing the task. Additional examples can be found at `Crazyflie Python library examples <https://github.com/bitcraze/crazyflie-lib-python/tree/master/examples>`_.