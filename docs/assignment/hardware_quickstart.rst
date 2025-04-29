Hardware project and Crazyflie software quickstart
===================================================

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
  * 4 x spare propellers CW
  * 4 x spare propellers CCW

* One flow deck v2
* One lighthouse deck

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

Onboard LEDs indicate drone states such as low power, self-test fail, etc. Refer to `bitcraze page <https://www.bitcraze.io/documentation/tutorials/getting-started-with-crazyflie-2-x/#leds>`_.

If you believe that your motors do not work correctly or you believe that your electronics are damaged, consult the `console debugging tool <https://www.bitcraze.io/2022/05/debug-tools-in-the-client-console-tab/>`_ to obtain debugging information and show it to one of the assistants.

Software installation
---------------------
For developing the code for the Crazyflie, you will first need to install Python 3.8+ if you don't have it yet.
Then, you can clone and install the Crazyflie Python Library, to communicate with and control the Crazyflie.

For both Ubuntu and Windows, you can install the library by running following commands in your terminal:

.. code-block:: bash

    git clone https://github.com/bitcraze/crazyflie-lib-python.git
    cd crazyflie-lib-python
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

Lighthouse positioning system information
------------------------------------------------

The Lighthouse positioning system is a motion capture system that uses infrared light to track the 3D position of the drone. The positioning accuracy is typically lower than 1 cm.

To set it up and connect it with your Crazyflie, follow the `Lighthouse instructions <https://www.bitcraze.io/documentation/tutorials/getting-started-with-lighthouse/#preparing-the-system>`_ in the section "Preparing the System".
It is sensible to perform the calibration steps given in this section at the start of every new session that that you have booked, as the stations might still slightly move after another group has used the setup.

Please DO NOT INTENTIONALLY MOVE the base stations and DO NOT MODIFY the Base station software settings. You must not perform any of the steps under "Preparing the base stations", this is already done for you.


Sensor information and readout
------------------------------------------------

The Crazyflie drone performs sensor fusion from all the onboard sensors and the Lighthouse system to obtain the optimal state estimate using an Extended Kalman Filter (EKF).
For background information on the measurement models and the state estimation pipeline, refer to this `link <https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/sensor-to-control/state_estimators/#extended-kalman-filter>`_.

In the Crazyflie software you may access the state estimates from these `logging variables <https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/api/logs/#stateestimate>`_.


Example - log
-------------
Now you can test the communication with the drone by downloading and running this log example:

.. `Download the script <_static/log.py>`_

.. literalinclude:: _static/log.py
   :language: python

.. .. raw:: html

   .. <a href="_static/log.py" download>Download the logging example file</a>

For this example you can put the drone on desk as there is no control.
If the library and radio driver is configured correctly, you should see sensor data printed in your terminal when running this example (remember changing the uri).
Try moving your hand closer and farther away from the multi-ranger sensors and observe the sensor data change.

 .. `log.py <https://github.com/dronecourse-epfl/crazy-practical-tutorial/tree/main/docs/log.py>`_.

Example - log_and_control
-------------------------
This example code will control the drone to fly a figure-eight trajectory, while also logging all sensor data at the same time.

.. .. raw:: html

   .. <a href="_static/log_and_control.py" download>Download the logging and control example file</a>

.. `Download the script <_static/log_and_control.py>`_

.. literalinclude:: _static/log_and_control.py
   :language: python

Please ensure that you place the drone on the ground before testing this example, as the drone is programmed to take off and fly.
Additionally, it is recommended to take off from a white part of the ground for best performance.

These two examples give you a sufficient framework to finish the task. Additional examples can be found at `Crazyflie Python library examples <https://github.com/bitcraze/crazyflie-lib-python/tree/master/examples>`_.