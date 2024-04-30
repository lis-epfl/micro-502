Hardware instructions
=====================

You should already be familiar with the project task through your work in simulation.
As a group, you will now fine-tune the best solution to control a real quadrotor in cluttered environments. 
In contrast to the simulation, your algorithms will now be executed in real-time. You will find that you may thus have to adjust your code such that it can be executed sufficiently fast to avoid crashing with the real Crazyflie. 
You should also keep in mind that the sensor signals on the actual drone are noisy and less accurate than in simulation.

You will lastly present your algorithm and demonstrate its performance live on a Crazyflie in an obstacle parcours at the end of the course.
A bonus for your demonstration in this project is applied for the quickest times to complete the parcours, so let the fastest drone win!

Learning objectives
-------------------
- Controlling a plug-and-play drone
- Interfacing with the Python library to control the drone
- Learning the difference between code deployment in simulation and on the real drone
- Mastering different flight phases in the real world
- Reporting performance results in a scientific manner

Tasks
-----
The final deliverables which you will be graded on for this hardware practical are the following:

- Short presentation (4 slides maximum)
- Video recording of one of your trials in an obstacle-rich task environment
- Demonstration on May 28th: 3 trials with obstacles (**best grade counts**), and each trial must be completed with only one battery

Presentation
------------
Length: up to 4 slides

Duration: up to 7 minutes

- 1 slide on the experimental setup (Environment layout)
- 1 slide on the strategy (Algorithm, what you spend most time on)
- 1 slide on the results (Statistics on mission time/success/...)
- 1 optional slide (with anything relevant to add)

Video of one trial
------------------
You will prepare and submit a video of one of your trials with your own obsatcles. The video should:

- Be at most 2 minutes long
- Contain no edits except speed-ups (indicate speed up factor in overlay) and text additions
- Show at least one of the team members
- Show the obstacles, take-off and landing zones
- Show a clock/stop watch with a physical device (i.e a timer on a phone) that indicates seconds at the beginning and at the end
- Be in mp4 format

Note that:

- You can create your own obstacles and landing pads (environment like the task setup, for an example check a video from the previous years in :ref:`Hardware Task overview`)
- Collaboration between groups to build the environment is encouraged
- To get sufficiently good position and velocity estimates from the optic flow sensor, the ground below the Crazyflie should contain some texture

Demonstrations
--------------

The demonstrations will take place at assigned timeslots on May 28th, between 15:15 and 19:00 (see :ref:`Demonstration schedule on May 28th`).

To test your algorithms in a setup as in the final demonstrations, we will offer 15 minute testing slots on May 22nd in MED -1 1422.
We will provide a booking method for you soon where you can select a timeslot.

Complementary information
-------------------------

- **Submit by Monday 27th May, 23:59** (your code with comments, the video as MP4, presentation as PPTX) in a zip file named GROUPNUMBER_LASTNAME1_LASTNAME2_LASTNAME3_LASTNAME4_LASTNAME5.zip on Moodle
- The initial position of the drone will be given one day before the final demonstration
- Each group will present the PPTX from our computer before the demonstration
- Return the drone directly after the demonstration
- Check that the returned material is complete, according to the lists in :ref:`Hardware unpacking`
- Make sure you set up and make space quickly

Overall grading for the hardware task
-------------------------------------

We will evaluate you as a group and determine your grade for this hardware task as a weighted average of the following elements:

- Presentation and video showcasing the algorithm on one of your trials (Grade 1-6, Weighting: 60%) 
- Demonstration (Grade 1-6, Weighting: 40%)

The assistants will judge your presentation right before your hardware demonstration. Your video submission to moodle will be judged separatly in the same week.

The demonstration performance will be graded according to the performance metrics defined under :ref:`Hardware Task overview`:

- **Grade 4.0**: Take off, avoid obstacles and reach the landing region whilst being airborne
- **Grade 4.5**: Land on the landing pad
- **Grade 5.0**: Take off from the landing pad and leave the landing region whilst being airborne
- **Grade 5.25**: Avoid obstacles and reach the starting region whilst being airborne
- **Grade 5.5**: Land on the take-off pad
- **Grade 5.5 + f(t)**: For students who complete the full task, extra grades will be based on the time taken to complete the task. The top 25% of students will receive a grade of f(t)=0.5, the next 50% will receive a grade of f(t)=0.25, and the bottom 25% will receive a grade of f(t)=0.

Solutions that go against the spirit of the exercise will not be accepted (e.g. flying above obstacles or outside of the arena).

You will get a 3.5 in your demonstration if you crash before reaching the landing region.

Demonstration schedule on May 28th
----------------------------------
============= ============= ==============
Time          Room MED11518 Room MED-11422
============= ============= ==============
15:15 - 15:30 group 1       group 11
15:35 - 15:50 group 2       group 12
15:55 - 16:10 group 3       group 13
16:15 - 16:30 group 4       group 14
16:35 - 16:50 group 5       group 15
16:55 - 17:10 group 6       group 16
17:15 - 17:30 group 7       group 17
17:35 - 17:50 group 8       group 18
17:55 - 18:10 group 9       group 19
18:15 - 18:30 group 10      group 20
============= ============= ==============
