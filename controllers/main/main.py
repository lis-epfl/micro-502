# Main simulation file called by the Webots
import sys
print("You are using python at this location:", sys.executable)

import numpy as np
from controller import Supervisor, Keyboard
from exercises.ex1_pid_control import quadrotor_controller
from exercises.ex2_kalman_filter import kalman_filter as KF
from exercises.ex3_motion_planner import MotionPlanner3D as MP
import assignment.my_assignment as assignment
import exercises.ex0_rotations as ex0_rotations
from scipy.spatial.transform import Rotation as R
import lib.mapping_and_planning_examples as mapping_and_planning_examples
import time, random
import threading

exp_num = 4                    # 0: Coordinate Transformation, 1: PID Tuning, 2: Kalman Filter, 3: Motion Planning, 4: Project
control_style = 'path_planner'      # 'keyboard' or 'path_planner'
rand_env = False                # Randomise the environment

# Global variables for handling threads
latest_sensor_data = None
latest_camera_data = None
sensor_lock = threading.Lock()

current_setpoint = np.zeros(4)
setpoint_lock = threading.Lock()

running = True

# Crazyflie drone class in webots
class CrazyflieInDroneDome(Supervisor):
    def __init__(self):
        super().__init__()
        self.timestep = int(self.getBasicTimeStep())

        # Actuators
        self.m1_motor = self.getDevice("m1_motor")
        self.m1_motor.setPosition(float('inf'))
        self.m1_motor.setVelocity(-1)
        self.m2_motor = self.getDevice("m2_motor")
        self.m2_motor.setPosition(float('inf'))
        self.m2_motor.setVelocity(1)
        self.m3_motor = self.getDevice("m3_motor")
        self.m3_motor.setPosition(float('inf'))
        self.m3_motor.setVelocity(-1)
        self.m4_motor = self.getDevice("m4_motor")
        self.m4_motor.setPosition(float('inf'))
        self.m4_motor.setVelocity(1)

        self.meas_state_gps = np.zeros((3,1))
        self.meas_state_accel = np.zeros((3,1))

        self.accel_read_last_time = 0.0
        self.gps_read_last_time = 0.0

        if exp_num == 2:
            # Kalman filter variables
            self.KF = KF()
            self.sensor_flag = 0
            self.dt_accel = 0.0
            self.dt_gps = 0.0
            self.dt_propagate = 0.0
            self.ctrl_update_period = int(self.timestep*3) #timestep equal to GPS time 2 or 3 works well
            self.gps_update_period = int(self.timestep*3) # 2*timestep
            self.accel_update_period = int(self.timestep*2) # 1*timestep
        else:
            self.ctrl_update_period = self.timestep
            self.gps_update_period = self.timestep
            self.accel_update_period = self.timestep

        # Sensors

        self.g = 9.81 #Used for accelerometer Z-direction correction

        self.imu = self.getDevice('inertial unit')
        self.imu.enable(self.timestep)
        self.gps = self.getDevice('gps')
        self.gps.enable(self.gps_update_period)
        self.accelerometer = self.getDevice('accelerometer')
        self.accelerometer.enable(self.accel_update_period)
        self.gyro = self.getDevice('gyro')
        self.gyro.enable(self.timestep)
        self.camera = self.getDevice('cf_camera')
        self.camera.enable(self.timestep)
        self.range_front = self.getDevice('range_front')
        self.range_front.enable(self.timestep)
        self.range_left = self.getDevice("range_left")
        self.range_left.enable(self.timestep)
        self.range_back = self.getDevice("range_back")
        self.range_back.enable(self.timestep)
        self.range_right = self.getDevice("range_right")
        self.range_right.enable(self.timestep)
        self.laser_down = self.getDevice("laser_down")
        self.laser_down.enable(self.timestep)
        
        # Crazyflie velocity PID controller
        self.PID_CF = quadrotor_controller(exp_num)
        self.PID_update_last_time = self.getTime()
        self.sensor_read_last_time = self.getTime()
        self.step_count = 0
        self.dt_ctrl = 0.0

        # History variables for calculating groundtruth velocity
        self.x_global_last = 0.0
        self.y_global_last = 0.0
        self.z_global_last = 0.0
        self.vx_global = 0.0
        self.vy_global = 0.0
        self.vz_global = 0.0

        # Tools
        self.keyboard = self.getKeyboard()
        self.keyboard.enable(self.timestep)

        # Simulation step update
        super().step(self.timestep)

        # Handle global setpoints to system depending on exercise
        if exp_num == 3:
            start = (0.0, 0.0, 0.5)
            goal = (5, 1, 1)
            grid_size = 0.25
            obstacles = [(0.75, 0.25, 0.0, 0.4, 0.4, 1.5),
                        (1.25, 1.625, 0.0, 0.4, 0.4, 1.5),
                        (3.25, 1.0, 0.0, 0.4, 0.4, 1.5),
                        (4.0, 2.25, 0.0, 0.4, 0.4, 1.5),
                        (4.0, 0.725, 0.0, 0.5, 0.25, 1.5),
                        (2.5, 0.0, 0.0, 0.5, 1.875, 1.5),
                        (2.5, 1.875, 0.0, 0.5, 1.125, 0.125),
                        (2.5, 1.875, 1.0, 0.5, 1.125, 0.5),
                        (2.5, 2.75, 0.125, 0.5, 0.25, 0.875)
                        ]  # (x, y, z, width_x, width_y, width_z)
            bounds = (0, 5, 0, 3, 0, 1.5)  # (x_min, x_max, y_min, y_max, z_min, z_max)
            mp_obj = MP(start, obstacles, bounds, grid_size, goal)
            self.setpoints = mp_obj.trajectory_setpoints
            self.timepoints = mp_obj.time_setpoints
            assert self.setpoints is not None, "No valid trajectory reference setpoints found"
            self.tol_goal = 0.25
        else:
            self.setpoints = [[0.0, 0.0, 1.0, 0.0], [0.0, 3.0, 1.25, np.pi/2], [5.0, 3.0, 1.5, np.pi], [5.0, 0.0, 0.25, 1.5*np.pi], [0.0, 0.0, 1.0, 0.0]]
            self.tol_goal = 0.1

        # For the assignment, randomise the positions of the drone, obstacles, goal, take-off pad and landing pad 
        if exp_num == 4:

            # Course parameters
            self.circle_centre = [4, 4]
            self.inner_radius = 1.5
            self.outer_radius = 3.5
            self.gate_height_bounds = [0.7, 2.0]
            self.goal_size_bounds = [0.3, 1.1]
            self.goal_rotation_bounds = [-np.pi/6, np.pi/6]
            self.num_gates = 5
            self.num_segments = self.num_gates + 1
            self.num_laps = 3
            self.segment_angular_size = np.pi / self.num_segments

            # Variables to track progress
            self.segment = 0
            self.segment_progress = [False] * self.num_segments
            self.gate_progress = [[False] * self.num_gates for _ in range(self.num_laps)]
            self.lap = 0
            self.lap_times = [1000] * self.num_laps
            self.start_time = 0
            
            # Get the angular segments of the gates
            self.angular_bounds = []
            for i in range(self.num_segments):
                angular_bound = [(2*i-0.5) * self.segment_angular_size % (2*np.pi), (2*i + 0.5) * self.segment_angular_size % (2*np.pi)]
                self.angular_bounds.append(angular_bound)
        
            # Randomise the positions of the drone and gates
            if rand_env:
                self.randomise_positions()

            # Get the position, size, and orientation of each of the gates
            self.gate_positions = []
            self.gate_sizes = []
            self.gate_orientations = []
            for i in range(5):
                goal_node = super().getFromDef('GATE' + str(i))
                self.gate_positions.append(goal_node.getField('translation').getSFVec3f())
                self.gate_sizes.append(goal_node.getField('goalSize').getSFVec3f())
                self.gate_orientations.append(goal_node.getField('rotation').getSFRotation())

    # Randomise the positions of the drone, obstacles, goal, take-off pad and landing pad
    def randomise_positions(self):
                               
        for i in range(self.num_segments):

            # Randomise the angular position of the gate in polar coordinates
            if i == 0:
                angular_position = random.uniform(self.angular_bounds[i][0] - 2*np.pi, self.angular_bounds[i][1])
            else:
                angular_position = random.uniform(self.angular_bounds[i][0], self.angular_bounds[i][1])
            radius = random.uniform(self.inner_radius, self.outer_radius)

            # Convert the polar coordinates to cartesian coordinates
            x = self.circle_centre[0] - radius * np.cos(angular_position)
            y = self.circle_centre[1] - radius * np.sin(angular_position)

            # if i == 0:
                # Set the take-off pose of the drone and take-off pad
                # takeoff_position = [x, y]
                # takeoff_orientation = angular_position - np.pi/2
                # self.set_take_off_position(takeoff_position, takeoff_orientation)
            # else:
            if i > 0:
                # Set the pose of the gate
                goal_node = super().getFromDef('GATE' + str(i-1))
                goal_height = np.random.uniform(self.gate_height_bounds[0], self.gate_height_bounds[1])
                goal_size = np.random.uniform(self.goal_size_bounds[0], self.goal_size_bounds[1])
                goal_position = [x, y, goal_height]
                goal_orientation = angular_position - np.pi/2 + np.random.uniform(self.goal_rotation_bounds[0], self.goal_rotation_bounds[1])
                self.set_goal_fields(goal_node, goal_size, goal_position, goal_orientation)
                            
    # Set the take-off position of the drone and take-off pad
    def set_take_off_position(self, take_off_position, take_off_orientation):

        # Set the initial position of the drone
        init_x_drone, init_y_drone = take_off_position
        drone = super().getSelf()
        translation_field = drone.getField('translation')
        translation_field.setSFVec3f([init_x_drone, init_y_drone, 0.25])
        rotation_field = drone.getField('rotation')
        rotation_field.setSFRotation([0, 0, 1, take_off_orientation])

        # Set the initial position of the take-off pad
        take_off_pad = super().getFromDef('TAKE_OFF_PAD')
        translation_field = take_off_pad.getField('translation')
        translation_field.setSFVec3f([init_x_drone, init_y_drone, 0.05])
        rotation_field = take_off_pad.getField('rotation')
        rotation_field.setSFRotation([0, 0, 1, take_off_orientation])
    
    # Set the fields of the obstacle node
    def set_goal_fields(self, goal_node, goal_size, goal_position, goal_orientation):
        
        # Default Beam dimensions
        w = 0.06
        h = 0.04
        
        # Update the translation of the gate
        translation_field = goal_node.getField('translation')
        translation_field.setSFVec3f(goal_position)

        # Get the gate height
        goal_height = goal_position[2]

        # Update the goal size
        goal_size_field = goal_node.getField('goalSize')
        goal_size_field.setSFVec3f([h, goal_size, goal_size])

        # Update the top beam
        top_beam_length = goal_size
        top_beam_scale_field = goal_node.getField('topBeamScale')
        top_beam_scale_field.setSFVec3f([top_beam_length, w, h])
        top_beam_translation_field = goal_node.getField('topBeamTranslation')
        top_beam_translation_field.setSFVec3f([0, 0, goal_size/2 + w/2])

        # Update the bottom beam
        bottom_beam_length = goal_size
        bottom_beam_scale_field = goal_node.getField('bottomBeamScale')
        bottom_beam_scale_field.setSFVec3f([bottom_beam_length, w, h])
        bottom_beam_translation_field = goal_node.getField('bottomBeamTranslation')
        bottom_beam_translation_field.setSFVec3f([0, 0, -goal_size/2 - w/2])

        # Update the left beam
        left_beam_length = goal_height + goal_size/2 + w - h
        left_beam_scale_field = goal_node.getField('leftBeamScale')
        left_beam_scale_field.setSFVec3f([left_beam_length, w, h])
        left_beam_translation_field = goal_node.getField('leftBeamTranslation')
        left_beam_translation_field.setSFVec3f([0, goal_size/2 + w/2, h + left_beam_length/2 - goal_height])

        # Update the right beam
        right_beam_length = goal_height + goal_size/2 + w - h
        right_beam_scale_field = goal_node.getField('rightBeamScale')
        right_beam_scale_field.setSFVec3f([right_beam_length, w, h])
        right_beam_translation_field = goal_node.getField('rightBeamTranslation')
        right_beam_translation_field.setSFVec3f([0, -goal_size/2 - w/2, h + right_beam_length/2 - goal_height])

        # Update the left leg
        left_leg_translation_field = goal_node.getField('leftLegTranslation')
        left_leg_translation_field.setSFVec3f([0, goal_size/2 + w/2, h/2 - goal_height])

        # Update the right leg
        right_leg_translation_field = goal_node.getField('rightLegTranslation')
        right_leg_translation_field.setSFVec3f([0, -goal_size/2 - w/2, h/2 - goal_height])

        # Update the orientation of the goal
        rotation_field = goal_node.getField('rotation')
        rotation_field.setSFRotation([0, 0, 1, goal_orientation])
  
    # Track the progress in the assignment
    def track_assignment_progress(self, sensor_data):
         # Check which segment the drone is in
        curr_segment = drone.check_segment(sensor_data)

        # Start timing when the drone leaves the first segment
        if curr_segment != 0 and drone.segment == 0 and drone.start_time == 0:
            drone.start_time = drone.getTime()
            print("Timing started...")

        # Stop timing when the drone returns to segment 0
        # print('curr_segment:', curr_segment, 'drone.segment:', drone.segment)
        if curr_segment == 0 and drone.segment == 5:
            elapsed_time = drone.getTime() - drone.start_time
            drone.start_time = 0
            drone.lap_times[drone.lap] = elapsed_time
            drone.lap += 1
            print(f"Lap completed. Total time elapsed: {elapsed_time:.2f} seconds") 
            drone.segment_progress = [False] * drone.num_segments
            drone.segment = 0
        
        # Update the current segment
        if curr_segment != -1:
            drone.segment = curr_segment

        # Mark the segment as completed
        if not drone.segment_progress[drone.segment]:
            drone.segment_progress[drone.segment] = True

            # Print the current progress
            if drone.segment > 1:
                if drone.gate_progress[drone.lap][drone.segment-2]:
                    print('Moving to the next segment after successfully passing gate', drone.segment-2)
                else:
                    print('Moving to the next segment after failing to pass gate', drone.segment-2)

        # Check if the drone has reached the gate in this segment
        if drone.segment != -1:
            drone.check_goal(sensor_data)
        
        # If finished all segments print the lap times
        if drone.lap == drone.num_laps:
            print("Lap times:", drone.lap_times)
            print("Gate progress:", drone.gate_progress)
            return False
        
        return True
    
    def wait_keyboard(self):
        while self.keyboard.getKey() != ord('Y'):
            super().step(self.timestep)

    def action_from_keyboard(self, sensor_data):
        forward_velocity = 0.0
        left_velocity = 0.0
        altitude_velocity = 0.0
        yaw_rate = 0.0
        key = self.keyboard.getKey()
        while key > 0:
            if key == ord('W'):
                forward_velocity = 2.0
            elif key == ord('S'):
                forward_velocity = -2.0
            elif key == ord('A'):
                left_velocity = 2.0
            elif key == ord('D'):
                left_velocity = -2.0
            elif key == ord('Q'):
                yaw_rate = 1.0
            elif key == ord('E'):
                yaw_rate = -1.0
            elif key == ord('X'):
                altitude_velocity = 0.3
            elif key == ord('Z'):
                altitude_velocity = -0.3
            key = self.keyboard.getKey()
        return [forward_velocity, left_velocity, altitude_velocity, yaw_rate]

    def read_KF_estimates(self):
        
        # Update time intervals for sensing and propagation
        self.dt_accel = self.getTime() - self.accel_read_last_time
        self.dt_gps = self.getTime() - self.gps_read_last_time

        # Data dictionary
        measured_data_raw = self.read_sensors().copy()
        measured_noisy_data = self.KF.add_noise(measured_data_raw.copy(), self.dt_gps, self.dt_accel, self.gps.getSamplingPeriod(), self.accelerometer.getSamplingPeriod())

        self.sensor_flag = 0

        if self.KF.use_accel_only and self.getTime() > 2.0:
            #Only propagate and measure accelerometer
            
            self.dt_propagate = self.dt_accel

            if np.round(self.dt_accel,3) >= self.accelerometer.getSamplingPeriod()/1000: 
                self.sensor_flag = 2
                self.meas_state_accel = np.array([[measured_noisy_data['ax_global'], measured_noisy_data['ay_global'], measured_noisy_data['az_global']]]).transpose()
                self.accel_read_last_time = self.getTime()
            if np.round(self.dt_gps,3) >= self.gps.getSamplingPeriod()/1000:
                self.gps_read_last_time = self.getTime() #Required to maintain ground truth state measured capability

        else:

            #Propagate and measure for both accelerometer and GPS

            self.dt_propagate = min(self.dt_accel, self.dt_gps)

            if np.round(self.dt_accel,3) >= self.accelerometer.getSamplingPeriod()/1000 and np.round(self.dt_gps,3) >= self.gps.getSamplingPeriod()/1000:
                self.sensor_flag = 3
                self.meas_state_accel = np.array([[measured_noisy_data['ax_global'], measured_noisy_data['ay_global'], measured_noisy_data['az_global']]]).transpose()
                self.accel_read_last_time = self.getTime()
                self.meas_state_gps = np.array([[measured_noisy_data['x_global'], measured_noisy_data['y_global'], measured_noisy_data['z_global']]]).transpose()
                self.gps_read_last_time = self.getTime()
            else:
                if np.round(self.dt_gps,3) >= self.gps.getSamplingPeriod()/1000:
                    self.sensor_flag = 1
                    self.meas_state_gps = np.array([[measured_noisy_data['x_global'], measured_noisy_data['y_global'], measured_noisy_data['z_global']]]).transpose()
                    self.gps_read_last_time = self.getTime()
                if np.round(self.dt_accel,3) >= self.accelerometer.getSamplingPeriod()/1000: 
                    self.sensor_flag = 2
                    self.meas_state_accel = np.array([[measured_noisy_data['ax_global'], measured_noisy_data['ay_global'], measured_noisy_data['az_global']]]).transpose()
                    self.accel_read_last_time = self.getTime()

        estimated_state, estimated_covariance = self.KF.KF_estimate(self.meas_state_gps, self.meas_state_accel, self.dt_propagate, self.sensor_flag)

        x_g_est, v_x_g_est, a_x_g_est, y_g_est, v_y_g_est, a_y_g_est, z_g_est, v_z_g_est, a_z_g_est = estimated_state.flatten()
        KF_state_outputs = measured_noisy_data.copy()
        KF_state_outputs['x_global'] = x_g_est
        KF_state_outputs['y_global'] = y_g_est
        KF_state_outputs['z_global'] = z_g_est
        KF_state_outputs['v_x'] = v_x_g_est
        KF_state_outputs['v_y'] = v_y_g_est
        KF_state_outputs['v_z'] = v_z_g_est
        KF_state_outputs['v_forward'] = v_x_g_est * np.cos(KF_state_outputs['yaw']) + v_y_g_est * np.sin(KF_state_outputs['yaw'])
        KF_state_outputs['v_left'] = -v_x_g_est * np.sin(KF_state_outputs['yaw']) + v_y_g_est * np.cos(KF_state_outputs['yaw'])
        KF_state_outputs['v_up'] = v_z_g_est
        KF_state_outputs['ax_global'] = a_x_g_est
        KF_state_outputs['ay_global'] = a_y_g_est
        KF_state_outputs['az_global'] = a_z_g_est

        # Call appending of states over run
        self.KF.aggregate_states(measured_data_raw, measured_noisy_data, KF_state_outputs, self.getTime())

        if self.KF.use_direct_noisy_measurement:
            if self.getTime() < 2.0:
                output_measurement = measured_data_raw
            else:
                output_measurement = measured_noisy_data.copy()
        elif self.KF.use_KF_measurement:
            if measured_data_raw['z_global'] < 0.49:
                output_measurement = measured_data_raw
            else:
                output_measurement = KF_state_outputs.copy()
        elif self.KF.use_direct_ground_truth_measurement:
            output_measurement = measured_data_raw.copy()

        return output_measurement
    
    def read_sensors(self):
        
        # Data dictionary
        data = {}

        # Time interval
        dt = self.getTime() - self.sensor_read_last_time
        data['t'] = self.getTime()
        self.sensor_read_last_time = self.getTime()

        # Position
        data['x_global'] = self.gps.getValues()[0]
        data['y_global'] = self.gps.getValues()[1]
        data['z_global'] = self.gps.getValues()[2]

        # Attitude
        data['roll'] = self.imu.getRollPitchYaw()[0]
        data['pitch'] = self.imu.getRollPitchYaw()[1]
        data['yaw'] = self.imu.getRollPitchYaw()[2]

        data['q_x'] = self.imu.getQuaternion()[0]
        data['q_y'] = self.imu.getQuaternion()[1]
        data['q_z'] = self.imu.getQuaternion()[2]
        data['q_w'] = self.imu.getQuaternion()[3]

        ax_body = self.accelerometer.getValues()[0]
        ay_body = self.accelerometer.getValues()[1]
        az_body = self.accelerometer.getValues()[2] 

        # Velocity
        if exp_num == 2:
            if np.round(self.dt_gps,3) >= self.gps_update_period/1000:
                self.vx_global = (data['x_global'] - self.x_global_last) / self.dt_gps
                self.vy_global = (data['y_global'] - self.y_global_last) / self.dt_gps
                self.vz_global = (data['z_global'] - self.z_global_last) / self.dt_gps
                self.x_global_last = data['x_global']
                self.y_global_last = data['y_global']
                self.z_global_last = data['z_global']
            else:
                data['x_global'] = self.x_global_last
                data['y_global'] = self.y_global_last
                data['z_global'] = self.z_global_last
        else:
            self.vx_global = (data['x_global'] - self.x_global_last) / dt
            self.vy_global = (data['y_global'] - self.y_global_last) / dt
            self.vz_global = (data['z_global'] - self.z_global_last) / dt
            self.x_global_last = data['x_global']
            self.y_global_last = data['y_global']
            self.z_global_last = data['z_global']

        data['v_x'] = self.vx_global
        data['v_y'] = self.vy_global
        data['v_z'] = self.vz_global

        data['v_forward'] =  self.vx_global * np.cos(data['yaw']) + self.vy_global * np.sin(data['yaw'])
        data['v_left'] =  -self.vx_global * np.sin(data['yaw']) + self.vy_global * np.cos(data['yaw'])
        data['v_up'] =  self.vz_global

        #Accleration from body to global frame
        r = R.from_euler('xyz', [data['roll'], data['pitch'], data['yaw']])
        R_T = r.as_matrix()

        a_global = (R_T @ np.array([[ax_body, ay_body, az_body]]).transpose()).flatten()

        data['ax_global'] = a_global[0]
        data['ay_global'] = a_global[1]
        data['az_global'] = a_global[2] - self.g     

        # Range sensor
        data['range_front'] = self.range_front.getValue() / 1000.0
        data['range_left']  = self.range_left.getValue() / 1000.0
        data['range_back']  = self.range_back.getValue() / 1000.0
        data['range_right'] = self.range_right.getValue() / 1000.0
        data['range_down'] = self.laser_down.getValue() / 1000.0

        # Yaw rate
        data['rate_roll'] = self.gyro.getValues()[0]
        data['rate_pitch'] = self.gyro.getValues()[1]
        data['rate_yaw'] = self.gyro.getValues()[2]

        return data

    # Read the camera feed
    def read_camera(self):

        # Read the camera image in BRGA format
        camera_image = self.camera.getImage()

        # Convert the image to a numpy array for OpenCV
        image = np.frombuffer(camera_image, np.uint8).reshape((self.camera.getHeight(), self.camera.getWidth(), 4))

        return image
    
    # Detect which segment the drone is in
    def check_segment(self, sensor_data):
        drone_pos = np.array([sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global']])
        drone_pos = drone_pos[:2]
        drone_pos = drone_pos - np.array(self.circle_centre)
        drone_pos = drone_pos / np.linalg.norm(drone_pos)

        # Compute the angle of the drone's position
        drone_angle = np.arctan2(drone_pos[1], drone_pos[0]) + np.pi

        # Determine the segment the drone is in
        for i in range(self.num_segments):
            if i == 0:
                if drone_angle >= self.angular_bounds[i][0] or drone_angle <= self.angular_bounds[i][1]:
                    return i
            if drone_angle >= self.angular_bounds[i][0] and drone_angle <= self.angular_bounds[i][1]:
                return i
        return -1

    # Detect if the drone has reached the gate, if it has set the GOAL object to be transparent
    def check_goal(self, sensor_data):

        # Get the current gate index
        gate_idx = drone.segment - 1
        
        # Get the gate parameters
        gate_position = self.gate_positions[gate_idx]
        gate_size = self.gate_sizes[gate_idx]
        gate_orientation = self.gate_orientations[gate_idx][3]
        
        # Use the drone's global position (using z_global rather than range_down)
        drone_pos = np.array([sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global']])
        gate_pos = np.array(gate_position)
        
        # Compute the relative position between drone and gate center
        rel_pos = drone_pos - gate_pos
        
        # Create a rotation matrix for a rotation about z by -gate_orientation
        cos_theta = np.cos(-gate_orientation)
        sin_theta = np.sin(-gate_orientation)
        Rz = np.array([[cos_theta, -sin_theta, 0],
                       [sin_theta,  cos_theta, 0],
                       [0,         0,         1]])
        
        # Transform the drone's position into the gate's local frame
        local_pos = Rz @ rel_pos
        
        # Determine half-dimensions of the gate's opening
        half_dims = np.array(gate_size) / 2.0

        # Check if the drone is within the gate bounds in the gate's local frame
        if (abs(local_pos[0]) <= half_dims[0] and
            abs(local_pos[1]) <= half_dims[1] and
            abs(local_pos[2]) <= half_dims[2]):
            
            if not self.gate_progress[self.lap][gate_idx]:
                print("Gate", gate_idx, "reached!")
                goal_node = super().getFromDef('GATE' + str(gate_idx))
                goal_visibility = goal_node.getField('goalVisible')
                goal_visibility.setSFFloat(1.0)
                self.gate_progress[self.lap][gate_idx] = True

    def reset(self):
        # Reset the simulation
        self.simulationResetPhysics()
        self.simulationReset()
        super().step(self.timestep)

    def step(self, motorPower, sensor_data):

        # Update motor command
        self.m1_motor.setVelocity(-motorPower[0])
        self.m2_motor.setVelocity(motorPower[1])
        self.m3_motor.setVelocity(-motorPower[2])
        self.m4_motor.setVelocity(motorPower[3])
        

        # Update drone states in simulation
        super().step(self.timestep)

# A thread that runs the path planner in parallel with the simulation
def path_planner_thread(drone):
    global latest_sensor_data, latest_camera_data, current_setpoint, running

    # Set the initial last planner time
    last_planner_time = drone.getTime()

    while running:
        
        # Make a local copy of the sensor data and camera data
        sensor_data_copy = None
        camera_data_copy = None

        # Lock the sensor data to prevent it from being updated while we are using it
        with sensor_lock:

            # Update sensor data if it is available
            if latest_sensor_data is not None:
                sensor_data_copy = latest_sensor_data.copy()

            # Update the camera data if it is available
            if latest_camera_data is not None:
                camera_data_copy = latest_camera_data.copy()

        # Call the path planner to get the new setpoint
        if sensor_data_copy is not None and camera_data_copy is not None:

            # Update the time interval for the planner
            current_time = drone.getTime()
            dt_planner = current_time - last_planner_time
            last_planner_time = current_time

            new_setpoint = assignment.get_command(sensor_data_copy, camera_data_copy, dt_planner)
            
            with setpoint_lock:
                current_setpoint = new_setpoint

        time.sleep(0.01)
    

if __name__ == '__main__':

    # Initialize the drone
    drone = CrazyflieInDroneDome()
    assert control_style in ['keyboard','path_planner'], "Variable control_style must either be 'keyboard' or 'path_planner'"
    assert exp_num in [0,1,2,3,4], "Exp_num must be a value between 0 and 4"

    # Start the path planner thread
    if control_style == 'path_planner' and exp_num == 4:
        planner_thread = threading.Thread(target=path_planner_thread, args=(drone,))
        planner_thread.daemon = True
        planner_thread.start()
   
    try:
        # Simulation loops
        for step in range(100000):
            
            if exp_num == 2:
                sensor_data = drone.read_KF_estimates()
                if np.round(drone.getTime(),2) == drone.KF.plot_time_limit:
                    drone.KF.plot_states()
            else:
                # Read sensor data including []
                sensor_data = drone.read_sensors()

            drone.dt_ctrl = drone.getTime() - drone.PID_update_last_time

            if drone.PID_update_last_time == 0.0 or np.round(drone.dt_ctrl,3) >= drone.ctrl_update_period/1000: #Only execute at first point and in control rate step

                if control_style == 'keyboard':
                    # Get the control commands from the keyboard
                    control_commands = drone.action_from_keyboard(sensor_data)
                    
                    # Rotate the control commands from the body reference frame to the inertial reference frame
                    euler_angles = [sensor_data['roll'], sensor_data['pitch'], sensor_data['yaw']]
                    quaternion = [sensor_data['q_x'], sensor_data['q_y'], sensor_data['q_z'], sensor_data['q_w']]
                    control_commands = ex0_rotations.rot_body2inertial(control_commands, euler_angles, quaternion)

                    # Call the PID controller to get the motor commands
                    motorPower = drone.PID_CF.keys_to_pwm(drone.dt_ctrl, control_commands, sensor_data)    

                elif control_style == 'path_planner':
                    # # Update the setpoint
                    if exp_num != 4:
                        if exp_num != 3:
                            if np.round(drone.dt_ctrl,3) >= drone.ctrl_update_period/1000: 
                                setpoint = mapping_and_planning_examples.path_planning(sensor_data,drone.dt_ctrl,drone.setpoints,drone.tol_goal)
                            else:
                                setpoint = [sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global'],0]
                        else:
                            setpoint = mapping_and_planning_examples.trajectory_tracking(sensor_data,drone.dt_ctrl,drone.timepoints,drone.setpoints, drone.tol_goal)

                        # Call the PID controller to get the motor commands
                        motorPower = drone.PID_CF.setpoint_to_pwm(drone.dt_ctrl, setpoint, sensor_data)

                    else:

                        # Read the camera feed
                        camera_data = drone.read_camera()
                        
                        # Update the sensor data in the thread
                        with sensor_lock:
                            latest_sensor_data = sensor_data
                            latest_camera_data = camera_data

                        # Call the PID controller to get the motor commands
                        motorPower = drone.PID_CF.setpoint_to_pwm(drone.dt_ctrl, current_setpoint, latest_sensor_data)
                        # motorPower = drone.PID_CF.setpoint_to_pwm(dt_ctrl, current_setpoint, latest_sensor_data)

                if exp_num == 4:
                    # Track the progress of the drone through the assignment world
                    running = drone.track_assignment_progress(sensor_data)
                    
                    # If the drone has completed the assignment, crash the drone
                    if not running:    
                        break

                # Update the PID control time
                drone.dt_ctrl = drone.getTime() - drone.PID_update_last_time # Time interval for PID control - Is refactored above for KF - why done twice?
                drone.PID_update_last_time = drone.getTime()

            # Update the drone status in simulation
            drone.step(motorPower, sensor_data)
    
    except KeyboardInterrupt:
        running = False
        planner_thread.join()



