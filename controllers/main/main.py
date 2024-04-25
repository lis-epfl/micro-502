# Main simulation file called by the Webots

import numpy as np
from controller import Supervisor, Keyboard
from control import quadrotor_controller_setpoint, quadrotor_controller_command
from kalman_filter import kalman_filter as KF
import utils
from scipy.spatial.transform import Rotation as R
import my_control
import time, random
import cv2

exp_num = 3                     # 0: Coordinate Transformation, 1: PID Tuning, 2: Kalman Filter, 3: Practical
control_style = 'autonomous'      # 'keyboard' or 'autonomous'
rand_env = True                # Randomise the environment for the practical exercise

path_around_arena = [[0.0, 0.0, 1.0, 0.0], [0.0, 3.0, 1.25, np.pi/2], [5.0, 3.0, 1.5, np.pi], [5.0, 0.0, 0.25, 1.5*np.pi], [0.0, 0.0, 1.0, 0.0]]

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

        # Kalman filter variables
        self.KF = KF()
        self.sensor_flag = 0
        self.dt_accel = 0.0
        self.dt_gps = 0.0
        self.dt_propagate = 0.0

        self.meas_state_gps = np.zeros((2,1))
        self.meas_state_accel = np.zeros((3,1))

        self.accel_read_last_time = 0.0
        self.gps_read_last_time = 0.0

        if exp_num == 2:
            self.ctrl_update_period = int(self.timestep*3) #timestep equal to GPS time 2 or 3 works well
            self.gps_update_period = int(self.timestep*3) # 2*timestep
            self.accel_update_period = int(self.timestep*2) # 1*timestep
        else:
            self.ctrl_update_period = self.timestep
            self.gps_update_period = self.timestep
            self.accel_update_period = self.timestep

        # Sensors

        #Update rates for excercise 2 (Kalman filter)

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
        self.PID_SP = quadrotor_controller_setpoint()
        self.PID_CM = quadrotor_controller_command()
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

        # Set a random initial yaw of the drone
        # drone = super().getSelf()
        # init_yaw_drone = random.uniform(-np.pi, np.pi)
        # # init_yaw_drone = np.pi/6
        # rotation_field = drone.getField('rotation')
        # rotation_field.setSFRotation([0, 0, 1, init_yaw_drone])

        # For the assignment, randomise the positions of the drone, obstacles, goal, take-off pad and landing pad 
        if exp_num == 3:

            # Variables to track progress
            self.reached_landing_zone = False
            self.reached_landing_pad = False
            self.reached_goal_first = False
            self.reached_goal_second = False
            self.reached_takeoff_zone = False
            self.returned_to_takeoff_pad = False
                
            if rand_env:
                # Set random initial position of the drone
                init_x_drone, init_y_drone = random.uniform(0.3, 1.2), random.uniform(0.3, 2.7)
                drone = super().getSelf()
                translation_field = drone.getField('translation')
                translation_field.setSFVec3f([init_x_drone, init_y_drone, 0.2])
            else:
                drone = super().getSelf()
                translation_field = drone.getField('translation')
                init_x_drone, init_y_drone, _ = translation_field.getSFVec3f()
                translation_field.setSFVec3f([init_x_drone, init_y_drone, 0.2])

            # Set random initial position of the take-off pad
            take_off_pad = super().getFromDef('TAKE_OFF_PAD')
            translation_field = take_off_pad.getField('translation')
            translation_field.setSFVec3f([init_x_drone, init_y_drone, 0.05])
            self.take_off_pad_position = [init_x_drone, init_y_drone]

            if rand_env:
                # Set random initial position of the landing pad
                self.landing_pad_position = [random.uniform(3.8, 4.7), random.uniform(0.3, 2.7)]
                landing_pad = super().getFromDef('LANDING_PAD')
                translation_field = landing_pad.getField('translation')
                translation_field.setSFVec3f([self.landing_pad_position[0], self.landing_pad_position[1], 0.05])
            else:
                landing_pad = super().getFromDef('LANDING_PAD')
                translation_field = landing_pad.getField('translation')
                self.landing_pad_position = translation_field.getSFVec3f()

            if rand_env:
                # Set random initial position of the Goal
                self.goal_position = [random.uniform(2.3, 2.7), random.uniform(0.3, 2.7), random.uniform(0.4,1.3)]
                goal = super().getFromDef('GOAL')
                translation_field = goal.getField('translation')
                translation_field.setSFVec3f(self.goal_position)
            else:
                goal = super().getFromDef('GOAL')
                translation_field = goal.getField('translation')
                self.goal_position = translation_field.getSFVec3f()

            # Set Goal dimensions
            self.goal_height = 0.4
            self.goal_width = 0.4
            self.goal_depth = 0.1    

            if rand_env:
                # Set random initial positions of obstacles
                existed_points = []
                existed_points.append([init_x_drone, init_y_drone])
                existed_points.append([self.landing_pad_position[0], self.landing_pad_position[1]])
                existed_points.append([self.goal_position[0], self.goal_position[1]])
                for i in range(1, 11):
                    find_appropriate_random_position = False
                    while not find_appropriate_random_position:
                        # Generate new random position
                        new_init_x_obs, new_init_y_obs = random.uniform(0, 5.0), random.uniform(0, 3.0)
                        min_distance = 1000
                        # Calculate the min distance to existed obstacles and pads
                        for point in existed_points:
                            distance = np.linalg.norm([point[0] - new_init_x_obs, point[1] - new_init_y_obs])
                            if distance < min_distance:
                                min_distance = distance
                        if min_distance > 0.8:
                            find_appropriate_random_position = True
                    # Accept position that is 0.8m far away from existed obstacles and pads
                    obstacle = super().getFromDef('OBSTACLE' + str(i))
                    translation_field = obstacle.getField('translation')
                    translation_field.setSFVec3f([new_init_x_obs, new_init_y_obs, 0.74])
                    existed_points.append([new_init_x_obs, new_init_y_obs])


            # # Start an OpenCV window to display the camera feed
            # cv2.startWindowThread()
            # cv2.namedWindow("Camera Feed")

        # Simulation step update
        super().step(self.timestep)

    def wait_keyboard(self):
        while self.keyboard.getKey() != ord('Y'):
            super().step(self.timestep)

    def action_from_keyboard(self, sensor_data):
        forward_velocity = 0.0
        left_velocity = 0.0
        yaw_rate = 0.0
        z_reference = 0.75
        key = self.keyboard.getKey()
        while key > 0:
            if key == ord('W'):
                forward_velocity = 1.0
            elif key == ord('S'):
                forward_velocity = -1.0
            elif key == ord('A'):
                left_velocity = 1.0
            elif key == ord('D'):
                left_velocity = -1.0
            elif key == ord('Q'):
                yaw_rate = 1.0
            elif key == ord('E'):
                yaw_rate = -1.0
            elif key == ord('V'):
                z_reference = 1.5
            elif key == ord('C'):
                z_reference = 0.0
            key = self.keyboard.getKey()
        return [forward_velocity, left_velocity, z_reference, yaw_rate]

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
        KF_state_outputs['v_down'] = v_z_g_est
        KF_state_outputs['ax_global'] = a_x_g_est
        KF_state_outputs['ay_global'] = a_y_g_est
        KF_state_outputs['az_global'] = a_z_g_est

        # Call appending of states over run
        self.KF.aggregate_states(measured_data_raw, measured_noisy_data, KF_state_outputs, self.getTime())

        if self.KF.use_noisy_measurement:
            if self.getTime() < 2.0:
                output_measurement = measured_data_raw
            else:
                output_measurement = measured_noisy_data.copy()
        elif self.KF.use_ground_truth_measurement:
            output_measurement = measured_data_raw.copy()
        else:
            output_measurement = KF_state_outputs.copy()

        return output_measurement
    
    def read_sensors(self):
        
        # Sensor data call values
        # "x_global": Global X position
        # "y_global": Global Y position
        # "z_global": Global Z position
        # "roll": Roll angle (rad)
        # "pitch": Pitch angle (rad)
        # "yaw": Yaw angle (rad)
        # "q_x": Quaternion x value
        # "q_y": Quaternion y value
        # "q_z": Quaternion z value
        # "q_w": Quaternion w value
        # "v_x": Global X velocity
        # "v_y": Global Y velocity
        # "v_z": Global Z velocity
        # "v_forward": Forward velocity (body frame)
        # "v_left": Leftward velocity (body frame)
        # "v_down": Downward velocity (body frame)
        # "ax_global": Global X acceleration
        # "ay_global": Global Y acceleration
        # "az_global": Global Z acceleration
        # "range_front": Front range finder distance
        # "range_down": Downward range finder distance
        # "range_left": Leftward range finder distance 
        # "range_back": Backward range finder distance
        # "range_right": Rightward range finder distance
        # "rate_roll": Roll rate (rad/s)
        # "rate_pitch": Pitch rate (rad/s)
        # "rate_yaw": Yaw rate (rad/s)

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
        data['v_down'] =  self.vz_global

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

    
    # Create a function to detect if the drone has reached the landing pad, if it has set the GOAL object to be transparent
    def check_landing_pads(self, sensor_data):
        
        drone_position = [sensor_data['x_global'], sensor_data['y_global'], sensor_data['range_down']]
        
        # Check if the drone is in the landing zone
        if drone_position[0] > 3.5 and drone_position[0] < 5.0 and drone_position[1] > 0.0 and drone_position[1] < 3.0 and not self.reached_landing_zone:
            print("Congratulations! You have reached the landing zone, now find the landing pad.")
            self.reached_landing_zone = True
        
        
        # Check if the drone has reached the landing pad
        landing_pad_distance = np.linalg.norm([drone_position[0] - self.landing_pad_position[0], drone_position[1] - self.landing_pad_position[1], drone_position[2] - 0.1])
        pad_half_size = 0.155
        if drone_position[0] > self.landing_pad_position[0] - pad_half_size and drone_position[0] < self.landing_pad_position[0] + pad_half_size \
            and drone_position[1] > self.landing_pad_position[1] - pad_half_size and drone_position[1] < self.landing_pad_position[1] + pad_half_size \
            and drone_position[2] < 0.05 and not self.reached_landing_pad:
            goal_node = super().getFromDef('GOAL')
            cam_node = super().getFromDef('CF_CAMERA')
            goal_node.setVisibility(cam_node, 0)
            print("Congratulations! You have reached the landing pad, the goal is now hidden.")
            self.reached_landing_pad = True

        
        if drone_position[0] > 0.0 and drone_position[0] < 1.5 and drone_position[1] > 0.0 and drone_position[1] < 3.0 and not self.reached_takeoff_zone and self.reached_landing_zone:
            print("Congratulations! You have made it back to the takeoff zone, now find the takeoff pad.")
            self.reached_takeoff_zone = True
        
        # Check if the drone has made it back to the takeoff pad after reaching the landing pad
        if self.reached_landing_zone and not self.returned_to_takeoff_pad:
            if drone_position[0] > self.take_off_pad_position[0] - 0.1 and drone_position[0] < self.take_off_pad_position[0] + 0.1 \
                and drone_position[1] > self.take_off_pad_position[1] - 0.1 and drone_position[1] < self.take_off_pad_position[1] + 0.1 \
                and drone_position[2] < 0.05:
                print("Congratulations! You have made it back to the takeoff pad.")
                self.returned_to_takeoff_pad = True     

    
    # Create a function to detect if the drone has reached the goal
    def check_goal(self, sensor_data):

        drone_position = [sensor_data['x_global'], sensor_data['y_global'], sensor_data['range_down']]

        # Check that the drone is within the goal
        goal_x_min = self.goal_position[0] - self.goal_depth / 2
        goal_x_max = self.goal_position[0] + self.goal_depth / 2

        goal_y_min = self.goal_position[1] - self.goal_width / 2
        goal_y_max = self.goal_position[1] + self.goal_width / 2

        goal_z_min = self.goal_position[2] - self.goal_height / 2
        goal_z_max = self.goal_position[2] + self.goal_height / 2

        if (goal_x_min < drone_position[0] < goal_x_max and goal_y_min < drone_position[1] < goal_y_max and goal_z_min < drone_position[2] < goal_z_max):
            
            if not self.reached_goal_first:
                print("Congratulations! You have reached the goal for the first time.")
                self.reached_goal_first = True

            elif self.reached_goal_first and not self.reached_goal_second and self.reached_landing_pad:
                print("Congratulations! You have reached the goal after it was hidden.")
                self.reached_goal_second = True


    def reset(self):
        # Reset the simulation
        self.simulationResetPhysics()
        self.simulationReset()
        super().step(self.timestep)

    def step_physics(self):
        super().step(self.timestep)

    def step_KF(self, KF_data):

        self.dt_ctrl = self.getTime() - self.PID_update_last_time

        if np.round(self.dt_ctrl,3) >= self.ctrl_update_period/1000:
            
            pp_cmds = my_control.path_to_setpoint(path_around_arena, KF_data, self.dt_ctrl)

            self.PID_update_last_time = self.getTime()
            # Low-level PID velocity control with fixed height
            motorPower = self.PID_SP.pid(self.dt_ctrl, pp_cmds, KF_data)
        
            # Update motor command
            self.m1_motor.setVelocity(-motorPower[0])
            self.m2_motor.setVelocity(motorPower[1])
            self.m3_motor.setVelocity(-motorPower[2])
            self.m4_motor.setVelocity(motorPower[3])

        if np.round(self.getTime(),2) == self.KF.plot_time_limit:
            self.KF.plot_states()

        # Update drone states in simulation
        super().step(self.timestep)


if __name__ == '__main__':

    # Initialize the drone
    drone = CrazyflieInDroneDome()
    assert control_style in ['keyboard','autonomous'], "Variable control_style must either be 'keyboard' or 'autonomous'"
    assert exp_num in [0,1,2,3], "Exp_num must be a value between 0 and 3"

    # Simulation loops
    for step in range(100000):
        # Default path around the arena
        if exp_num == 2:
            assert control_style == 'autonomous', "Variable control_style must be set to 'autonomous' for this exercise"
            state_data = drone.read_KF_estimates()
            # Update the drone status in simulation with KF
            drone.step_KF(state_data)

        else:
            # Read sensor data including []
            sensor_data = drone.read_sensors()
            camera_data = drone.read_camera()
            dt_ctrl = drone.getTime() - drone.PID_update_last_time

            if exp_num == 3:
                drone.check_landing_pads(sensor_data)
                drone.check_goal(sensor_data)
                control_commands = my_control.get_command(sensor_data, camera_data, dt_ctrl)
            else:
                setpoint = my_control.path_to_setpoint(path_around_arena,sensor_data,dt_ctrl)

            if control_style == 'keyboard':
                control_commands = drone.action_from_keyboard(sensor_data)

            # Update the drone status in simulation
            dt_ctrl = drone.getTime() - drone.PID_update_last_time
            # Time interval for PID control
            drone.PID_update_last_time = drone.getTime()
            # Low-level PID velocity control with fixed height
            if exp_num != 3 and control_style != 'keyboard':
                motorPower = drone.PID_SP.pid(dt_ctrl, setpoint, sensor_data)
            else:
                motorPower = drone.PID_CM.pid(dt_ctrl, control_commands, sensor_data)
                
            # Update motor command
            drone.m1_motor.setVelocity(-motorPower[0])
            drone.m2_motor.setVelocity(motorPower[1])
            drone.m3_motor.setVelocity(-motorPower[2])
            drone.m4_motor.setVelocity(motorPower[3])
            
            drone.step_physics()

        # ---- end --- #



