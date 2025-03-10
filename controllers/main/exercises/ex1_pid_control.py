# Low-level PID control of velocity and attitude
import numpy as np
import matplotlib.pyplot as plt
from lib.simple_pid import PID
from scipy.spatial.transform import Rotation as R

class quadrotor_controller():
    def __init__(self, exp_num):
        # Exercise 1: Choose what to tune ["vel_z", "pos_z", "vel_xy", "pos_xy"]
        self.tuning_level = "off"
        
        # Only change the gains you are asked to, the others are already tuned by us (INITIAL GAINS)
        # gains = {
        #             "P_pos_z": 8.0,     "I_pos_z": 0.0,     "D_pos_z": 0.8,
        #             "P_pos_xy": 0.5,    "I_pos_xy": 0.0,    "D_pos_xy": 0.0,
        #             "P_vel_z": 2.0,     "I_vel_z": 0.0,     "D_vel_z": 1.0,
        #             "P_vel_xy": 0.2,    "I_vel_xy": 0.0,    "D_vel_xy": 0.0,
        #             "P_att_rp": 10.0,   "I_att_rp": 0.0,    "D_att_rp": 0.2,
        #             "P_att_y": 4.0,     "I_att_y": 0.0,     "D_att_y": 0.3,
        #             "P_rate_rp": 1.5,   "I_rate_rp":0.0,    "D_rate_rp": 0.1,
        #             "P_rate_y": 0.02,   "I_rate_y": 0.0,    "D_rate_y": 0.001
        #             }
        
        if exp_num == 2 or exp_num == 3:
            # KF gains and limits
            
            gains = {
                        "P_pos_z": 5.0,     "I_pos_z": 0.0,     "D_pos_z": 2.5,
                        "P_pos_xy": 1.5,    "I_pos_xy": 0.0,    "D_pos_xy": 0.0,
                        "P_vel_z": 7.0,     "I_vel_z": 0.1,     "D_vel_z": 2.0,
                        "P_vel_xy": 0.3,    "I_vel_xy": 0.0,    "D_vel_xy": 0.010,
                        "P_att_rp": 8.0,   "I_att_rp": 0.0,    "D_att_rp": 0.9,
                        "P_att_y": 2.0,     "I_att_y": 0.0,     "D_att_y": 1.0,
                        "P_rate_rp": 1.5,   "I_rate_rp":0.0,    "D_rate_rp": 0.15,
                        "P_rate_y": 0.01,   "I_rate_y": 0.0,    "D_rate_y": 0.002
                        }
            
            self.limits = {
                        "L_rate_rp": 2.0,
                        "L_rate_y": 2.0,
                        "L_acc_rp": np.pi/6,
                        "L_vel_z": 0.75,
                        "L_vel_xy": 2.0
            }
        else:
            # ### SOLUTION GAINS EXERCISE 1###
            gains = {
                        "P_pos_z": 5.0,     "I_pos_z": 0.0,     "D_pos_z": 0.8,
                        "P_pos_xy": 1.5,    "I_pos_xy": 0.0,    "D_pos_xy": 0.0,
                        "P_vel_z": 7.0,     "I_vel_z": 0.0,     "D_vel_z": 2.0,
                        "P_vel_xy": 0.5,    "I_vel_xy": 0.0,    "D_vel_xy": 0.015,
                        "P_att_rp": 10.0,   "I_att_rp": 0.0,    "D_att_rp": 0.2,
                        "P_att_y": 4.0,     "I_att_y": 0.0,     "D_att_y": 0.3,
                        "P_rate_rp": 1.5,   "I_rate_rp":0.0,    "D_rate_rp": 0.1,
                        "P_rate_y": 0.02,   "I_rate_y": 0.0,    "D_rate_y": 0.001
                        }

                        
            self.limits = {
                        "L_rate_rp": 2.0,
                        "L_rate_y": 3.0,
                        "L_acc_rp": np.pi/6,
                        "L_vel_z": 0.75,
                        "L_vel_xy": 2.0
            }
                
        self.global_time = 0
        self.mass = 0.0552 #[kg]

        self.tuning_on = False
        self.tuning_start = 7
        self.tuning_iter = 2
        self.tuning_time = 0.0
        self.tuning_ts = []
        self.tuning_desired = []
        self.tuning_actual = []

        # Position controller
        self.pid_pos_x = PID(gains["P_pos_xy"], gains["I_pos_xy"], gains["D_pos_xy"])
        self.pid_pos_y = PID(gains["P_pos_xy"], gains["I_pos_xy"], gains["D_pos_xy"])
        self.pid_pos_z = PID(gains["P_pos_z"], gains["I_pos_z"], gains["D_pos_z"])
        
        self.pid_pos_x.output_limits = (-self.limits["L_vel_xy"],self.limits["L_vel_xy"])
        self.pid_pos_y.output_limits = (-self.limits["L_vel_xy"],self.limits["L_vel_xy"])
        self.pid_pos_z.output_limits = (-self.limits["L_vel_z"],self.limits["L_vel_z"])

        # Velocity controller
        self.pid_vel_x = PID(gains["P_vel_xy"], gains["I_vel_xy"], gains["D_vel_xy"])
        self.pid_vel_y = PID(gains["P_vel_xy"], gains["I_vel_xy"], gains["D_vel_xy"])
        self.pid_vel_z = PID(gains["P_vel_z"], gains["I_vel_z"], gains["D_vel_z"])

        self.pid_vel_x.output_limits = (-self.limits["L_acc_rp"],self.limits["L_acc_rp"])
        self.pid_vel_y.output_limits = (-self.limits["L_acc_rp"],self.limits["L_acc_rp"])
        self.pid_vel_z.output_limits = (None,None)

        # Attitude controller
        self.pid_att_x = PID(gains["P_att_rp"], gains["I_att_rp"], gains["D_att_rp"])
        self.pid_att_y = PID(gains["P_att_rp"], gains["I_att_rp"], gains["D_att_rp"])
        self.pid_att_z = PID(gains["P_att_y"], gains["I_att_y"], gains["D_att_y"])
        
        self.pid_att_x.output_limits = (-self.limits["L_rate_rp"],self.limits["L_rate_rp"])
        self.pid_att_y.output_limits = (-self.limits["L_rate_rp"],self.limits["L_rate_rp"])
        self.pid_att_z.output_limits = (-self.limits["L_rate_y"],self.limits["L_rate_y"])

        # Rate controller
        self.pid_rate_roll = PID(gains["P_rate_rp"], gains["I_rate_rp"], gains["D_rate_rp"])
        self.pid_rate_pitch = PID(gains["P_rate_rp"], gains["I_rate_rp"], gains["D_rate_rp"])
        self.pid_rate_yaw = PID(gains["P_rate_y"], gains["I_rate_y"], gains["D_rate_y"])
        
        self.pid_rate_roll.output_limits = (None,None)
        self.pid_rate_pitch.output_limits = (None,None)
        self.pid_rate_yaw.output_limits = (None,None)


    def setpoint_to_pwm(self, dt, setpoint, sensor_data):
        if self.tuning_level != "off":
            setpoint = [0.97,0.84,3,0]

        ### START EXERCISE 1 SOLUTION ###
        ### Position control loop ###
        # For tuning (use later)
        if self.tuning_level == "pos_xy":
            setpoint[1] = self.tuning(-3, 3, 5, dt, setpoint[1], sensor_data["y_global"], "y position [m]")
        if self.tuning_level == "pos_z":
            setpoint[2] = self.tuning(0.5, 1.5, 5, dt, setpoint[2], sensor_data["z_global"], "z position [m]")
        # Position error in inertial frame (use sensor_data["x_global"], sensor_data["y_global"], sensor_data["z_global"])
        pos_x_error = setpoint[0] - sensor_data["x_global"]
        pos_y_error = setpoint[1] - sensor_data["y_global"]
        pos_z_error = setpoint[2] - sensor_data["z_global"]
        yaw_setpoint = setpoint[3]
        # Calculate rotation
        R_current = R.from_quat([sensor_data["q_x"], sensor_data["q_y"], sensor_data["q_z"], sensor_data["q_w"]])
        R_body_to_inertial = R_current.as_matrix()  # Rotation from body to inertial frame
        R_inertial_to_body = R_body_to_inertial.T  # Inverse (transpose for rotation matrices)
        # Rotate position error into body frame
        pos_error_inertial = np.array([pos_x_error, pos_y_error, pos_z_error])
        pos_error_body = R_inertial_to_body @ pos_error_inertial  # Rotate into body frame
        pos_x_error, pos_y_error, pos_z_error = pos_error_body
        # Put setpoint of PID controller
        self.pid_pos_x.setpoint = 0
        self.pid_pos_y.setpoint = 0
        self.pid_pos_z.setpoint = 0
        # Call PID controller
        vel_x_setpoint = self.pid_pos_x(-pos_x_error, dt=dt)
        vel_y_setpoint = self.pid_pos_y(-pos_y_error, dt=dt)
        vel_z_setpoint = self.pid_pos_z(-pos_z_error, dt=dt)
        ### Velocity control loop ###
        # For tuning (use later)
        if self.tuning_level == "vel_xy":
            vel_y_setpoint = self.tuning(-self.limits["L_vel_xy"], self.limits["L_vel_xy"], 3, dt, vel_y_setpoint, sensor_data["v_y"], "y velocity [m/s]")
        if self.tuning_level == "vel_z":
            vel_z_setpoint = self.tuning(-self.limits["L_vel_z"], self.limits["L_vel_z"], 2, dt, vel_z_setpoint, sensor_data["v_z"], "z velocity [m/s]")
        # Put setpoint of PID controller
        self.pid_vel_x.setpoint = vel_x_setpoint
        self.pid_vel_y.setpoint = vel_y_setpoint
        self.pid_vel_z.setpoint = vel_z_setpoint
        # Call PID controller (use sensor_data["v_forward"], sensor_data["v_left"], sensor_data["v_up"])
        acc_x_setpoint = self.pid_vel_x(sensor_data["v_forward"], dt=dt)
        acc_y_setpoint = self.pid_vel_y(sensor_data["v_left"], dt=dt)
        acc_z_setpoint = self.pid_vel_z(sensor_data["v_up"], dt=dt)
        return self.acceleration_and_yaw_to_pwm(dt, [acc_x_setpoint, acc_y_setpoint, acc_z_setpoint], yaw_setpoint, sensor_data)
        ### END EXERCISE 1 SOLUTION ###
    
    def keys_to_pwm(self, dt, keys, sensor_data):
        # keys = acc_x, acc_y, altitude, yaw
        vel_z_setpoint = keys[2]
        self.pid_vel_z.setpoint = vel_z_setpoint
        acc_z_setpoint = self.pid_vel_z(sensor_data["v_z"],dt=dt)
        yaw = sensor_data["yaw"] + keys[3]
        
        return self.acceleration_and_yaw_to_pwm(dt, [keys[0], keys[1], acc_z_setpoint], yaw, sensor_data)

    def acceleration_and_yaw_to_pwm(self, dt, acceleration, yaw, sensor_data):
        # for tuning
        if self.tuning_level == "att_rp":
            acceleration[1] = self.tuning(-self.limits["L_acc_rp"],self.limits["L_acc_rp"],2,dt,acceleration[1], -sensor_data["roll"], "roll [rad]", transform=True)
        if self.tuning_level == "att_y":
            yaw = self.tuning(-2,2,2,dt,yaw, sensor_data["yaw"], "yaw [rad]")

        # Attitude control loop
        self.pid_att_x.setpoint = np.clip(-acceleration[1],-np.pi/6,np.pi/6)
        self.pid_att_y.setpoint = np.clip(acceleration[0],-np.pi/6,np.pi/6)
        yaw = self.convert_yaw_setpoint(yaw, sensor_data["yaw"])
        self.pid_att_z.setpoint = yaw
        
        rate_roll_setpoint = self.pid_att_x(sensor_data["roll"],dt=dt)
        rate_pitch_setpoint = self.pid_att_y(sensor_data["pitch"],dt=dt)
        rate_yaw_setpoint = self.pid_att_z(sensor_data["yaw"],dt=dt)

        # Body Rate control loop
        if self.tuning_level == "rate_rp":
            rate_roll_setpoint = self.tuning(-self.limits["L_rate_rp"],self.limits["L_rate_rp"],0.4,dt,rate_roll_setpoint, sensor_data["rate_roll"], "roll rate [rad/s]")
        if self.tuning_level == "rate_y":
            rate_yaw_setpoint = self.tuning(-self.limits["L_rate_y"],self.limits["L_rate_y"],2.0,dt,rate_yaw_setpoint, sensor_data["rate_yaw"], "yaw rate [rad]")

        self.pid_rate_roll.setpoint = rate_roll_setpoint
        self.pid_rate_pitch.setpoint = rate_pitch_setpoint
        self.pid_rate_yaw.setpoint = rate_yaw_setpoint

        rollCommand = self.pid_rate_roll(sensor_data["rate_roll"],dt=dt)
        pitchCommand = self.pid_rate_pitch(sensor_data["rate_pitch"],dt=dt)
        yawCommand = self.pid_rate_yaw(sensor_data["rate_yaw"],dt=dt)

        k_thrust = 100
        k_rollpitch = k_thrust*0.7
        k_yaw = k_thrust*10

        commanded_thrust = self.mass * np.array([acceleration[0], acceleration[1], acceleration[2] + 9.81])  # Acceleration in body frame + gravity
        combined_thrust = np.linalg.norm(commanded_thrust)  # Magnitude of the thrust vector

        # Motor mixing
        m1 =  (k_thrust*combined_thrust - k_rollpitch*rollCommand - k_rollpitch*pitchCommand + k_yaw*yawCommand)
        m2 =  (k_thrust*combined_thrust - k_rollpitch*rollCommand + k_rollpitch*pitchCommand - k_yaw*yawCommand)
        m3 =  (k_thrust*combined_thrust + k_rollpitch*rollCommand + k_rollpitch*pitchCommand + k_yaw*yawCommand)
        m4 =  (k_thrust*combined_thrust + k_rollpitch*rollCommand - k_rollpitch*pitchCommand - k_yaw*yawCommand)

        # Limit the motor command
        m1 = np.clip(m1, 0, 600)
        m2 = np.clip(m2, 0, 600)
        m3 = np.clip(m3, 0, 600)
        m4 = np.clip(m4, 0, 600)

        self.global_time += dt
        return [m1, m2, m3, m4]
    
    def convert_yaw_setpoint(self, yaw_setpoint, yaw_measurement):
        if yaw_setpoint - yaw_measurement > np.pi:
            yaw_setpoint -= 2*np.pi
        elif yaw_setpoint - yaw_measurement < -np.pi:
            yaw_setpoint += 2*np.pi
        return yaw_setpoint

    # Only for tuning
    def set_tuning(self,level):
        self.tuning_level = level

    def tuning(self,input_min,input_max,T,dt,desired,actual,ylabel,transform=False):
        if self.global_time > self.tuning_start:
            self.tuning_on = True
        if self.tuning_on:
            if (self.tuning_iter > 0):
                desired = self.step_function(dt,input_min,input_max,T)
                # if transform:
                #     rot, _ = self.acc_to_rotation_and_thrust(0,desired,0,0)
                #     desired_att = rot.as_euler('zyx', degrees=False)[2]

                #     self.tuning_desired.append(desired_att)
                # else:
                #     self.tuning_desired.append(desired)
                self.tuning_desired.append(desired)
                self.tuning_actual.append(actual)
                self.tuning_ts.append(self.global_time)
            else:
                self.plot(ylabel)
                self.tuning_on = False
                self.tuning_start = np.inf
        return desired

    def step_function(self,dt,input_min,input_max,T):
        # Calculate step function
        if self.tuning_time < T:
            input = input_max
        else:
            input = input_min
        
        # Keep track of cycle
        self.tuning_time += dt
        if self.tuning_time >= 2*T:
            self.tuning_time = 0
            self.tuning_iter -= 1
        return input
    
    def plot(self,ylabel):
        # Plot tuning relevant data
        c_actual = "black"
        c_desired = "grey"
        c_os = [0/255,109/255,143/255]
        c_ss = [181/255,78/255,44/255]
        c_rt = [196/255,158/255,69/255]

        fig,ax = plt.subplots(1,1,figsize=(7,5))
        ax.plot(self.tuning_ts,self.tuning_desired,label="desired",color=c_desired)
        ax.plot(self.tuning_ts,self.tuning_actual,label="actual",color=c_actual)

        # Calculate steadt state error
        des_min = np.min(self.tuning_desired)
        des_max = np.max(self.tuning_desired)
        std = (des_max - des_min)/2

        idx_ss_low = -1
        desired_reverse = self.tuning_desired[::-1]
        idx_ss_high = len(self.tuning_ts) - np.argmax(np.gradient(desired_reverse)) - 2
        perc_min = (self.tuning_actual[idx_ss_low]-self.tuning_desired[idx_ss_low])/std*100
        perc_max = (self.tuning_actual[idx_ss_high]-self.tuning_desired[idx_ss_high])/std*100
        if (abs(perc_max) >= 1):
            ax.plot([self.tuning_ts[idx_ss_high],self.tuning_ts[idx_ss_high]],[self.tuning_desired[idx_ss_high],self.tuning_actual[idx_ss_high]],
                    linewidth=3,color=c_ss,label="steady state error")
            ax.text(x=self.tuning_ts[idx_ss_high],y=self.tuning_actual[idx_ss_high],s=" "+str(int(abs(perc_max)))+" [%]",
                    color=c_ss,fontsize="x-large",horizontalalignment="left",verticalalignment="center")
        if (abs(perc_min) >= 1):
            ax.plot([self.tuning_ts[idx_ss_low],self.tuning_ts[idx_ss_low]],[self.tuning_desired[idx_ss_low],self.tuning_actual[idx_ss_low]],
                    linewidth=3,color=c_ss)
            ax.text(x=self.tuning_ts[idx_ss_low],y=self.tuning_actual[idx_ss_low],s=" "+str(int(abs(perc_min)))+" [%]",
                    color=c_ss,fontsize="x-large",horizontalalignment="left",verticalalignment="center")

        # Calculate overshoot
        last = idx_ss_high + 1
        phase = len(self.tuning_ts) - last
        crop = len(self.tuning_ts) - 2*phase + 1
        idx_os_low = last + np.argmin(self.tuning_actual[last::])
        idx_os_high = crop + np.argmax(self.tuning_actual[crop:last])
        perc_min = (self.tuning_actual[idx_os_low]-self.tuning_desired[idx_os_low])/std*100
        perc_max = (self.tuning_actual[idx_os_high]-self.tuning_desired[idx_os_high])/std*100
        if (self.tuning_desired[idx_os_low] > self.tuning_actual[idx_os_low]) & (abs(perc_min) >= 1):
            ax.plot([self.tuning_ts[idx_os_low],self.tuning_ts[idx_os_low]],[self.tuning_desired[idx_os_low],self.tuning_actual[idx_os_low]],
                    linewidth=3,color=c_os,label="overshoot")
            ax.text(x=self.tuning_ts[idx_os_low],y=self.tuning_actual[idx_os_low],s=str(int(abs(perc_min)))+" [%] ",
                    color=c_os,fontsize="x-large",horizontalalignment="right",verticalalignment="center")
        if (self.tuning_desired[idx_os_high] < self.tuning_actual[idx_os_high]) & (abs(perc_max) >= 1):
            ax.plot([self.tuning_ts[idx_os_high],self.tuning_ts[idx_os_high]],[self.tuning_desired[idx_os_high],self.tuning_actual[idx_os_high]],
                    linewidth=3,color=c_os)
            ax.text(x=self.tuning_ts[idx_os_high],y=self.tuning_actual[idx_os_high],s=str(int(abs(perc_max)))+" [%] ",
                    color=c_os,fontsize="x-large",horizontalalignment="right",verticalalignment="center")
        
        # # Calculate rise time
        limit = 0.05
        idx_rt_low = last + np.argmax((self.tuning_actual[last::]-des_min)/abs(des_min) < limit)
        idx_rt_high = crop + np.argmax((des_max-self.tuning_actual[crop:last+1])/des_max < limit)

        rt_high = self.tuning_ts[idx_rt_high] - self.tuning_ts[crop]
        rt_low = self.tuning_ts[idx_rt_low] - self.tuning_ts[last]
        if idx_rt_high > crop:
            ax.plot([self.tuning_ts[crop],self.tuning_ts[idx_rt_high]],[self.tuning_desired[idx_rt_high],self.tuning_desired[idx_rt_high]],
                    linewidth=3,color=c_rt,label="rise time")
            ax.text(x=self.tuning_ts[idx_rt_high],y=self.tuning_actual[idx_rt_high],s=str(np.round(rt_high,1))+"[s]",
                    color=c_rt,fontsize="x-large",horizontalalignment="right",verticalalignment="top")
        if idx_rt_low > last:
            ax.plot([self.tuning_ts[last],self.tuning_ts[idx_rt_low]],[self.tuning_desired[idx_rt_low],self.tuning_desired[idx_rt_low]],
                    linewidth=3,color=c_rt)
            ax.text(x=self.tuning_ts[idx_rt_low],y=self.tuning_actual[idx_rt_low],s=str(np.round(rt_low,1))+"[s]",
                        color=c_rt,fontsize="x-large",horizontalalignment="right",verticalalignment="bottom")

        ax.set_xlabel("time [s]")
        ax.set_ylabel(ylabel)
        plt.legend(loc="upper left")
        plt.tight_layout()
        plt.show()

        self.tuning_level = "off"
