# Low-level PID control of velocity and attitude
import numpy as np
import matplotlib.pyplot as plt
from simple_pid import PID
from scipy.spatial.transform import Rotation as R

class quadrotor_controller_setpoint():
    def __init__(self):
        # Exercise 1: Choose what to tune ["vel_z", "pos_z", "rate_rp", "rate_y", "att", "vel_xy", "pos_xy"]
        self.tuning_level = "off"

        # Exercise 1: Tune gains (we suggest: P < 25, I = 0 in most cases, D < 2)
        # gains = {
        #             "P_vel_z": 6.0,     "I_vel_z": 1.0,      "D_vel_z": 0.8,
        #             "P_pos_z": 2.5,     "I_pos_z": 0.0,      "D_pos_z": 1.0,
        #             "P_rate_rp": 0.5,   "I_rate_rp":0.0,     "D_rate_rp": 0.03,
        #             "P_rate_y": 0.01,   "I_rate_y": 0.0,     "D_rate_y": 0.001,
        #             "P_att": 18.0,      "I_att":0.0,         "D_att": 0.3,
        #             "P_vel_xy": 5.0,    "I_vel_xy": 1.0,     "D_vel_xy": 0.15,
        #             "P_pos_xy": 2.2,    "I_pos_xy": 0.0,     "D_pos_xy": 0.03
        # }

        # KF Gains
        gains = {"P_vel_z": 6.0,     "I_vel_z": 1.0,     "D_vel_z": 0.8,
            "P_pos_z": 2.5,     "I_pos_z": 0.0,     "D_pos_z": 1.0,
            "P_rate_rp": 0.2,     "I_rate_rp":0.0,      "D_rate_rp": 0.03,
            "P_rate_y": 0.01,      "I_rate_y": 0.0,      "D_rate_y": 0.001,
            "P_att": 12.0,      "I_att": 0.0,      "D_att": 0.2,
            "P_vel_xy": 2.0,     "I_vel_xy": 0.0,     "D_vel_xy": 0.10,
            "P_pos_xy": 1.5,     "I_pos_xy": 0.0,     "D_pos_xy": 0.02}
        
        # Bonus: Increase limits and retune
        self.limits = {
                    "L_rate_rp": 2.0,
                    "L_rate_y": 3.0,
                    "L_acc_rp": 10.0,
                    "L_vel_z": 1.25,
                    "L_vel_xy": 3.0
        }
        
        # DO NOT CHANGE ANYTHING BELOW
        self.global_time = 0
        self.mass = 0.05 #[kg]

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
        self.pid_att_x = PID(gains["P_att"], gains["I_att"], gains["D_att"])
        self.piD_att = PID(gains["P_att"], gains["I_att"], gains["D_att"])
        self.pid_att_z = PID(gains["P_att"], gains["I_att"], gains["D_att"])
        
        self.pid_att_x.output_limits = (-self.limits["L_rate_rp"],self.limits["L_rate_rp"])
        self.piD_att.output_limits = (-self.limits["L_rate_rp"],self.limits["L_rate_rp"])
        self.pid_att_z.output_limits = (-self.limits["L_rate_y"],self.limits["L_rate_y"])

        # Rate controller
        self.pid_rate_roll = PID(gains["P_rate_rp"], gains["I_rate_rp"], gains["D_rate_rp"])
        self.pid_rate_pitch = PID(gains["P_rate_rp"], gains["I_rate_rp"], gains["D_rate_rp"])
        self.pid_rate_yaw = PID(gains["P_rate_y"], gains["I_rate_y"], gains["D_rate_y"])
        
        self.pid_rate_roll.output_limits = (None,None)
        self.pid_rate_pitch.output_limits = (None,None)
        self.pid_rate_yaw.output_limits = (None,None)


    def pid(self, dt, setpoint, sensor_data):

        if self.tuning_level != "off":
            setpoint = [0.966047,0.837248,1,0]

        pos_x_setpoint = setpoint[0]
        pos_y_setpoint = setpoint[1]
        pos_z_setpoint = setpoint[2]
        att_z_setpoint = setpoint[3]

        # Position control loop
        if self.tuning_level == "pos_xy":
            pos_y_setpoint = self.tuning(-3,3,5,dt,pos_y_setpoint, sensor_data["y_global"], "y position [m]")
        if self.tuning_level == "pos_z":
            pos_z_setpoint = self.tuning(0.5,1.5,5,dt,pos_z_setpoint, sensor_data["z_global"], "z position [m]")

        self.pid_pos_x.setpoint = pos_x_setpoint
        self.pid_pos_y.setpoint = pos_y_setpoint
        self.pid_pos_z.setpoint = pos_z_setpoint

        vel_x_setpoint = self.pid_pos_x(sensor_data["x_global"],dt=dt)
        vel_y_setpoint = self.pid_pos_y(sensor_data["y_global"],dt=dt)
        vel_z_setpoint = self.pid_pos_z(sensor_data["z_global"],dt=dt)

        # Velocity control loop
        if self.tuning_level == "vel_xy":
            vel_y_setpoint = self.tuning(-self.limits["L_vel_xy"],self.limits["L_vel_xy"],3,dt,vel_y_setpoint, sensor_data["v_y"], "y velocity [m/s]")
        if self.tuning_level == "vel_z":
            vel_z_setpoint = self.tuning(-self.limits["L_vel_z"],self.limits["L_vel_z"],2,dt,vel_z_setpoint, sensor_data["v_z"], "z velocity [m/s]")

        self.pid_vel_x.setpoint = vel_x_setpoint
        self.pid_vel_y.setpoint = vel_y_setpoint
        self.pid_vel_z.setpoint = vel_z_setpoint

        acc_x_setpoint = self.pid_vel_x(sensor_data["v_x"],dt=dt)
        acc_y_setpoint = self.pid_vel_y(sensor_data["v_y"],dt=dt)
        acc_z_setpoint = self.pid_vel_z(sensor_data["v_z"],dt=dt)

        # Convert linear accelerations to orientation        
        if self.tuning_level == "att":
            acc_y_setpoint = self.tuning(-self.limits["L_acc_rp"],self.limits["L_acc_rp"],2,dt,acc_y_setpoint, sensor_data["roll"], "roll [rad]", transform=True)
        if self.tuning_level == "att_y":
            att_z_setpoint = self.tuning(-2,2,1,dt,att_z_setpoint, sensor_data["yaw"], "yaw [rad]")

        R_setpoint, combined_thrust = self.acc_to_rotation(acc_x_setpoint, acc_y_setpoint, acc_z_setpoint, att_z_setpoint)

        # Calculate error quaternion
        R_current = R.from_quat([sensor_data["q_x"], sensor_data["q_y"], sensor_data["q_z"], sensor_data["q_w"]])        
        R_current_inv = R_current.inv()
        error_quat = (R_current_inv*R_setpoint).as_quat()


        # Attitude control loop
        self.pid_att_x.setpoint = 0
        self.piD_att.setpoint = 0
        self.pid_att_z.setpoint = 0
        
        rate_roll_setpoint = self.pid_att_x(-error_quat[0]*np.sign(error_quat[3]),dt=dt)
        rate_pitch_setpoint = self.piD_att(-error_quat[1]*np.sign(error_quat[3]),dt=dt)
        rate_yaw_setpoint = self.pid_att_z(-error_quat[2]*np.sign(error_quat[3]),dt=dt)

        # Body Rate control loop
        if self.tuning_level == "rate_rp":
            rate_roll_setpoint = self.tuning(-self.limits["L_rate_rp"],self.limits["L_rate_rp"],0.2,dt,rate_roll_setpoint, sensor_data["rate_roll"], "roll rate [rad/s]")
        if self.tuning_level == "rate_y":
            rate_yaw_setpoint = self.tuning(-self.limits["L_rate_y"],self.limits["L_rate_y"],1.0,dt,rate_yaw_setpoint, sensor_data["rate_yaw"], "yaw rate [rad]")

        self.pid_rate_roll.setpoint = rate_roll_setpoint
        self.pid_rate_pitch.setpoint = rate_pitch_setpoint
        self.pid_rate_yaw.setpoint = rate_yaw_setpoint

        rollCommand = self.pid_rate_roll(sensor_data["rate_roll"],dt=dt)
        pitchCommand = self.pid_rate_pitch(sensor_data["rate_pitch"],dt=dt)
        yawCommand = self.pid_rate_yaw(sensor_data["rate_yaw"],dt=dt)

        k_thrust = 100
        k_rollpitch = k_thrust*0.7
        k_yaw = k_thrust*10

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

    def acc_to_rotation(self, acc_x, acc_y, acc_z, yaw):
        commanded_thrust = self.mass*np.array([acc_x, acc_y, acc_z + 9.81])
        # commanded_thrust = [1,0,0]
        combined_thrust = np.linalg.norm(commanded_thrust)

        # Build quaternion from direction (commanded thrust) and yaw
        z_b = commanded_thrust/combined_thrust
        yaw_matrix = R.from_euler('z', yaw).as_matrix()
        x_b_prime = yaw_matrix@np.array([1,0,0])
        y_b = np.cross(z_b,x_b_prime)
        y_b /= np.linalg.norm(y_b)
        x_b = np.cross(y_b,z_b)
        x_b /= np.linalg.norm(x_b)
        r_matrix = np.array([x_b,y_b,z_b]).T
        R_setpoint = R.from_matrix(r_matrix)
        
        return R_setpoint, combined_thrust
    
    # Only for tuning
    def set_tuning(self,level):
        self.tuning_level = level

    def tuning(self,input_min,input_max,T,dt,desired,actual,ylabel,transform=False):
        if self.global_time > self.tuning_start:
            self.tuning_on = True
        if self.tuning_on:
            if (self.tuning_iter > 0):
                desired = self.step_function(dt,input_min,input_max,T)

                if transform:
                    rot, _ = self.acc_to_rotation(0,desired,0,0)
                    desired_att = rot.as_euler('zyx', degrees=False)[2]

                    self.tuning_desired.append(desired_att)
                else:
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


class quadrotor_controller_command():
    def __init__(self):
        self.pastVxError = 0
        self.pastVyError = 0
        self.pastAltError = 0
        self.pastPitchError = 0
        self.pastRollError = 0
        self.altIntegrator = 0
        self.last_time = 0.0

    def pid(self, dt, command, sensor_data):
        actual_roll = sensor_data["roll"]
        actual_pitch = sensor_data["pitch"]
        actual_yaw_rate = sensor_data["rate_yaw"]
        actual_alt = sensor_data["z_global"]
        actual_vforward = sensor_data["v_forward"]
        actual_vleft = sensor_data["v_left"]

        gains = {"kp_att_y": 1, "kd_att_y": 0.5, "kp_att_rp": 0.5, "kd_att_rp": 0.1,
                "kp_vel_xy": 2, "kd_vel_xy": 0.5, "kp_z": 10, "ki_z": 5, "kd_z": 5}

        # Actions
        desired_vx, desired_vy, desired_alt, desired_yaw_rate = command[0], command[1], command[2], command[3]

        # Velocity PID control
        vxError = desired_vx - actual_vforward
        vxDeriv = (vxError - self.pastVxError) / dt
        vyError = desired_vy - actual_vleft
        vyDeriv = (vyError - self.pastVyError) / dt
        desired_pitch = gains["kp_vel_xy"] * np.clip(vxError, -1, 1) + gains["kd_vel_xy"] * vxDeriv
        desired_roll = -gains["kp_vel_xy"] * np.clip(vyError, -1, 1) - gains["kd_vel_xy"] * vyDeriv
        self.pastVxError = vxError
        self.pastVyError = vyError

        # Altitude PID control
        altError = desired_alt - actual_alt
        altDeriv = (altError - self.pastAltError) / dt
        self.altIntegrator += altError * dt
        altCommand = gains["kp_z"] * altError + gains["kd_z"] * altDeriv + gains["ki_z"] * np.clip(self.altIntegrator, -2, 2) + 48
        self.pastAltError = altError

        # Attitude PID control
        pitchError = desired_pitch - actual_pitch
        pitchDeriv = (pitchError - self.pastPitchError) / dt
        rollError = desired_roll - actual_roll
        rollDeriv = (rollError - self.pastRollError) / dt
        yawRateError = desired_yaw_rate - actual_yaw_rate
        rollCommand = gains["kp_att_rp"] * np.clip(rollError, -1, 1) + gains["kd_att_rp"] * rollDeriv
        pitchCommand = -gains["kp_att_rp"] * np.clip(pitchError, -1, 1) - gains["kd_att_rp"] * pitchDeriv
        yawCommand = gains["kp_att_y"] * np.clip(yawRateError, -1, 1)
        self.pastPitchError = pitchError
        self.pastRollError = rollError

        # Motor mixing
        m1 =  altCommand - rollCommand + pitchCommand + yawCommand
        m2 =  altCommand - rollCommand - pitchCommand - yawCommand
        m3 =  altCommand + rollCommand - pitchCommand + yawCommand
        m4 =  altCommand + rollCommand + pitchCommand - yawCommand

        # Limit the motor command
        m1 = np.clip(m1, 0, 600)
        m2 = np.clip(m2, 0, 600)
        m3 = np.clip(m3, 0, 600)
        m4 = np.clip(m4, 0, 600)

        return [m1, m2, m3, m4]