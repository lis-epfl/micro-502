# Examples of basic methods for simulation competition
import numpy as np
import matplotlib.pyplot as plt
import time

# Global variables
on_ground = True
height_desired = 0.5
timer = None
startpos = None
timer_done = None

# Obstacle avoidance with range sensors
def obstacle_avoidance(sensor_data):
    global on_ground, height_desired, startpos

    if startpos is None:
        startpos = [sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global']]
    if on_ground and sensor_data['z_global'] < 0.49:
        current_setpoint = [startpos[0], startpos[1], height_desired, 0.0]
        return current_setpoint
    else:
        on_ground = False

    # Obstacle avoidance with distance sensors
    if sensor_data['range_front'] < 0.2:
        if sensor_data['range_left'] > sensor_data['range_right']:
            control_command = [sensor_data['x_global'], sensor_data['y_global']+0.1, height_desired, sensor_data['yaw']]
        else:
            control_command = [sensor_data['x_global'], sensor_data['y_global']-0.1, height_desired, sensor_data['yaw']]
    else:
        control_command = [sensor_data['x_global']+0.1, sensor_data['y_global'], height_desired, sensor_data['yaw']]

    return control_command

# Coverage path planning
index_current_setpoint = 0
def path_planning(sensor_data, dt, setpoints, tol):
    global on_ground, height_desired, index_current_setpoint, timer, timer_done, startpos

    # Take off
    if startpos is None:
        startpos = [sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global']]
        # print(startpos)    
    if on_ground and sensor_data['z_global'] < 0.49:
        #current_setpoint = [0.97,0.84,height_desired,0]
        current_setpoint = [startpos[0], startpos[1], height_desired, 0.0]
        # print(current_setpoint)
        return current_setpoint
    else:
        on_ground = False

    # Start timer
    if (index_current_setpoint == 1) & (timer is None):
        timer = 0
        print("Time recording started")
    if timer is not None:
        timer += dt
    # Hover at the final setpoint
    if index_current_setpoint == len(setpoints):
        control_command = [0.0, 0.0, height_desired, 0.0] #[startpos[0], startpos[1], startpos[2]-0.05, 0.0]

        if timer_done is None:
            timer_done = True
            print("Path planning took " + str(np.round(timer,1)) + " [s]")
        return control_command

    # Get the goal position and drone position
    current_setpoint = setpoints[index_current_setpoint]
    x_drone, y_drone, z_drone, yaw_drone = sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global'], sensor_data['yaw']
    distance_drone_to_goal = np.linalg.norm([current_setpoint[0] - x_drone, current_setpoint[1] - y_drone, current_setpoint[2] - z_drone, current_setpoint[3] - yaw_drone%(2*np.pi)])

    # When the drone reaches the goal setpoint, e.g., distance < 0.1m
    if distance_drone_to_goal < tol:
        # Select the next setpoint as the goal position
        index_current_setpoint += 1
        # Hover at the final setpoint
        if index_current_setpoint == len(setpoints):
            current_setpoint = [0.0, 0.0, height_desired, 0.0]
            return current_setpoint

    return current_setpoint

def trajectory_tracking(sensor_data, dt, timepoints, setpoints, tol, repeat = False):
    global on_ground, index_current_setpoint, timer, timer_done

    start_point = setpoints[0]
    end_point = setpoints[-1]

    # Take off 
    if on_ground and sensor_data['z_global'] < start_point[2] - 0.01:
        current_setpoint = [sensor_data['x_global'], sensor_data['y_global'], sensor_data['z_global'] + 0.5, sensor_data['yaw']]
        return current_setpoint
    else:
        on_ground = False
        if timer is None:
            # Begin timer and start trajectory
            timer = 0
            print("Trajectory tracking started")
            index_current_setpoint = 1
        else:
            timer += dt

    # Determine the current setpoint based on the time
    if not on_ground and timer is not None:
        if index_current_setpoint < len(timepoints) - 1:
            # Update new setpoint
            if timer >= timepoints[index_current_setpoint]:
                index_current_setpoint += 1
            current_setpoint = setpoints[index_current_setpoint,:]
        else:
            # Hover at the final setpoint
            current_setpoint = end_point
            if timer_done is None and np.linalg.norm([sensor_data['x_global'] - end_point[0], sensor_data['y_global'] - end_point[1], sensor_data['z_global'] - end_point[2]]) < tol:
                timer_done = True
                print("Trajectory took " + str(np.round(timer,1)) + " [s]")
                if repeat:
                    timer_done = None
                    timer = None
                
    return current_setpoint
    
    
# Occupancy map based on distance sensor
min_x, max_x = 0, 5.0 # meter
min_y, max_y = 0, 5.0 # meter
range_max = 2.0 # meter, maximum range of distance sensor
res_pos = 0.2 # meter
conf = 0.2 # certainty given by each measurement
t = 0 # only for plotting

map = np.zeros((int((max_x-min_x)/res_pos), int((max_y-min_y)/res_pos))) # 0 = unknown, 1 = free, -1 = occupied

def occupancy_map(sensor_data):
    global map, t
    pos_x = sensor_data['x_global']
    pos_y = sensor_data['y_global']
    yaw = sensor_data['yaw']
    
    for j in range(4): # 4 sensors
        yaw_sensor = yaw + j*np.pi/2 #yaw positive is counter clockwise
        if j == 0:
            measurement = sensor_data['range_front']
        elif j == 1:
            measurement = sensor_data['range_left']
        elif j == 2:
            measurement = sensor_data['range_back']
        elif j == 3:
            measurement = sensor_data['range_right']
        
        for i in range(int(range_max/res_pos)): # range is 2 meters
            dist = i*res_pos
            idx_x = int(np.round((pos_x - min_x + dist*np.cos(yaw_sensor))/res_pos,0))
            idx_y = int(np.round((pos_y - min_y + dist*np.sin(yaw_sensor))/res_pos,0))

            # make sure the current_setpoint is within the map
            if idx_x < 0 or idx_x >= map.shape[0] or idx_y < 0 or idx_y >= map.shape[1] or dist > range_max:
                break

            # update the map
            if dist < measurement:
                map[idx_x, idx_y] += conf
            else:
                map[idx_x, idx_y] -= conf
                break
    
    map = np.clip(map, -1, 1) # certainty can never be more than 100%

    # only plot every Nth time step (comment out if not needed)
    if t % 50 == 0:
        plt.imshow(np.flip(map,1), vmin=-1, vmax=1, cmap='gray', origin='lower') # flip the map to match the coordinate system
        plt.savefig("map.png")
    t +=1

    return map