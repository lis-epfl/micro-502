import numpy as np

# Create a rotation matrix from the world frame to the body frame using euler angles
def euler2rotmat(euler_angles):
    
    R = np.eye(3)
    
    # Here you need to implement the rotation matrix
    # First calculate the rotation matrix for each angle (roll, pitch, yaw)
    # Then multiply the matrices together to get the total rotation matrix

    # Inputs:
    #           euler_angles: A list of 3 Euler angles [roll, pitch, yaw] in radians
    # Outputs:
    #           R: A 3x3 numpy array that represents the rotation matrix of the euler angles
    
    # --- YOUR CODE HERE ---

    # R_roll = 
    # R_pitch = 
    # R_yaw = 

    # R =
    
    return R

# Rotate the control commands from the body reference frame to the inertial reference frame
def rot_body2inertial(control_commands, euler_angles, quaternion):
    
    # Here you need to rotate the control commands from the body reference frame to the inertial reference frame
    # You should use the euler2rotmat function to get the rotation matrix
    # Keep in mind that you only want to rotate the velocity commands, which are the first two elements of the control_commands array
    # Think carefully about which direction you need to perform the rotation

    # Inputs:
    #           control_commands: A list of 4 control commands [vel_x, vel_y, altitude, yaw_rate] in the body reference frame
    #           euler_angles: A list of 3 Euler angles [roll, pitch, yaw] in radians
    #           quaternion: A list of 4 elements [x, y, z, w] representing the quaternion of the drone
    # Outputs:
    #           control_commands: A list of 4 control commands [vel_x, vel_y, altitude, yaw_rate] in the inertial reference frame

    # --- YOUR CODE HERE ---

    # vel_world = 
    
    # R = 
    
    # vel_body = 
    
    # control_commands = 

    return control_commands


   
