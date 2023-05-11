import math


#new value = (x - a) * (d - c) / (b - a) + c
def LMC_space_to_UR5_space(x, y, z):

    '''
    converts the values from LMC to the coresponding values for UR5. This provides the mapping
    between the two devices.
    '''

    new_x = (x - 130) * (-0.6 - (-0.23)) / (-170 - 130) - 0.23
    new_y = (y - (-200)) * (0.43 - (-0.43)) / (200 - (-200)) - 0.43
    new_z = (z - 200) * (0.5 - 0.2) / (500 - 200) + 0.2

    return new_x, new_y, new_z


def apply_bounds_TCP(new_x, new_y, new_z):

    '''
    Set limitations for work area for the UR5. By defining lower and upper bounds for
    x-values, y-values and z-values, this provides a 3D cube where the TCP is allowed
    to move
    '''

    new_x = max(-0.6, min(-0.23, new_x))
    new_y = max(-0.43, min(0.43, new_y))
    new_z = max(0.2, min(0.5, new_z))

    return new_x, new_y, new_z


def apply_bounds_wrist(rx, ry, rz):

    '''
    Set limitations for roll, pitch and yaw positions for the wrist. By defining lower and upper bounds for
    rx-values, ry-values and rz-values, this provides a limitation for rpy-values. 
    '''
    rx = max(-3.14, min(3.14, rx))
    ry = max(-1, min(1, ry))
    rz = max(-1.3, min(1.3, rz))

    return rx, ry, rz


def gripper_pos(i,t):

    '''
    Calculates the distance between the thumb and index finger, and converts this value to
    coresponding value for the position of the gripper.

    Position is limited til open or closed

    i = tip of index finger, t = tip of thumb
    '''

    f_dist = math.sqrt(((i[0]-t[0])**2 + (i[1]-t[1])**2 + (i[2]-t[2])**2))
    f_conv = (f_dist - 15) * (0 - 226) / (120 - 15) + 226
    f_conv = max(15, min(120, f_conv))
    return f_conv, f_dist

def print_gripper_pos(f_conv):

    '''
    Prints the status of the gripper, opern/closed
    '''

    if f_conv > 120:
        print("Gripper status: Closed")
    if f_conv < 15:
        print("Gripper status: Open")


def print_TCP(new_x, new_y, new_z):

    '''
    Prints the Target for the TCP given by the position of the hand
    '''
    print("TCP Pose", "x:", round((new_x)*100,3), "cm     |", "y:", round((new_y)*100,3), "cm     |", "z:", round((new_z)*100,3), "cm")

def wrist_pos(rx, ry, rz):

    '''
    Mapping of the wrist positions
    '''

    rx_new = (rx - (-0.8)) * (0.293 - 0.4) / (0.8 - (-0.8)) + 0.4
    ry_new = (ry - (-0.8)) * (-2.614 - (-3.7)) / (0.8 - (-0.8)) + (-3.7)
    rz_new = (rz - (-0.8)) * (-0.112 - 0.22) / (0.8 - (-0.8)) + 0.22

    return rx_new, ry_new, rz_new

def space():
    print("################################################\n")

def thumb_index_distance(i,t):
    dist = math.sqrt((t[0]-i[0])**2 + (t[1]-i[1])**2 + (t[2]-i[2])**2)
    print("Finger distance", round(dist/10,2), "cm")

def print_joint_pos(jpos):
    print("Joint Positions:", round(jpos[0],3), "  |", round(jpos[1],3), "  |", round(jpos[2],3), "  |", round(jpos[3],3), "  |", round(jpos[4],3), "  |", round(jpos[5],3))

def print_wrist_RPYs(rx,ry,rz):
    rx_d = math.degrees(rx)
    ry_d = math.degrees(ry)
    rz_d = math.degrees(rz)
    print("Wrist RPY:", round(ry_d,3),"deg", "  |", round(rx_d,3),"deg", "  |", round(rz_d,3),"deg")

