
#new value = (x - a) * (d - c) / (b - a) + c
def LMC_space_to_UR5_space(x, y, z):
    new_x = (x - 130) * (-0.6 - (-0.23)) / (-170 - 130) - 0.23
    new_y = (y - (-200)) * (0.43 - (-0.43)) / (200 - (-200)) - 0.43
    new_z = (z - 200) * (0.5 - 0.2) / (500 - 200) + 0.2
    return new_x, new_y, new_z

def apply_bounds(new_x, new_y, new_z):
    if new_x < -0.6:
        new_x = -0.6
    if new_x > -0.23:
        new_x = -0.23
        
    if new_y < -0.43:
        new_y = -0.43
    if new_y > 0.43:
        new_y = 0.43
        
    if new_z < 0.2:
        new_z = 0.2
    if new_z > 0.5:
        new_z = 0.5
    
    return new_x, new_y, new_z
