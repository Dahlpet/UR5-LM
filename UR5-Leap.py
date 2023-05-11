import Leap, sys, URBasic, URBasic.urScript, time, socket, URleap

#set speed and acceleration of the robot
acc, vel = 0.7, 2.5

#host = '169.254.226.180'  #Real Robot IP
#host = '192.168.81.128'    #Simulation IP
host = '192.168.12.128'    #Simulation2 IP
port = 63352 #PORT used by robotiq gripper

#Create a UR5 robot 
robotModle = URBasic.robotModel.RobotModel()
robot = URBasic.urScriptExt.UrScriptExt(host=host,robotModel=robotModle)
robot.reset_error()

# Activate the gripper, and set speed and force
# with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
#     s.connect((host, port))
#     s.sendall(b'SET ACT 1\n')
#     s.sendall(b'SET SPE 255\n')
#     s.sendall(b'SET FOR 0\n')


class SampleListener(Leap.Listener):


    #Control framerate from Leap Motion to the robot
    last_print_time = 0
    print_interval = 0.8 # Print once per unit time

    def on_frame(self, controller):

        # Get the most recent frame and report some basic information
        frame = controller.frame()

        # Get hands
        for hand in frame.hands:


            # Get the hand's normal vector and direction            
            normal = hand.palm_normal
            direction = hand.direction

            #Reduce framrate from LMC
            if (time.time() - self.last_print_time) > self.print_interval:
                self.last_print_time = time.time()
                
                #Get position from LMC and declare
                x, y, z = (hand.palm_position[2], hand.palm_position[0], hand.palm_position[1])
                rx, ry, rz = (direction.pitch, -normal.roll, direction.yaw)

                #Convert from LMC space to UR5 space
                new_x, new_y, new_z = URleap.LMC_space_to_UR5_space(x, y, z)

                #Limit the work area of the UR5
                new_x, new_y, new_z = URleap.apply_bounds_TCP(new_x, new_y, new_z)

                #Limit rpy movements of the wrist
                rx, ry, rz = URleap.apply_bounds_wrist(rx, ry, rz)
  

                #Move command to the robot, type: movej or movel
                #robot.movej(pose=[new_x,new_y,new_z, -rz,-3.14,0], a=acc, v=vel)
                robot.movel(pose=[new_x,new_y,new_z, rz,rx-3.14,ry], a=acc, v=vel)

                #Get fingers
                for finger in hand.fingers:

                    # Get bones
                    for b in range(0, 4):
                        bone = finger.bone(b)

                #Get position on tip of thumb and tip of index finger        
                t = hand.fingers[0].bone(bone.type).next_joint
                i = hand.fingers[1].bone(bone.type).next_joint

                #Print information to the terminal
                URleap.print_TCP(new_x, new_y, new_z)
                jpos = robot.get_actual_joint_positions()
                URleap.print_joint_pos(jpos)
                URleap.print_wrist_RPYs(rx, ry, rz)
                
                f_conv, f_dist = URleap.gripper_pos(t,i)
                
                URleap.print_gripper_pos(f_conv)
                URleap.thumb_index_distance(i,t)
                URleap.space()

                #Control the gripper
                # with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                #     s.connect((host, port))
                #     if dg < 113:
                #         s.sendall(b'SET POS 0\n') #Open gripper
                #     if dg > 113:
                #         s.sendall(b'SET POS 255\n') #Close gripper


def ExampleurScriptLEAP():

        listener = SampleListener()
        controller = Leap.Controller()
        controller.add_listener(listener)
     
        print("Press Enter to quit...")
        try:
            sys.stdin.readline()
        except KeyboardInterrupt:
            pass
        finally:
            controller.remove_listener(listener)
            robot.close()
        SampleListener.frame = 0

if __name__ == '__main__':
    ExampleurScriptLEAP()