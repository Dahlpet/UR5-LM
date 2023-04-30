import Leap, sys, URBasic, URBasic.urScript, time, socket, math, URleap
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import numpy as np

#host = '169.254.226.180'  #Real Robot IP
host = '192.168.81.128'    #Simulation IP
#host = '192.168.12.128'    #Simulation2 IP
port = 63352 #PORT used by robotiq gripper

#UR5
robotModle = URBasic.robotModel.RobotModel()
robot = URBasic.urScriptExt.UrScriptExt(host=host,robotModel=robotModle)
robot.reset_error()
#dg=0

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.set_xlim3d([-1.0, 1.0])
# ax.set_ylim3d([-1.0, 1.0])
# ax.set_zlim3d([0.0, 1.5])
# new_x, new_y, new_z = 0, 0, 0
# line, = ax.plot([], [], [], lw=2)

#Gripper
# with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
#     s.connect((host, port))
#     s.sendall(b'SET ACT 1\n')
#     s.sendall(b'SET SPE 255\n')
#     s.sendall(b'SET FOR 0\n')


class SampleListener(Leap.Listener):


    #Control framerate from Leap Motion
    last_print_time = 0.5
    print_interval = 0.9 # Print once per second 

    def on_frame(self, controller):
        # Get the most recent frame and report some basic information
        frame = controller.frame()

        # Get hands
        for hand in frame.hands:

            # Get the hand's normal vector and direction            
            normal = hand.palm_normal
            direction = hand.direction

            # line.set_data([new_x, new_y])
            # line.set_3d_properties([new_z])
            # fig.canvas.draw()


            #Reduce framrate from LMC
            if (time.time() - self.last_print_time) > self.print_interval:
                self.last_print_time = time.time()
                
                #Get position from LMC and declare
                x, y, z = (hand.palm_position[2], hand.palm_position[0], hand.palm_position[1])

                #Convert from LMC space to UR5 space
                new_x, new_y, new_z = URleap.LMC_space_to_UR5_space(x, y, z)

                #Limit the work area of the UR5
                new_x, new_y, new_z = URleap.apply_bounds(new_x, new_y, new_z)

                pitch, roll, yaw = (direction.pitch, normal.roll, direction.yaw)


                def rotation_matrix(pitch, roll, yaw):
                    Rx = np.array([[1, 0, 0],
                        [0, np.cos(pitch), -np.sin(pitch)],
                        [0, np.sin(pitch), np.cos(pitch)]])

                    Ry = np.array([[np.cos(roll), 0, np.sin(roll)],
                        [0, 1, 0],
                        [-np.sin(roll), 0, np.cos(roll)]])

                    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                        [np.sin(yaw), np.cos(yaw), 0],
                        [0, 0, 1]])

                    R = Rz.dot(Ry).dot(Rx)

                    return R

            # URleap.rotation_matrix(pitch, roll, yaw)
            # URleap.wrist_pos(pitch, roll, yaw, R)
   
                R = rotation_matrix(pitch, roll, yaw)
                rz = math.atan2(R[1,0], R[0,0])
                ry = math.atan2(-R[2,0], math.sqrt(R[2,1]**2 + R[2,2]**2))
                rx = math.atan2(R[2,1], R[2,2]) 
                print("WRIST", rx, ry, rz)

                #Move command to the robot
                robot.movej(pose=[new_x,new_y,new_z, 0,3.14,0], a=0.7, v=2.5)

                for finger in hand.fingers:

                    # Get bones
                    for b in range(0, 4):
                        bone = finger.bone(b)

                #Get position on tip of thumb and tip of index finger        
                t = hand.fingers[0].bone(bone.type).next_joint
                i = hand.fingers[1].bone(bone.type).next_joint

                

                # line.set_data([new_x, new_y])
                # line.set_3d_properties([new_z])
                # fig.canvas.draw()

                #Calculate distance between tip pf index finger and thumb and convert to gripper pos.
                # URleap.print_TCP(new_x, new_y, new_z)
                # robot.print_actual_tcp_pose()
                # jpos = robot.get_actual_joint_positions()
                # print("Joint Positions:", jpos)
                
                ######### dg = URleap.gripper_pos(t,i)

               ############ URleap.print_gripper_pos(dg)

                # print("  pitch: %f degrees, roll: %f degrees, yaw: %f degrees" % (
                #     direction.pitch * Leap.RAD_TO_DEG,
                #     normal.roll * Leap.RAD_TO_DEG,
                #     direction.yaw * Leap.RAD_TO_DEG))
                
                # pitch, roll, yaw = (direction.pitch, normal.roll, direction.yaw)
                

                # def rotation_matrix(pitch, roll, yaw):
                #     Rx = np.array([[1, 0, 0],
                #         [0, np.cos(pitch), -np.sin(pitch)],
                #         [0, np.sin(pitch), np.cos(pitch)]])

                #     Ry = np.array([[np.cos(roll), 0, np.sin(roll)],
                #         [0, 1, 0],
                #         [-np.sin(roll), 0, np.cos(roll)]])

                #     Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                #         [np.sin(yaw), np.cos(yaw), 0],
                #         [0, 0, 1]])

                #     R = Rz.dot(Ry).dot(Rx)

                #     return R

            # URleap.rotation_matrix(pitch, roll, yaw)
            # URleap.wrist_pos(pitch, roll, yaw, R)
   
                # R = rotation_matrix(pitch, roll, yaw)
                # print("Matrix", R)

                # rz = math.atan2(R[1,0], R[0,0])
                # ry = math.atan2(-R[2,0], math.sqrt(R[2,1]**2 + R[2,2]**2))
                # rx = math.atan2(R[2,1], R[2,2]) 
                # print("WRIST", rx, ry, rz)

    # Perform the desired operations based on the updated orientation
    ...

   
                # with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                #     s.connect((host, port))
                #     if dg < 113:
                #         s.sendall(b'SET POS 0\n')
                #     if dg > 113:
                #         s.sendall(b'SET POS 255\n')
                #URleap.plotter(new_x, new_y, new_z)


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