import Leap, sys, URBasic, URBasic.urScript, time, socket, math

#host = '169.254.226.180'  #Real Robot IP
host = '192.168.81.128'    #Simulation IP
port=63352 #PORT used by robotiq gripper

#UR5
robotModle = URBasic.robotModel.RobotModel()
robot = URBasic.urScriptExt.UrScriptExt(host=host,robotModel=robotModle)
robot.reset_error()
dg=0

##Gripper
# with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
#     s.connect((host, port))
#     s.sendall(b'SET ACT 1\n')


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

            #Reduce framrate of coordinates to be used for the arm
            if (time.time() - self.last_print_time) > self.print_interval:
                self.last_print_time = time.time()
                
                #Convertion from Leap Motion space to UR space
                x, y, z = (hand.palm_position[0]/500, hand.palm_position[1]/1000, hand.palm_position[2]/500)
                rx, ry, rz = (direction.pitch, (1.3*normal.roll + 2.9), direction.yaw)

                #Limit area of freedom for UR5 arm
                if x < -0.6: x = -0.6 
                if x > -0.2: x = -0.2
                if z < -0.4: z = -0.4
                if z >  0.4: z = 0.4
                if y <  0.2: y = 0.2
                if y >  0.6: y = 0.6

                #Set arm position
                robot.movej(pose=[z,x,y, -0,ry,0], a=1, v=50)
                tcp = (round(x, 3), round(y, 3), round(z, 3), round(rx, 3), round(ry, 3), round(rz, 3))
                print("TCP position: " + str(tcp))
            
                for finger in hand.fingers:

                    # Get bones
                    for b in range(0, 4):
                        bone = finger.bone(b)

                #Get position, tip of thumb and tip of index finger        
                t = hand.fingers[0].bone(bone.type).next_joint
                i = hand.fingers[1].bone(bone.type).next_joint

                #Calculate distance between tip pf index finger and thumb
                d = math.sqrt(((i[0]-t[0])**2 + (i[1]-t[1])**2 + (i[2]-t[2])**2))
                if d > 120:
                    d = 120
                if d < 15:
                    d = 15

                #Convert distance between fingers to value of gripper
                dg = (d - 15) * (0 - 226) / (120 - 15) + 226
                print("Gripper position: " + str(int(dg)))

            #     #set position of gripper
            #     with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            #         s.connect((host, port))
            #         #s.sendall(b'SET ACT 1\n')   ##Activate gripper
            #         s.sendall(b'SET SPE 200\n')
            #         if x > -0.34:
            #             s.sendall(b'SET POS 0\n')
            #         elif x <= -0.34:
            #             s.sendall(b'SET POS 226\n')
            #     s.sendall(b'GET POS\n')
            #     data = s.recv(2**10)
            # print('Gripper finger position is: ', data)
            # print(round(d,0))

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