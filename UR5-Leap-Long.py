import Leap, sys, URBasic, URBasic.urScript, time, socket, math

#host = '169.254.226.180'  #Real Robot IP
#host = '192.168.81.128'    #Simulation IP
host = '192.168.12.128'    #Simulation2 IP
port=63352 #PORT used by robotiq gripper

#UR5
robotModle = URBasic.robotModel.RobotModel()
robot = URBasic.urScriptExt.UrScriptExt(host=host,robotModel=robotModle)
robot.reset_error()
dg=0


# #Gripper
# with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
#     s.connect((host, port))
#     s.sendall(b'SET ACT 1\n')


class SampleListener(Leap.Listener):
    finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
    bone_names = ['Metacarpal', 'Proximal', 'Intermediate', 'Distal']

    last_print_time = 0.0
    print_interval = 0.8 # Print once per second

    def on_init(self, controller):
        pass

    def on_connect(self, controller):
        pass

    def on_disconnect(self, controller):
        # Note: not dispatched when running in a debugger.
        pass

    def on_exit(self, controller):
        pass

    def on_frame(self, controller):
        # Get the most recent frame and report some basic information
        frame = controller.frame()

        print("Frame id: %d, timestamp: %d, hands: %d, fingers: %d" % (
              frame.id, frame.timestamp, len(frame.hands), len(frame.fingers)))

        # Get hands
        for hand in frame.hands:

            handType = "Left hand" if hand.is_left else "Right hand"

            print("  %s, id %d, position: %s" % (
                handType, hand.id, hand.palm_position))
            
            # Get the hand's normal vector and direction            
            normal = hand.palm_normal
            direction = hand.direction

            # Calculate the hand's pitch, roll, and yaw angles
            print("  pitch: %f degrees, roll: %f degrees, yaw: %f degrees" % (
                direction.pitch * Leap.RAD_TO_DEG,
                normal.roll * Leap.RAD_TO_DEG,
                direction.yaw * Leap.RAD_TO_DEG))
            
            # Get arm bone
            arm = hand.arm
            print("  Arm direction: %s, wrist position: %s, elbow position: %s" % (
                arm.direction,
                arm.wrist_position,
                arm.elbow_position))
            
            # Get fingers
            for finger in hand.fingers:

                print("    %s finger, id: %d, length: %fmm, width: %fmm" % (
                    self.finger_names[finger.type],
                    finger.id,
                    finger.length,
                    finger.width))
                
                # Get bones
                for b in range(0, 4):
                    bone = finger.bone(b)
                    print("      Bone: %s, start: %s, end: %s, direction: %s" % (
                        self.bone_names[bone.type],
                        bone.prev_joint,
                        bone.next_joint,
                        bone.direction))
            
            #Reduce framrate of coordinates to be used for the arm
            if (time.time() - self.last_print_time) > self.print_interval:
                self.last_print_time = time.time()
                
                #Convertion from Leap Motion space to UR space
                x, y, z = (hand.palm_position[0]/500, hand.palm_position[1]/1000, hand.palm_position[2]/500)
                rx, ry, rz = direction.pitch, (1.3*normal.roll + 2.9), direction.yaw

                #Limit area of freedom for UR5 arm
                if x < -0.6: x = -0.6 
                if x > -0.2: x = -0.2
                if z < -0.4: z = -0.4
                if z >  0.4: z = 0.4
                if y <  0.2: y = 0.2
                if y >  0.6: y = 0.6

                #Set arm position
                robot.movej(pose=[z,x,y, -0,ry,0], a=1, v=50)
                print(round(x, 3), round(y, 3), round(z, 3), round(rx, 3), round(ry, 3), round(rz, 3))
            
                # Calculate the hand's pitch, roll, and yaw angles
                print("  pitch: %f degrees, roll: %f degrees, yaw: %f degrees" % (
                    direction.pitch * Leap.RAD_TO_DEG,
                    normal.roll * Leap.RAD_TO_DEG,
                    direction.yaw * Leap.RAD_TO_DEG))

            # Get arm bone
                arm = hand.arm
                print("  Arm direction: %s, wrist position: %s, elbow position: %s" % (
                    arm.direction,
                    arm.wrist_position,
                    arm.elbow_position))

                for finger in hand.fingers:

                    # Get bones
                    for b in range(0, 4):
                        bone = finger.bone(b)
                        
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
                print(dg)


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