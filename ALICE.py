import Leap, sys
from scipy.spatial.transform import Rotation as R

'''
host = '169.254.226.180'  #Real Robot IP

#UR5
robotModle = URBasic.robotModel.RobotModel()
robot = URBasic.urScriptExt.UrScriptExt(host=host,robotModel=robotModle)
robot.reset_error()
dg=0
'''
class SampleListener(Leap.Listener):

    def on_frame(self, controller):
        
        frame = controller.frame()

        for hand in frame.hands:
            
            #Palm position
            palm_pos = hand.palm_position           
            
            #Palm orientation
            palm_norm = hand.palm_normal
            palm_dir = hand.direction
            rx, ry, rz = palm_dir.pitch, palm_norm.roll, palm_dir.yaw

            #Convert from RPY to Axis-Angle
            Rot = R.from_euler('ZYX',[rx,ry,rz])
            Mat = R.as_matrix(Rot)
            axis_angle = R.as_rotvec(Rot)
            
            print("------------------------------------------------------------------------")
            print("Pitch: ",rx, "Roll: ", ry, "Yaw: ", rz)
            print("Rotasjonsmatrise:\n",Mat,
                  "\n Axis angle: ", axis_angle) 
            
            #Inverse kinematics
            targetPosition = [palm_pos[0],palm_pos[1],palm_pos[2],axis_angle[0],axis_angle[1],axis_angle[2]]
            print("Target position", targetPosition)
            print("------------------------------------------------------------------------")
            '''
            actualJointPosition = robot.get_actual_joint_positions()
            jointPosition = Invkine_manip(targetPosition,actualJointPosition)

            #Move to target position
            robot.movej(q=jointPosition, a=1, v=50)
            '''
            
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
            #robot.close()
        SampleListener.frame = 0

if __name__ == '__main__':
    ExampleurScriptLEAP()