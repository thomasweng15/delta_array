import numpy as np
import matplotlib.pyplot as plt
import time

from serial import Serial
from pyparsing import delimitedList
from Prismatic_Delta import Prismatic_Delta
from get_coords import RoboCoords
import telnetlib
import delta_array_pb2
from control_delta_arrays import DeltaArrayAgent

arduino = Serial('/dev/ttyACM0', 57600)  

DDA = DeltaArrayAgent(arduino, 0)
s_p = 1.5 #side length of the platform
s_b = 4.3 #side length of the base
l = 4.5 #length of leg attached to platform

Delta = Prismatic_Delta(s_p, s_b, l)

def create_joint_positions(val):
    a = []
    for i in range(3):
        for j in range(3):
            a.append(val[j])
    a += [0, 0, 0]
    return a


if __name__=="__main__":
    # env = control_delta_arrays.DeltaArrayEnv("COM7")
    
    wave = 3*np.sin(np.linspace(-2*np.pi, 2*np.pi, 20))
    # plt.plot(wave)
    # plt.show()
    
    delta_message = delta_array_pb2.DeltaMessage()

    delta_message.id = 0
    delta_message.request_joint_pose = False
    delta_message.request_done_state = False
    delta_message.reset = False
    for i in range(10000):
        # ee_pts = list(np.array(x.split(","), dtype=float))
        
        x = input("Go signal")
        for j in range(20):
            print(j)
            ee_pts = [0,0,10+wave[j]]
            pts = Delta.IK(ee_pts)
            pts = np.array(pts) * 0.01
            pts = np.clip(pts,0.005,0.095)
            jts = create_joint_positions(pts)
            _ = [delta_message.joint_pos.append(jts[i]) for i in range(12)]
            # print(delta_message)
            serialized = delta_message.SerializeToString()
            
            if b"\xa7" in serialized:
                print("HAKUNA")
                
            arduino.write(b'\xa6~~'+ serialized + b'\xa7~~\r\n')
            # reachedPos = ['__']
            # while reachedPos!='126': 
                # reachedPos = str(arduino.readline().strip().decode("utf-8"))
                # print(reachedPos)
            # print(str(arduino.readline()))

            del delta_message.joint_pos[:]


    # for i in np.arange(-2,2,0.1):
    #     for j in np.arange(-2,2,0.1):
    #         for k in np.arange(6,16,0.1):
    #             val = Delta.IK([i,j,k])
    #             val = np.array(val)/100
    #             vals = np.concatenate([val,val,val,val])
                
    #             for n in range(20):
    #                 _ = [delta_message.trajectory.append(vals[i]) for i in range(12)]
    #             serialized= delta_message.SerializeToString()
    #             del delta_message.trajectory[:]
    #             if b'p' in serialized:
    #                 print("HAKUNA")
    #                 # print(i,j,k)
    #                 print(len(serialized))
    #             else:
    #                 # print(serialized)
    #                 continue
    #                 # print(serialized)