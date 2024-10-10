from RobotRaconteur.Client import *
import time
import numpy as np

c = RRN.ConnectService('rr+tcp://localhost:58660?service=robot')

robot_info = c.robot_info
print(robot_info)

time.sleep(0.1)

c.enable()

print(c.robot_state.PeekInValue()[0].command_mode)

robot_const = RRN.GetConstants("com.robotraconteur.robotics.robot", c)

halt_mode = robot_const["RobotCommandMode"]["halt"]
jog_mode = robot_const["RobotCommandMode"]["jog"]

c.command_mode = halt_mode
time.sleep(0.1)
c.command_mode = jog_mode

joint_pos = np.zeros((14,),dtype=np.float64)
joint_vel = 1.5*np.ones((14,),dtype=np.float64)

i = 0

for i in range(10):
    if i % 2 == 0:
        joint_pos = np.ones((14,))*-0.5
    else:
        joint_pos = np.ones((14,))*0.5
    c.jog_freespace(joint_pos, joint_vel, True)
    print(hex(c.robot_state.PeekInValue()[0].robot_state_flags))
    time.sleep(1)
    i+=1