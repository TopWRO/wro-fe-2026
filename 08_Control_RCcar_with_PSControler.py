#!/usr/bin/env python3
'''
'''

import os
import time
import sys
import serial
import time
import pygame




# ======================================================
"""
init joystick
"""
joystick_data={"axis0":0,"axis1":0,"axis2":0,"axis3":0,"axis4":0,"axis5":0,"button0":0,"button1":0,"button2":0,"button3":0,}
pygame.init()
pygame.joystick.init()
joystick_count = pygame.joystick.get_count()
try:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    time.sleep(0.1)
    timer01=time.time()
    while time.time()-timer01<3:
        event=pygame.event.get()
        if event != []:
            print("joystick  No"+str(joystick_count)+"  is connected")
            
            break
        else:
            print("joystick has not data send,please try again.")
            time.sleep(0.01)
except :
    print("no joystick connected")
#=======================================================

'''
run Main CODE
'''
# 



if __name__ == '__main__':

    
    while True:
        


# wireless controller data      
        event=pygame.event.get()
        if event != []:
            joystick_data["axis0"] = round(joystick.get_axis( 0 ),2)
            joystick_data["axis4"] = round(joystick.get_axis( 4 ),2)
            print("axis0",joystick_data["axis0"],"                             axis4",joystick_data["axis4"])

#Prepare the PMW data
        


        time.sleep(0.05)