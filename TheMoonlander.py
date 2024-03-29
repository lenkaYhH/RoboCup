#!/usr/bin/env python
# coding: utf-8

# In[ ]:


from mirte_robot import robot
import time
mirte = robot.createRobot()

MOTOR_SPEED = 30
RIGHT = True
LINE = 600

def move(t, speedL=MOTOR_SPEED, speedR=MOTOR_SPEED):
    mirte.setMotorSpeed('left', speedL)
    mirte.setMotorSpeed('right', speedR)
    time.sleep(t)
    mirte.stop()
    
def on_line():
    R = mirte.getIntensity('right')
    L = mirte.getIntensity('left')

    if R > LINE:
        return 1

    elif L > LINE:
        return -1

    else: 
        return 0

def sweep():
    global RIGHT
    steps = 60
    
    if RIGHT == True and 9 < mirte.getDistance("right") <= 10:
        print("obstacle detected: " + str(mirte.getDistance("right")))
        for i in range(1, steps):
            mirte.setServoAngle("left", i*(180//steps))
            time.sleep(0.01)
        RIGHT = False
        
    elif RIGHT == False and 9 < mirte.getDistance("right") <= 10:
        print("obstacle detected: " + str(mirte.getDistance("right")))
        for i in range(1, steps):
            mirte.setServoAngle("left", 180 - i*(180//steps))
            time.sleep(0.01)
            
        RIGHT = True
    
    
mirte.setServoAngle("left", 3)

while mirte.getIntensity('right') < 3000 and mirte.getIntensity('left') < 3000:  
  
    sweep()
    move(0.01)

    output = on_line()
    if output == 1 or output == -1:
        mirte.stop()
        time.sleep(0.5)
        move(0.05, 2* output * MOTOR_SPEED, 2* -1 * output * MOTOR_SPEED)
        
#     elif output == 0:
#         mirte.stop()
#         time.sleep(0.001)
      
mirte.stop()


print("Terminated")


# In[ ]:




