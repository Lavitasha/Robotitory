import time
from gpiozero import CamJamKitRobot

robot = CamJamKitRobot()

leftmotorspeed=0.5
rightmotorspeed=0.5

motorforward=(leftmotorspeed, rightmotorspeed)
motorbackward=(-leftmotorspeed, -rightmotorspeed)
motorleft=(leftmotorspeed,0)
motorright=(0,rightmotorspeed)

robot.value=motorforward
time.sleep(1)

robot.value=motorbackward
time.sleep(1)

robot.value=motorleft
time.sleep(1)

robot.value=motorright
time.sleep(1)


robot.stop()


#right-hand motor, forward = pin 10, backward = pin 9
#left-hand motor, forward = pin 8, backward = pin 7
        
            

