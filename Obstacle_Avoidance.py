import time
from gpiozero import CamJamKitRobot, DistanceSensor


pinTrigger = 17
pinEcho = 18

robot=CamJamKitRobot()
sensor = DistanceSensor(echo=pinEcho, trigger=pinTrigger)

#Distance Variables
hownear=15.0
reversetime=0.5
turntime=0.75

leftmotorspeed=0.5
rightmotorspeed=0.5

motorforward=(leftmotorspeed, rightmotorspeed)
motorbackward=(-leftmotorspeed, -rightmotorspeed)
motorleft=(leftmotorspeed,0)
motorright=(0,rightmotorspeed)

#Return True if the ultrasonic sensor sees an obstacle
def isnearobstacle(localhownear):
    distance=sensor.distance*100
    print("IsNearObstacle: " + "str(distance)")
    if distance < localhownear:
        return True
    else:
        return False 

# move back a little, then turn right
def avoidobstacle():
    print("Backwards")
    robot.value=motorbackward
    time.sleep(reversetime)
    robot.stop()
    print("Right")
    robot.value=motorright
    time.sleep(turntime)
    robot.stop()

try:
    while True:
        robot.value=motorforward
        time.sleep(0.1)
        if isnearobstacle(hownear):
            robot.stop()
            avoidobstacle()
except KeyboardInterrupt:
    robot.stop()

