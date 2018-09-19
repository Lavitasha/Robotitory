""" is near obstacle tryout """
import time
from gpiozero import CamJamKitRobot, DistanceSensor

# Distance Variables
hownear=15.0
reversetime=0.5
turntime=0.75
pinTrigger = 17
pinEcho = 18
sensor = DistanceSensor(echo=pinEcho, trigger=pinTrigger)

robot=CamJamKitRobot()
leftmotorspeed=0.5
rightmotorspeed=0.5

motorforward=(leftmotorspeed, rightmotorspeed)
motorbackward=(-leftmotorspeed, -rightmotorspeed)
motorleft=(leftmotorspeed,0)
motorright=(0,rightmotorspeed)

def Nearobstacle(localhownear):
    distance=sensor.distance*100
    print("IsNearObstacle: " + str(distance))
    if distance < localhownear:
        
        return True
    else:
        return False

    
def Avoidobstacle():
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
        if Nearobstacle(hownear):
            robot.stop()
            Avoidobstacle()
except KeyboardInterrupt:
    robot.stop()
