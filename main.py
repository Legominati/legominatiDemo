#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from legominati import Legominati

import threading

ev3 = EV3Brick()
#driveSpeed=-50

minati=Legominati
sound=SoundFile()
import time


left_motor=Motor(Port.B)
right_motor=Motor(Port.C)
big_lifter=Motor(Port.D,Direction.COUNTERCLOCKWISE)
small_lifter=Motor(Port.A,Direction.COUNTERCLOCKWISE)
gyro1=GyroSensor(Port.S2,Direction.CLOCKWISE)
#gyro2=GyroSensor(Port.S3,Direction.CLOCKWISE)

line_sensor = ColorSensor(Port.S4)
# Calculate the light threshold. Choose values based on your measurements.

minati.test_method

robo=DriveBase(left_motor,right_motor,62.4,110)
robo.settings(100,500,500,500)

# Write your program here.
ev3.light.off()
time.sleep(1)
#
#ev3.light.animate([Color.BLUE,Color.GREEN,Color.RED],1)

ev3.light.on(Color.RED)
time.sleep(0.1)
ev3.light.on(Color.GREEN)
time.sleep(0.1)
ev3.light.on(Color.ORANGE)
time.sleep(0.11)

def resetGyros(winkel):
    gyro1.reset_angle(winkel)
   # gyro2.reset_angle(winkel)


def meanGyro():
    #resetGyros
    return (gyro1.angle())#+gyro2.angle())/2)

def turnPid(winkel):
    resetGyros(0)
    p=0.9
    i=0.0
    d=0.1

    w1=winkel-meanGyro()
    w2=winkel-meanGyro()
    w3=winkel-meanGyro()
    w4=winkel-meanGyro()

    for a in range(100):
        korrektur=(winkel-meanGyro())*p+(w4-w3)*d+i*(w1+w2+w3+w4)
        robo.turn(-korrektur)
        #ev3.screen.draw_text(5,20,korrektur)
          
def straightPid(v0,dist):
    resetGyros(0)
    robo.reset()
    
    while robo.distance() <= dist:
        p=0.7
        i=0.05
        d=0.2

        winkel=0

        w1=winkel-meanGyro()
        w2=winkel-meanGyro()
        w3=winkel-meanGyro()
        w4=winkel-meanGyro()

       
        korrektur=(winkel-meanGyro())*p+(w4-w3)*d+i*(w1+w2+w3+w4)
        robo.drive(v0,-korrektur)
    
    
def thread_big_lifter():
    big_lifter.run_until_stalled(1000,then=Stop.COAST,duty_limit=70)

while True:
    
    
    if Button.UP in ev3.buttons.pressed():
        thread = threading.Thread(target=thread_big_lifter)
        thread.start()
        wait(100)
        small_lifter.run_until_stalled(1000, then=Stop.COAST,duty_limit=70)
        #time.sleep(1)
    elif Button.LEFT in ev3.buttons.pressed():
        ev3.screen.clear()
        ev3.screen.draw_text(5,10,"Button left pressed") 
        ev3.speaker.play_file(sound.LEFT)
        ev3.light.on(Color.ORANGE)
        
        file1 = open("data.txt", "w")
        
        # bestimme weiss und schwarz-Wertr
        weiss=50
        schwarz=50
        t=0

        for t in range(60):
            if(line_sensor.reflection()>weiss):
                weiss=line_sensor.reflection()
            if(line_sensor.reflection()<schwarz):
                schwarz=line_sensor.reflection()
            robo.turn(1)
        
        
        ausw="Weisswert: "+ str(weiss)
        auss="Schwarzwert: "+ str(schwarz)
        while line_sensor.reflection()>1.2*schwarz:
            robo.turn(-1)

    

        print(ausw)
        print(auss)
        mittelwert=(weiss+schwarz)/2
            


        
        p = 0.5  # P-Regler
        d = -0.5    # D-Regler, soll das Taumeln dämpfen (weniger zucken)
        i = 0.03  # I-Regler, soll die Kurvenfahrt verbessern

        # Start following the line endlessly.
        a=0
        summe=0
        devOld = line_sensor.reflection() - mittelwert
        devNew = line_sensor.reflection() - mittelwert
        
        while a in range(450):
            a=a+1
            summe=summe+devNew
            if a%4==True:
                devOld=line_sensor.reflection() - mittelwert
            
            # Calculate the deviation from the threshold.
            devNew = line_sensor.reflection() - mittelwert

            aus="devNew: "+ str(devNew)       
            aus1= "devOld: " + str(devOld)
            aus3="summe: "+str(summe)
            print(aus)
            print(aus1)
            #print(aus2)
            
            
         # Calculate the turn rate.
            turn_rate = p*devNew+d*(devOld-devNew)+ i*summe
          
            aus2 = "turn_Rate: "+str(turn_rate)
            print(aus2)
            print(aus3)
            print("next")
            # Set the drive base speed and turn rate.
            L = [aus,aus1, aus2, aus3]
            file1.write(str(a)+";"+str(summe)+";"+str(devNew)+";"+str(devOld)+";"+str(turn_rate)+"\n") 
            
            
            robo.drive(driveSpeed, turn_rate)
            wait(1)
        robo.stop() 
        file1.close()   
    # You can wait for a short time or do other things in this loop.
    #wait(10)


        #time.sleep(1)
    elif Button.RIGHT in ev3.buttons.pressed():
        ev3.screen.clear()
        ev3.screen.draw_text(5,10,"Button right pressed")  
        ev3.speaker.play_file(sound.RIGHT) 
        ev3.light.on(Color.GREEN) 
        #time.sleep(1)
    elif Button.CENTER in ev3.buttons.pressed():
        ev3.screen.clear()
        ev3.screen.draw_text(5,10,"Button center")  
        ev3.speaker.play_file(sound.EV3) 
        i=0
    
        robo.straight(-330)
        
        robo.stop()
        left_motor.hold()
        right_motor.hold()
        wait(500)
        robo.straight(100)
        robo.turn(-65)
        robo.straight(-470)
        robo.turn(130)
        big_lifter.run_until_stalled(200, then=Stop.COAST, duty_limit=50)
        #big_lifter.run_time(100,5000)
        robo.straight(-100)
        #robo.turn(90)
        
       # time.sleep(1)
    elif Button.DOWN in ev3.buttons.pressed():
        ev3.screen.clear()
        
        while True:
            ev3.screen.draw_text(5,20,line_sensor.reflection())
            wait(10)
            ev3.screen.clear()


        straightPid(100,500)
        robo.stop()
        ev3.screen.draw_text(5,30,robo.distance()) 
        left_motor.hold()
        right_motor.hold()   
        ev3.screen.draw_text(5,10,"Button down pressed") 
        ev3.speaker.play_file(sound.DOWN) 
        
        #time.sleep(1)
    
    
     

for i in range (2):
    robo.straight(1000)
    time.sleep(10)  # Hält das Hold Kommando für 10 Sekunde, Räder blockieren
    robo.straight(-1000)  
    time.sleep(10)
    
    i=i+1

