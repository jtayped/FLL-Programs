#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.media.ev3dev import Font
import time
from threading import Thread

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information. 


# Create your objects here.

font_of = Font(family="Terminal", size=1)
# Create your objects here.
ev3 = EV3Brick()
ev3.screen.set_font(font_of)
#ev3.screen.set_font("Lat15-Terminus12x6")
ev3.screen.print("[ NEXT ]: Medium Motors A/C: ")
medium_motorL = Motor(Port.A)
medium_motorR = Motor(Port.C)
ev3.screen.print("[ NEXT ]: Wheel Motors B/D: ")
left_wheel = Motor(Port.B) 
right_wheel = Motor(Port.D)
ev3.screen.print("[ NEXT ]: Drive Base: ")
robot = DriveBase(left_wheel, right_wheel, wheel_diameter=90, axle_track=130)
ev3.screen.print("[ NEXT ]: Gyro S3: ")
gyro = GyroSensor(Port.S3)
ev3.screen.print("[ NEXT ]: Color Sensor S1: ")
light_left = ColorSensor(Port.S1)
ev3.screen.print("[ NEXT ]: Color Sensor S4: ")
light_right = ColorSensor(Port.S4)
ev3.screen.print("[ NEXT ]: Color Sensor S2: ")
color_sensor= ColorSensor(Port.S2)

def start_sound(beep_num):
    freq = 100
    for i in range(beep_num):
        freq += 300
        ev3.speaker.beep(freq, 100)
        time.sleep(0.1)

ev3.screen.print("[ OK ]: Ready to go!")
ev3.speaker.set_speech_options("ca", "f1", 250, 50)
time_wait = 1
while True:
    ev3.screen.print(ev3.battery.voltage())
    if ev3.battery.voltage() > 8470:
        ev3.speaker.say("Ja he acabat de carregar!")
        for i in range(200):
            
            ev3.speaker.beep(700, 50)

            time.sleep(1/time_wait)

            time_wait += 0.4
        time_wait = 1

        ev3.speaker.play_file(SoundFile.MOTOR_STOP)


