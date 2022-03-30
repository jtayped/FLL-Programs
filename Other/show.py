#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import time
from pybricks.media.ev3dev import Font

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

ev3.speaker.set_speech_options("es", "f4", 120, 65)
ev3.speaker.set_volume(500)

ev3.screen.print("[ OK ]: Ready to go!")

robot.settings(straight_speed= -700, straight_acceleration= 300, turn_rate= 600, turn_acceleration= 1000)

robot.reset()
while True:

    if Button.LEFT in ev3.buttons.pressed():
        medium_motorL.run_angle(-1000, 360, wait=False)
        medium_motorR.run_angle(-1000, 360)

    if Button.RIGHT in ev3.buttons.pressed():
        medium_motorL.run_angle(1000, 360, wait=False)
        medium_motorR.run_angle(1000, 360)
    

    if Button.UP in ev3.buttons.pressed():
        robot.straight(-500)

    if Button.DOWN in ev3.buttons.pressed():
        robot.straight(500)
    

    if Button.CENTER in ev3.buttons.pressed():
        ev3.speaker.say("Hola my nombre es PiBot!")
        start_sound(3)
        for i in range(2):
            medium_motorL.run_angle(-2000, 140, wait=False)
            medium_motorR.run_angle(-2000, 140)


    if color_sensor.color() == Color.GREEN:
        ev3.screen.print("[ COL ]: Color Verde!")
        ev3.speaker.say("Verde!")

    if color_sensor.color() == Color.RED:
        ev3.screen.print("[ COL ]: Color Rojo!")
        ev3.speaker.say("Rojo!")        

    if color_sensor.color() == Color.BLUE:
        ev3.screen.print("[ COL ]: Color Azul!")
        ev3.speaker.say("Azul!")