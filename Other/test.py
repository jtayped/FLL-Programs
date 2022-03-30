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

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information. 


# Create your objects here.
font_of = Font(family="Terminal", size=1)
ev3 = EV3Brick()
ev3.screen.set_font(font_of)
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


class Drive_base:
    def stop(self):
        robot.stop()
        left_wheel.brake() 
        right_wheel.brake()

    def complement(self):
        speed = 100
        ms = 100
        medium_motorL.run_time(speed, ms, wait= False)
        medium_motorL.run_time(-speed, ms, wait= False)
        medium_motorR.run_time(speed, ms, wait= False)
        medium_motorR.run_time(-speed, ms)
        ev3.screen.print("[ OK ]: Compliment Fitted!")

    def start_sound(self, beep_num):
        freq = 100
        ev3.screen.print("[ RUN ]: Run " + str(beep_num) + "!")
        for i in range(beep_num):
            freq += 300
            ev3.speaker.beep(freq, 100)
            time.sleep(0.1)

    def gyro_calibrate(self, angle=0):
        while not gyro.speed() == 0:
            gyro.speed()
            gyro.angle()
            time.sleep(0.05)
        gyro.reset_angle(angle)
        ev3.screen.print("[ OK ]: Gyro Calibrated!")
bot = Drive_base()

class Moving:
    def straight(self, distance):
        robot.straight(-distance)
        robot.stop()

    def right(self, target_angle, speed, turn_rate):
        ev3.screen.print("[ MV ]: S(" + str(gyro.angle()) + ") T(" + str(target_angle) + ")")

        while gyro.angle() < target_angle:
            robot.drive(-speed, -turn_rate)
        robot.stop()

    def left(self, target_angle, speed, turn_rate):
        ev3.screen.print("[ MV ]: S(" + str(gyro.angle()) + ") T(" + str(target_angle) + ")")

        while gyro.angle() > target_angle:
            robot.drive(-speed, turn_rate)
        robot.stop()
    
    def line_squaring(self, speed):
        left_wheel.reset_angle(0)
        right_wheel.reset_angle(0)
        while not light_left.reflection() <= 10:
            robot.drive(speed, 0)
        stop_robot()

        # Left motor goes straight till white line
        while not light_right.reflection() <= 10:
            right_wheel.run(-70)
        stop_robot()

        left_wheel.reset_angle(0)
        while not light_left.reflection() >= 90:
            left_wheel.run(30)
            wait(200)
        stop_robot()
        distance_left = left_wheel.angle() /2
        left_wheel.reset_angle(0)
        while not left_wheel.angle() <= distance_left:
            left_wheel.run(-30)
        stop_robot()

        while not light_right.reflection() >= 90:
            right_wheel.run(30)
        stop_robot()
        right_wheel.reset_angle(0)

        # Right motor goes backwards till other white line 
        while not light_right.reflection() >= 90:
            right_wheel.run(30)
            wait(200)
        stop_robot()
        distance_right = right_wheel.angle() /2
        right_wheel.reset_angle(0)
        while not right_wheel.angle() <= distance_right:
            right_wheel.run(-30)
        stop_robot()
        left_wheel.reset_angle(0)
        right_wheel.reset_angle(0)
movement = Moving()

bot.start_sound(3)

bot.gyro_calibrate()

movement.straight(-300)

movement.right(40, 0, 60)
        
movement.straight(620)

