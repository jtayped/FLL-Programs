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


# Initializes the motors and sensors and prints them on the screen
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
color_sensor = ColorSensor(Port.S2)

########################################################################################################
########################################################################################################
########################################################################################################


class Drive_base:
    def stop(self):
        # Stops all the robot
        robot.stop()
        left_wheel.brake()
        right_wheel.brake()

    def complement(self):
        # Adjusts the complement medium motors so the complements fall in the gears perfectly
        speed = 300
        ms = 140
        medium_motorL.run_time(speed, ms, wait=False)
        medium_motorR.run_time(speed, ms)
        medium_motorL.run_time(-speed, ms, wait=False)
        medium_motorR.run_time(-speed, ms)
        ev3.screen.print("[ OK ]: Compliment Fitted!")

    def start_sound(self, beep_num, say_run=True):
        # Makes a beep with an incremental frequency
        freq = 100
        # If you don't wan't to print out the run nº it will simply not, just specify it in the say_run paramenter, if not it will be True
        if say_run == True:
            ev3.screen.print("[ RUN ]: Run " + str(beep_num) + "!")

        for i in range(beep_num):
            freq += 300
            ev3.speaker.beep(freq, 100)
            time.sleep(0.1)

    def gyro_calibrate(self, angle=0):
        # It will calibrate the gyro sensor, and if it takes more than 0.5s it will give an error
        # It calibrates the sensor by teaching it what is staying still aka speed==0, by changing it's mode till it's 0
        # As it's a while True loop, we were scared it would stick in it forever
        gyro_timer = time.time()
        while not gyro.speed() == 0:
            gyro.speed()
            gyro.angle()
            time.sleep(0.05)

            if (time.time()-gyro_timer) > 0.8:
                ev3.screen.print("[ ERR ]: Gyro Cal. Error!")
                for i in range(2):
                    ev3.speaker.beep(300, 200)
                break

        gyro.reset_angle(angle)
        ev3.screen.print("[ OK ]: Gyro Calibrated!")

bot = Drive_base()

########################################################################################################
########################################################################################################


class Moving:
    def straight(self, distance):
        # Prints on the ev3 screen the amount of distance specified in the parameters
        ev3.screen.print("[ F ]: Distance(" + str(distance) + ")")

        # Moves straight till at the specified distance accelerating and decelerating at the values specified in the settings
        robot.straight(-distance)

        # Stops the robot
        robot.stop()

    def right(self, target_angle, speed, turn_rate, time_limit=7):
        # Prints on the ev3 screen the current angle and the target_angle
        ev3.screen.print("[ MV ]: S(" + str(gyro.angle()) +
                         "º) T(" + str(target_angle) + "º)")

        # Calculates a limit time specified in the paramenters and if gone over that time (default time in parameters), it will go on with the next line of code
        turn_timer = time.time()
        # Turn to the target_angle at the speed and turn_rate specified in the paramenters
        while gyro.angle() < target_angle:
            if (time.time()-turn_timer) > time_limit:
                ev3.screen.print("[ ERR ]: Time Limit Exceeded!")
                for i in range(2):
                    ev3.speaker.beep(300, 200)
                break
            robot.drive(-speed, -turn_rate)

        # Stops the robot
        robot.stop()

    def left(self, target_angle, speed, turn_rate, time_limit=7):
        # Prints on the ev3 screen the current angle and the target_angle
        ev3.screen.print("[ MV ]: S(" + str(gyro.angle()) +
                         "º) T(" + str(target_angle) + "º)")

        # Calculates a limit time specified in the paramenters and if gone over that time (default time in parameters), it will go on with the next line of code
        turn_timer = time.time()
        # Turn to the target_angle at the speed and turn_rate specified in the paramenters
        while gyro.angle() > target_angle:
            if (time.time()-turn_timer) > time_limit:
                ev3.screen.print("[ ERR ]: Time Limit Exceeded!")
                for i in range(2):
                    ev3.speaker.beep(300, 200)
                break
            robot.drive(-speed, turn_rate)

        # Stops the robot
        robot.stop()

    def line_squaring(self, speed):

        left_wheel.reset_angle(0)
        right_wheel.reset_angle(0)
        while not light_left.reflection() <= 10:
            robot.drive(-speed, 0)
        bot.stop()

        # Left motor goes straight till white line
        while not light_right.reflection() <= 10:
            right_wheel.run(-70)
        bot.stop()

        left_wheel.reset_angle(0)
        while not light_left.reflection() >= 90:
            left_wheel.run(30)
            wait(200)
        bot.stop()

        distance_left = left_wheel.angle() /2
        left_wheel.reset_angle(0)
        while not left_wheel.angle() <= distance_left:
            left_wheel.run(-30)
        bot.stop()

        while not light_right.reflection() >= 90:
            right_wheel.run(30)
        bot.stop()
        right_wheel.reset_angle(0)

        # Right motor goes backwards till other white line 
        while not light_right.reflection() >= 90:
            right_wheel.run(30)
            wait(200)
        bot.stop()
        distance_right = right_wheel.angle() /2
        right_wheel.reset_angle(0)
        while not right_wheel.angle() <= distance_right:
            right_wheel.run(-30)
        bot.stop()
        left_wheel.reset_angle(0)
        right_wheel.reset_angle(0)

    def exponential_turn(self, target_angle, time_limit=5):
        # Starts a timer
        start_time = time.time()
        while gyro.angle() != target_angle:
            # If the robot glitches (common) it will compare it to the time limit
            # If it's over the time limit, it will break the loop and continue
            if (time.time()-start_time) > time_limit:
                break
            # The correction variable is in charge of varying the turn_rate according to the robot angle
            # The target_angle has 3 degrees more because like that it will do the turn faster
            correction = ((target_angle+3) - gyro.angle()) *3
            robot.drive(0, correction)

    
movement = Moving()

########################################################################################################
########################################################################################################


class Runs:
    def run1(self):
        # Before run preparations
        bot.stop()
        robot.settings(straight_speed=-700, straight_acceleration=300)
        bot.start_sound(1)
        bot.complement()
        bot.gyro_calibrate()
        gyro.reset_angle(0)

        # Goes straight till next to the back of the plane
        movement.straight(630)

        # Moves forward and right to leave the cargo in the circle
        movement.right(35, 160, 35)

        # Moves backward and adjusts to have the perfect angle
        movement.straight(-30)
        movement.right(40, 0, 35)
        movement.straight(-265)

        # Moves the stick down to align with the cargo lever
        medium_motorL.run_target(300, 130)
        movement.straight(100)

        # Moves the lever down
        medium_motorL.run_time(600, 1100)

        # Goes back to base
        movement.straight(-800)
        bot.stop()

    ########################################################################################################

    def run2(self):
        # Before run preparations
        bot.stop()
        robot.settings(straight_speed=-700, straight_acceleration=575)
        bot.start_sound(2)
        bot.complement()
        bot.gyro_calibrate()
        gyro.reset_angle(0)
        # Adjusts the motor to fold in
        medium_motorL.run_until_stalled(4000, duty_limit=50)

        # First straight
        movement.straight(200)

        # Turns to get to the truck
        movement.right(35, 0, 80)
        movement.straight(320)
        movement.right(83, 0, 80)

        # Pushes the truck in to the bridge
        movement.straight(190)
        movement.straight(-190)

        # Turns to get to 90º to bring down the bridges
        movement.left(60, 0, 50)

        # Uses the color sensor to have the perfect distance
        while light_right.reflection() > 30:
            robot.drive(-100, 0)
        bot.stop()
        movement.straight(240)

        # Turns to 90º for the straight, has big multiplying value due to the accuracy
        movement.right(90, 0, 80)

        # Big straight till train tracks, then goes back a bit to adjust for the helicopter
        robot.reset()
        while robot.distance() >= -980:
            correction = (91 - gyro.angle()) * 10
            robot.drive(-400, -correction)
        bot.stop()
        robot.reset()

        # Moves back a bit
        while robot.distance() <= 40:
            correction = (90 - gyro.angle()) * 10
            robot.drive(200, -correction)
        bot.stop()
        robot.reset()

        # Turns to 180º to turn back in to the helicopter
        movement.right(175, 0, 80)

        # Comes in to do the helicopter
        movement.straight(-520)
        gyro.reset_angle(0)

        # Change in settings to go slower for more accuracy
        robot.settings(straight_speed=-300, straight_acceleration=300)
        movement.straight(245)
        robot.settings(straight_speed=-700, straight_acceleration=575)

        # Turns in to the helicopter, the turn_rate has to be negative because it would bump in to the lever
        movement.left(-81, -40, 100)
        movement.straight(120)

        # Goes back
        movement.straight(-70)
        movement.right(0, 0, 80)

        # Straightens with the wall and
        movement.straight(-300)
        gyro.reset_angle(0)

        # Goes for the train track repair
        movement.straight(600)

        # Turns in to the train tracks
        movement.left(-70, -35, 100)

        movement.straight(130)

        # Puts down the train track and retracts the arm
        medium_motorR.run_time(-700, 800)
        medium_motorR.run_time(700, 800)

        # Goes back and moves till the robot is behind the truck
        movement.straight(-20)
        # (We put 5 degrees so we save time in adjustments)
        movement.right(5, -10, 65)
        movement.straight(-410)

        # Folds the stick out to move the train
        medium_motorL.run_angle(-1000, 230, wait=False)
        time.sleep(0.5)

        # Moves straight till the train is locked in
        robot.reset()
        while robot.distance() >= -405:
            correction = (0 - gyro.angle()) * 6
            robot.drive(-300, -correction)
        bot.stop()

        # Goes back, retracts the stick
        movement.straight(-300)
        medium_motorL.run_until_stalled(4000, duty_limit=50)

        # Turns till the desired angle of 35 degrees to go back to base and goes straight
        movement.right(35, 0, 50)
        movement.straight(675)

        # Turns to 90 degrees to aim back at base
        movement.right(90, 0, 80)

        # Returns back to base
        robot.reset()
        while robot.distance() >= -1600:
            if Button.RIGHT in ev3.buttons.pressed():
                break
            correction = (90 - gyro.angle()) * 10
            robot.drive(-700, -correction)
        bot.stop()
        second_run = True

    ########################################################################################################

    def run3(self):
        # Before run preparations
        bot.stop()
        robot.settings(straight_speed=-400, straight_acceleration=200)
        bot.start_sound(3)
        bot.complement()
        bot.gyro_calibrate()
        gyro.reset_angle(0)
        # (Makes sure the cargo won't fall out in the middle of the mission)
        medium_motorR.run_until_stalled(4000, duty_limit=30)

        # First straight
        movement.straight(390)

        # Turns to aim at the crane and goes there
        movement.right(60, 0, 30)
        movement.straight(675)

        # For precision purposes we use the color sensor to find the black line
        while light_right.reflection() > 30:
            robot.drive(-100, 0)
        bot.stop()

        # Moves straight the rest of the distance
        movement.straight(39)

        movement.right(87, 0, 40)

        # Moves the crane to the desired place
        robot.reset()
        while robot.distance() >= -430:
            correction = (90 - gyro.angle()) * 15
            robot.drive(-100, -correction)
        bot.stop()

        # Goes back and starts to go to leave the blue cargo box
        movement.straight(-175)
        movement.right(110, 0, 20)
        movement.straight(300)
        movement.left(90, 0, 60)

        # Smacks in to the train tracks to have the correct distance inbetween the crane and the stick
        movement.straight(265)
        movement.straight(-13)

        # Rotates to look at the blue circle, due to the weight of the complement angles are adjusted for error
        movement.left(-1, 0, 60)
        movement.straight(130)

        # Leaves the blue cargo box in circle, we add a sleep moment so the flap has time to open
        medium_motorL.run_time(1000, 2000, wait=False)
        time.sleep(0.5)

        # Goes back and starts to aim for the next mission
        movement.straight(-90)
        movement.left(90, -30, 60)

        # Initializes the travel to the last mission, and in the process leaves a gray cargo missiona and the innovation prodject
        robot.reset()
        while robot.distance() <= 280:
            correction = (90 - gyro.angle()) * 5
            robot.drive(110, -correction)
        bot.stop()
        # (In the middle of the journey it leaves the cargo missions)
        medium_motorR.run_angle(-500, 120)
        robot.reset()
        while robot.distance() <= 360:
            correction = (100 - gyro.angle()) * 2.5
            robot.drive(200, -correction)
        bot.stop()

        # Turns to aim at the last mission
        movement.left(-25, 0, 60)

        # Changes the settings to get a lower speed and aims at the last mission
        robot.settings(straight_speed=-100, straight_acceleration=50)
        movement.straight(110)

        # Calculates the final_turn_time of the turn and if it's over 4 seconds (or getting stuck) it will play the FANFARE sound
        final_turn_time = time.time()
        while gyro.angle() > -68:
            if (time.time() - final_turn_time) > 4:
                ev3.speaker.print(gyro.angle())
                ev3.speaker.play_file(SoundFile.FANFARE)
                break
            robot.drive(-100, 2)
        bot.stop()

        second_run = False
        third_run = True


run = Runs()

########################################################################################################
########################################################################################################

second_run = False
third_run = False
freq = 100

# Prints "Ready to go!" on the EV3 screen
ev3.screen.print("[ OK ]: Ready to go!")
bot.start_sound(4, say_run=False)

# While loop for the runs!! Note: Can't "ANDs" or "ORs" because the firmware of the robot does not work with them, that's why there is so many IFs together
# There is also a switch (ex: if second_run == False:), to prevent the same run to execute straight after the previous one
while True:

    # First complement doesn't require a color so we dedicated a button to it
    if Button.LEFT in ev3.buttons.pressed():
        # Makes a timer to calculate the final time at the end
        starting_time = time.time()

        # Executes the first run
        run.run1()

    # If it detects the color blue it will go to the next if (if second_run == False:), and if it hasn't done it yet it will go to the next (the button)
    if color_sensor.color() == Color.BLUE:
        if second_run == False:
            if Button.CENTER in ev3.buttons.pressed():

                # Executes the second run
                run.run2()

    # Same with this one
    if color_sensor.color() == Color.RED:
        if third_run == False:
            if Button.CENTER in ev3.buttons.pressed():
                # Executes the third run
                run.run3()

                # It will try to calculate the final time, but if we were not doing full runs it won't give an error
                try:
                    # Calculates the final_time and prints it on the screen
                    final_time = time.time()-starting_time
                    ev3.screen.print("[ OK ]: Final time: " + str(final_time))

                    # If it's under or over 2:30 it will do a different beep
                    if final_time > 150:
                        for i in range(2):
                            ev3.speaker.beep(500, 700)

                    if final_time < 150:
                        ev3.speaker.beep(300, 2000)

                # If it gave error it will simply pass
                except NameError:
                    pass

    # Program just in case the complement doesn't go in right
    if Button.RIGHT in ev3.buttons.pressed():
        bot.complement()