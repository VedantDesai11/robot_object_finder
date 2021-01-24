#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.media.ev3dev import SoundFile, ImageFile
import time
from time import sleep
from math import pi



def reset_angles(left_motor, right_motor):
    # reset motor angles to zero

    left_motor.reset_angle(0)
    right_motor.reset_angle(0)


def turn(angle):

    reset_angles(left_motor, right_motor)

    # calculate amount of rotation needed by wheels to cover distance to rotate given angle
    distance = ROBOT_WIDTH * angle
    degree_rotation = distance/WHEEL_DIAMETER

    # run motor till it reaches target rotation 
    left_motor.run_target(SPEED, degree_rotation, wait=False)
    right_motor.run_target(SPEED, -degree_rotation, wait=True)

    # reset angles to zero for other rotations
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)


def drive(distance, speed = 180):

    reset_angles(left_motor, right_motor)

    if distance == 0:
        left_motor.run(speed)
        right_motor.run(speed)
    else:
        # calculate amount of rotation needed by wheel to cover distance
        degree_rotation = round((360*distance)/CIRCUMFERENCE_WHEEL)

        # run motor till it reaches target rotation 
        left_motor.run_target(speed, degree_rotation, wait=False)
        right_motor.run_target(speed, degree_rotation, then=Stop.HOLD, wait=True)

        # reset angles to zero for other rotations
        reset_angles(left_motor, right_motor)



def find_Can(minimum_radius):

    print('Looking for Can')

    start_angle = 0
    end_angle = 0
    can_distance = []

    reset_angles(left_motor, right_motor)
    while True:

        # begin while loop to turn 170degrees looking for the can

        # run motors to turn in clockwise direction
        left_motor.run(SPEED)
        right_motor.run(-SPEED)

        # calculate distance measured by US sensor
        distance = ultrasonic.distance()

        # check if can is found within minimum distance
        if distance < minimum_radius:

            # append to list containing all distances calcuated of can across all angles
            can_distance.append(distance)

            # if can is found for the first time, make this as start angle
            if start_angle == 0:

                # formula to calculate rotation angle according to turns of motor
                start_angle = int(left_motor.angle() * ANGLE_CONVERSION)
                print('Found the can')
        else:
            if start_angle != 0:
                
                # check when US sensor stops seeing the can, meaning we got our end angle
                left_motor.brake()
                right_motor.brake()

                print('Calculating angle to can')

                # formula to calculate rotation angle according to turns of motor
                end_angle = int(left_motor.angle() * ANGLE_CONVERSION)
                reset_angles(left_motor, right_motor)

                # calculate angle at which can is inside the start and end angles
                can_location_angle = int((end_angle - start_angle)/2)

                # turn the robot by that angle times 1.5 because of the placement of transmitter not being exactly in the center
                turn(-can_location_angle*(3/2))

                print('Heading towards the can')

                # drive towards the can
                while ultrasonic.distance() > 200:
                    
                    # check to make sure there is no wall in the way
                    if colorsensor.color() == WALL_COLOR:
                        left_motor.brake()
                        right_motor.brake()
                        reset_angles(left_motor, right_motor)
                        print('Found wall in the way. Following wall.')
                        return False

                    left_motor.run(SPEED)
                    right_motor.run(SPEED)

                # stop when robot is 200mm from the can to person 2nd angle check
                left_motor.brake()
                right_motor.brake()
                print('Final angle check')
                reset_angles(left_motor, right_motor)

                # turn by 45 degrees away from the can to calculate start and end angles again
                turn(-45)
                start_angle = 0
                end_angle = 0

                # performing 2nd check
                while True:

                    # rotating the robot at half speed for more precise measurments
                    left_motor.run(SPEED//2)
                    right_motor.run(-SPEED//2)

                    # calculate distances again
                    distance = ultrasonic.distance()

                    # find start and end angles again
                    if distance < 250:
                        if start_angle == 0:
                            start_angle = int(left_motor.angle() * ANGLE_CONVERSION)
                    else:
                        if start_angle != 0:
                            end_angle = int(left_motor.angle() * ANGLE_CONVERSION)
                            left_motor.brake()
                            right_motor.brake()
                            reset_angles(left_motor, right_motor)
                            break

                    # check for a wall if its in the way
                    if colorsensor.color() == WALL_COLOR:
                        left_motor.brake()
                        right_motor.brake()
                        reset_angles(left_motor, right_motor)
                        print('Found wall in the way. Following wall.')
                        return False

                     # if we cant find the can, meaning we were wrong earlier so reset by driving to a wall and return
                    if left_motor.angle() > SEARCH_ROTATIONS and start_angle != 0:
                        print("Lost can, heading back to a wall")
                        find_Wall()
                        return False

                # once can is found, calcualte precise angles again
                can_location_angle = int((end_angle - start_angle)/2)

                # turn to that calculated angle, should be exactly towards the can
                turn(-can_location_angle*(3/2))

                # drive towards the can
                while True:
                    left_motor.run(SPEED)
                    right_motor.run(SPEED)

                    # checking for a wall in the way, just in case
                    if colorsensor.color() == WALL_COLOR:
                        left_motor.brake()
                        right_motor.brake()
                        reset_angles(left_motor, right_motor)
                        print('Found wall in the way. Following wall.')
                        return False

                    # once we are 200mm close, stop the robot
                    if ultrasonic.distance() > 200:
                        break
                    
                # sound the alarm as mentioned in the question
                ev3.speaker.beep()

                # drive 300mm straight into the can, pushing it 100mm away from position
                drive(300, 500)
                
                return True
                
        # if the can is not found, we stop the search and return to original position
        if left_motor.angle() > SEARCH_ROTATIONS:
            print('Stopping search')
            left_motor.brake()
            right_motor.brake()
            break
            
    # rotating back to original position meaning back to the wall to continue wall following
    while colorsensor.color() != WALL_COLOR:
        left_motor.run(-SPEED)
        right_motor.run(SPEED)

    left_motor.brake()
    right_motor.brake()
    reset_angles(left_motor, right_motor)
    print('Found wall in the way. Following wall.')
    return False


def find_Wall():
    print('Looking for wall')

    # drive the robot straight till color sensor finds a wall
    while colorsensor.color() != WALL_COLOR:
        left_motor.run(SPEED)
        right_motor.run(SPEED)

    left_motor.brake()
    right_motor.brake()

    print('Found wall')
    print('Aligning to wall')
    
    # turn the robot to the right so it is aligned to the wall so it can do wall following
    while colorsensor.color() == WALL_COLOR:
        left_motor.stop()
        right_motor.run(-SPEED)

    left_motor.brake()
    right_motor.brake()

    print('Aligned to wall')


def wall_following():

    # initialize times for can checks, left turn time, right turn time and for can checking time.
    can_check = time.time()
    leftTime = time.time()
    rightTime = time.time()
    can_checks = 0


    while True:
        # while loop to keep the robot following the wall

        # check if 20 seconds have passed since last can check
        if time.time() - can_check > 20:
            can_checks += 1
            left_motor.brake()
            right_motor.brake()

            # if 5 can checks are performed, turn 90 degrees and look for another wall (wander)
            if can_checks == 5:
                turn(90)
                find_Wall()

            # performing can check with minimum radius for distance check at 600mm
            else:
                
                # if find_can returns true, means we found and hit the can so we can exit the while loop and end the function
                if find_Can(600):
                    break
        
            # if can isn't found, we reinitialize the times 
            can_check = time.time()
            leftTime = time.time()
            rightTime = time.time()

        # while loop to check if we are on the wall, then we perform a right arc turn
        while colorsensor.color() == WALL_COLOR:

            # if we are turning right for 0.1 second, means we need to make a 90deg right turn
            if time.time() - leftTime > 0.1:
                print('Turning Right')

                # while loop to turn the robot 90 degrees right till it finds the white floor again
                while colorsensor.color() == WALL_COLOR:

                    left_motor.run(SPEED)
                    right_motor.run(-SPEED)

                    # special check if we keep turning right forever without finding wall
                    if time.time() - leftTime > 15:
                        print("Lost wall, finding it again")
                        find_Wall()
                        break

                leftTime = time.time()
                break

            # performing the right arc turn
            rightTime = time.time()
            left_motor.run(SPEED+20)
            right_motor.run(SPEED-15)
            
        # while loop to check if we are not on the wall, then we perform a left arc turn
        while colorsensor.color() != WALL_COLOR:

             # if we are turning left for 1.5 seconds, means we need to make a 90deg left turn
            if time.time() - rightTime > 1.5:
                print('Turning Left')

                # performing the left turn/ u-turn
                while colorsensor.color() != WALL_COLOR:
                    right_motor.run(SPEED+70)
                    left_motor.run(35)

                    if time.time() - rightTime > 15:
                        print("Lost wall, finding it again")
                        find_Wall()
                        break

                rightTime = time.time()
                break

            # performing the left arc turn
            leftTime = time.time()
            right_motor.run(SPEED+20)
            left_motor.run(SPEED-15)


if __name__ == "__main__":

    WHEEL_DIAMETER = 56 #mm
    CIRCUMFERENCE_WHEEL = pi * WHEEL_DIAMETER # 3.14 * 56
    SPEED = 100 # degrees/second
    ROBOT_WIDTH = 119#mm
    WALL_COLOR = Color.BLUE
    TARGET_COLOR = Color.RED
    SEARCH_ANGLE = 170
    SEARCH_ROTATIONS = ROBOT_WIDTH * SEARCH_ANGLE/WHEEL_DIAMETER
    ANGLE_CONVERSION = SEARCH_ANGLE/SEARCH_ROTATIONS
    
    # Create your objects here.
    ev3 = EV3Brick()

    # Connect two motors on output ports A and D
    left_motor = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE, gears=None)
    right_motor = Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE, gears=None)

    # Connect US sensor and color sensor
    ultrasonic = UltrasonicSensor(Port.S4)
    colorsensor = ColorSensor(Port.S3)

    #find_Can(600)

    find_Wall()
    wall_following()



    

    

    

    




    

    


        
