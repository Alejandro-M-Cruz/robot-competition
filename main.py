#!/usr/bin/env python3

from time import sleep

from ev3dev2.led import Leds
from ev3dev2.motor import LargeMotor, MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, MoveDifferential
from ev3dev2.sensor.lego import TouchSensor, ColorSensor, UltrasonicSensor, GyroSensor
from ev3dev2.wheel import EV3EducationSetTire
from ev3dev2.sound import Sound

WHEEL_DISTANCE_MM = 118
MOTOR_SPEED_PERCENT = 25
MOTOR_SPEED_PERCENT_WHEN_TURNING = 15
INITIAL_CM_FROM_CAN = 100
DESIRED_CM_FROM_CAN = 11.2
BRAKE_DEFAULT = True

leds = Leds()
speaker = Sound()
crane_motor = MediumMotor(OUTPUT_A)
move_differential = MoveDifferential(
    left_motor_port=OUTPUT_B,
    right_motor_port=OUTPUT_C,
    wheel_class=EV3EducationSetTire,
    wheel_distance_mm=WHEEL_DISTANCE_MM,
    motor_class=LargeMotor
)
touch_sensor = TouchSensor()
color_sensor = ColorSensor()
ultrasonic_sensor = UltrasonicSensor()
# move_differential.gyro = GyroSensor()


def move_forward(distance_cm: float, speed=MOTOR_SPEED_PERCENT, brake=BRAKE_DEFAULT, block=True):
    move_differential.on_for_distance(distance_mm=distance_cm * 10, speed=speed, brake=brake, block=block)


def turn_left(degrees: int, speed=MOTOR_SPEED_PERCENT_WHEN_TURNING, brake=BRAKE_DEFAULT, block=True):
    move_differential.turn_left(degrees=degrees, speed=speed, brake=brake, use_gyro=True, block=block)


def turn_right(degrees: int, speed=MOTOR_SPEED_PERCENT_WHEN_TURNING, brake=BRAKE_DEFAULT, block=True):
    turn_left(degrees=-degrees, speed=speed, brake=brake, block=block)


def stop(brake=BRAKE_DEFAULT):
    move_differential.off(brake=brake)


def floor_is_white():
    return color_sensor.color == ColorSensor.COLOR_WHITE


def set_leds_color(color: str):
    """Supported colors are BLACK, RED, GREEN, AMBER, ORANGE and YELLOW"""
    leds.set_color("LEFT", color)
    leds.set_color("RIGHT", color)


def turn_until_finding_can(left: bool):
    while ultrasonic_sensor.distance_centimeters > INITIAL_CM_FROM_CAN:
        if left:
            turn_left(degrees=60, brake=False, block=False)
        else:
            turn_right(degrees=120, brake=False, block=False)



if __name__ == "__main__":
    with open("initial_distance_from_can.txt", "w") as f:
        f.write(str(ultrasonic_sensor.distance_centimeters))
    speaker.beep()


    crane_motor.on_for_degrees(degrees=30, speed=5, brake=False, block=False)

    set_leds_color("RED")

    if ultrasonic_sensor.distance_centimeters > INITIAL_CM_FROM_CAN:
        turn_right(degrees=60, speed=15, brake=True, block=False)

    while ultrasonic_sensor.distance_centimeters > INITIAL_CM_FROM_CAN:
        pass

    if ultrasonic_sensor.distance_centimeters > INITIAL_CM_FROM_CAN:
        turn_left(degrees=120, speed=15, brake=True, block=False)
    else:
        stop(brake=False)

    while ultrasonic_sensor.distance_centimeters > INITIAL_CM_FROM_CAN:
        pass

    stop(brake=False)

    move_forward(distance_cm=INITIAL_CM_FROM_CAN + 50, speed=30, brake=False, block=False)

    while ultrasonic_sensor.distance_centimeters > DESIRED_CM_FROM_CAN:
        pass

    stop(brake=True)
    crane_motor.on_for_degrees(degrees=-60, speed=20, brake=False, block=False)

    while not touch_sensor.is_pressed:
        pass

    crane_motor.off(brake=False)
    set_leds_color("YELLOW")
    # speaker.play_file("coin_sound.wav", play_type=Sound.PLAY_NO_WAIT_FOR_COMPLETE, volume=100)
    speaker.beep()
    crane_motor.on_for_degrees(degrees=60, speed=30, brake=False, block=False)
    move_forward(distance_cm=-150, speed=100, brake=False, block=False)

    sleep(1)

    while not floor_is_white():
        pass

    stop(brake=True)
    set_leds_color("GREEN")
    # speaker.play_file("coin_sound.wav", volume=100)
    speaker.beep()