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
MAX_OBJECT_DISTANCE_CM = 100
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
move_differential.gyro = GyroSensor()


def move_forward(distance_cm: int, speed: int, *, brake: bool, block: bool):
    move_differential.on_for_distance(speed=speed, distance_mm=distance_cm * 10, brake=brake, block=block)


def set_leds_color(color: str):
    """Supported colors are BLACK, RED, GREEN, AMBER, ORANGE and YELLOW"""
    leds.set_color("LEFT", color)
    leds.set_color("RIGHT", color)


def can_is_in_front():
    return ultrasonic_sensor.distance_centimeters <= MAX_OBJECT_DISTANCE_CM


def turn_until_can_is_in_front():
    if can_is_in_front():
        return
    else:
        move_differential.turn_right(degrees=45, speed=15, brake=True, block=False)

    while not can_is_in_front() and move_differential.is_running:
        pass

    if not can_is_in_front():
        move_differential.turn_left(degrees=90, speed=15, brake=True, block=False)
    else:
        move_differential.off(brake=False)

    while not can_is_in_front() and move_differential.is_running:
        pass

    move_differential.off(brake=False)


def touch_can():
    crane_motor.on_for_degrees(degrees=-60, speed=20, brake=False, block=False)
    times_attempted = 1

    while not touch_sensor.is_pressed:
        if not crane_motor.is_running:
            for _ in range(10):
                sleep(0.1)
                if touch_sensor.is_pressed:
                    on_can_touched()
                    return
            sleep(9)

            crane_motor.on_to_position(position=0, speed=30, brake=False, block=True)

            if not can_is_in_front():
                turn_until_can_is_in_front()
            else:
                if ultrasonic_sensor.distance_centimeters < DESIRED_CM_FROM_CAN - 5:
                    move_forward(distance_cm=-5, speed=15, brake=True, block=True)
                elif ultrasonic_sensor.distance_centimeters > DESIRED_CM_FROM_CAN + 5:
                    move_forward(distance_cm=5, speed=15, brake=True, block=True)
                if times_attempted < 3:
                    move_differential.turn_right(degrees=5, speed=15, brake=True, block=True)
                else:
                    move_differential.turn_left(degrees=5, speed=15, brake=True, block=True)

            crane_motor.on_for_degrees(degrees=-60, speed=20, brake=False, block=False)
            times_attempted += 1

    on_can_touched()


def on_can_touched():
    set_leds_color("YELLOW")
    speaker.beep(play_type=Sound.PLAY_NO_WAIT_FOR_COMPLETE)
    crane_motor.on_to_position(position=0, speed=30, brake=False, block=False)


if __name__ == "__main__":
    with open("initial_distance_from_can.txt", "w") as f:
        f.write(str(ultrasonic_sensor.distance_centimeters))

    crane_motor.on_to_position(position=0, speed=5, brake=False, block=True)

    set_leds_color("RED")
    speaker.set_volume(100)
    speaker.beep()

    turn_until_can_is_in_front()

    move_forward(distance_cm=MAX_OBJECT_DISTANCE_CM + 50, speed=30, brake=False, block=False)

    while ultrasonic_sensor.distance_centimeters > DESIRED_CM_FROM_CAN:
        if not can_is_in_front():
            turn_until_can_is_in_front()

    move_differential.off(brake=True)

    touch_can()

    move_forward(distance_cm=-150, speed=100, brake=False, block=False)

    sleep(1)

    while color_sensor.color != ColorSensor.COLOR_WHITE:
        pass

    move_differential.off(brake=True)
    set_leds_color("GREEN")
    # speaker.play_file("coin_sound.wav", volume=100)
    speaker.beep()
    speaker.speak("Facilito")
