#!/usr/bin/env python3

from ev3dev2.led import Leds
from ev3dev2.motor import LargeMotor, MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, MoveDifferential
from ev3dev2.sensor.lego import TouchSensor, ColorSensor, UltrasonicSensor, GyroSensor
from ev3dev2.wheel import EV3EducationSetTire

WHEEL_DISTANCE_MM = 118
MOTOR_SPEED_PERCENT = 25
MOTOR_SPEED_PERCENT_WHEN_TURNING = 15

leds = Leds()
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
gyro_sensor = GyroSensor()


def move_forward(distance_cm: float, speed=MOTOR_SPEED_PERCENT, brake=True):
    move_differential.on_for_distance(distance_mm=distance_cm * 10, speed=speed, brake=brake)


if __name__ == "__main__":
    move_forward(distance_cm=10, brake=False)
    crane_motor.on_for_degrees(speed=20, degrees=30)
    while True:
        if touch_sensor.is_pressed:
            move_forward(distance_cm=5, brake=False)
            break
