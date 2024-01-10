# robot-competition

Para nuestro proyecto final de Sistemas Robóticos Autónomos, hemos elegido la competición. En las pruebas realizadas el miércoles 10/01/2024, logramos que el robot cumpliera su objetivo, siendo este el código final, contenido en el fichero `main.py`:
```python
#!/usr/bin/env python3

import time

from ev3dev2.led import Leds
from ev3dev2.motor import LargeMotor, MediumMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, MoveDifferential
from ev3dev2.sensor.lego import TouchSensor, ColorSensor, UltrasonicSensor, GyroSensor
from ev3dev2.wheel import EV3EducationSetTire
from ev3dev2.sound import Sound

WHEEL_DISTANCE_MM = 118
MAX_OBJECT_DISTANCE_CM = 100
DESIRED_CM_FROM_CAN = 11.2

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


def turn_until_can_is_in_front(turn_speed: float, first_turn_right: bool = True):
    if first_turn_right:
        move_differential.turn_right(degrees=45, speed=turn_speed, brake=True, block=False)
    else:
        move_differential.turn_left(degrees=45, speed=turn_speed, brake=True, block=False)

    while not can_is_in_front() and move_differential.is_running:
        pass

    if not can_is_in_front():
        if first_turn_right:
            move_differential.turn_left(degrees=90, speed=turn_speed, brake=True, block=False)
        else:
            move_differential.turn_right(degrees=90, speed=turn_speed, brake=True, block=False)
        while not can_is_in_front() and move_differential.is_running:
            pass
        return "left" if first_turn_right else "right"
    else:
        return "right" if first_turn_right else "left"


def touch_can(max_attempts=5):
    for _ in range(max_attempts):
        crane_motor.on_for_degrees(degrees=-60, speed=20, brake=False, block=False)
        start = time.perf_counter()
        while time.perf_counter() - start < 3:
            if touch_sensor.is_pressed:
                on_can_touched()
                return
        time.sleep(10)
        crane_motor.on_to_position(position=0, speed=30, brake=False, block=True)
        if not can_is_in_front():
            turn_until_can_is_in_front(turn_speed=2)


def on_can_touched():
    set_leds_color("YELLOW")
    speaker.beep(play_type=Sound.PLAY_NO_WAIT_FOR_COMPLETE)
    crane_motor.on_to_position(position=0, speed=30, brake=False, block=False)


if __name__ == "__main__":
    crane_motor.on_to_position(position=0, speed=5, brake=False, block=True)

    set_leds_color("RED")
    speaker.set_volume(100)
    speaker.beep()

    if can_is_in_front():
        move_forward(distance_cm=MAX_OBJECT_DISTANCE_CM + 50, speed=30, brake=False, block=False)

    can_side = "unknown"
    while ultrasonic_sensor.distance_centimeters > DESIRED_CM_FROM_CAN:
        if not can_is_in_front():
            move_differential.off(brake=True)
            can_side = turn_until_can_is_in_front(turn_speed=5,first_turn_right=can_side == "right")
            move_differential.off(brake=True)
        else:
            move_forward(distance_cm=MAX_OBJECT_DISTANCE_CM + 50, speed=30, brake=False, block=False)
    move_differential.off(brake=True)
    touch_can()
    move_forward(distance_cm=-150, speed=100, brake=False, block=False)
    time.sleep(1)

    while color_sensor.color != ColorSensor.COLOR_WHITE:
        pass

    move_differential.off(brake=True)
    set_leds_color("GREEN")
    speaker.beep()
    speaker.speak("Facilito")

```
En primer lugar, el robot hace subir el brazo, en caso de que no se encuentre ya arriba. Seguidamente, cambia el color de sus luces LED a rojo y emite un pitido. Este es el momento en que el robot comienza a moverse. 
En caso de detectar algún objeto a menos de 1 metro, la lata en este caso, se mueve en línea recta hacia el mismo. Si no, rota sobre sí mismo, primero hacia la izquierda y luego hacia la derecha, hasta encontrarlo.

Una vez detecta el objeto, comienza a moverse hacia él. Si en algún momento, durante la trayectoria en línea recta, el robot deja de detectar el objeto, este para, rota hasta volver a tenerlo a la vista, y continúa acercándose. Este proceso 
se repite tantas veces como sea necesario.

En el momento en que el robot se encuentra a menos de 11,2 centímetros del objeto, baja el brazo con el sensor de contacto en la punta, hasta que dicho sensor se encuentre presionado, momento
en el cual los LEDs cambian a amarillo y se reproduce un segundo pitido. En caso de no haber acertado al objeto, espera 10 segundos y vuelve a 
intentarlo, rotando previamente hasta encontrar el objeto si fuera necesario.

Por último, una vez el sensor de contacto ha sido presionado, el robot exprime al máximo la potencia de sus servomotores para alejarse de la lata marcha atrás lo más rápido posible. Al salir del círculo exterior, 
detectado gracias a la cinta blanca y al sensor de color, cambia las luces LED a verde, hace sonar un último pitido y articula una chulesca celebración.
