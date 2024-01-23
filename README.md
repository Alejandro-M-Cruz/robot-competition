# robot-competition

Para nuestro trabajo final de Sistemas Robóticos Autónomos, los integrantes del grupo 10 hemos decidido participar en la competición. He aquí una imagen de los sujetos en cuestión:

![foto de grupo](https://github.com/Alejandro-M-Cruz/robot-competition/assets/113340373/57a0b6ac-7aa3-47a3-a2ed-174c112c8ec4)

<br>

## Montaje del robot 
El primer paso fue el ensamblado de todos los sensores y actuadores necesarios para el robot, al que hemos decidido bautizar como "El Bicho". Además de los dos servomotores acoplados a las ruedas, ya utilizados en las prácticas anteriores, se incorporó un 
nuevo motor para el brazo mecánico, que a su vez tiene un sensor de contacto en su extremo. 

Asimismo, se añadió un sensor ultrasónico en la parte frontal, que permite detectar objetos delante del robot, un giroscopio 
para mejorar la precisión de las rotaciones y un sensor de color que apunta hacia el suelo, con el fin de determinar el momento en que el robot cruza la circunferencia exterior. 

Aunque ya vienen incorporados y no requieren montaje, cabe mencionar también el uso de las luces LED y el altavoz. 

Se puede observar a continuación el estado final del robot:

![robot](https://github.com/Alejandro-M-Cruz/robot-competition/assets/113340373/bfce5975-4405-4b42-8227-b28ff56547ee)

<br>

## Código 
En las pruebas realizadas el miércoles 10/01/2024, logramos que el robot cumpliera su objetivo, siendo este el código final, contenido en el fichero `main.py`:
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
            turn_until_can_is_in_front(turn_speed=5)


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
            can_side = turn_until_can_is_in_front(turn_speed=5, first_turn_right=can_side == "right")
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
    speaker.play_file("celebration.wav")

```
En primer lugar, el robot hace subir el brazo, en caso de que no se encuentre ya arriba. Seguidamente, cambia el color de sus luces LED a rojo y emite un pitido. Este es el momento en que el robot comienza a moverse. 
En caso de detectar algún objeto a menos de 1 metro, la lata en este caso, se mueve en línea recta hacia el mismo. Si no, rota sobre sí mismo, primero hacia la izquierda y luego hacia la derecha, hasta encontrarlo.

Una vez detecta el objeto, comienza a moverse hacia él. Si en algún momento, durante la trayectoria en línea recta, el robot deja de detectar el objeto, este se detiene, rota hasta volver a tenerlo a la vista, y continúa acercándose. El movimiento rotatorio se efectúa en el sentido en el cual se estaba rotando la última vez que se encontró la lata. De esta manera, el robot vuelve a encontrarla más rápidamente que si, por ejemplo, girara siempre primero a la izquierda y luego a la derecha, sin importar a qué lado se encuentre la lata al comienzo del recorrido. Este proceso se repite tantas veces como sea necesario hasta hallarse a una distancia adecuada.

En el momento en que el robot se encuentra a menos de 11,2 centímetros del objeto, baja el brazo con el sensor de contacto en la punta, hasta que dicho sensor se encuentre presionado, momento
en el cual los LEDs cambian a amarillo, se reproduce un segundo pitido y el brazo vuelve a su posición inicial. En caso de no haber acertado al objetivo, espera 10 segundos y vuelve a 
intentarlo, rotando previamente hasta encontrar de nuevo la lata si fuera necesario.

Por último, una vez que ha tocado la lata, el robot exprime al máximo la potencia de sus servomotores para alejarse del objeto marcha atrás lo más rápido posible. Al salir del círculo exterior, 
detectado gracias a la cinta blanca y al sensor de color, cambia las luces LED a verde, hace sonar un último pitido y articula una efusiva celebración.

<br>

## Proceso de realización
La primera versión funcional, desarrollada a partir del script de la primera práctica (véase [aquí](https://github.com/Alejandro-M-Cruz/robot-sra/blob/main/square/main.py)), fue la siguiente:
```python
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
move_differential.gyro = GyroSensor()


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


if __name__ == "__main__":
    crane_motor.on_for_degrees(degrees=30, speed=5, brake=False, block=False)

    set_leds_color("RED")
    speaker.beep()

    move_forward(distance_cm=INITIAL_CM_FROM_CAN + 50, speed=30, brake=False, block=False)

    while ultrasonic_sensor.distance_centimeters > DESIRED_CM_FROM_CAN:
        pass

    stop(brake=True)
    crane_motor.on_for_degrees(degrees=-60, speed=20, brake=False, block=False)

    while not touch_sensor.is_pressed:
        pass

    crane_motor.off(brake=False)
    set_leds_color("YELLOW")
    speaker.beep()
    crane_motor.on_for_degrees(degrees=60, speed=30, brake=False, block=False)
    move_forward(distance_cm=-150, speed=100, brake=False, block=False)

    sleep(1)

    while not floor_is_white():
        pass

    stop(brake=True)
    set_leds_color("GREEN")
    speaker.beep()

```
Las pruebas realizadas con esta ya permitían al robot detectar la distancia a la lata, accionar el brazo adecuadamente y detectar su salida del círculo. Solo había un ligero inconveniente. El robot debía estar perfectamente alineado con la lata desde el principio, 
o simplemente pasaba de largo. 

Fue en este momento cuando se añadió la lógica de detección de la lata, mediante sucesivas rotaciones del robot. Además, se eliminaron algunas funciones y variables innecesarias, priorizando la simplicidad y legibilidad del código, así como un buen rendimiento. Por supuesto, todo este proceso no fue inmediato y se ejecutaron multitud de pruebas de por medio. 

