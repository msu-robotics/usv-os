from machine import Pin, PWM

import time
from machine import Pin, PWM


class ESC:
    def __init__(self, pin, min_pulse=1000, max_pulse=2000, frequency=50):
        """
        Класс для управления ESC через PWM сигнал.

        :param pin: Номер пина для управления ESC
        :param min_pulse: Минимальная ширина импульса (микросекунды)
        :param max_pulse: Максимальная ширина импульса (микросекунды)
        :param frequency: Частота PWM сигнала (Гц)
        """
        self.pwm = PWM(Pin(pin))
        self.pwm.freq(frequency)
        self.min_pulse = min_pulse
        self.max_pulse = max_pulse
        self.frequency = frequency

    def calibrate(self):
        """
        Калибровка ESC. Обычно включает установку максимального и минимального
        значения сигнала.
        """
        print("Calibrating ESC...")
        # Установить на максимальное значение
        self.set_speed(100)
        time.sleep(2)
        # Установить на минимальное значение
        self.set_speed(0)
        time.sleep(2)
        print("Calibration complete")

    def set_speed(self, speed):
        """
        Установить скорость ESC.

        :param speed: Скорость, от 100 до 100
        """
        if speed < -100:
            speed = -100
        elif speed > 100:
            speed = 100

        mid_pulse = (self.max_pulse + self.min_pulse) / 2
        scale = (mid_pulse - self.min_pulse)/100

        speed = int(scale * speed)

        pulse_width = mid_pulse + speed
        duty = self.pulse_width_to_duty(pulse_width)
        self.pwm.duty_u16(duty)

    def pulse_width_to_duty(self, pulse_width):
        """
        Преобразовать ширину импульса в значение duty для PWM.

        :param pulse_width: Ширина импульса в микросекундах
        :return: Значение duty для метода duty_u16
        """
        period = 1000000 / self.frequency  # Период в микросекундах
        duty_cycle = pulse_width / period
        return int(duty_cycle * 65535)

    def stop(self):
        """
        Остановить ESC.
        """
        self.set_speed(0)


class SurfaceVehicle:
    def __init__(self, pins, motor_matrix=None, min_pulse=1000, max_pulse=2000, frequency=50):
        """
        Класс для управления надводным аппаратом с матрицей управления двигателями,
        поддерживающей движение вперед, поворот и лаг (боковое движение).

        :param pins: Список пинов для подключения ESC двигателей (4 пина).
        :param motor_matrix: Матрица направлений для каждого двигателя (если нет, используется стандартная конфигурация).
        :param min_pulse: Минимальная ширина импульса для ESC (микросекунды).
        :param max_pulse: Максимальная ширина импульса для ESC (микросекунды).
        :param frequency: Частота PWM сигнала (Гц).
        """
        if len(pins) != 4:
            raise ValueError("Должны быть указаны 4 пина для двигателей.")

        # Создаем объекты ESC для каждого двигателя
        self.motors = [ESC(pin, min_pulse, max_pulse, frequency) for pin in pins]

        # Определение матрицы: строки - двигатели, столбцы - оси движения, x, y, rz(поворот по оси z)
        if motor_matrix is None:
            self.motor_matrix = [
                [1,  1,  1],  # Левый передний двигатель (LF)
                [1, -1, -1],  # Правый передний двигатель (RF)
                [-1, 1, -1],  # Левый задний двигатель (LB)
                [-1,-1,  1]  # Правый задний двигатель (RB)
            ]
        else:
            self.motor_matrix = motor_matrix

        self.calibrate_all_esc()

    def calibrate_all_esc(self):
        """
        Калибровка всех ESC одновременно. Устанавливаем максимальный и минимальный сигнал для всех двигателей.
        """
        print("Starting ESC calibration...")

        # Устанавливаем максимальный сигнал на всех ESC
        for motor in self.motors:
            motor.set_speed(100)  # Максимальный сигнал (100%)
        time.sleep(2)  # Ждем 2 секунды

        # Устанавливаем минимальный сигнал на всех ESC
        for motor in self.motors:
            motor.set_speed(-100)  # Реверсивный сигнал (-100%)
        time.sleep(2)  # Ждем 2 секунды

        for motor in self.motors:
            motor.set_speed(0)  # Нулевое положение (0%)

        print("ESC calibration complete.")

    def set_motors(self, forward_speed, lateral_speed, yaw_speed):
        """
        Устанавливает скорость двигателей в зависимости от направлений по матрице.

        :param forward_speed: Скорость вперед/назад (-100 до 100).
        :param yaw_speed: Скорость поворота (влево/вправо) (-100 до 100).
        :param lateral_speed: Скорость лагового движения (боковое движение) (-100 до 100).
        """
        for i, motor in enumerate(self.motors):
            # Рассчитываем скорость для каждого двигателя на основе матрицы
            speed = (self.motor_matrix[i][0] * forward_speed) + \
                    (self.motor_matrix[i][1] * lateral_speed) + \
                    (self.motor_matrix[i][2] * yaw_speed)
            motor.set_speed(speed)

    def move_forward(self, speed):
        """
        Движение вперед на заданной скорости.

        :param speed: Скорость от -100 до 100.
        """
        self.set_motors(speed, 0, 0)

    def move_backward(self, speed):
        """
        Движение назад на заданной скорости.

        :param speed: Скорость от -100 до 100.
        """
        self.set_motors(-speed, 0, 0)

    def turn_right(self, speed):
        """
        Поворот вправо.

        :param speed: Скорость поворота (от -100 до 100).
        """
        self.set_motors(0, speed, 0)

    def turn_left(self, speed):
        """
        Поворот влево.

        :param speed: Скорость поворота (от -100 до 100).
        """
        self.set_motors(0, -speed, 0)

    def move_right(self, speed):
        """
        Движение вправо (лагом) на заданной скорости.

        :param speed: Скорость от -100 до 100.
        """
        self.set_motors(0, 0, speed)

    def move_left(self, speed):
        """
        Движение влево (лагом) на заданной скорости.

        :param speed: Скорость от -100 до 100.
        """
        self.set_motors(0, 0, -speed)

    def stop(self):
        """
        Остановить все двигатели.
        """
        self.set_motors(0, 0, 0)
