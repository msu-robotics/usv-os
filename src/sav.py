import time
from machine import Pin, PWM
from pid_controller import PIDController


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
        self.current_speed = 0
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

        self.current_speed = speed

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

    def get_pulse_width(self):
        """
        Возвращает текущую ширину импульса в микросекундах.

        :return: Ширина импульса в микросекундах.
        """
        # Преобразуем текущую скорость в значение ШИМ
        pulse_width = ((self.current_speed + 100) / 200) * (self.max_pulse - self.min_pulse) + self.min_pulse
        return pulse_width


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

        # Определение матрицы: строки - двигатели, столбцы - оси движения: [вперед/назад, боковое, поворот]
        if motor_matrix is None:
            self.motor_matrix = [
                [-1,  -1,  -1],   # Левый передний двигатель (LF)
                [1, -1, -1],   # Правый передний двигатель (RF)
                [1, -1, 1],   # Левый задний двигатель (LB)
                [1, 1,  -1]    # Правый задний двигатель (RB)
            ]
        else:
            self.motor_matrix = motor_matrix

        self.calibrate_all_esc()

        self.mode = 'manual'  # Режим управления: 'manual' или 'stabilization'
        self.pid_controller = PIDController(0, 0, 0)  # Инициализация PID-регулятора с коэффициентами по умолчанию
        self.last_time = time.time()
        self.forward_speed = 0
        self.lateral_speed = 0
        self.user_yaw_speed = 0  # Заданный пользователем yaw_speed
        self.probe_motor_1 = Pin(26, Pin.OUT)
        self.probe_motor_2 = Pin(33, Pin.OUT)
        self.probe_motor_3 = Pin(32, Pin.OUT)
        self.probe_motor_4 = Pin(25, Pin.OUT)
        # Список для хранения текущих значений ШИМ в микросекундах
        self.current_pwm_values = [0.0] * len(self.motors)

    def calibrate_all_esc(self):
        """
        Калибровка всех ESC одновременно. Устанавливаем максимальный и минимальный сигнал для всех двигателей.
        """
        print("Начало калибровки ESC...")

        # Устанавливаем максимальный сигнал на всех ESC
        for motor in self.motors:
            motor.set_speed(100)  # Максимальный сигнал (100%)
        time.sleep(2)  # Ждем 2 секунды

        # Устанавливаем минимальный сигнал на всех ESC
        for motor in self.motors:
            motor.set_speed(-100)  # Минимальный сигнал (-100%)
        time.sleep(2)  # Ждем 2 секунды

        # Устанавливаем нулевой сигнал на всех ESC
        for motor in self.motors:
            motor.set_speed(0)  # Нулевой сигнал (0%)

        print("Калибровка ESC завершена.")

    def probe(self, direction):
        if direction == 'up':
            self.probe_motor_1.off()
            self.probe_motor_2.on()
            self.probe_motor_3.off()
            self.probe_motor_4.on()
        elif direction == 'down':
            self.probe_motor_2.off()
            self.probe_motor_1.on()
            self.probe_motor_4.off()
            self.probe_motor_3.on()
        else:
            self.probe_motor_1.off()
            self.probe_motor_2.off()
            self.probe_motor_3.off()
            self.probe_motor_4.off()
        print(f'Установленно значение для пробоотборника {direction}')

    def set_mode(self, mode):
        """
        Установка режима управления.

        :param mode: Режим управления: 'manual' или 'stabilization'.
        """
        if mode not in ['manual', 'stabilization']:
            raise ValueError("Недопустимый режим. Используйте 'manual' или 'stabilization'.")
        self.mode = mode
        print(f"Режим установлен: {mode}")
        # Сброс PID-регулятора при переключении в режим стабилизации
        if mode == 'stabilization':
            self.pid_controller.reset()

    def update_pid_settings(self, p_gain, i_gain, d_gain):
        """
        Обновление коэффициентов PID-регулятора.

        :param p_gain: Пропорциональный коэффициент.
        :param i_gain: Интегральный коэффициент.
        :param d_gain: Дифференциальный коэффициент.
        """
        self.pid_controller.Kp = p_gain
        self.pid_controller.Ki = i_gain
        self.pid_controller.Kd = d_gain
        print(f"Обновлены настройки PID: P={p_gain}, I={i_gain}, D={d_gain}")

    def set_motors(self, forward_speed, lateral_speed, yaw_speed):
        """
        Устанавливает скорость двигателей в зависимости от направлений по матрице.

        :param forward_speed: Скорость вперед/назад (-100 до 100).
        :param lateral_speed: Скорость лагового движения (боковое движение) (-100 до 100).
        :param yaw_speed: Скорость поворота (влево/вправо) (-100 до 100).
        """
        self.forward_speed = forward_speed
        self.lateral_speed = lateral_speed
        self.user_yaw_speed = yaw_speed  # Сохраняем yaw_speed от пользователя

        if self.mode == 'manual':
            # В ручном режиме используем yaw_speed от пользователя
            actual_yaw_speed = yaw_speed

            # Рассчитываем и устанавливаем скорость для каждого двигателя
            for i, motor in enumerate(self.motors):
                speed = (self.motor_matrix[i][0] * self.forward_speed) + \
                        (self.motor_matrix[i][1] * self.lateral_speed) + \
                        (self.motor_matrix[i][2] * actual_yaw_speed)
                motor.set_speed(speed)
                # Преобразуем скорость в значение ШИМ в микросекундах и сохраняем
                self.current_pwm_values[i] = motor.get_pulse_width()
        elif self.mode == 'stabilization':
            # В режиме стабилизации yaw_speed управляется PID-регулятором
            # Устанавливаем целевое значение yaw в PID-регуляторе
            self.pid_controller.setpoint = self.user_yaw_speed
            # Двигатели будут обновлены в методе update_control()
        else:
            # Если режим неизвестен, останавливаем двигатели
            self.stop()

    def update_control(self, current_yaw):
        """
        Обновление управления двигателями на основе текущего угла yaw.

        :param current_yaw: Текущее значение yaw (в градусах).
        """
        if self.mode != 'stabilization':
            return  # В режиме 'manual' ничего не делаем

        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0:
            dt = 0.01  # Предотвращаем деление на ноль
        self.last_time = current_time

        # Вычисляем управляющее воздействие PID-регулятора
        yaw_correction = self.pid_controller.update(current_yaw, dt)

        # Ограничиваем yaw_correction в диапазоне допустимых значений скорости
        yaw_correction = max(-100, min(100, yaw_correction))

        # Рассчитываем и устанавливаем скорость для каждого двигателя
        for i, motor in enumerate(self.motors):
            speed = (self.motor_matrix[i][0] * self.forward_speed) + \
                    (self.motor_matrix[i][1] * self.lateral_speed) + \
                    (self.motor_matrix[i][2] * yaw_correction)
            motor.set_speed(speed)
            # Преобразуем скорость в значение ШИМ в микросекундах и сохраняем
            self.current_pwm_values[i] = motor.get_pulse_width()

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
        self.set_motors(0, 0, speed)

    def turn_left(self, speed):
        """
        Поворот влево.

        :param speed: Скорость поворота (от -100 до 100).
        """
        self.set_motors(0, 0, -speed)

    def move_right(self, speed):
        """
        Движение вправо (лагом) на заданной скорости.

        :param speed: Скорость от -100 до 100.
        """
        self.set_motors(0, speed, 0)

    def move_left(self, speed):
        """
        Движение влево (лагом) на заданной скорости.

        :param speed: Скорость от -100 до 100.
        """
        self.set_motors(0, -speed, 0)

    def stop(self):
        """
        Остановить все двигатели.
        """
        self.set_motors(0, 0, 0)
        # Останавливаем двигатели
        for motor in self.motors:
            motor.set_speed(0)

    def get_pwm_values(self):
        """
        Возвращает текущие значения ШИМ в микросекундах для каждого двигателя.

        :return: Список значений ШИМ в микросекундах.
        """
        return self.current_pwm_values
