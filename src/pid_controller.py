

class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint=0, output_limits=(-100, 100)):
        """
        Инициализация PID-регулятора.

        :param Kp: Пропорциональный коэффициент
        :param Ki: Интегральный коэффициент
        :param Kd: Дифференциальный коэффициент
        :param setpoint: Желаемое значение (уставка)
        :param output_limits: Кортеж с минимальным и максимальным значением выходного сигнала
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint

        self._integral = 0
        self._prev_error = None
        self._output_limits = output_limits

    def update(self, measurement, dt):
        """
        Обновление PID-регулятора на основе текущего измерения и времени.

        :param measurement: Текущее значение измерения (например, текущий угол yaw)
        :param dt: Время, прошедшее с последнего вызова (в секундах)
        :return: Вычисленное управляющее воздействие
        """
        error = self.setpoint - measurement

        # Пропорциональная составляющая
        P = self.Kp * error

        # Интегральная составляющая
        self._integral += error * dt
        I = self.Ki * self._integral

        # Дифференциальная составляющая
        D = 0
        if self._prev_error is not None:
            derivative = (error - self._prev_error) / dt
            D = self.Kd * derivative
        self._prev_error = error

        # Общий выход PID
        output = P + I + D

        # Применение ограничений на выход
        min_output, max_output = self._output_limits
        if min_output is not None:
            output = max(min_output, output)
        if max_output is not None:
            output = min(max_output, output)

        return output

    def reset(self):
        """Сброс внутреннего состояния PID-регулятора."""
        self._integral = 0
        self._prev_error = None
