

class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint=0, output_limits=(-100, 100)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint

        self._integral = 0
        self._prev_error = None
        self._output_limits = output_limits

    def update(self, measurement, dt):
        # Вычисление угловой ошибки с учетом цикличности
        error = (self.setpoint - measurement + 180) % 360 - 180
        print(f'error: {error}, measurement: {measurement}, setpoint: {self.setpoint}')

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
