import time


def clamp(x, limits):
    """Clamp x to (low, high). Use None for no limit."""
    low, high = limits
    if x is None:
        return None
    if high is not None and x > high:
        return high
    if low is not None and x < low:
        return low
    return x


class PID:
    """
      pid = PID(Kp, Ki, Kd)
      pid.output_limits = (low, high)
      u = pid(measurement, dt=dt)
    """

    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd

        self.setpoint = 0.0              # stays 0 for "rate" control (as in your use)
        self.sample_time = None          # always update
        self.output_limits = (None, None)

        self._last_time = None
        self._last_measurement = None
        self._integral = 0.0

        self.time_fn = getattr(time, "monotonic", time.time)
        self.reset()

    def reset(self):
        self._integral = clamp(0.0, self.output_limits)
        self._last_time = self.time_fn()
        self._last_measurement = None

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint

    def call(self, measurement, dt):

        # Compute Error
        error = self.setpoint - measurement

        # Derivative on measurement (to avoid "derivative kick" when the setpoint changes)
        last_in = self._last_measurement if (self._last_measurement is not None) else measurement
        d_measurement = measurement - last_in

        # PID terms
        p = self.Kp * error

        self._integral += self.Ki * error * dt
        self._integral = clamp(self._integral, self.output_limits)  # anti-windup

        d = -self.Kd * d_measurement / dt

        # Output saturation
        output = p + self._integral + d
        output = clamp(output, self.output_limits)

        # Save state
        self._last_measurement = measurement

        return output
