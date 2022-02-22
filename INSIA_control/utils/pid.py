import numpy as np
import time


class PIDF(object):
    kp = None
    td = None
    ti = None
    prev_i = 0
    prev_d = 0
    prev_value = 0
    prev_target = 0
    _antiWindUp = None
    error = 0
    n = 8.0
    h = 0.1 / 5  # TODO DEPENDE DE LA FRECUENCIA

    def __init__(self, kp, td, ti, anti_wind_up, pro_wind_up=0):
        self.kp = kp
        self.td = td
        self.ti = ti
        self._antiWindUp = anti_wind_up
        self._pro_wind_up = pro_wind_up

    def reset_values(self):
        self.prev_i=0
        self.prev_d=0
        self.prev_value = 0
        self.prev_target = 0
        self.error = 0
        self.n = 8.0

    def calcValue(self, target_value, current_value, anti_wind_up_change=False, use=(True, True, True)):
        self.error = target_value - current_value

        if use[0] and self.kp != 0:
            p = self.kp * self.error
        else:
            p = 0

        if np.sign(self.prev_target) != np.sign(target_value) and anti_wind_up_change:
            i = 0
            self.prev_i = 0

        if use[1] and self.ti != 0:
            i = self.prev_i + (((self.kp * self.h) / self.ti) * self.error)
            i = self.antiwindup(i=i)
            self.prev_i = i
        else:
            i = 0

        if use[2] and self.td != 0:
            d = ((self.td / (self.td + (self.n * self.h)) * self.prev_d) - (
                    ((self.kp * self.td * self.n) / (self.td + (self.n * self.h))) * (current_value - self.prev_value)))
            self.prev_d = d
        else:
            d = 0

        self.prev_value = current_value

        if p + i + d < -1:
            return -1
        if p + i + d > 1:
            return 1

        self.prev_target = target_value

        return p + i + d

    def calcValue_2(self, target_value, current_value, kp, td, ti, anti_wind_up_change=False, use=(True, True, True)):
        self.error = target_value - current_value

        if use[0] and self.kp != 0:
            p = kp * self.error
        else:
            p = 0

        if np.sign(self.prev_target) != np.sign(target_value) and anti_wind_up_change:
            i = 0
            self.prev_i = 0

        if use[1] and self.ti != 0:
            i = self.prev_i + (((kp * self.h) / ti) * self.error)
            i = self.antiwindup(i=i)
            self.prev_i = i
        else:
            i = 0

        if use[2] and self.td != 0:
            d = ((td / (td + (self.n * self.h)) * self.prev_d) - (
                    ((kp * td * self.n) / (td + (self.n * self.h))) * (current_value - self.prev_value)))
            self.prev_d = d
        else:
            d = 0

        self.prev_value = current_value

        if p + i + d < -1:
            return -1
        if p + i + d > 1:
            return 1

        self.prev_target = target_value

        return p + i + d

    def antiwindup(self, i):
        if i > self._antiWindUp:
            i = self._antiWindUp
        elif i < -self._antiWindUp:
            i = -self._antiWindUp

        if 0 < i < self._pro_wind_up:
            i = self._pro_wind_up
        elif 0 > i > -self._pro_wind_up:
            i = -self._pro_wind_up

        return i

    def setKp(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.kp = proportional_gain

    def setKi(self, integral_gain):
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        self.ti = integral_gain

    def setKd(self, derivative_gain):
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        self.td = derivative_gain


class PID(object):
    """
    PID Controller
    """

    __slots__ = (
        'Kp', 'Ti', 'Td', 'windup_guard', 'sample_time', 'current_time', 'last_time', 'PTerm', 'ITerm', 'DTerm',
        'last_error', 'int_error', 'output')

    def __init__(self, kp=0.2, ti=0.0, td=0.0, anti_wind_up=0.2):

        self.Kp = kp
        self.Ti = ti
        self.Td = td
        self.windup_guard = anti_wind_up
        self.current_time = time.time()
        self.last_time = self.current_time
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.output = 0.0

    def calcValue(self, target_value, current_value):
        """Calculates PID value for given reference feedback

        .. math::
            u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}

        .. figure:: images/pid_1.png
           :align:   center

           Test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)

        """

        error = target_value - current_value

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        self.PTerm = self.Kp * error
        self.ITerm += error * delta_time

        if self.ITerm < -self.windup_guard:
            self.ITerm = -self.windup_guard
        elif self.ITerm > self.windup_guard:
            self.ITerm = self.windup_guard

        self.DTerm = 0.0
        if delta_time > 0:
            self.DTerm = delta_error / delta_time

        # Remember last time and last error for next calculation
        self.last_time = self.current_time
        self.last_error = error

        self.output = self.PTerm + (self.Ti * self.ITerm) + (self.Td * self.DTerm)
        return self.output

    def setWindup(self, windup):
        """Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in setpoint occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        """
        self.windup_guard = windup

    # def setKp(self, proportional_gain):
    #     """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
    #     self.Kp = proportional_gain
    #
    # def setKi(self, integral_gain):
    #     """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
    #     self.Ti = integral_gain
    #
    # def setKd(self, derivative_gain):
    #     """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
    #     self.Td = derivative_gain


if __name__ == '__main__':

    import matplotlib.pyplot as plt


    def test_pid(kp=0.2, ki=0.0, kd=0.0, end=100):
        """Self-test PID class

        .. note::
            ...
            for i in range(1, END):
                pid.update(feedback)
                output = pid.output
                if pid.SetPoint > 0:
                    feedback += (output - (1/i))
                if i>9:
                    pid.SetPoint = 1
                time.sleep(0.02)
            ---
        """
        pid = PID(kp, ki, kd)

        END = end
        feedback = 0

        feedback_list = []
        time_list = []
        setpoint_list = []
        v = 0
        for i in range(1, END):
            pid.calcValue(v, feedback)
            output = pid.output
            if v > 0:
                feedback += (output - (1 / i))
            if i > 9:
                v = 1
            time.sleep(0.02)

            feedback_list.append(feedback)
            setpoint_list.append(v)
            time_list.append(i)

        plt.plot(list(range(0, len(feedback_list))), feedback_list)
        plt.plot(time_list, setpoint_list)
        plt.xlim((0, end))
        plt.ylim((min(feedback_list) - 0.5, max(feedback_list) + 0.5))
        plt.xlabel('time (s)')
        plt.ylabel('PID (PV)')
        plt.title('TEST PID')

        plt.ylim((1 - 0.5, 1 + 0.5))

        plt.grid(True)
        plt.savefig('piddddd.png')


    test_pid(1.2, 1, 0.001, end=50)
    # test_pid(0.8, end=50)
