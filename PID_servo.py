import time
from decimal import Decimal

class PID_SV:
    def __init__(self):
        #self.kp = 4.755    # Proportional factor
        #self.ki = 4.555   # Integrating factor
        #self.kd = 4.444 # Differential factor

        self.kp = 4.7455    # Proportional factor
        self.ki = -25.555   # Integrating factor -35..555 cua duoc thang lac, -25.555 chay thang 
        self.kd = 90 # Differential factor

        #self.kp = 4.7455    # Proportional factor
        #self.ki = -25.555   # Integrating factor -35..555 cua duoc thang lac, -25.555 chay thang 
        #self.kd = 90 # Differential factor

        self.previous_error = 0
        self.previous_time = time.time()

        self.base = 100
        self.max = 130
        self.min = 60

    def pid(self, error):

        error = error * -1 # Dao gia tri 
        curr_time = time.time()
        elapsedTime = curr_time - self.previous_time
        pid_p = self.kp * error
        pid_i = error * elapsedTime
        pid_d = self.kd * ((error - self.previous_error) / elapsedTime)
        pid = pid_p + self.ki * pid_i + pid_d
        servo = self.base + pid
        if servo > self.max:
            servo = self.max
        if servo < self.min:
            servo = self.min
        self.previous_error = error
        self.previous_time = curr_time

        value = Decimal(servo)
        rounded_value = round(value, 3)
        return float(rounded_value)


