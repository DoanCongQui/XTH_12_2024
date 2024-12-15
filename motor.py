import RPi.GPIO as GPIO
import time


class MotorController:
    #INT1_PIN=17, INT2_PIN=4, ENA_PIN=26, INT3_PIN=27, INT4_PIN=22, ENB_PIN=5
    def __init__(self, enA=26, in1=4, in2=17, in3=27, in4=22, enB=5):
        self.enA = enA
        self.in1 = in1
        self.in2 = in2
        self.in3 = in3
        self.in4 = in4
        self.enB = enB

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.enA, GPIO.OUT)
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        GPIO.setup(self.in3, GPIO.OUT)
        GPIO.setup(self.in4, GPIO.OUT)
        GPIO.setup(self.enB, GPIO.OUT)

        # Create PWM objects
        self.pwm_A = GPIO.PWM(self.enA, 100)  # PWM frequency: 100 Hz
        self.pwm_B = GPIO.PWM(self.enB, 100)  # PWM frequency: 100 Hz

        # Start PWM
        self.pwm_A.start(0)  # Initial speed: 0
        self.pwm_B.start(0)  # Initial speed: 0

    def move_forward(self, speed1, speed2):
        GPIO.output(self.in1, GPIO.HIGH)
        GPIO.output(self.in2, GPIO.LOW)
        self.pwm_A.ChangeDutyCycle(speed1)

        GPIO.output(self.in3, GPIO.LOW)
        GPIO.output(self.in4, GPIO.HIGH)
        self.pwm_B.ChangeDutyCycle(speed2)

    def move_DC(self, speed):
        # Giới hạn giá trị speed trong khoảng 0.0 đến 100.0
        speed = max(0.0, min(100.0, speed))
        GPIO.output(self.in1, GPIO.HIGH)
        GPIO.output(self.in2, GPIO.LOW)
        self.pwm_A.ChangeDutyCycle(speed)

        GPIO.output(self.in3, GPIO.LOW)
        GPIO.output(self.in4, GPIO.HIGH)
        self.pwm_B.ChangeDutyCycle(speed)

    def stop(self):
        self.pwm_A.ChangeDutyCycle(0)
        self.pwm_B.ChangeDutyCycle(0)

    def cleanup(self):
        self.stop()
        GPIO.cleanup()

# # Example usage:
if __name__ == "__main__":
    motor_controller = MotorController()

    try:
        while True:
            # Assuming you have some PID control mechanism here to calculate speed_A and speed_B
            speed_A = 100
            speed_B = 100

            motor_controller.move_DC(speed_A)
            time.sleep(0.1)

    except KeyboardInterrupt:
        motor_controller.cleanup()
