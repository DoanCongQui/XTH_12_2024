import RPi.GPIO as GPIO
import time

class Motor:
    def __init__(self, ENA=26, IN1=4, IN2=17, ENB=5, IN3=27, IN4=22):
        self.ENA = ENA
        self.ENB = ENB
        self.IN1 = IN1
        self.IN2 = IN2
        self.IN3 = IN3
        self.IN4 = IN4

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.ENA, GPIO.OUT)
        GPIO.setup(self.ENB, GPIO.OUT)
        GPIO.setup(self.IN1, GPIO.OUT)
        GPIO.setup(self.IN2, GPIO.OUT)
        GPIO.setup(self.IN3, GPIO.OUT)
        GPIO.setup(self.IN4, GPIO.OUT)

        # Setup PWM
        self.PWMA = GPIO.PWM(self.ENA, 100)
        self.PWMB = GPIO.PWM(self.ENB, 100)

        # Start PWM
        self.PWMA.start(0)
        self.PWMB.start(0)

    # Forward motor
    def move_forward(self, speed):
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)
        self.PWMA.ChangeDutyCycle(speed)
        self.PWMB.ChangeDutyCycle(speed)
    
    # Stop motor
    def stop(self):
        self.PWMA.ChangeDutyCycle(0)
        self.PWMB.ChangeDutyCycle(0)

    # Clear motor
    def cleanup(self):
        self.stop()
        GPIO.cleanup()

class Servo:
    def __init__(self, INS=1):
        self.INS = INS
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.INS, GPIO.OUT)

        self.PWMS = GPIO.PWM(self.INS, 50)

    def set_angle(self, angle):
        duty_cycle = angle / 18.0 + 2.5
        self.PWMS.start(0)
        self.PWMS.ChangeDutyCycle(duty_cycle)
        # time.sleep(0.01) 
    
    # Stop servo
    def stop(self):
        self.PWMS.stop()

    # Clear servo
    def cleanup(self):
        GPIO.cleanup()

if __name__ == "__main__":
    motor_controller = Motor()
    servo_motor = Servo()

    try:
        while True:
            # Assuming you have some PID control mechanism here to calculate speed_A and speed_B
            speed_A = 50
            speed_B = 50

            motor_controller.move_forward(speed_A)
            servo_motor.set_angle(38) 
            time.sleep(0.1)

    except KeyboardInterrupt:
        servo_motor.stop()
        servo_motor.cleanup()
        motor_controller.cleanup()
