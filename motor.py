import RPi.GPIO as GPIO
import time
import serial

arduino_port = '/dev/ttyACM0' 
baud_rate = 9600    

# Connect Arduino
try:
    ser = serial.Serial(arduino_port, baud_rate, timeout=1)
    print("Concect Arduino!")
    time.sleep(2)
except:
    print("No Connect Arduino.")
    exit()

def send_command(command):
        ser.write((command + '\n').encode())

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

class Sensor:
    def __init__(self):
        pass

    def sensor(self, mt):
        if ser.in_waiting > 0:
            distance = ser.readline().decode('utf-8').strip()  
            try:
                distance = int(distance)  
                s = "Khong co vat can"
                if 0 < distance <= 25:  
                    s = "Dung xe, co vat can"
                    mt.stop()
                else:
                    s = "Khong co vat can"
                    mt.move_forward(15)
                return s, distance
            
            except ValueError:
                print("Error")

# class Servo:
#     def __init__(self, INS=1):
#         self.INS = INS
#         GPIO.setmode(GPIO.BCM)
#         GPIO.setup(self.INS, GPIO.OUT)

#         self.PWMS = GPIO.PWM(self.INS, 50)

#     def set_angle(self, angle):
#         duty_cycle = angle / 18.0 + 2.5
#         self.PWMS.start(0)
#         self.PWMS.ChangeDutyCycle(duty_cycle)
#         # time.sleep(0.01) 
    
#     # Stop servo
#     def stop(self):
#         self.PWMS.stop()

#     # Clear servo
#     def cleanup(self):
#         GPIO.cleanup()
class Servo:
    def __init__(self):
        pass

    def on(self, angle):
        send_command(f"O {int(angle)}")
        print("On Servo")

    def stop(self):
        send_command(f"S")
        print("Stop Servo")

def send_command(command):
        ser.write((command + '\n').encode())

if __name__ == "__main__":
    motor_controller = Motor()
    servo_motor = Servo()
    sensor = Sensor()

    try:
        while True:
            # Assuming you have some PID control mechanism here to calculate speed_A and speed_B
            # speed_A = 50
            # speed_B = 50

            # motor_controller.move_forward(speed_A)
            s, d = sensor.sensor(motor_controller)
            print(d)
            print(s)
            # servo_motor.servo(85) 
            time.sleep(0.1)

    except KeyboardInterrupt:
        servo_motor.stop()
        # servo_motor.cleanup()
        motor_controller.cleanup()
