# pip install pyserial

import cv2
import RPi.GPIO as GPIO  # Thêm thư viện RPi.GPIO
from line_point import ImageProcessor
from PID_servo import PID_SV
from Dieu_khien_dong_co import MotorController
from Servo import ServoMotor
# from ultralytics import YOLO
import time
import numpy as np


class UART:
    def __init__(self, weights_path, target_class_name, confidence_min, confidence_max, led_pin_1, led_pin_2):
        # self.model = YOLO(weights_path)
        self.target_class_name = target_class_name
        self.confidence_min = confidence_min
        self.confidence_max = confidence_max

        # Bien toan cuc
        self.K = 1

        # Khởi tạo các đối tượng điều khiển
        self.contour = ImageProcessor()
        self.pid_servo = PID_SV()
        self.control_DC = MotorController()
        self.Servo = ServoMotor()

        # Thiết lập GPIO cho đèn LED
        GPIO.setmode(GPIO.BCM)  # Sử dụng sơ đồ chân BCM
        self.led_pin_1 = led_pin_1
        self.led_pin_2 = led_pin_2
        GPIO.setup(self.led_pin_1, GPIO.OUT)
        GPIO.setup(self.led_pin_2, GPIO.OUT)

    def Xanh_lam(self, frame):
        # Lưu frame hiện tại vào biến image
        xanh_lam = frame.copy()

        # Chuyển đổi hình ảnh từ không gian màu BGR sang HSV
        hsv = cv2.cvtColor(xanh_lam, cv2.COLOR_BGR2HSV)

        # Đặt ngưỡng màu xanh lam (giá trị có thể điều chỉnh)
        lower_blue = np.array([100, 150, 0])
        upper_blue = np.array([140, 255, 255])

        # Lọc ngưỡng màu xanh lam
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Tìm các contour từ mặt nạ
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        has_box = 0

        # Vẽ hộp bao quanh các contour có kích thước lớn hơn hoặc bằng 20x20
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if w >= 20 and h >= 20:  # Điều kiện kiểm tra kích thước
                cv2.rectangle(xanh_lam, (x, y), (x + w, y + h), (255, 0, 0), 2)  # Hộp màu xanh lam
                has_box = 1

        return xanh_lam, has_box


    def Xanh_la(self, frame):
        # Lưu frame hiện tại vào biến image
        Xanh_la = frame.copy()

        # Chuyển đổi hình ảnh từ không gian màu BGR sang HSV
        hsv = cv2.cvtColor(Xanh_la, cv2.COLOR_BGR2HSV)

        # Đặt ngưỡng màu xanh lam (giá trị có thể điều chỉnh)
        lower_blue = np.array([35, 50, 50])
        upper_blue = np.array([85, 255, 255])

        # Lọc ngưỡng màu xanh lam
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Tìm các contour từ mặt nạ
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        has_box = 0

        # Vẽ hộp bao quanh các contour có kích thước lớn hơn hoặc bằng 20x20
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if w >= 20 and h >= 20:  # Điều kiện kiểm tra kích thước
                cv2.rectangle(Xanh_la, (x, y), (x + w, y + h), (0, 0, 255), 2)  # Hộp màu xanh lam
                has_box = 1

        return Xanh_la, has_box

    def image_thresh(self):
        return self.contour.image_thresh()

    def number_to_send(self, value, speed_DC):
        self.Servo.set_angle(value)
        self.control_DC.move_forward(speed_DC, speed_DC)

    def speed_DC(self, value):
        # if value < 55 or value > 110:
        #     speed = 13  # Giá trị mặc định nếu không nằm trong các khoảng trên
        # else:
        speed = 8
        return speed

    def Nhan_ID(self, servo, speed):
        ID = servo * speed
        return ID


    def perform_actions(self):
        """Thực hiện các hành động khi nhận diện đạt ngưỡng"""
        # self.control_leds(True, False)
        # self.number_to_send(110, 10) # servo, dc
        # time.sleep(0.1)
        self.number_to_send(135, 12)
        time.sleep(2.5)
        # self.control_leds(True, True)
        # self.s_DC(0)
        self.K = 1


    def goc(self, frame):
        # Tạo bản sao của khung hình để vẽ mà không ảnh hưởng đến khung hình gốc
        processed_frame = frame.copy()

        results = self.model(processed_frame)
        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                conf = box.conf[0]
                cls = int(box.cls[0])
                class_name = self.model.names[cls]

                if class_name == self.target_class_name and self.confidence_min <= conf <= self.confidence_max:
                    print(f"Class {class_name} đạt ngưỡng với độ tin cậy: {conf:.2f}")

                    # # Bật đèn LED khi đạt ngưỡng
                    # self.control_led(True)
                    self.K = 0

                    # Vẽ bounding box và nhãn
                    label = f'{class_name} {conf:.2f}'
                    cv2.rectangle(processed_frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
                    cv2.putText(processed_frame, label, (int(x1), int(y1)-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                else:
                    # Tắt đèn LED nếu không đạt ngưỡng
                    self.control_led(False)

        return processed_frame

    def control_led(self, state):
        """Điều khiển trạng thái đèn LED."""
        GPIO.output(self.led_pin_1, GPIO.HIGH if state else GPIO.LOW)
        GPIO.output(self.led_pin_2, GPIO.HIGH if state else GPIO.LOW)

    def cleanup(self):
        """Hàm dọn dẹp GPIO."""
        GPIO.cleanup()

# Khởi tạo đối tượng UART với các tham số cần thiết
uart = UART(
    weights_path='/home/dell/Desktop/DO_AN/best_bien_bao_14_11_2024_yolov8n_50.pt',
    target_class_name='goc_cua',
    confidence_min=0.75,
    confidence_max=0.9,
    led_pin_1=23,  # Sử dụng chân GPIO 23 cho LED thứ nhất
    led_pin_2=24   # Sử dụng chân GPIO 24 cho LED thứ hai
)

cap = cv2.VideoCapture(0)
has_box =0
has_box02 = 0

if not cap.isOpened():
    print("Cannot open video.")
else:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("Cannot read more frames (end of video or error).")
            break

        frame = cv2.resize(frame, (640, 480))
        frameA = frame.copy()
        frameB = frame.copy()
        xanh_lam, has_box = uart.Xanh_lam(frameA)
        xanh_la, has_box02 = uart.Xanh_la(frameB)
            # Xử lý frame với ImageProcessor

        thresh = uart.contour.Process(frame)
        value = uart.contour.get_contour()

        if has_box == 1:
            #value_pid_servo = 130
            print("OK")

       # if has_box == 0:
         #   value_pid_servo = uart.pid_servo.pid(value)
            
        value_pid_servo = uart.pid_servo.pid(value)
        # print(value)
        speed_DC = uart.speed_DC(value_pid_servo)
        uart.number_to_send(value_pid_servo, speed_DC)

        print(value_pid_servo)

        #uart.control_led(True)
        # Gửi giá trị servo sau khi mã hóa qua UART
        #uart.number_to_send(value_pid_servo, speed_DC)

        # Hiển thị hình ảnh đã xử lý
        cv2.imshow('Masked Image', uart.contour.image)
        # cv2.imshow('Original Frame', frame)
#        cv2.imshow('xanh', xanh_lam)
#        cv2.imshow('xanh_la', xanh_la)
        #cv2.imshow('Doi tuong', frame_doi_tuong)

        # Thoát khi nhấn phím 'q'
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()

# Dọn dẹp GPIO khi kết thúc
uart.cleanup()
