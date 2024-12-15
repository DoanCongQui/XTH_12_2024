# -*- coding: utf-8 -*-
"""
Created on Wed Oct  9 11:08:38 2024

@author: trann
"""

import cv2
import numpy as np


class Gian_no:
    def gian_no(self, frame):
        # Tạo kernel cho quá trình giãn nở và co
        kernel = np.ones((3, 3), np.uint8)

        # Áp dụng giãn nở
        result_image = cv2.dilate(frame, kernel, iterations=10)

        # Áp dụng co
        result_image = cv2.erode(result_image, kernel, iterations=1)

        return result_image


class ImageProcessor:
    def __init__(self):
        self.image = None
        self.thresh = None
        self.result_image = None
        self.contourCenterX = 0
        self.MainContour = None
        self.last_value = 0  # Biến lưu giá trị 3 giây trước đó
        self.GN = Gian_no()
    


    
    def thresh(self):
        return self.thresh

    def Process(self, frame):

        # Kích thước khung hình gốc
        height, width, _ = frame.shape

        # Tính toán vị trí cắt (x, y)
        x_start = (width - 640) // 2
        y_start = (height - 300) // 2

        # Cắt khung hình từ vị trí chính giữa
        frame = frame[y_start:y_start + 300, x_start:x_start + 640]
        
        # Nhận hình ảnh từ frame của video capture
        self.image = frame

        self.image = cv2.GaussianBlur(self.image, (5, 5), 0)

        # Chuyển đổi sang ảnh grayscale
        imgray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

        # Them chuc nang dieu chinh lay nguong xam 

        # Lấy ngưỡng ảnh
        ret, thresh = cv2.threshold(imgray, 125, 255, cv2.THRESH_BINARY_INV)

        # Đảo ngược ảnh nhị phân từ trắng sang đen
        thresh = cv2.bitwise_not(thresh)

        # Đảo ngược ảnh nhị phân từ trắng sang đen
        thresh = cv2.bitwise_not(thresh)  # Đổi từ trắng sang đen

        self.thresh = thresh

        # # Tạo kernel cho quá trình giãn nở và co
        # kernel = np.ones((3, 3), np.uint8)

        # # Áp dụng giãn nở
        # result_image = cv2.dilate(thresh, kernel, iterations=5)

        # # Áp dụng co
        # result_image = cv2.erode(result_image, kernel, iterations=1)

        result_image = self.GN.gian_no(thresh)

        self.result_image = result_image

        # Tìm contour từ ảnh đã qua giãn nở và co
        self.contours, _ = cv2.findContours(result_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Lưu contour lớn nhất trước đó
        self.prev_MC = self.MainContour

        # Nếu tìm thấy contours
        if self.contours:
            # Lấy contour lớn nhất
            self.MainContour = max(self.contours, key=cv2.contourArea)
        
            # Lấy chiều cao và chiều rộng của hình ảnh
            self.height, self.width = self.image.shape[:2]

            # Lấy tọa độ X và Y của điểm giữa hình ảnh
            self.middleX = int(self.width / 2)
            self.middleY = int(self.height / 2)

            # Lưu tọa độ trung tâm contour trước đó
            self.prev_cX = self.contourCenterX

            # Nếu contour hiện tại hợp lệ
            if self.getContourCenter(self.MainContour) != 0:
                # Lấy tọa độ trung tâm contour hiện tại
                self.contourCenterX = self.getContourCenter(self.MainContour)[0]

                # Nếu sự thay đổi vị trí của trung tâm lớn hơn 5 pixel, điều chỉnh contour
                if abs(self.prev_cX - self.contourCenterX) > 5:
                    self.correctMainContour(self.prev_cX)
            else:
                # Nếu không tìm thấy trung tâm contour, đặt tọa độ trung tâm là 0
                self.contourCenterX = 0

            # Tính hướng di chuyển của contour
            self.dir = int((self.middleX - self.contourCenterX) * self.getContourExtent(self.MainContour))

            # Vẽ contour và các điểm trung tâm
            cv2.drawContours(self.image, [self.MainContour], -1, (0, 255, 0), 3)
            cv2.circle(self.image, (self.contourCenterX, self.middleY), 7, (255, 255, 255), -1)
            cv2.circle(self.image, (self.middleX, self.middleY), 3, (0, 0, 255), -1)

            # Hiển thị khoảng cách giữa tâm hình ảnh và tâm contour
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(self.image, str(self.middleX - self.contourCenterX), (self.contourCenterX + 20, self.middleY), font, 1, (200, 0, 200), 2, cv2.LINE_AA)
            cv2.putText(self.image, "Weight:%.3f" % self.getContourExtent(self.MainContour), (self.contourCenterX + 20, self.middleY + 35), font, 0.5, (200, 0, 200), 1, cv2.LINE_AA)

    def getContourCenter(self, contour):
        # Tính các moments của contour
        M = cv2.moments(contour)
        # Kiểm tra nếu diện tích contour là 0
        if M["m00"] == 0:
            return 0
        # Tính toán tọa độ trung tâm x và y của contour
        x = int(M["m10"] / M["m00"])
        y = int(M["m01"] / M["m00"])
        return [x, y]
    
    def getContourExtent(self, contour):
        # Tính diện tích của contour
        area = cv2.contourArea(contour)
        # Lấy hình chữ nhật bao quanh contour
        x, y, w, h = cv2.boundingRect(contour)
        rect_area = w * h
        # Trả về tỷ lệ contour
        return (float(area) / rect_area) if rect_area > 0 else 0

    def correctMainContour(self, prev_cx):
        # Duyệt qua các contours để tìm contour gần nhất với vị trí cũ
        for i in range(len(self.contours)):
            if self.getContourCenter(self.contours[i]) != 0:
                tmp_cx = self.getContourCenter(self.contours[i])[0]
                if abs(tmp_cx - prev_cx) <= 5:
                    self.MainContour = self.contours[i]
                    if self.getContourCenter(self.MainContour) != 0:
                        self.contourCenterX = self.getContourCenter(self.MainContour)[0]
                    break

    def show_distance(self):
        # Tính khoảng cách giữa tâm hình ảnh và tâm contour
        return self.middleX - self.contourCenterX
    
    def get_contour(self):
        position = self.show_distance()
        value = None  # Khởi tạo giá trị mặc định cho value
        if position is not None:  # Kiểm tra xem position có khác None không
            if position > -25 and position < 25:
                value = 1
            if position >= 25 and position <= 55 :
                value = -1.5
            elif position > 55 and position <= 75:
                value = -2.5
            elif position > 75 and position <= 100:
                value = -3
            elif position > 100 and position <= 320:
                value = -4
            
            if position <= -25 and position >= -55:
                value = 1.5
            elif position < -55 and position >= -75:
                value = 2.5
            elif position < -75 and position >= -100:
                value = 3
            elif position < -100 and position >= -320:
                value = 4

        # Nếu value vẫn là None, sử dụng giá trị 3 giây trước đó
        if value is None:
            value = self.last_value

        # Cập nhật giá trị trước đó
        self.last_value = value

        return value

# Phần mã chính để chạy chương trình
if __name__ == "__main__":
    cap = cv2.VideoCapture(0)  # Mở camera
    processor = ImageProcessor()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        processor.Process(frame)
        cv2.imshow("Processed Image", processor.image)
        cv2.imshow("nhi phan", processor.thresh)
        cv2.imshow("result_image", processor.result_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
