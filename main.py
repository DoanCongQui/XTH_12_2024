import cv2 as cv
import numpy as np
import time

from motor import Motor, Servo
import traffic_sign 


if __name__ == "__main__":
    cap = cv.VideoCapture(0)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("Không thể đọc từ camera")
            break

        __, cls = traffic_sign.run_traffic(frame)

        print(cls)
        # cv.imshow("Frame",frame)
        if cv.waitKey(25) & 0xFF == ord('q'):
            break
    cap.release()
    cv.destroyAllWindows()

