import cv2 as cv
import numpy as np
import time

from motor import Motor, Servo
import traffic_sign

model = cv.dnn.readNetFromONNX("traffic_sign_classifier_lenet_v2.onnx")

if __name__ == "__main__":
    # Initialize camera
    cap = cv.VideoCapture(0)
    if not cap.isOpened():
        print("Failed to open the camera.")
        exit()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Cannot read from camera.")
            break

        # Bien bao
        signs = traffic_sign.detect_traffic_signs(frame, model)

        # Vẽ kết quả lên khung hình
        for sign in signs:
            cls, x, y, w, h, score = sign
            text = f"{cls} {round(score, 2)}"
            cv.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 255), 4)
            cv.putText(frame, text, (x, y-10),
            cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # Show frame (optional)
        # cv.imshow("Frame", frame)
        print("Nhan dien: ", cls)

        # Exit on pressing 'q'
        if cv.waitKey(25) & 0xFF == ord('q'):
            break

    # Release resources
    cap.release()
    cv.destroyAllWindows()
