import cv2
import numpy as np

# Tải mô hình đã huấn luyện
# model = cv2.dnn.readNetFromONNX("traffic_sign_classifier_lenet_v2.onnx")


def filter_signs_by_color(image):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower1, upper1 = np.array([0, 70, 50]), np.array([10, 255, 255])
    lower2, upper2 = np.array([170, 70, 50]), np.array([180, 255, 255])
    
    mask_1 = cv2.inRange(image, lower1, upper1) 
    mask_2 = cv2.inRange(image, lower2, upper2)  
    mask_r = cv2.bitwise_or(mask_1, mask_2) 
   
    lower3, upper3 = np.array([85, 50, 200]), np.array([135, 250, 250])
    mask_b = cv2.inRange(image, lower3, upper3)
   
    mask_final = cv2.bitwise_or(mask_r, mask_b)
    return mask_final

def get_boxes_from_mask(mask):
    bboxes = []
    nccomps = cv2.connectedComponentsWithStats(mask, 4, cv2.CV_32S)
    numLabels, labels, stats, centroids = nccomps
    im_height, im_width = mask.shape[:2]
    for i in range(numLabels):
        x = stats[i, cv2.CC_STAT_LEFT]
        y = stats[i, cv2.CC_STAT_TOP]
        w = stats[i, cv2.CC_STAT_WIDTH]
        h = stats[i, cv2.CC_STAT_HEIGHT]
        area = stats[i, cv2.CC_STAT_AREA]
        # Lọc các vật quá nhỏ, có thể là nhiễu
        if w < 20 or h < 20:
            continue
        # Lọc các vật quá lớn
        if w > 0.8 * im_width or h > 0.8 * im_height:
            continue
        # Loại bỏ các vật có tỷ lệ dài / rộng quá khác biệt
        if w / h > 2.0 or h / w > 2.0:
            continue
        bboxes.append([x, y, w, h])
    return bboxes

def detect_traffic_signs(img, model):
    """Phát hiện biển báo"""
    # Các lớp biển báo
    classes = ['unknown', 'left', 'no_left', 'right',
               'no_right', 'straight', 'stop']

    # Phát hiện biển báo theo màu sắc
    mask = filter_signs_by_color(img)
    bboxes = get_boxes_from_mask(mask)

    # Tiền xử lý
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img_rgb = img_rgb.astype(np.float32) / 255.0

    # Phân loại biển báo dùng CNN
    signs = []
    for bbox in bboxes:
        # Cắt vùng cần phân loại
        x, y, w, h = bbox
        sub_img = img_rgb[y:y+h, x:x+w]

        if sub_img.shape[0] < 20 or sub_img.shape[1] < 20:
            continue

        # Tiền xử lý
        sub_img_resized = cv2.resize(sub_img, (32, 32))
        sub_img_expanded = np.expand_dims(sub_img_resized, axis=0)

        # Sử dụng CNN để phân loại biển báo
        model.setInput(sub_img_expanded)
        preds = model.forward()
        preds = preds[0]
        cls = preds.argmax()
        score = preds[cls]

        # Loại bỏ các vật không phải biển báo - thuộc lớp unknown
        if cls == 0:
            continue

        # Loại bỏ các vật có độ tin cậy thấp
        if score < 0.9:
            continue

        signs.append([classes[cls], x, y, w, h, score])

    return signs

# Khởi tạo camera
if __name__ == "__main__":
    model = cv2.dnn.readNetFromONNX("traffic_sign_classifier_lenet_v2.onnx")
    cap0 = cv2.VideoCapture(0)
    cap1 = cv2.VideoCapture(2)
    # trg = TrafficSign()
    while True:
        ret, frame = cap0.read()
        if not ret:
            print("Không thể đọc từ camera")
            break
        
        signs = detect_traffic_signs(frame, model)

        # Vẽ kết quả lên khung hình
        for sign in signs:
            cls, x, y, w, h, score = sign
            text = f"{cls} {round(score, 2)}"
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 255), 4)
            cv2.putText(frame, text, (x, y-10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            print(cls)
        # print(c)

        # cv2.imshow("Traffic Sign Detection", frame)

        # Nhấn phím 'q' để thoát
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Giải phóng tài nguyên
    cap1.release()
    cv2.destroyAllWindows()
