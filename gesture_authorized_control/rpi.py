gesture car

rpi:-

import cv2
import numpy as np
import serial
import time
import face_recognition
import os
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # Update port if needed
time.sleep(2)
known_encodings = []
known_names = []

# Load authorized faces from "known_faces" directory
for filename in os.listdir("known_faces"):
    if filename.endswith((".jpg", ".png")):
        img = face_recognition.load_image_file(f"known_faces/{filename}")
        enc = face_recognition.face_encodings(img)[0]
        known_encodings.append(enc)
        known_names.append(os.path.splitext(filename)[0])

print("[INFO] Loaded authorized faces:", known_names)

def count_fingers(thresh, drawing):
    contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return 0

    max_contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(max_contour) < 1000:
        return 0

    hull = cv2.convexHull(max_contour, returnPoints=False)
    if hull is None or len(hull) < 3:
        return 0

    defects = cv2.convexityDefects(max_contour, hull)
    if defects is None:
        return 0

    finger_count = 0
    for i in range(defects.shape[0]):
        s, e, f, d = defects[i, 0]
        start = tuple(max_contour[s][0])
        end = tuple(max_contour[e][0])
        far = tuple(max_contour[f][0])

        a = np.linalg.norm(np.array(end) - np.array(start))
        b = np.linalg.norm(np.array(far) - np.array(start))
        c = np.linalg.norm(np.array(end) - np.array(far))

        # FIX: corrected formula (used ** instead of *2 typo)
        angle = np.arccos((b**2 + c**2 - a**2) / (2 * b * c))

        if angle <= np.pi / 2 and d > 10000:
            finger_count += 1
            cv2.circle(drawing, far, 8, [255, 0, 0], -1)

    return finger_count + 1
def skin_mask(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = np.array([0, 30, 60], dtype=np.uint8)
    upper = np.array([20, 150, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower, upper)

    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=4)
    mask = cv2.GaussianBlur(mask, (5, 5), 0)

    return mask

def main():
    ip_url = 'http://192.168.1.100:8080/video'  # Change to your IP cam
    cap = cv2.VideoCapture(ip_url)

    if not cap.isOpened():
        print("Error: Cannot open IP webcam stream")
        return

    last_command = ''
    last_time = time.time()
    command_delay = 1.0
    authorized = False

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
        rgb_small = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)

        face_locations = face_recognition.face_locations(rgb_small)
        face_encodings = face_recognition.face_encodings(rgb_small, face_locations)

        authorized = False
        for enc in face_encodings:
            matches = face_recognition.compare_faces(known_encodings, enc, tolerance=0.5)
            if True in matches:
                authorized = True
                break

        if authorized:
            cv2.putText(frame, "AUTHORIZED", (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            roi = frame[100:400, 100:400]
            cv2.rectangle(frame, (100, 100), (400, 400), (0, 255, 0), 2)

            mask = skin_mask(roi)
            _, thresh = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)

            drawing = np.zeros(roi.shape, np.uint8)
            fingers = count_fingers(thresh, drawing)

            if fingers == 1:
                command = 'F'
            elif fingers == 2:
                command = 'B'
            elif fingers == 3:
                command = 'L'
            elif fingers == 4:
                command = 'R'
            else:
                command = 'S'

            current_time = time.time()
            if command != last_command and (current_time - last_time) > command_delay:
                ser.write(command.encode())
                print(f"Fingers: {fingers} -> Command sent: {command}")
                last_command = command
                last_time = current_time

            cv2.imshow('Threshold', thresh)
            cv2.imshow('Drawing', drawing)

        else:
            cv2.putText(frame, "NOT AUTHORIZED", (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            ser.write(b'S')  # force STOP if unauthorized

        cv2.imshow('Frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    ser.close()

if __name__ == "__main__":
    main()
