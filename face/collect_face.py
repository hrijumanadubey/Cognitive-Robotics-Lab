import cv2
import os

# Load Haar cascade for face detection
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Ask user for name
person_name = input("Enter the person's name: ").strip()
dataset_path = 'dataset'
person_folder = os.path.join(dataset_path, person_name)

# Create directory if not exists
os.makedirs(person_folder, exist_ok=True)

# Use phone camera stream
# Replace this with your actual IP Webcam URL
video_url = 'http://192.0.0.4:8080/video'
cap = cv2.VideoCapture(video_url)

count = 0
max_images = 50  # Number of face images to collect

print(f"Collecting images for {person_name}. Press 'q' to quit early.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame.")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)

    for (x, y, w, h) in faces:
        count += 1
        face_img = frame[y:y+h, x:x+w]
        face_img = cv2.resize(face_img, (200, 200))

        file_path = os.path.join(person_folder, f"{person_name}_{count}.jpg")
        cv2.imwrite(file_path, face_img)

        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.putText(frame, f"{count}/{max_images}", (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

    cv2.imshow("Collecting Faces", frame)

    if cv2.waitKey(1) & 0xFF == ord('q') or count >= max_images:
        break

cap.release()
cv2.destroyAllWindows()

print(f"Saved {count} images to '{person_folder}'")
