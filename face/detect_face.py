import face_recognition
import cv2	
import os
import numpy as np

# Load known face encodings
dataset_path = 'dataset'
known_face_encodings = []
known_face_names = []

print("Loading known faces...")
for person_name in os.listdir(dataset_path):
    person_folder = os.path.join(dataset_path, person_name)
    if not os.path.isdir(person_folder):
        continue

    for filename in os.listdir(person_folder):
        img_path = os.path.join(person_folder, filename)
        img = face_recognition.load_image_file(img_path)
        encodings = face_recognition.face_encodings(img)

        if encodings:
            known_face_encodings.append(encodings[0])
            known_face_names.append(person_name)

print(f"Loaded encodings for {len(known_face_names)} images.")

# Use phone camera stream
video_url = 'http://192.0.0.4:8080/video'
cap = cv2.VideoCapture(video_url)

print("Starting face recognition. Press 'q' to exit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame.")
        break

    # Resize for faster processing
    small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
    rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)

    # Find faces and encodings
    face_locations = face_recognition.face_locations(rgb_small_frame)
    face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

    for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
        # Compare with known faces
        matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
        name = "Unknown"

        # Use the closest match
        face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
        if face_distances.size > 0:
            best_match_index = np.argmin(face_distances)
            if matches[best_match_index]:
                name = known_face_names[best_match_index]

        # Scale back up face locations
        top *= 4
        right *= 4
        bottom *= 4
        left *= 4

        # Draw box and name
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)
        cv2.putText(frame, name, (left, top - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

    cv2.imshow('Face Recognition', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
