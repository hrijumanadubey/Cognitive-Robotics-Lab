import cv2
import time
import argparse
import serial
from collections import Counter, deque
import mediapipe as mp
import threading

# -------- Configurable parameters --------
GESTURE_WINDOW = 6            # majority window frames
NO_HAND_TIMEOUT = 1.2         # seconds to consider no hand -> stop
SEND_REPEAT_INTERVAL = 0.15   # seconds before resending same command (reduction)
DEFAULT_SPEED = 180           # 0..255 default PWM sent to Arduino for move commands
BACKWARD_SPEED = 160
TURN_SPEED = 170
# mapping gesture_count -> (cmd_char, speed)
GESTURE_MAP = {
    0: ('S', 0),               # fist -> stop
    1: ('F', DEFAULT_SPEED),   # one finger -> forward
    2: ('B', BACKWARD_SPEED),  # two -> backward
    3: ('L', TURN_SPEED),      # three -> turn left
    4: ('R', TURN_SPEED),      # four -> turn right
}
# -----------------------------------------

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

# finger tip and pip ids for index..pinky (thumb ignored)
FINGER_TIP_IDS = [8, 12, 16, 20]
FINGER_PIP_IDS = [6, 10, 14, 18]

def count_raised_fingers(hand_landmarks, h, w):
    lm = hand_landmarks.landmark
    cnt = 0
    for tip, pip in zip(FINGER_TIP_IDS, FINGER_PIP_IDS):
        tip_y = lm[tip].y * h
        pip_y = lm[pip].y * h
        if tip_y < pip_y:  # tip above pip -> finger up
            cnt += 1
    return cnt

class SerialSender:
    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.last_sent = None
        self.last_time = 0

    def send(self, cmd_char, speed):
        # format "F150\n"
        line = f"{cmd_char}{int(speed)}\n"
        now = time.time()
        # avoid flooding identical commands too fast
        if line == self.last_sent and (now - self.last_time) < SEND_REPEAT_INTERVAL:
            return
        try:
            self.ser.write(line.encode('ascii'))
            self.last_sent = line
            self.last_time = now
        except Exception as e:
            print("Serial write err:", e)

    def close(self):
        try:
            self.ser.close()
        except:
            pass

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True, help="IP Webcam URL or camera index")
    parser.add_argument("--serial", required=True, help="Serial port to Arduino, e.g. /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--show", action="store_true", help="Show debug window")
    args = parser.parse_args()

    # open serial
    sender = SerialSender(args.serial, args.baud)
    print("Opened serial", args.serial)

    # video capture (IP webcam URL or index)
    try:
        src = int(args.source)
    except Exception:
        src = args.source
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        print("Failed to open source:", src)
        return

    recent = deque(maxlen=GESTURE_WINDOW)
    last_hand_time = time.time()
    last_command_time = 0

    with mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=1,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    ) as hands:

        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.05)
                continue
            # IP Webcam streams are often rotated / smaller -> flip for natural gestures
            frame = cv2.flip(frame, 1)
            h, w = frame.shape[:2]
            img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = hands.process(img_rgb)

            gesture_here = None

            if results.multi_hand_landmarks:
                hand = results.multi_hand_landmarks[0]
                fingers = count_raised_fingers(hand, h, w)
                gesture_here = fingers
                recent.append(fingers)
                last_hand_time = time.time()
                # draw landmarks for debugging
                if args.show:
                    mp_drawing.draw_landmarks(frame, hand, mp_hands.HAND_CONNECTIONS)
                    cv2.putText(frame, f"fingers:{fingers}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,0),2)
            # if no hand, we do not append; majority will remain until timeout then send stop
            # compute majority
            if recent:
                maj = Counter(recent).most_common(1)[0][0]
            else:
                maj = None

            # decide target command
            if time.time() - last_hand_time > NO_HAND_TIMEOUT:
                cmd_char, speed = ('S', 0)  # no-hand -> stop
            elif maj is not None and maj in GESTURE_MAP:
                cmd_char, speed = GESTURE_MAP[maj]
            else:
                cmd_char, speed = ('S', 0)

            # send to Arduino if changed
            now = time.time()
            # send whenever majority changes or periodically for safety
            if (now - last_command_time) > 0.05:  # 20 Hz cap
                sender.send(cmd_char, speed)
                last_command_time = now

            # overlay
            if args.show:
                cv2.putText(frame, f"CMD:{cmd_char}{speed}", (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,0),2)
                cv2.imshow("GestureCar", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    cap.release()
    sender.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    from collections import Counter
    main()

