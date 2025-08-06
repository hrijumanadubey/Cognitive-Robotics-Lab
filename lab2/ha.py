import RPi.GPIO as GPIO
import time

# --- CONFIGURATION ---

# Ultrasonic sensor pins (Backdoor)
BACK_TRIG = 2
BACK_ECHO = 3

# LEDs for alert
LED1 = 27  # Red LED
LED2 = 22  # Yellow LED

# Keypad pins
ROW_PINS = [5, 6, 13, 19]    # Rows 1-4
COL_PINS = [12, 16, 20]      # Columns 1-3

# Servo pin
SERVO_PIN = 18

# Password
PASSWORD = "1234"

# Keypad layout
KEYPAD = [
    ['1', '2', '3'],
    ['4', '5', '6'],
    ['7', '8', '9'],
    ['*', '0', '#']
]

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Setup ultrasonic pins
GPIO.setup(BACK_TRIG, GPIO.OUT)
GPIO.setup(BACK_ECHO, GPIO.IN)

# Setup LEDs
GPIO.setup(LED1, GPIO.OUT)
GPIO.setup(LED2, GPIO.OUT)

# Setup keypad rows as outputs
for row in ROW_PINS:
    GPIO.setup(row, GPIO.OUT)
    GPIO.output(row, GPIO.LOW)

# Setup keypad columns as inputs with pull-down resistors
for col in COL_PINS:
    GPIO.setup(col, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Setup servo
GPIO.setup(SERVO_PIN, GPIO.OUT)
servo = GPIO.PWM(SERVO_PIN, 50)  # 50Hz PWM frequency
servo.start(0)

def set_servo_angle(angle):
    duty = 2 + (angle / 18)
    servo.ChangeDutyCycle(duty)
    time.sleep(0.5)
    servo.ChangeDutyCycle(0)

def get_key():
    # Scan keypad for pressed key
    for row_num, row_pin in enumerate(ROW_PINS):
        GPIO.output(row_pin, GPIO.HIGH)
        for col_num, col_pin in enumerate(COL_PINS):
            if GPIO.input(col_pin) == GPIO.HIGH:
                GPIO.output(row_pin, GPIO.LOW)
                return KEYPAD[row_num][col_num]
        GPIO.output(row_pin, GPIO.LOW)
    return None

def get_distance(trig, echo):
    # Trigger ultrasonic sensor
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    pulse_start = None
    pulse_end = None

    timeout = time.time() + 0.04

    while GPIO.input(echo) == 0:
        pulse_start = time.time()
        if pulse_start > timeout:
            return None

    timeout = time.time() + 0.04

    while GPIO.input(echo) == 1:
        pulse_end = time.time()
        if pulse_end > timeout:
            return None

    if pulse_start is None or pulse_end is None:
        return None

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 34300 / 2  # cm
    return distance

try:
    input_password = ""
    print("System ready. Enter password on keypad.")

    while True:
        # Read ultrasonic distance
        back_distance = get_distance(BACK_TRIG, BACK_ECHO)
        backdoor_alert = back_distance is not None and back_distance < 50

        if backdoor_alert:
            # Blink LEDs alternately 10 times (total 4 seconds)
            for i in range(10):
                GPIO.output(LED1, True)
                GPIO.output(LED2, False)
                time.sleep(0.2)
                GPIO.output(LED1, False)
                GPIO.output(LED2, True)
                time.sleep(0.2)
            # Turn off LEDs after blinking
            GPIO.output(LED1, False)
            GPIO.output(LED2, False)

        else:
            # LEDs off if no alert
            GPIO.output(LED1, False)
            GPIO.output(LED2, False)

        # Check keypad for key press
        key = get_key()
        if key:
            print(f"Key pressed: {key}")

            if key == '#':  # '#' acts as Enter
                if input_password == PASSWORD:
                    print("Password correct! Unlocking door...")
                    set_servo_angle(90)  # Open door
                    time.sleep(5)        # Keep door open for 5 seconds
                    set_servo_angle(0)   # Close door
                    print("Door locked.")
                else:
                    print("Incorrect password!")
                input_password = ""

            elif key == '*':  # '*' clears input
                input_password = ""
                print("Password cleared.")

            else:
                input_password += key

            time.sleep(0.3)  # Debounce delay for keypad

        # Small delay to prevent CPU hogging
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nExiting and cleaning up GPIO...")
finally:
    servo.stop()
    GPIO.cleanup()
