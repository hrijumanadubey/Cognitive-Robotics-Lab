from gpiozero import LED, DistanceSensor
from time import sleep

# --- Traffic light LEDs ---

# Light 1
red1 = LED(17)
yellow1 = LED(27)
green1 = LED(22)

# Light 2
red2 = LED(26)
yellow2 = LED(6)
green2 = LED(5)

# Light 3
red3 = LED(21)
yellow3 = LED(20)
green3 = LED(16)

# --- Ultrasonic Sensors ---

sensor1 = DistanceSensor(echo=18, trigger=23, max_distance=2.0)
sensor2 = DistanceSensor(echo=24, trigger=25, max_distance=2.0)
sensor3 = DistanceSensor(echo=4, trigger=12, max_distance=2.0)

# --- Constants ---

THRESHOLD = 0.3      # meters, distance below which congestion is assumed
GREEN_TIME = 5       # default green light duration (seconds)
YELLOW_TIME = 2      # yellow light duration (seconds)
ALL_RED_TIME = 1     # all red duration for safety

# Adaptive priority green time parameters
MIN_PRIORITY_TIME = 3   # minimum green time when congestion detected
MAX_PRIORITY_TIME = 10  # maximum green time when congestion detected

# --- Functions to control lights ---

def all_red():
    red1.on(); yellow1.off(); green1.off()
    red2.on(); yellow2.off(); green2.off()
    red3.on(); yellow3.off(); green3.off()

def set_light_green(light_num):
    if light_num == 1:
        green1.on(); yellow1.off(); red1.off()
        red2.on(); yellow2.off(); green2.off()
        red3.on(); yellow3.off(); green3.off()
    elif light_num == 2:
        red1.on(); yellow1.off(); green1.off()
        green2.on(); yellow2.off(); red2.off()
        red3.on(); yellow3.off(); green3.off()
    elif light_num == 3:
        red1.on(); yellow1.off(); green1.off()
        red2.on(); yellow2.off(); green2.off()
        green3.on(); yellow3.off(); red3.off()

def set_light_yellow(light_num):
    if light_num == 1:
        green1.off(); yellow1.on(); red1.off()
        red2.on(); yellow2.off(); green2.off()
        red3.on(); yellow3.off(); green3.off()
    elif light_num == 2:
        red1.on(); yellow1.off(); green1.off()
        green2.off(); yellow2.on(); red2.off()
        red3.on(); yellow3.off(); green3.off()
    elif light_num == 3:
        red1.on(); yellow1.off(); green1.off()
        red2.on(); yellow2.off(); green2.off()
        green3.off(); yellow3.on(); red3.off()

def get_priority_time(distance):
    """
    Calculate adaptive green time based on congestion distance.
    Closer distance => longer green time (between MIN_PRIORITY_TIME and MAX_PRIORITY_TIME).
    """
    if distance >= THRESHOLD:
        return GREEN_TIME  # No congestion, use default green time
    
    # Linear interpolation between MIN_PRIORITY_TIME and MAX_PRIORITY_TIME
    ratio = (THRESHOLD - distance) / THRESHOLD
    adaptive_time = MIN_PRIORITY_TIME + ratio * (MAX_PRIORITY_TIME - MIN_PRIORITY_TIME)
    
    # Clamp the time within min and max bounds
    return max(MIN_PRIORITY_TIME, min(adaptive_time, MAX_PRIORITY_TIME))

# --- Main loop ---

try:
    while True:
        # Debug: Print sensor distances
        print(f"Sensor1 Distance: {sensor1.distance:.2f} m")
        print(f"Sensor2 Distance: {sensor2.distance:.2f} m")
        print(f"Sensor3 Distance: {sensor3.distance:.2f} m")

        congested_road = None
        if sensor1.distance < THRESHOLD:
            congested_road = 1
        elif sensor2.distance < THRESHOLD:
            congested_road = 2
        elif sensor3.distance < THRESHOLD:
            congested_road = 3

        if congested_road:
            print(f"🚦 Congestion detected on road {congested_road}. Giving priority.")
            all_red()
            sleep(ALL_RED_TIME)

            # Get adaptive green time based on congestion severity
            if congested_road == 1:
                priority_time = get_priority_time(sensor1.distance)
            elif congested_road == 2:
                priority_time = get_priority_time(sensor2.distance)
            else:
                priority_time = get_priority_time(sensor3.distance)

            print(f"Priority green time: {priority_time:.2f} seconds")
            set_light_green(congested_road)
            sleep(priority_time)

            set_light_yellow(congested_road)
            sleep(YELLOW_TIME)

            all_red()
            sleep(ALL_RED_TIME)
        else:
            print("No congestion detected. Running normal cycle.")

            # Normal cycle road 1
            print("🟢 Light 1 GREEN | Others RED")
            set_light_green(1)
            sleep(GREEN_TIME)

            print("🟡 Light 1 YELLOW")
            set_light_yellow(1)
            sleep(YELLOW_TIME)

            all_red()
            sleep(ALL_RED_TIME)

            # Normal cycle road 2
            print("🟢 Light 2 GREEN | Others RED")
            set_light_green(2)
            sleep(GREEN_TIME)

            print("🟡 Light 2 YELLOW")
            set_light_yellow(2)
            sleep(YELLOW_TIME)

            all_red()
            sleep(ALL_RED_TIME)

            # Normal cycle road 3
            print("🟢 Light 3 GREEN | Others RED")
            set_light_green(3)
            sleep(GREEN_TIME)

            print("🟡 Light 3 YELLOW")
            set_light_yellow(3)
            sleep(YELLOW_TIME)

            all_red()
            sleep(ALL_RED_TIME)

except KeyboardInterrupt:
    print("\nStopping traffic system safely...")
    red1.off(); yellow1.off(); green1.off()
    red2.off(); yellow2.off(); green2.off()
    red3.off(); yellow3.off(); green3.off()
