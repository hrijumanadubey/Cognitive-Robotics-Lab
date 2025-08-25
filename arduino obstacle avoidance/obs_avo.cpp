#include <Servo.h>

// --- Pin Definitions ---

// Motor Driver Pins (from PDF)
#define ENA 5  // Enable A for Left Motors
#define ENB 6  // Enable B for Right Motors
#define IN1 7  // Input 1 for Left Motors
#define IN2 8  // Input 2 for Left Motors
#define IN3 9  // Input 3 for Right Motors
#define IN4 11 // Input 4 for Right Motors

// Ultrasonic Sensor Pins (Updated with correct pins)
#define trigPin A5 // Trigger pin is connected to A5 on the V5 shield
#define echoPin A4 // Echo pin is connected to A4 on the V5 shield

// Servo Motor Pin (Corrected Pin)
#define servoPin 3 // Signal pin for the servo motor

// --- Constants ---
const int obstacleDistance = 15; // Distance in cm to stop and check for obstacles
const int centerAngle = 130;     // Calibrated center position for the servo

// --- Global Variables ---
Servo ultrasonicServo; // Create a servo object to control the sensor rotation
long duration;         // Variable to store the time taken for the ultrasonic pulse
int distance;          // Variable to store the calculated distance

void setup()
{
    // Initialize Serial Monitor for debugging
    Serial.begin(9600);

    // Set all motor control pins to OUTPUT
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // Set ultrasonic sensor pins
    pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
    pinMode(echoPin, INPUT);  // Sets the echoPin as an Input

    // Attach the servo on its pin
    ultrasonicServo.attach(servoPin);
    ultrasonicServo.write(centerAngle); // Start with the sensor looking forward
    delay(1000);                        // Wait for the servo to reach the position
}

void loop()
{
    // Always start by ensuring the servo is centered before moving
    ultrasonicServo.write(centerAngle);

    // Check the distance in front of the car first
    distance = calculateDistance();
    Serial.print("Forward Distance: ");
    Serial.println(distance);

    // If an obstacle is detected within the threshold distance
    if (distance <= obstacleDistance && distance > 0)
    { // Added check for distance > 0 to avoid false readings
        Serial.println("Obstacle detected!");

        // Stop, then reverse for a moment (approx. 5cm, adjust delay if needed)
        stopMotors();
        delay(100);
        moveBackward();
        delay(300); // This delay controls how far back it goes.
        stopMotors();
        delay(300);

        // Scan for the best path in 5 directions
        scanForBestPath();
    }
    else
    {
        // If no obstacle, move forward
        Serial.println("Path is clear. Moving forward.");
        moveForward();
    }
}

// --- New Scanning and Decision Function ---
void scanForBestPath()
{
    Serial.println("Scanning for the best path...");
    int distances[5];
    int angles[] = {centerAngle - 90, centerAngle - 45, centerAngle, centerAngle + 45, 180}; // Right, F-Right, Front, F-Left, Left

    // Scan all 5 angles
    for (int i = 0; i < 5; i++)
    {
        ultrasonicServo.write(angles[i]);
        delay(300);
        distances[i] = calculateDistance();
        Serial.print("Angle ");
        Serial.print(angles[i]);
        Serial.print(": ");
        Serial.println(distances[i]);
    }

    // Reset servo to center
    ultrasonicServo.write(centerAngle);

    // Find the direction with the maximum distance
    int maxDist = 0;
    int bestAngleIndex = -1;
    for (int i = 0; i < 5; i++)
    {
        if (distances[i] > maxDist)
        {
            maxDist = distances[i];
            bestAngleIndex = i;
        }
    }

    // --- Decision Making ---
    // Turn towards the direction with the most free space
    switch (bestAngleIndex)
    {
    case 0: // Full Right
        Serial.println("Path is clearest to the right. Turning right.");
        turnRight();
        delay(360); // Longer turn for full right (Reduced by 40%)
        break;
    case 1: // Front-Right
        Serial.println("Path is clearest to the front-right. Turning slightly right.");
        turnRight();
        delay(180); // Shorter turn (Reduced by 40%)
        break;
    case 2: // Front
        Serial.println("Path is clearest straight ahead. Moving forward.");
        moveForward();
        delay(200);
        break;
    case 3: // Front-Left
        Serial.println("Path is clearest to the front-left. Turning slightly left.");
        turnLeft();
        delay(180); // Shorter turn (Reduced by 40%)
        break;
    case 4: // Full Left
        Serial.println("Path is clearest to the left. Turning left.");
        turnLeft();
        delay(360); // Longer turn for full left (Reduced by 40%)
        break;
    default: // If no path is clear (e.g., in a corner), turn around
        Serial.println("No clear path. Turning around.");
        turnRight();
        delay(1000);
        break;
    }
}

// --- Movement Functions ---

// Function to make the car move forward (Corrected Direction)
void moveForward()
{
    analogWrite(ENA, 200);
    analogWrite(ENB, 200);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

// Function to make the car move backward
void moveBackward()
{
    analogWrite(ENA, 200);
    analogWrite(ENB, 200);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

// Function to stop the car
void stopMotors()
{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

// Function to make the car turn right on the spot (Corrected Direction)
void turnRight()
{
    analogWrite(ENA, 200);
    analogWrite(ENB, 200);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

// Function to make the car turn left on the spot (Corrected Direction)
void turnLeft()
{
    analogWrite(ENA, 200);
    analogWrite(ENB, 200);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

// --- Sensor Function ---

// Function to calculate the distance from the ultrasonic sensor
int calculateDistance()
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = duration * 0.034 / 2;
    return distance;
}
