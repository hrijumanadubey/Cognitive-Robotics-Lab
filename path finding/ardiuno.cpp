#include <NewPing.h>

// === Ultrasonic Sensor ===
#define TRIG_PIN A5

#define ECHO_PIN A4
#define MIN_DISTANCE 2   // cm
#define MAX_DISTANCE 200 // cm
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

// === Motor Pins (for rotation) ===
#define IN1 5
#define IN2 6
#define IN3 7
#define IN4 8
#define ENA 9
#define ENB 11

// === Mapping Settings ===
#define STEPS 8      // number of rotation steps (360° / 8 = 45° per step)
#define CELL_SIZE 20 // cm per grid cell

void setup()
{
    Serial.begin(9600);

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
}

void loop()
{
    for (int i = 0; i < STEPS; i++)
    {
        // Read ultrasonic sensor
        int distance = sonar.ping_cm();
        if (distance < MIN_DISTANCE)
            distance = MIN_DISTANCE;
        if (distance > MAX_DISTANCE)
            distance = MAX_DISTANCE;

        // Convert distance to grid cells relative to car
        int obsX = round((distance * cos(i * 2 * 3.14159 / STEPS)) / CELL_SIZE);
        int obsY = round((distance * sin(i * 2 * 3.14159 / STEPS)) / CELL_SIZE);

        // Send obstacle coordinates to Raspberry Pi
        Serial.print(obsX);
        Serial.print(",");
        Serial.print(obsY);
        Serial.print(",");
        Serial.println(distance);

        // Rotate car by one step
        rotateStep(360.0 / STEPS);
    }

    while (1)
        ; // Stop after one full 360° scan
}

// Rotate car in place by given degree
void rotateStep(float degree)
{
    analogWrite(ENA, 150);
    analogWrite(ENB, 150);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    int delayTime = degree * 10; // adjust for your kit
    delay(delayTime);

    stopMotors();
}

void stopMotors()
{
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}
